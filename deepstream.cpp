#include "deepstream.h"
#include <stdio.h>
#include <gst/rtsp-server/rtsp-server.h>
#include "gst-nvmessage.h"
#include "nvdsgstutils.h"
#include "deepstream_common.h"
#include "gst-nvdssr.h"
#include "gstnvdsmeta.h"
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "config_mgr.h"
#include <deepstream_config_file_parser.h>

enum
{
    HW_NOIMAGE = 0,
};

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

#define TILED_OUTPUT_WIDTH 1280 // 1920
#define TILED_OUTPUT_HEIGHT 720 // 1080

#define GST_CAPS_FEATURES_NVMM "memory:NVMM"

/* YOLO class ID */
#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2
#define PGIE_CLASS_ID_CAR 2
#define PGIE_CLASS_ID_BUS 5
#define PGIE_CLASS_ID_TRUCK 7

#define MAX_STUCK_CARS 5
#define MAX_DISPLAY_LEN 64
#define RED "\033[0;32;31m"
#define LIGHT_GREEN "\033[1;32m"
#define NONE "\033[m"
#define CYAN "\033[0;36m"

typedef struct
{
    struct timeval ts;
    struct timeval te;
    int32_t id;
} VechicleStuck;

static GstRTSPServer *server;
static guint bus_watch_id;
static guint lane_vehicle_cnt[MAX_LANES];
static uint32_t vehicle_overload_time[MAX_LANES];
static VechicleStuck lanes_stuck[MAX_LANES][MAX_STUCK_CARS];
extern GstElement *pipeline;
extern NvDsSRContext *ctx;
extern guint line_cnt;
extern guint event_lane;
extern guint display_resolution[2];
extern guint label_display;
extern GstElement *plugin;
extern gdouble gravity_hori;
extern gdouble gravity_vert;
extern uint32_t hw_status;
extern gchar *src_uri;
extern gchar *out_rtsp_name;
extern uint32_t event[128];
extern GMainLoop *loop;
extern Line *lines;
extern gchar *lane_line;
extern gint stuck_car_count;

extern void record_handle(bool isStuck, int _event_lane);
extern gboolean parse_lane(gchar *lane_line);
static gdouble lane_colors[10][3] = {
    {0.9, 0.5, 0.0},
    {0.0, 0.9, 0.0},
    {0.0, 0.0, 0.9},
    {0.9, 0.9, 0.0},
    {0.0, 0.9, 0.9},
    {0.9, 0.0, 0.9},
    {0.5, 0.5, 0.0},
    {0.0, 0.9, 0.5},
    {0.5, 0.0, 0.9},
    {0.9, 0.9, 0.9}};

/*
 * y[n] = a*y[n-1]+b*x[n]
 */
static gdouble iir_filter(gint new_x, gdouble old_y)
{
    gdouble y;

    y = 0.8 * old_y + 0.2 * new_x;

    return y;
}

static int calculate_lane_vehicles(Line *line, guint x, guint y)
{
    gint val = 0;
    gint val_old = 0;
    guint lane_id = 0;

    for (int i = 0; i < line_cnt; i++)
    {
        val = line[i].a * x + line[i].b * y + line[i].c;
        if ((val_old != 0) && ((val ^ val_old) < 0))
            break;
        val_old = val;
        lane_id++;
    }

    return lane_id;
}

gboolean check_lane_stuck(int lane_id, int vehicle_id, struct timeval t, int *duration)
{
    int i = 0;

    for (i = 0; i < MAX_STUCK_CARS; i++)
    {
        if (lanes_stuck[lane_id][i].id == vehicle_id)
        {
            lanes_stuck[lane_id][i].te = t;
            int diff_t = t.tv_sec - lanes_stuck[lane_id][i].ts.tv_sec;
            // g_print(CYAN"id: %d, duration = %d, t = %d\n", vehicle_id, diff_t, t.tv_sec);
            *duration = diff_t;
            if (diff_t > 15)
            {
                // g_print(RED"id %d: (%d, %d) stuck...%d\n"NONE,
                //		vehicle_id, lane_id, i, diff_t);
                return true;
            }
            return false;
        }

        if (((t.tv_sec - lanes_stuck[lane_id][i].te.tv_sec) > 2) &&
            (lanes_stuck[lane_id][i].id != -1))
        {
            // g_print("id: %d(%d) exit....te: %ld, t: %ld\n",
            //		    lanes_stuck[lane_id][i].id, i,
            //		    lanes_stuck[lane_id][i].te.tv_sec,
            //		    t.tv_sec);
            lanes_stuck[lane_id][i].id = -1;
        }
    }

    for (i = 0; i < MAX_STUCK_CARS; i++)
    {
        if (lanes_stuck[lane_id][i].id == -1)
        {
            lanes_stuck[lane_id][i].id = vehicle_id;
            lanes_stuck[lane_id][i].ts = t;
            lanes_stuck[lane_id][i].te = t;
            *duration = 0;
            // g_print(LIGHT_GREEN"id: %d add(%d, %d): ....%ld\n"NONE,
            //		    vehicle_id,
            //		    lane_id, i, t.tv_sec);
            return false;
        }
    }
    return false;
}

/* tiler_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */
static GstPadProbeReturn
tiler_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info,
                           gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *)info->data;
    NvDsObjectMeta *obj_meta = NULL;
    guint vehicle_size = 0;
    guint vehicle_pos_x = 0;
    guint vehicle_pos_y = 0;
    guint lane_no = 0;
    guint vehicle_total_cnt = 0;
    gboolean alert = FALSE;
    guint obj_size_limit = (TILED_OUTPUT_WIDTH / 15) * (TILED_OUTPUT_HEIGHT / 15);
    NvDsMetaList *l_frame = NULL;
    NvDsMetaList *l_obj = NULL;
    NvDsDisplayMeta *display_meta = NULL;
    NvOSD_ColorParams text_clr;
    int lane_vehicle_len[line_cnt];
    int duration;
    struct timeval tv;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    gettimeofday(&tv, NULL);
    event_lane = 0;

    for (int i = 0; i < (int)(line_cnt - 1); i++)
    {
        lane_vehicle_cnt[i] = 0;
        lane_vehicle_len[i] = 0;
        vehicle_overload_time[i] = 0;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
         l_frame = l_frame->next)
    {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
        int offset = 0;
        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next)
        {
            obj_meta = (NvDsObjectMeta *)(l_obj->data);
            obj_meta->text_params.font_params.font_size = 20;
            obj_meta->text_params.font_params.font_color.alpha = label_display;
            // g_print("osd txt: %s\n", obj_meta->text_params.display_text);
            if ((obj_meta->class_id == PGIE_CLASS_ID_CAR) ||
                (obj_meta->class_id == PGIE_CLASS_ID_BUS) ||
                (obj_meta->class_id == PGIE_CLASS_ID_TRUCK))
            {
                vehicle_total_cnt++;
                if ((lane_no > 0) && (lane_no < line_cnt))
                {
                    lane_vehicle_cnt[lane_no - 1]++;
                    // g_print("lane no = %d, cnt = %d, avg = %f\n", lane_no -1, lane_vehicle_cnt[lane_no-1], vehicle_avg[lane_no-1]);
                }
            }
            else
                continue;

            if (line_cnt != 0)
            {
                // vehicle_size = (guint)(obj_meta->rect_params.width*obj_meta->rect_params.height);
                vehicle_pos_x = (guint)(obj_meta->rect_params.left + obj_meta->rect_params.width * gravity_hori);
                vehicle_pos_y = (guint)(obj_meta->rect_params.top + obj_meta->rect_params.height * gravity_vert);
                lane_no = calculate_lane_vehicles(lines, vehicle_pos_x, vehicle_pos_y);

                // g_print("%s(%ld)\n",obj_meta->text_params.display_text, obj_meta->object_id, lane_no);

                text_clr.red = lane_colors[lane_no - 1][0];
                text_clr.green = lane_colors[lane_no - 1][1];
                text_clr.blue = lane_colors[lane_no - 1][2];

                obj_meta->text_params.text_bg_clr = text_clr;
                obj_meta->rect_params.border_color = text_clr;
                obj_meta->rect_params.border_width = label_display * 2;
                obj_meta->text_params.text_bg_clr.alpha = 0.7 * label_display;
                obj_meta->rect_params.border_color.alpha = 1.0 * label_display;
                if ((obj_meta->rect_params.width < 30) || (obj_meta->rect_params.height < 30))
                {
                    obj_meta->text_params.text_bg_clr.alpha = 0;
                    obj_meta->rect_params.border_color.alpha = 0;
                    obj_meta->rect_params.border_width = 0;
                    obj_meta->text_params.font_params.font_color.alpha = 0;
                    // g_print("skip...\n");
                    continue;
                }
            }
        }

        if (label_display)
        {
            display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
            NvOSD_TextParams *txt_params = &display_meta->text_params[0];
            NvOSD_TextParams *title = &display_meta->text_params[1];
            display_meta->num_labels = 2;
            txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);
            title->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);

            offset += snprintf(txt_params->display_text + offset, MAX_DISPLAY_LEN, "車道 %d: %d     ", 1, lane_vehicle_cnt[0]);

            if (line_cnt == 0)
            {
                offset += snprintf(txt_params->display_text + offset, MAX_DISPLAY_LEN, "總車輛: %d ", vehicle_total_cnt);
            }
            snprintf(title->display_text, MAX_DISPLAY_LEN, "事件偵測攝影機");

            // g_print("%s\n", txt_params->display_text);
            if (lane_vehicle_cnt[0] >= stuck_car_count)
            {
                record_handle(true, 0);
                txt_params->font_params.font_color.red = 1.0;
                txt_params->font_params.font_color.green = 0.2;
                txt_params->font_params.font_color.blue = 0.1;
            }
            else
            {
                txt_params->font_params.font_color.red = 0.0;
                txt_params->font_params.font_color.green = 0.8;
                txt_params->font_params.font_color.blue = 0.4;
            }

            title->font_params.font_color.red = 1.0;
            title->font_params.font_color.green = 0.2;
            title->font_params.font_color.blue = 0.1;
            /* Now set the offsets where the string should appear */
            txt_params->x_offset = 0;
            txt_params->y_offset = display_resolution[1] - 50;

            title->x_offset = 0;
            title->y_offset = 50;

            /* Font , font-color and font-size */
            txt_params->font_params.font_name = (char *)"Serif";
            txt_params->font_params.font_size = 25;
            txt_params->font_params.font_color.alpha = 1.0;
            title->font_params.font_name = (char *)"Serif";
            title->font_params.font_size = 25;
            title->font_params.font_color.alpha = 1.0;

            /* Text background color */
            txt_params->set_bg_clr = 1;
            txt_params->text_bg_clr.red = 0.0;
            txt_params->text_bg_clr.green = 0.0;
            txt_params->text_bg_clr.blue = 0.0;
            txt_params->text_bg_clr.alpha = 0.8;

            title->set_bg_clr = 1;
            title->text_bg_clr.red = 0.0;
            title->text_bg_clr.green = 0.0;
            title->text_bg_clr.blue = 0.0;
            title->text_bg_clr.alpha = 0.8;

            nvds_add_display_meta_to_frame(frame_meta, display_meta);
        }
    }
    return GST_PAD_PROBE_OK;
}

static gboolean set_tracker_properties(GstElement *nvtracker)
{
    gboolean ret = FALSE;
    GError *error = NULL;
    gchar **keys = NULL;
    gchar **key = NULL;
    GKeyFile *key_file = g_key_file_new();

    if (!g_key_file_load_from_file(key_file, TRACKER_CONFIG_FILE, G_KEY_FILE_NONE,
                                   &error))
    {
        g_printerr("Failed to load config file: %s\n", error->message);
        return FALSE;
    }

    keys = g_key_file_get_keys(key_file, CONFIG_GROUP_TRACKER, NULL, &error);
    CHECK_ERROR(error);

    for (key = keys; *key; key++)
    {
        if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_WIDTH))
        {
            gint width =
                g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                       CONFIG_GROUP_TRACKER_WIDTH, &error);
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "tracker-width", width, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_HEIGHT))
        {
            gint height =
                g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                       CONFIG_GROUP_TRACKER_HEIGHT, &error);
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "tracker-height", height, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GPU_ID))
        {
            guint gpu_id =
                g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                       CONFIG_GPU_ID, &error);
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "gpu_id", gpu_id, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_CONFIG_FILE))
        {
            char *ll_config_file = get_absolute_file_path((char *)TRACKER_CONFIG_FILE,
                                                          g_key_file_get_string(key_file,
                                                                                (char *)CONFIG_GROUP_TRACKER,
                                                                                (char *)CONFIG_GROUP_TRACKER_LL_CONFIG_FILE, &error));
            CHECK_ERROR(error);
            g_print("load ll-config-file....\n");
            g_object_set(G_OBJECT(nvtracker), "ll-config-file", ll_config_file, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE))
        {
            char *ll_lib_file = get_absolute_file_path((char *)TRACKER_CONFIG_FILE,
                                                       g_key_file_get_string(key_file,
                                                                             (char *)CONFIG_GROUP_TRACKER,
                                                                             (char *)CONFIG_GROUP_TRACKER_LL_LIB_FILE, &error));
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "ll-lib-file", ll_lib_file, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS))
        {
            gboolean enable_batch_process =
                g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                       CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS, &error);
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "enable-batch-process",
                         enable_batch_process, NULL);
        }
        else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_ENABLE_PAST_FRAME))
        {
            gboolean enable_past_frame =
                g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                       CONFIG_GROUP_TRACKER_ENABLE_PAST_FRAME, &error);
            CHECK_ERROR(error);
            g_object_set(G_OBJECT(nvtracker), "enable-past-frame",
                         enable_past_frame, NULL);
        }
        else
        {
            g_printerr("Unknown key '%s' for group [%s]", *key,
                       CONFIG_GROUP_TRACKER);
        }
    }

    ret = TRUE;
done:
    if (error)
    {
        g_error_free(error);
    }
    if (keys)
    {
        g_strfreev(keys);
    }
    if (!ret)
    {
        g_printerr("%s failed", __func__);
    }
    return ret;
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_WARNING:
    {
        gchar *debug;
        GError *error;
        gst_message_parse_warning(msg, &error, &debug);
        g_printerr("WARNING from element %s: %s\n",
                   GST_OBJECT_NAME(msg->src), error->message);
        g_free(debug);
        g_printerr("Warning: %s\n", error->message);
        g_error_free(error);
        break;
    }
    case GST_MESSAGE_ERROR:
    {
        gchar *debug;
        GError *error;
        int cnt = 10;
        gst_message_parse_error(msg, &error, &debug);
        g_printerr("ERROR from element %s: %s\n",
                   GST_OBJECT_NAME(msg->src), error->message);
        if (debug)
            g_printerr("Error details: %s\n", debug);
        memset(src_uri, 0, 1024);
        hw_status |= (1 << HW_NOIMAGE);
        if (strstr(error->message, "resource"))
        {
            while ((strlen(src_uri) == 0) && (cnt > 0))
            {
                g_print("img N/A....\n");
                cnt--;
                sleep(1);
            }
        }
        g_free(debug);
        g_error_free(error);
        g_main_loop_quit(loop);
        break;
    }
#ifndef PLATFORM_TEGRA
    case GST_MESSAGE_ELEMENT:
    {
        if (gst_nvmessage_is_stream_eos(msg))
        {
            guint stream_id;
            if (gst_nvmessage_parse_stream_eos(msg, &stream_id))
            {
                g_print("Got EOS from stream %d\n", stream_id);
            }
        }
        break;
    }
#endif
    default:
        break;
    }
    return TRUE;
}

gboolean start_rtsp_streaming(guint rtsp_port_num, guint updsink_port_num,
                              guint64 udp_buffer_size)
{
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;
    GstRTSPAuth *auth;
    GstRTSPToken *token;
    gchar *basic;
    char udpsrc_pipeline[512];
    char port_num_Str[64] = {0};
    char *encoder_name;

    if (udp_buffer_size == 0)
        udp_buffer_size = 512 * 1024;

    sprintf(udpsrc_pipeline,
            "( udpsrc name=pay0 port=%d buffer-size=%lu caps=\"application/x-rtp, media=video, "
            "clock-rate=90000, encoding-name=H264, payload=96 \" )",
            updsink_port_num, udp_buffer_size);

    sprintf(port_num_Str, "%d", rtsp_port_num);

    server = gst_rtsp_server_new();
    g_object_set(server, "service", port_num_Str, NULL);

    mounts = gst_rtsp_server_get_mount_points(server);

    factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, udpsrc_pipeline);
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_mount_points_add_factory(mounts, out_rtsp_name, factory);
    g_object_unref(mounts);

    gst_rtsp_media_factory_add_role(factory, "admin",
                                    GST_RTSP_PERM_MEDIA_FACTORY_ACCESS, G_TYPE_BOOLEAN, TRUE,
                                    GST_RTSP_PERM_MEDIA_FACTORY_CONSTRUCT, G_TYPE_BOOLEAN, TRUE, NULL);
    /* make user token */
    token =
        gst_rtsp_token_new(GST_RTSP_TOKEN_MEDIA_FACTORY_ROLE, G_TYPE_STRING,
                           "admin", NULL);
    auth = gst_rtsp_auth_new();
    basic = gst_rtsp_auth_make_basic("admin", "admin");
    gst_rtsp_auth_add_basic(auth, basic, token);
    g_free(basic);
    gst_rtsp_token_unref(token);

    /* configure in the server */
    gst_rtsp_server_set_auth(server, auth);
    g_object_unref(auth);

    gst_rtsp_server_attach(server, NULL);

    g_print("\n *** DeepStream: Launched RTSP Streaming at rtsp://localhost:%d%s ***\n\n",
            rtsp_port_num, out_rtsp_name);

    return TRUE;
}

static void cb_new_rtspsrc_pad(GstElement *element, GstPad *pad, gpointer data)
{
    gchar *name;
    GstCaps *p_caps;
    gchar *description;
    GstElement *p_rtph264depay;

    name = gst_pad_get_name(pad);

    // here, you would setup a new pad link for the newly created pad
    // sooo, now find that rtph264depay is needed and link them?
    p_caps = gst_pad_get_pad_template_caps(pad);

    description = gst_caps_to_string(p_caps);
    g_free(description);

    p_rtph264depay = GST_ELEMENT(data);

    g_print("%s: name = %s\n", __func__, name);
    // try to link the pads then ...
    if (!gst_element_link_pads(element, name, p_rtph264depay, "sink"))
    {
        printf("Failed to link rtsp elements 3\n");
    }

    g_free(name);
}

static void qtdemux_pad_added_cb(GstElement *qtdemux, GstPad *pad, GstElement *mjpegdec)
{
    gst_element_link_pads(qtdemux, GST_PAD_NAME(pad), mjpegdec, NULL);
}

static gpointer smart_record_callback(NvDsSRRecordingInfo *info, gpointer userData)
{
    static GMutex mutex;
    char file_name_new[256];
    char file_name_old[256];
    g_return_val_if_fail(info, NULL);

    // g_print(LIGHT_GREEN"Smart record....\n"NONE);
    g_mutex_lock(&mutex);
    sprintf(file_name_old, "%s/%s", info->dirpath, info->filename);
    sprintf(file_name_new, "%s/Event-%d%02d%02d-%02d%02d%02d.mp4", info->dirpath,
            event[2], event[3], event[4], event[5], event[6], event[7]);
    rename(file_name_old, file_name_new);
    // g_print("%s->%s\n", file_name_old, file_name_new);
    memset(event, 0, sizeof(event));
    g_mutex_unlock(&mutex);

    return NULL;
}

#if 1
GstElement *create_rtsp_bin(guint index, gchar *uri, GstElement *pipeline, NvDsSRContext **ctx)
{
    GstElement *source = NULL, *parser = NULL,
               *decoder = NULL, *demux = NULL, *bin = NULL,
               *decque = NULL, *tee = NULL, *depay = NULL;
    gchar name[128];

    g_print("url: %s\n", uri);
    memset(name, 0, sizeof(name));
    sprintf(name, "nvv4l2-decoder-%d", index);
    decoder = gst_element_factory_make("nvv4l2decoder", name);
    memset(name, 0, sizeof(name));
    sprintf(name, "h264-parser-%d", index);
    parser = gst_element_factory_make("h264parse", name);

    memset(name, 0, sizeof(name));
    sprintf(name, "dec-que-%d", index);
    decque = gst_element_factory_make("queue", name);
    /*
        memset(name, 0, sizeof(name));
        sprintf(name, "srcc-tee-%d", index);
        tee = gst_element_factory_make ("tee", name);
        if (!parser || !decoder || !tee || !decque)
        {
            g_printerr("%s: One element could not be created...\n", __func__);
            return NULL;
        }
    */
    // gst_bin_add_many(GST_BIN(pipeline), parser, decoder, tee, decque, NULL);
    gst_bin_add_many(GST_BIN(pipeline), parser, decoder, NULL);
    /*
        if (!gst_element_link_many (parser, tee, NULL)) {
            g_printerr ("h254paser, tee could not be linked: 1. Exiting.\n");
            return NULL;
        }
    */
    if (!strncmp(uri, "rtsp", 4))
    {
        memset(name, 0, sizeof(name));
        sprintf(name, "rtspsrc%d", index);
        source = gst_element_factory_make("rtspsrc", name);
        memset(name, 0, sizeof(name));
        sprintf(name, "rtsp-depay-%d", index);
        depay = gst_element_factory_make("rtph264depay", name);
        if (!source || !depay)
        {
            g_print("source or rtph264 couldn't create...\n");
            return NULL;
        }
        gst_bin_add_many(GST_BIN(pipeline), source, depay, NULL);
        g_signal_connect(source, "pad-added", G_CALLBACK(cb_new_rtspsrc_pad), depay);
        if (!gst_element_link_many(depay, parser, NULL))
        {
            g_printerr("Elements could not be linked: 1. Exiting.\n");
            return NULL;
        }

        if (!gst_element_sync_state_with_parent(depay))
        {
            g_print(" failed to sync state with parent");
            return NULL;
        }
        gst_element_sync_state_with_parent(parser);

        g_object_set(G_OBJECT(source), "latency", 100, NULL);
        g_object_set(G_OBJECT(source), "drop-on-latency", TRUE, NULL);
        g_object_set(G_OBJECT(source), "protocols", 0x7, NULL);
        configure_source_for_ntp_sync(source);
    }
    /*
        if (!link_element_to_tee_src_pad(tee, decque)) {
           g_print("%s: tee, decque can't be linked.\n", __func__);
           return NULL;
        }
     */
    // if (!gst_element_link_many (decque, decoder, NULL)) {
    if (!gst_element_link_many(parser, decoder, NULL))
    {
        g_printerr("h254paser, tee could not be linked: 1. Exiting.\n");
        return NULL;
    }

    g_object_set(G_OBJECT(source), "location", uri, NULL);

    return decoder;
}

static void
cb_newpad(GstElement *decodebin, GstPad *decoder_src_pad, gpointer data)
{
    g_print("In cb_newpad\n");
    GstCaps *caps = gst_pad_get_current_caps(decoder_src_pad);
    const GstStructure *str = gst_caps_get_structure(caps, 0);
    const gchar *name = gst_structure_get_name(str);
    GstElement *source_bin = (GstElement *)data;
    GstCapsFeatures *features = gst_caps_get_features(caps, 0);

    /* Need to check if the pad created by the decodebin is for video and not
     * audio. */
    if (!strncmp(name, "video", 5))
    {
        /* Link the decodebin pad only if decodebin has picked nvidia
         * decoder plugin nvdec_*. We do this by checking if the pad caps contain
         * NVMM memory features. */
        if (gst_caps_features_contains(features, GST_CAPS_FEATURES_NVMM))
        {
            /* Get the source bin ghost pad */
            GstPad *bin_ghost_pad = gst_element_get_static_pad(source_bin, "src");
            if (!gst_ghost_pad_set_target(GST_GHOST_PAD(bin_ghost_pad),
                                          decoder_src_pad))
            {
                g_printerr("Failed to link decoder src pad to source bin ghost pad\n");
            }
            gst_object_unref(bin_ghost_pad);
        }
        else
        {
            g_printerr("Error: Decodebin did not pick nvidia decoder plugin.\n");
        }
    }
}

static void
decodebin_child_added(GstChildProxy *child_proxy, GObject *object,
                      gchar *name, gpointer user_data)
{
    g_print("Decodebin child added: %s\n", name);
    if (g_strrstr(name, "decodebin") == name)
    {
        g_signal_connect(G_OBJECT(object), "child-added",
                         G_CALLBACK(decodebin_child_added), user_data);
    }
}

static GstElement *create_source_bin2(guint index, gchar *uri)
{
    GstElement *bin = NULL, *uri_decode_bin = NULL;
    gchar bin_name[16] = {};

    g_snprintf(bin_name, 15, "source-bin-%02d", index);
    /* Create a source GstBin to abstract this bin's content from the rest of the
     * pipeline */
    bin = gst_bin_new(bin_name);

    /* Source element for reading from the uri.
     * We will use decodebin and let it figure out the container format of the
     * stream and the codec and plug the appropriate demux and decode plugins. */
    uri_decode_bin = gst_element_factory_make("uridecodebin", "uri-decode-bin");

    if (!bin || !uri_decode_bin)
    {
        g_printerr("One element in source bin could not be created.\n");
        return NULL;
    }

    /* We set the input uri to the source element */
    g_object_set(G_OBJECT(uri_decode_bin), "uri", uri, NULL);

    /* Connect to the "pad-added" signal of the decodebin which generates a
     * callback once a new pad for raw data has beed created by the decodebin */
    g_signal_connect(G_OBJECT(uri_decode_bin), "pad-added",
                     G_CALLBACK(cb_newpad), bin);
    g_signal_connect(G_OBJECT(uri_decode_bin), "child-added",
                     G_CALLBACK(decodebin_child_added), bin);

    gst_bin_add(GST_BIN(bin), uri_decode_bin);

    /* We need to create a ghost pad for the source bin which will act as a proxy
     * for the video decoder src pad. The ghost pad will not have a target right
     * now. Once the decode bin creates the video decoder and generates the
     * cb_newpad callback, we will set the ghost pad target to the video decoder
     * src pad. */
    if (!gst_element_add_pad(bin, gst_ghost_pad_new_no_target("src",
                                                              GST_PAD_SRC)))
    {
        g_printerr("Failed to add ghost pad in source bin\n");
        return NULL;
    }

    return bin;
}
#endif

int create_pipeline()
{
    GstElement *streammux = NULL, *pgie = NULL,
               *nvvidconv = NULL, *nvosd = NULL, *tiler = NULL,
               *rtph264pay = NULL, *sink_rtsp = NULL,
               *h264enc = NULL, *h264parse2 = NULL,
               *caps = NULL, *nvvidconv_postosd = NULL;
    GstElement *tee = NULL, *queue1 = NULL, *queue2 = NULL;
    GstElement *decque = NULL, *nvtracker = NULL, *sgie = NULL;
    GstBus *bus = NULL;
    GstPad *tiler_src_pad = NULL;
    GstPad *tee_render_pad = NULL;
    GstPad *tee_msg_pad = NULL;
    GstPad *sink_pad = NULL, *src_pad = NULL;
    guint pgie_batch_size;
    guint tiler_rows, tiler_columns;
    int num_sources = 1;

    /* Create gstreamer elements */
    /* Create Pipeline element that will form a connection of other elements */
    pipeline = gst_pipeline_new("truck_scale-pipeline");

    /* Create nvstreammux instance to form batches from one or more sources. */
    streammux = gst_element_factory_make("nvstreammux", "stream-muxer");

    if (!pipeline || !streammux)
    {
        g_printerr("pipeline/streammux element could not be created. Exiting.\n");
        return -1;
    }

    gst_bin_add(GST_BIN(pipeline), streammux);

    for (int i = 0; i < num_sources; i++)
    {
        gchar pad_name[16] = {};
        gchar ele_name[64];
        GstElement *decoder;

        g_print("create source bin....1, %d: %s\n", i, src_uri);
        if (src_uri && strlen(src_uri) != 0)
            decoder = create_source_bin2(i, src_uri);
        // decoder = create_rtsp_bin (i, src_uri, pipeline, &ctx);

        gst_bin_add(GST_BIN(pipeline), decoder);
        g_snprintf(pad_name, 15, "sink_%u", i);
        sink_pad = gst_element_get_request_pad(streammux, pad_name);
        if (!sink_pad)
        {
            g_printerr("Streammux request sink pad failed. Exiting.\n");
            return -1;
        }

        src_pad = gst_element_get_static_pad(decoder, "src");
        if (!src_pad)
        {
            g_printerr("Failed to get src pad of source bin. Exiting.\n");
            return -1;
        }

        if (gst_pad_link(src_pad, sink_pad) != GST_PAD_LINK_OK)
        {
            g_printerr("Failed to link source bin to stream muxer. Exiting.\n");
            return -1;
        }

        gst_object_unref(src_pad);
        gst_object_unref(sink_pad);
    }

    /* Use nvinfer to infer on batched frame. */
    pgie = gst_element_factory_make("nvinfer", "primary-nvinference-engine");

    /* We need to have a tracker to track the identified objects */
    nvtracker = gst_element_factory_make("nvtracker", "tracker");

    /* Use nvtiler to composite the batched frames into a 2D tiled array based
     * on the source of the frames. */
    tiler = gst_element_factory_make("nvmultistreamtiler", "nvtiler");

    /* Use convertor to convert from NV12 to RGBA as required by nvosd */
    nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");

    /* Create OSD to draw on the converted RGBA buffer */
    nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");

    /* Add plugin */
    plugin = gst_element_factory_make("dsexample", "plugin");

    /* Finally render the osd output */

    /* Create tee to render buffer and send message simultaneously*/
    tee = gst_element_factory_make("tee", "nvsink-tee");

    /* Create sink queues */
    queue1 = gst_element_factory_make("queue", "nvtee-que1");
    queue2 = gst_element_factory_make("queue", "nvtee-que2");

    /* Create rtsp */
    sink_rtsp = gst_element_factory_make("udpsink", "udp-sink");
    rtph264pay = gst_element_factory_make("rtph264pay", "rtppay");
    h264enc = gst_element_factory_make("nvv4l2h264enc", "enc");
    h264parse2 = gst_element_factory_make("h264parse", "h264-parser2");
    caps = gst_element_factory_make("capsfilter", "filter");
    nvvidconv_postosd = gst_element_factory_make("nvvideoconvert", "nvvidconv_postosd");

    if (!pgie || !tiler || !nvvidconv || !nvosd || !sink_rtsp ||
        !tee || !rtph264pay || !h264enc || !h264parse2 || !caps || !nvvidconv_postosd ||
        !queue1 || !queue2 || !nvtracker || !plugin)
    {
        g_printerr("pgie/tiler/nvvidconv/nvosd/sink_rtsp element could not be created. Exiting.\n");
        return -1;
    }

    g_object_set(G_OBJECT(streammux), "batch-size", num_sources, NULL);

    g_object_set(G_OBJECT(streammux), "width", display_resolution[0],
                 "height", display_resolution[1],
                 "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

    /* Configure the nvinfer element using the nvinfer config file. */
    g_object_set(G_OBJECT(pgie), "config-file-path", PGIE_CONFIG_FILE, NULL);

    /* Override the batch-size set in the config file with the number of sources. */
    g_object_get(G_OBJECT(pgie), "batch-size", &pgie_batch_size, NULL);
    if (pgie_batch_size != num_sources)
    {
        g_printerr("WARNING: Overriding infer-config batch-size (%d) with number of sources (%d)\n",
                   pgie_batch_size, num_sources);
        g_object_set(G_OBJECT(pgie), "batch-size", num_sources, NULL);
    }

    /* Configure rtsp property */
    g_object_set(G_OBJECT(caps), "caps", gst_caps_from_string("video/x-raw(memory:NVMM), format=I420"), NULL);
    g_object_set(G_OBJECT(h264enc), "bitrate", 8000000, NULL);
    g_object_set(G_OBJECT(h264enc), "iframeinterval", 1, NULL);
    // g_object_set (G_OBJECT (h264enc), "profile", 2, "insert-sps-pps", 1, "bufapi-version", 1, "preset-level", 1, NULL);
    g_object_set(G_OBJECT(h264enc), "profile", 2, NULL);
    g_object_set(G_OBJECT(sink_rtsp), "host", "127.0.0.1", "port", 5400, "async", FALSE, "sync", 1, NULL);

    /* Configure plugin property */
    g_object_set(G_OBJECT(plugin), "full-frame", 0, NULL);
    g_object_set(G_OBJECT(plugin), "blur-objects", 1, NULL);
    g_object_set(G_OBJECT(plugin), "lane-id", 1, NULL);

    gchar tmp_buf[1024];
    sprintf(tmp_buf, lane_line, 1024);
    if (parse_lane(lane_line))
        g_object_set(G_OBJECT(plugin), "lane-line", tmp_buf, NULL);
    else
        g_print("no lanes...\n");

    g_object_set(G_OBJECT(plugin), "lane-display", label_display, NULL);
    /* Set necessary properties of the tracker element. */
    if (!set_tracker_properties(nvtracker))
    {
        g_printerr("Failed to set tracker properties. Exiting.\n");
        return -1;
    }

    tiler_rows = (guint)sqrt(num_sources);
    tiler_columns = (guint)ceil(1.0 * num_sources / tiler_rows);
    /* we set the tiler properties here */
    g_object_set(G_OBJECT(tiler), "rows", tiler_rows, "columns", tiler_columns,
                 "width", display_resolution[0], "height", display_resolution[1], NULL);

    /* we add a message handler */
    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    /* Set up the pipeline */
    /* we add all elements into the pipeline */
#ifdef PLATFORM_TEGRA
    gst_bin_add_many(GST_BIN(pipeline), pgie, tiler, nvvidconv, plugin, nvosd,
                     rtph264pay, h264enc, caps, nvvidconv_postosd, h264parse2,
                     sink_rtsp, tee, queue1, nvtracker, NULL);
#else
    gst_bin_add_many(GST_BIN(pipeline), pgie, tiler, nvvidconv, plugin, nvosd,
                     rtph264pay, h264enc, caps, nvvidconv_postosd, h264parse2,
                     sink_rtsp, tee, queue1, nvtracker, NULL);
#endif

    /* we link the elements together
     * nvstreammux -> nvinfer -> nvtiler -> nvvidconv -> nvosd -> video-renderer */
    if (!gst_element_link_many(streammux, pgie, nvtracker,
                               tiler, nvvidconv, plugin, nvosd, tee, NULL))
    {
        g_printerr("Elements could not be linked. Exiting. 1\n");
        return -1;
    }

    if (!gst_element_link_many(queue1, nvvidconv_postosd, caps,
                               h264enc, h264parse2, rtph264pay, sink_rtsp, NULL))
    {
        g_printerr("Elements could not be linked. Exiting. 3\n");
        return -1;
    }
    /* Lets add probe to get informed of the meta data generated, we add probe to
     * the sink pad of the osd element, since by that time, the buffer would have
     * had got all the metadata. */
    tiler_src_pad = gst_element_get_static_pad(nvosd, "sink");
    if (!tiler_src_pad)
        g_print("Unable to get src pad\n");
    else
        gst_pad_add_probe(tiler_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          tiler_src_pad_buffer_probe, NULL, NULL);

    sink_pad = gst_element_get_static_pad(queue1, "sink");
    tee_msg_pad = gst_element_get_request_pad(tee, "src_%u");
    tee_render_pad = gst_element_get_request_pad(tee, "src_%u");
    if (!tee_msg_pad || !tee_render_pad)
    {
        g_printerr("Unable to get request pads\n");
        return -1;
    }

    if (gst_pad_link(tee_msg_pad, sink_pad) != GST_PAD_LINK_OK)
    {
        g_printerr("Unable to link tee and message converter\n");
        gst_object_unref(sink_pad);
        return -1;
    }
    gst_object_unref(sink_pad);

    /* Create smart record */
    GstElement *queue_sr = gst_element_factory_make("queue", "queue-sr");
    GstElement *encoder_sr = gst_element_factory_make("nvv4l2h264enc", "encoder-sr");
    GstElement *parser_sr = gst_element_factory_make("h264parse", "parser-sr");
    GstElement *nvvidconv_sr = gst_element_factory_make("nvvideoconvert", "nvvidconv-sr");
    GstElement *caps_sr = gst_element_factory_make("capsfilter", "cap-sr");

    if (!encoder_sr || !parser_sr || !queue_sr || !nvvidconv_sr || !caps_sr)
    {
        g_printerr("SR elements could not be created. Exiting.\n");
        return -1;
    }

    NvDsSRInitParams params = {0};
    params.containerType = NVDSSR_CONTAINER_MP4;
    params.dirpath = (char *)"/home/nvidia/ftp";
    params.videoCacheSize = 60;
    params.defaultDuration = 10;
    params.callback = smart_record_callback;
    if (NvDsSRCreate(&ctx, &params) != NVDSSR_STATUS_OK)
    {
        g_print("Failed to create smart record bin");
        return -1;
    }

    gst_bin_add_many(GST_BIN(pipeline), queue_sr, encoder_sr,
                     parser_sr, nvvidconv_sr,
                     caps_sr, ctx->recordbin, NULL);

    if (!gst_element_link_many(tee, queue_sr, nvvidconv_sr, caps_sr,
                               encoder_sr, parser_sr, ctx->recordbin, NULL))
    {
        g_print("Elements not linked. Exiting. \n");
        return -1;
    }

    g_object_set(G_OBJECT(nvvidconv), "nvbuf-memory-type", 3, NULL);
}

void deepstream_init(const char *config)
{

    int i, j;

    create_pipeline();

    /* Initialize lanes_stuck */
    for (i = 0; i < line_cnt; i++)
    {
        for (j = 0; j < 5; j++)
        {
            lanes_stuck[i][j].id = -1;
        }
    }

    start_rtsp_streaming(8554, 5400, 0);

    /* Set the pipeline to "playing" state */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void deepstream_exit()
{

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_watch_id);
}
