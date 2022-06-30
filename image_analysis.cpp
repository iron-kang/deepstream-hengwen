/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/socket.h>
//#include "gstnvstreammeta.h"
#include "config_mgr.h"
#include "nvdsgstutils.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <deepstream_config_file_parser.h>
#include "gst-nvdssr.h"
#include "deepstream.h"

GST_DEBUG_CATEGORY(NVDS_APP);
#define FPS_PRINT_INTERVAL 300
/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1280 // 1920
#define MUXER_OUTPUT_HEIGHT 720 // 1080

enum SOCKET_CMD
{
    THRESHOLD = 0x1,
    START_RECORD = 0x2,
    SOURCE_RTSP = 0x3,
    DEST_RTSP = 0x4,
    START_REC_TIME = 0x5,
    REC_DURATION = 0x6,
    LABEL_DISPLAY = 0x7,
    EVENT = 0x8,
    HW_STATUS = 0x9,
    RESET_PROG = 0xA,
    REBOOT_SYS = 0xB,
    RESOLUTION = 0xC,
    LANE_LINE = 0xD,
    STUCK_TIME = 0xE,
    STUCK_CAR_COUNT = 0xF,
};

static GThread *thread = NULL;
static GKeyFile *cfg_file = NULL;
static gint rec_duration = 0;
static gint start_rec_time = 0;
static gdouble class_threshold = 0.0;
uint32_t hw_status = 0;
uint32_t event[128] = {0};
gchar *lane_line;
gchar config[128];
GMainLoop *loop = NULL;
GstElement *pipeline = NULL;
gchar *out_rtsp_name;
gchar *src_uri;
gdouble gravity_hori = 0.0;
gdouble gravity_vert = 0.0;
guint line_cnt = 0;
guint event_lane = 0;
gint display_resolution[2] = {0};
GstElement *plugin = NULL;
NvDsSRContext *ctx = NULL;
Line *lines;
gint label_display = false;
gint stuck_time = 0;
gint stuck_cart_count = 0;

/*
 *   | x   y   1|
 *   | x1  y1  1|
 *   | x2  y2  1|
 *
 *   ax + by + c = 0
 *   a = y1 - y2
 *   b = x2 - x1
 *   c = x1*y2 - x2*y1
 */
static void get_line_func(Line *line, guint x1, guint y1, guint x2, guint y2)
{
    line->a = y1 - y2;
    line->b = x2 - x1;
    line->c = x1 * y2 - x2 * y1;
}

gboolean parse_lane(gchar *lane_line)
{
    char *pch;
    char *delim = ";";
    int cnt = 0;
    int coord[4 * MAX_LANES];

    pch = strtok(lane_line, delim);
    while (pch != NULL)
    {
        // g_print("%s\n", pch);
        coord[cnt++] = atoi(pch);
        pch = strtok(NULL, delim);
    }
    g_print("cnt = %d\n", cnt);
    if (cnt % 4 == 0)
    {
        line_cnt = cnt / 4;

        if ((line_cnt > 10) || (line_cnt < 2))
            return FALSE;

        for (int i = 0; i < line_cnt; i++)
            get_line_func(&lines[i], coord[i * 4 + 0], coord[i * 4 + 1],
                          coord[i * 4 + 2], coord[i * 4 + 3]);
        return TRUE;
    }

    return FALSE;
}

static void threshold_property(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    gdouble threshold = 0.0;
    int ret = 0;
    float tmp;

    switch (cmd)
    {
    case 'w':
        read(*fd, &tmp, sizeof(float));
        threshold = tmp;
        g_print("%s: cmd[%x]->val = %f\n", __func__, cmd, threshold);

        update_class_attrs_all(cfg_file, CONFIG_CLASS_ATTRS_ALL_PRE_THRESHOLD, threshold);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        if (pipeline)
        {
            elem = gst_bin_get_by_name(GST_BIN(pipeline), "primary-nvinference-engine");
            g_object_set(G_OBJECT(elem), "config-file-reload", config, NULL);
        }
        break;
    case 'r':
        get_class_attrs_double_value(cfg_file, CONFIG_CLASS_ATTRS_ALL,
                                     CONFIG_CLASS_ATTRS_ALL_PRE_THRESHOLD, &threshold);
        g_print("threshold = %f\n", threshold);
        ret = (int)send(*fd, &threshold, (size_t)sizeof(gdouble), 0);
        break;
    default:
        break;
    }
}

static void
source_uri_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        memset(src_uri, 0, 1024);
        read(*fd, src_uri, 1024);
        g_print("srcuri: %s\n", src_uri);
        set_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_URI, &src_uri[0]);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        sleep(1);
        g_main_loop_quit(loop);
        break;
    case 'r':
        ret = (int)send(*fd, src_uri, strlen(src_uri) + 1, 0);
        break;
    default:
        break;
    }
}

static void
out_rtsp_name_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        memset(out_rtsp_name, 0, 1024);
        read(*fd, out_rtsp_name, 1024);
        g_print("out_uri: %s\n", out_rtsp_name);
        set_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_OUT_RTSP_NAME, out_rtsp_name);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, out_rtsp_name, (size_t)strlen(out_rtsp_name), 0);
        break;
    default:
        break;
    }
}

static void start_rec_time_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, &start_rec_time, sizeof(int));
        g_print("start rec time: %d\n", start_rec_time);
        set_class_attrs_int_value(cfg_file, CONFIG_SMART_RECORD,
                                  CONFIG_SMART_RECORD_START_REC_TIME, start_rec_time);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, &start_rec_time, sizeof(int), 0);
        break;
    default:
        break;
    }
}

static void label_display_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, &label_display, sizeof(int));
        g_print("label_display write: %d\n", label_display);

        g_object_set(G_OBJECT(plugin), "lane-display", label_display, NULL);
        set_class_attrs_int_value(cfg_file, CONFIG_SOURCE,
                                  CONFIG_SOURCE_LABEL_DISPLAY, label_display);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, &label_display, sizeof(int), 0);
        break;
    default:
        break;
    }
}

static void stuck_time_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, &stuck_time, sizeof(int));
        g_print("stuck time write: %d\n", stuck_time);

        g_object_set(G_OBJECT(plugin), "stuck-time", stuck_time, NULL);
        set_class_attrs_int_value(cfg_file, CONFIG_SOURCE,
                                  CONFIG_SOURCE_STUCK_TIME, stuck_time);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, &stuck_time, sizeof(int), 0);
        break;
    default:
        break;
    }
}

static void stuck_car_count_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, &stuck_car_count, sizeof(int));
        g_print("stuck car count write: %d\n", stuck_car_count);

        set_class_attrs_int_value(cfg_file, CONFIG_SOURCE,
                                  CONFIG_SOURCE_STUCK_CAR_COUNT, stuck_car_count);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, &stuck_car_count, sizeof(int), 0);
        break;
    default:
        break;
    }
}

static void rec_duration_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, &rec_duration, sizeof(int));
        g_print("rec duration: %d\n", rec_duration);
        set_class_attrs_int_value(cfg_file, CONFIG_SMART_RECORD,
                                  CONFIG_SMART_RECORD_REC_DURATION, rec_duration);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, &rec_duration, sizeof(int), 0);
        break;
    default:
        break;
    }
}

static void resolution_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'w':
        read(*fd, display_resolution, sizeof(int) * 2);
        g_print("resolution: %dx%d\n", display_resolution[0], display_resolution[1]);
        set_class_attrs_int_value(cfg_file, CONFIG_SOURCE,
                                  CONFIG_SOURCE_WIDTH, display_resolution[0]);
        set_class_attrs_int_value(cfg_file, CONFIG_SOURCE,
                                  CONFIG_SOURCE_HEIGHT, display_resolution[1]);
        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        ret = 1;
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, display_resolution, sizeof(int) * 2, 0);
        break;
    default:
        break;
    }
}

static void lane_line_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;
    gchar tmp_buf[1024];
    gchar tmp_buf2[1024];

    switch (cmd)
    {
    case 'w':
        // memset(lane_line, 0, 1024);
        read(*fd, tmp_buf, 1024);
        g_print("lane_line: %s\n", tmp_buf);
        memcpy(tmp_buf2, tmp_buf, 1024);
        if (parse_lane(tmp_buf))
        {
            ret = 1;
            g_object_set(G_OBJECT(plugin), "lane-line", tmp_buf2, NULL);
            set_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_LANE_LINE, tmp_buf2);
        }

        if (!g_key_file_save_to_file(cfg_file, config, &error))
        {
            g_warning("Error saving key file: %s", error->message);
        }
        g_print("%s.....\n", __func__);
        send(*fd, &ret, (size_t)sizeof(int), 0);
        break;
    case 'r':
        ret = (int)send(*fd, lane_line, strlen(lane_line), 0);
        break;
    default:
        break;
    }
}

void record_handle(bool isStuck, int _event_lane)
{
    static GMutex mutex;
    static gboolean rec_en = FALSE;
    static struct timeval rec_time;
    struct timeval current_time;
    time_t t;
    struct tm tm;
    NvDsSRSessionId sessId = 0;
    gint diff_t;

    gettimeofday(&current_time, NULL);

    diff_t = current_time.tv_sec - rec_time.tv_sec;

    // g_print("diff t = %d\n", diff_t);
    if (diff_t > rec_duration)
    {
        t = time(NULL);
        tm = *localtime(&t);
        gettimeofday(&rec_time, NULL);
        NvDsSRStart(ctx, &sessId, start_rec_time, rec_duration, NULL);
        if (isStuck)
        {
            event[0] = 1;
            event[1] = _event_lane;
            event[2] = tm.tm_year + 1900;
            event[3] = tm.tm_mon + 1;
            event[4] = tm.tm_mday;
            event[5] = tm.tm_hour;
            event[6] = tm.tm_min;
            event[7] = tm.tm_sec;
            g_print("start record.%d%d%d-%d%d%d\n", event[2], event[3], event[4], event[5], event[6], event[7]);
        }
    }
}

static void event_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'r':
        ret = (int)send(*fd, event, sizeof(event), 0);
        break;
    default:
        break;
    }
}

static void hw_status_handle(uint8_t cmd, int *fd)
{
    GError *error = NULL;
    GstElement *elem = NULL;
    int ret = 0;

    switch (cmd)
    {
    case 'r':
        ret = (int)send(*fd, &hw_status, sizeof(hw_status), 0);
        break;
    default:
        break;
    }
}

int readn(int fd, char *buffer, int size)
{
    ssize_t numRead;
    size_t totRead;
    char *buf;

    buf = buffer;
    for (totRead = 0; totRead < size;)
    {

        numRead = read(fd, buf, size - totRead);

        if (numRead == 0)
            return totRead;
        if (numRead == -1)
        {
            if (errno == EINTR)
                continue;
            else
                return -1;
        }

        totRead += numRead;
        buf += numRead;
    }
    return totRead;
}

/* cmd
 *    0x1: threshold
 *    0x2: start record
 *    0x3: source rtsp
 *    0x4: dest rtsp
 *    0x5: start rec time
 *    0x6: rec duration
 *    0x7: vehicle limit
 *    0x8: event
 *    0x9: hw status
 *    0xA: reset program
 *    0xB: reboot
 *    0xC: resloltuion
 *    0xD: Lane line
 */
static gpointer
socket_server(gpointer data)
{
    int fd_server = -1, fd_client = -1;
    int enable = 1;
    int ret = 0;
    int backLog = 10;
    int keepalive = 1;
    int keepidle = 10;
    int keepinterval = 1;
    int keepcount = 10;
    struct sockaddr_in addr_in, addr;
    socklen_t addr_len;
    uint8_t buf[1024];

    fd_server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (fd_server == -1)
    {
        g_printerr("local sock fail !\n");
        return NULL;
    }

    setsockopt(fd_server, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    addr_in.sin_family = AF_INET;
    addr_in.sin_addr.s_addr = INADDR_ANY;
    addr_in.sin_port = htons(8756);

    ret = bind(fd_server, (struct sockaddr *)&addr_in, sizeof(addr_in));
    if (ret != 0)
    {
        g_printerr("local Bind failed\n");
        return NULL;
    }

    ret = listen(fd_server, backLog);

    while (fd_server)
    {
        g_print("%s: accept=======\n", __func__);
        fd_client = accept(fd_server, (struct sockaddr *)&addr, &addr_len);
        setsockopt(fd_client, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof(keepalive));
        setsockopt(fd_client, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&keepidle, sizeof(keepidle));
        setsockopt(fd_client, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepinterval, sizeof(keepinterval));
        setsockopt(fd_client, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepcount, sizeof(keepcount));
        setsockopt(fd_client, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));

        while (TRUE)
        {
            if (read(fd_client, &buf, 4) <= 0)
            {
                break;
            }
            g_print("head: %x, %x\n", buf[0], buf[1]);
            /* Check header */
            if ((buf[0] != 0xBA) || (buf[1] != 0xDC))
                continue;
            g_print("cmd: 0x%x\n", buf[2]);
            switch (buf[2])
            {
            case THRESHOLD:
                threshold_property(buf[3], &fd_client);
                break;
            case START_RECORD:
                record_handle(false, 0);
                break;
            case SOURCE_RTSP:
                source_uri_handle(buf[3], &fd_client);
                break;
            case DEST_RTSP:
                out_rtsp_name_handle(buf[3], &fd_client);
                break;
            case START_REC_TIME:
                start_rec_time_handle(buf[3], &fd_client);
                break;
            case LABEL_DISPLAY:
                label_display_handle(buf[3], &fd_client);
                break;
            case REC_DURATION:
                rec_duration_handle(buf[3], &fd_client);
                break;
            case EVENT:
                event_handle(buf[3], &fd_client);
                break;
            case HW_STATUS:
                hw_status_handle(buf[3], &fd_client);
                break;
            case RESET_PROG:
                g_print("reset pipeline\n");
                g_main_loop_quit(loop);
                break;
            case REBOOT_SYS:
                system("echo nvidia | sudo -S reboot");
                break;
            case RESOLUTION:
                resolution_handle(buf[3], &fd_client);
                break;
            case LANE_LINE:
                lane_line_handle(buf[3], &fd_client);
                break;
            case STUCK_TIME:
                stuck_time_handle(buf[3], &fd_client);
                break;
            case STUCK_CAR_COUNT:
                stuck_car_count_handle(buf[3], &fd_client);
                break;
            default:
                break;
            }
        }
    }
    return NULL;
}

void load_config()
{
    GError *error = NULL;
    /* Create config mgr */
    cfg_file = g_key_file_new();
    if (!g_key_file_load_from_file(cfg_file, config, G_KEY_FILE_NONE, &error))
    {
        g_print("%s", error->message);
    }

    /* Load config */
    get_class_attrs_double_value(cfg_file, CONFIG_CLASS_ATTRS_ALL, CONFIG_CLASS_ATTRS_ALL_PRE_THRESHOLD, &class_threshold);
    get_class_attrs_int_value(cfg_file, CONFIG_SMART_RECORD, CONFIG_SMART_RECORD_START_REC_TIME, &start_rec_time);
    get_class_attrs_int_value(cfg_file, CONFIG_SMART_RECORD, CONFIG_SMART_RECORD_REC_DURATION, &rec_duration);
    get_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_URI, &src_uri);
    get_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_OUT_RTSP_NAME, &out_rtsp_name);
    get_class_attrs_double_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_GRAVITY_HORI, &gravity_hori);
    get_class_attrs_double_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_GRAVITY_VERT, &gravity_vert);
    get_class_attrs_int_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_LABEL_DISPLAY, &label_display);
    get_class_attrs_int_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_STUCK_TIME, &stuck_time);
    get_class_attrs_int_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_STUCK_CAR_COUNT, &stuck_car_count);
    get_class_attrs_int_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_WIDTH, &display_resolution[0]);
    get_class_attrs_int_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_HEIGHT, &display_resolution[1]);
    get_class_attrs_string_value(cfg_file, CONFIG_SOURCE, CONFIG_SOURCE_LANE_LINE, &lane_line);

    g_print("class threshold: %f\n", class_threshold);
    g_print("gravity hori: %f\n", gravity_hori);
    g_print("gravity vert: %f\n", gravity_vert);
    g_print("label display: %d\n", label_display);
    g_print("start rec time: %d\n", start_rec_time);
    g_print("rec duration: %d\n", rec_duration);
    g_print("src uri: %s\n", src_uri);
    g_print("rtsp name: %s\n", out_rtsp_name);
    g_print("lane line: %s\n", lane_line);
    g_print("stuck car count: %d\n", stuck_car_count);
    if ((display_resolution[0] == 0) || (display_resolution[1] == 0))
    {
        display_resolution[0] = MUXER_OUTPUT_WIDTH;
        display_resolution[1] = MUXER_OUTPUT_HEIGHT;
    }
    g_print("%dx%d\n", display_resolution[0], display_resolution[1]);

    /* Create lane line */
    lines = (Line *)malloc(sizeof(Line) * (MAX_LANES + 1));
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        g_print("usage: image_analysis config file\n");
        return 0;
    }

    memcpy(config, argv[1], sizeof(config));
    g_print("config: %s\n", config);
    /* Load deepstream & settings parameter */
    load_config();

    /* Create TCP server */
    /* Standard GStreamer initialization */
    gst_init(&argc, &argv);
    loop = g_main_loop_new(NULL, FALSE);

    deepstream_init(config);

    thread = g_thread_new("Truck Scale server", socket_server, NULL);
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_MEDIA_TYPE, "image-analysis");
    /* Wait till pipeline encounters an error or EOS */
    g_main_loop_run(loop);

    deepstream_exit();
    g_key_file_free(cfg_file);
    g_free(lines);
    g_main_loop_unref(loop);

    return 0;
}
