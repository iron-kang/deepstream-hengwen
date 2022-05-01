#ifndef __CONFIG_MGR__
#define __CONFIG_MGR__

#include <glib.h>

#define TRACKER_CONFIG_FILE "tracker_config.txt"
#define MAX_TRACKING_ID_LEN 16

#define PGIE_CONFIG_FILE "pgie_config_yolo.txt"
//#define PGIE_CONFIG_FILE "pgie_config.txt"
#define SGIE_CONFIG_FILE "sgie_config.txt"
/* class-attrs-all group*/
#define CONFIG_CLASS_ATTRS_ALL "class-attrs-all"
#define CONFIG_CLASS_ATTRS_ALL_PRE_THRESHOLD "pre-cluster-threshold"
#define CONFIG_CLASS_ATTRS_ALL_EPS "eps"
#define CONFIG_CLASS_ATTRS_ALL_GROUP_THRESHOLD "group-threshold"
/* smart-record group */
#define CONFIG_SMART_RECORD "smart-record"
#define CONFIG_SMART_RECORD_START_REC_TIME "start-rec-time"
#define CONFIG_SMART_RECORD_REC_DURATION "rec-duration"
/* source group */
#define CONFIG_SOURCE "source"
#define CONFIG_SOURCE_URI "uri"
#define CONFIG_SOURCE_OUT_RTSP_NAME "out-rtsp-name"
#define CONFIG_SOURCE_STUCK_TIME "stuck-time"
#define CONFIG_SOURCE_WIDTH "width"
#define CONFIG_SOURCE_HEIGHT "height"
#define CONFIG_SOURCE_LANE_LINE "lane-line"
#define CONFIG_SOURCE_GRAVITY_HORI "gravity-hori"
#define CONFIG_SOURCE_GRAVITY_VERT "gravity-vert"
#define CONFIG_SOURCE_LABEL_DISPLAY "label-display"
/* tracker group*/
#define CONFIG_GROUP_TRACKER "tracker"
#define CONFIG_GROUP_TRACKER_WIDTH "tracker-width"
#define CONFIG_GROUP_TRACKER_HEIGHT "tracker-height"
#define CONFIG_GROUP_TRACKER_LL_CONFIG_FILE "ll-config-file"
#define CONFIG_GROUP_TRACKER_LL_LIB_FILE "ll-lib-file"
#define CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS "enable-batch-process"
#define CONFIG_GROUP_TRACKER_ENABLE_PAST_FRAME "enable-past-frame"
#define CONFIG_GPU_ID "gpu-id"

#define CHECK_ERROR(error) \
    if (error) { \
        g_printerr ("Error while parsing config file: %s\n", error->message); \
        goto done; \
    }

void update_class_attrs_all (GKeyFile *key_file, gchar *field, float val);
void get_class_attrs_double_value(GKeyFile *key_file, gchar *group, gchar *field, gdouble *val);
void set_class_attrs_doublet_value (GKeyFile *key_file, gchar *group, gchar *field, gdouble val);
void get_class_attrs_int_value(GKeyFile *key_file, gchar *group, gchar *field, gint *val);
void set_class_attrs_int_value (GKeyFile *key_file, gchar *group, gchar *field, gint val);
void get_class_attrs_string_value(GKeyFile *key_file, gchar *group, gchar *field, gchar **val);
void set_class_attrs_string_value (GKeyFile *key_file, gchar *group, gchar *field, gchar *val);

#endif
