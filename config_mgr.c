#include "config_mgr.h"

void update_class_attrs_all (GKeyFile *key_file, gchar *field, float val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;

  keys = g_key_file_get_keys (key_file, CONFIG_CLASS_ATTRS_ALL, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
      	g_key_file_set_double (key_file, CONFIG_CLASS_ATTRS_ALL, *key, val);
	g_print("set [%s]%s: %f\n", CONFIG_CLASS_ATTRS_ALL, field, val);
      }
  }

}
void set_class_attrs_doublet_value (GKeyFile *key_file, gchar *group, gchar *field, gdouble val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;

  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
      	g_key_file_set_double (key_file, group, *key, val);
      }
  }

}

void get_class_attrs_double_value(GKeyFile *key_file, gchar *group, gchar *field, gdouble *val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;


  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
          *val  = g_key_file_get_double (key_file, group, *key, &error);
      }
  }

}

void set_class_attrs_int_value (GKeyFile *key_file, gchar *group, gchar *field, gint val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;

  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
      	g_key_file_set_integer (key_file, group, *key, val);
      }
  }

}

void get_class_attrs_int_value(GKeyFile *key_file, gchar *group, gchar *field, gint *val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;


  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
          *val  = g_key_file_get_integer (key_file, group, *key, &error);
      }
  }

}

void set_class_attrs_string_value (GKeyFile *key_file, gchar *group, gchar *field, gchar *val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;

  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
      	g_key_file_set_string (key_file, group, *key, val);
      }
  }

}

void get_class_attrs_string_value(GKeyFile *key_file, gchar *group, gchar *field, gchar **val)
{
  gboolean ret = FALSE;
  gchar **keys = NULL;
  gchar **key = NULL;
  GError *error = NULL;
  guint rett;


  keys = g_key_file_get_keys (key_file, group, NULL, &error);

  for (key = keys; *key; key++)
  {
      if (!strcmp (*key, field)) {
          *val  = g_key_file_get_string (key_file, group, *key, &error);
	  //g_print("%s = %s\n", __func__, val);
      }
  }

}

