#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "param_util.h"

#define PLANAR_LIDAR_PREFIX "planar_lidars"
#define CAMERA_PREFIX "cameras"

//utility functions for cameras and lidars
//get all sensor names
char**
bot_param_get_all_camera_names(BotParam *param)
{
  return bot_param_get_subkeys(param, CAMERA_PREFIX);
}
char**
bot_param_get_all_planar_lidar_names(BotParam *param)
{
  return bot_param_get_subkeys(param, PLANAR_LIDAR_PREFIX);
}

//get prefix for the sensors
int bot_param_get_camera_prefix(BotParam *param, const char *cam_name, char *result, int result_size)
{
  int n = snprintf(result, result_size, "%s.%s", CAMERA_PREFIX, cam_name);
  if (n >= result_size)
    return -1;
  if (param)
    return bot_param_has_key(param, result) ? 0 : -1;
  else
    return 0;
}

int bot_param_get_planar_lidar_prefix(BotParam *param, const char *plidar_name, char *result, int result_size)
{
  int n = snprintf(result, result_size, "%s.%s", PLANAR_LIDAR_PREFIX, plidar_name);
  if (n >= result_size)
    return -1;
  if (param)
    return bot_param_has_key(param, result) ? 0 : -1;
  else
    return 0;
}

char * bot_param_get_sensor_name_from_lcm_channel(BotParam *param, const char * prefix, const char *channel)
{
  char * sensor_name = NULL;
  char **sensor_names = bot_param_get_subkeys(param, prefix);
  for (int i = 0; sensor_names && sensor_names[i]; i++) {
    char key[2048];
    snprintf(key, sizeof(key), "%s.%s.lcm_channel", prefix, sensor_names[i]);
    char *lcm_channel_str = NULL;
    int key_status = bot_param_get_str(param, key, &lcm_channel_str);

    if ((0 == key_status) && (0 == strcmp(channel, lcm_channel_str))) {
      sensor_name = strdup(sensor_names[i]);
      free(lcm_channel_str);
      break;
    }
    else {
      free(lcm_channel_str);
    }
  }
  g_strfreev(sensor_names);

  return sensor_name;
}

char * bot_param_get_camera_name_from_lcm_channel(BotParam *param, const char *channel)
{
  return bot_param_get_sensor_name_from_lcm_channel(param, CAMERA_PREFIX, channel);
}

char * bot_param_get_planar_lidar_name_from_lcm_channel(BotParam *param, const char *channel)
{
  return bot_param_get_sensor_name_from_lcm_channel(param, PLANAR_LIDAR_PREFIX, channel);
}


//get coord frame name
char * bot_param_get_sensor_coord_frame(BotParam *bot_param, const char * sensor_prefix, const char *sensor_name)
{
  char key[1024];
  snprintf(key, sizeof(key), "%s.%s.coord_frame", sensor_prefix, sensor_name);
  return bot_param_get_str_or_fail(bot_param, key);
}
char *
bot_param_get_camera_coord_frame(BotParam *bot_param, const char *camera_name)
{
  return bot_param_get_sensor_coord_frame(bot_param, CAMERA_PREFIX, camera_name);
}
char *
bot_param_get_planar_lidar_coord_frame(BotParam *bot_param, const char *lidar_name)
{
  return bot_param_get_sensor_coord_frame(bot_param, PLANAR_LIDAR_PREFIX, lidar_name);
}


//get lcm channel
char * bot_param_get_sensor_lcm_channel(BotParam *bot_param, const char * sensor_prefix, const char *sensor_name)
{
  char key[1024];
  snprintf(key, sizeof(key), "%s.%s.lcm_channel", sensor_prefix, sensor_name);
  return bot_param_get_str_or_fail(bot_param, key);
}
char * bot_param_get_camera_lcm_channel(BotParam *bot_param, const char *camera_name)
{
  return bot_param_get_sensor_lcm_channel(bot_param, CAMERA_PREFIX, camera_name);
}
char * bot_param_get_planar_lidar_lcm_channel(BotParam *bot_param, const char *lidar_name)
{
  return bot_param_get_sensor_lcm_channel(bot_param, PLANAR_LIDAR_PREFIX, lidar_name);
}


/* ================ general ============== */
int bot_param_get_quat(BotParam *param, const char *name, double quat[4])
{
  char key[2048];
  sprintf(key, "%s.quat", name);
  if (bot_param_has_key(param, key)) {
    int sz = bot_param_get_double_array(param, key, quat, 4);
    assert(sz == 4);
    return 0;
  }

  sprintf(key, "%s.rpy", name);
  if (bot_param_has_key(param, key)) {
    double rpy[3];
    int sz = bot_param_get_double_array(param, key, rpy, 3);
    assert(sz == 3);
    for (int i = 0; i < 3; i++)
      rpy[i] = bot_to_radians (rpy[i]);
    bot_roll_pitch_yaw_to_quat(rpy, quat);
    return 0;
  }

  sprintf(key, "%s.rodrigues", name);
  if (bot_param_has_key(param, key)) {
    double rod[3];
    int sz = bot_param_get_double_array(param, key, rod, 3);
    assert(sz == 3);
    bot_rodrigues_to_quat(rod, quat);
    return 0;
  }

  sprintf(key, "%s.angleaxis", name);
  if (bot_param_has_key(param, key)) {
    double aa[4];
    int sz = bot_param_get_double_array(param, key, aa, 4);
    assert(sz == 4);

    bot_angle_axis_to_quat(aa[0], aa + 1, quat);
    return 0;
  }
  return -1;
}

int bot_param_get_translation(BotParam *param, const char *name, double translation[3])
{
  char key[2048];
  sprintf(key, "%s.translation", name);
  if (bot_param_has_key(param, key)) {
    int sz = bot_param_get_double_array(param, key, translation, 3);
    assert(sz == 3);
    return 0;
  }
  // not found
  return -1;
}

int bot_param_get_trans(BotParam *param, const char *name, BotTrans * trans)
{
  if (bot_param_get_quat(param, name, trans->rot_quat))
    return -1;

  if (bot_param_get_translation(param, name, trans->trans_vec))
    return -1;

  return 0;
}

BotCamTrans*
bot_param_get_new_camtrans(BotParam *param, const char *cam_name)
{
  char prefix[2048];
  snprintf(prefix, sizeof(prefix), "%s.%s.intrinsic_cal", CAMERA_PREFIX, cam_name);
  if (!bot_param_has_key(param, prefix))
    goto fail;

  char key[2048];
  double width;
  sprintf(key, "%s.width", prefix);
  if (0 != bot_param_get_double(param, key, &width))
    goto fail;

  double height;
  sprintf(key, "%s.height", prefix);
  if (0 != bot_param_get_double(param, key, &height))
    goto fail;

  double pinhole_params[5];
  snprintf(key, sizeof(key), "%s.pinhole", prefix);
  if (5 != bot_param_get_double_array(param, key, pinhole_params, 5))
    goto fail;
  double fx = pinhole_params[0];
  double fy = pinhole_params[1];
  double cx = pinhole_params[3];
  double cy = pinhole_params[4];
  double skew = pinhole_params[2];

  char * distortion_model;
  sprintf(key, "%s.distortion_model", prefix);
  if (0 != bot_param_get_str(param, key, &distortion_model))
    goto fail;

  if (strcmp(distortion_model, "null") == 0) {
    BotDistortionObj* null_dist = bot_null_distortion_create();
    BotCamTrans* null_camtrans = bot_camtrans_new(cam_name, width, height, fx, fy, cx, cy, skew, null_dist);
    return null_camtrans;
  }
  else if (strcmp(distortion_model, "spherical") == 0) {
    double distortion_param;
    sprintf(key, "%s.distortion_params", prefix);
    if (1 != bot_param_get_double_array(param, key, &distortion_param, 1))
      goto fail;

    BotDistortionObj* sph_dist = bot_spherical_distortion_create(distortion_param);
    BotCamTrans* sph_camtrans = bot_camtrans_new(cam_name, width, height, fx, fy, cx, cy, skew, sph_dist);
    return sph_camtrans;
  }
  else if (strcmp(distortion_model, "plumb-bob") == 0) {
    double dist_k[3];
    sprintf(key, "%s.distortion_k", prefix);
    if (3 != bot_param_get_double_array(param, key, dist_k, 3))
      goto fail;

    double dist_p[2];
    sprintf(key, "%s.distortion_p", prefix);
    if (2 != bot_param_get_double_array(param, key, dist_p, 2))
      goto fail;

    BotDistortionObj* pb_dist = bot_plumb_bob_distortion_create(dist_k[0], dist_k[1], dist_k[2], dist_p[0], dist_p[1]);
    BotCamTrans* pb_camtrans = bot_camtrans_new(cam_name, width, height, fx, fy, cx, cy, skew, pb_dist);
    return pb_camtrans;
  }
  else if (strcmp(distortion_model, "angular-poly") == 0) {
    double coeffs[64];
    sprintf(key, "%s.distortion_coeffs", prefix);
    int num_coeffs = bot_param_get_double_array(param, key, coeffs, -1);
    if (0 >= num_coeffs)
      goto fail;

    BotDistortionObj* ang_dist = bot_angular_poly_distortion_create(coeffs, num_coeffs);
    BotCamTrans* ang_camtrans = bot_camtrans_new(cam_name, width, height, fx, fy, cx, cy, skew, ang_dist);
    return ang_camtrans;
  }

  fail: return NULL;
}
