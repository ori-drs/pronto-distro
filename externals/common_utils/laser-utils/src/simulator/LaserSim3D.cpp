#include "LaserSim3D.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bot_core/bot_core.h>
#include <geom_utils/geometry.h>
#include <octomap/octomap.h>

using namespace std;
using namespace octomap;
using namespace laser_util;

const bot_core_planar_lidar_t * LaserSim3D::simulate(BotTrans * curr_pose, int64_t utime)
{
  bot_tictoc("publishLaser");

  //do the ray tracing
  double local_xyz[3];
  float prev_range = laser_max_range;
  for (int i = 0; i < laser_msg->nranges; i++) {

    if (body_frame_scan->point_status[i] >= laser_valid_projection) {
      laser_msg->ranges[i] = prev_range;
      continue;
    }
    double * laser_xyz = point3d_as_array(&body_frame_scan->points[i]);
    bot_trans_apply_vec(curr_pose, laser_xyz, local_xyz);

    point3d origin(curr_pose->trans_vec[0], curr_pose->trans_vec[1], curr_pose->trans_vec[2]);
    point3d direction(local_xyz[0], local_xyz[1], local_xyz[2]);
    direction -= origin;
    point3d hitPoint;

    if (map->castRay(origin, direction, hitPoint, true, laser_max_range)) {
      laser_msg->ranges[i] = origin.distance(hitPoint); //TODO: resonable noise?
    }
    else {
      laser_msg->ranges[i] = laser_max_range;
    }
    prev_range = laser_msg->ranges[i];
  }
  laser_msg->utime = utime;
  return laser_msg;

}

LaserSim3D::LaserSim3D(const octomap::OcTree * ocTree, BotParam * param, BotFrames * frames,
    const std::string & laser_name)
{

  string prefix = "planar_lidars." + laser_name;

  Laser_projector * proj = laser_projector_new(param, frames, laser_name.c_str(), 1);

  laser_msg = (bot_core_planar_lidar_t *) calloc(1, sizeof(bot_core_planar_lidar_t));
  laser_msg->nranges = bot_param_get_int_or_fail(param, (prefix + ".nranges").c_str());
  laser_msg->ranges = (float *) calloc(laser_msg->nranges, sizeof(float));
  laser_msg->nintensities = 0;
  laser_msg->rad0 = bot_to_radians(bot_param_get_double_or_fail(param,(prefix+".rad0").c_str()));
  laser_msg->radstep = bot_to_radians(bot_param_get_double_or_fail(param,(prefix+".radstep").c_str()));
  laser_max_range = proj->max_range;

  decimation_factor = bot_param_get_int_or_fail(param, (prefix + ".decimation_factor").c_str());

  for (int i = 0; i < laser_msg->nranges; i++)
    laser_msg->ranges[i] = laser_max_range*.95;
  body_frame_scan = laser_create_projected_scan_from_planar_lidar(proj, laser_msg,
      proj->coord_frame);

  laser_decimate_projected_scan(body_frame_scan, decimation_factor, 0, 1000);

  laser_projector_destroy(proj);

  map = ocTree;
}

LaserSim3D::~LaserSim3D()
{

  laser_destroy_projected_scan(body_frame_scan);
  bot_core_planar_lidar_t_destroy(laser_msg);
}
