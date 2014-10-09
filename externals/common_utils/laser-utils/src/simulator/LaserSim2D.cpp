#include "LaserSim2D.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bot_core/bot_core.h>
#include <geom_utils/geometry.h>
#include <occ_map/PixelMap.hpp>

using namespace std;
using namespace occ_map;
using namespace laser_util;

bool LaserSim2D::isNearMapBorder(const double location[2], double range)
{
  return location[0] - map->xy0[0] < range ||
      map->xy1[0] - location[0] < range ||
      location[1] - map->xy0[1] < range ||
      map->xy1[1] - location[1] < range;

}

bool LaserSim2D::getMapBorderInstersection(const double P0[2], const double P1[2], double intersect[2])
{

  double P2[2], P3[2];
  P2[0] = map->xy0[0];
  P2[1] = map->xy0[1];

  P3[0] = map->xy0[0];
  P3[1] = map->xy1[1];
  if (geom_line_seg_line_seg_intersect_2d(POINT2D(P0), POINT2D(P1), POINT2D(P2), POINT2D(P3), POINT2D(intersect)))
    return true;
  P3[0] = map->xy1[0];
  P3[1] = map->xy0[1];
  if (geom_line_seg_line_seg_intersect_2d(POINT2D(P0), POINT2D(P1), POINT2D(P2), POINT2D(P3), POINT2D(intersect)))
    return true;

  P2[0] = map->xy1[0];
  P2[1] = map->xy1[1];

  P3[0] = map->xy0[0];
  P3[1] = map->xy1[1];
  if (geom_line_seg_line_seg_intersect_2d(POINT2D(P0), POINT2D(P1), POINT2D(P2), POINT2D(P3), POINT2D(intersect)))
    return true;
  P3[0] = map->xy1[0];
  P3[1] = map->xy0[1];
  if (geom_line_seg_line_seg_intersect_2d(POINT2D(P0), POINT2D(P1), POINT2D(P2), POINT2D(P3), POINT2D(intersect)))
    return true;

  return false;

}

const bot_core_planar_lidar_t * LaserSim2D::simulate(BotTrans * curr_pose, int64_t utime)
{
  bool nearMapBorder = isNearMapBorder(curr_pose->trans_vec, laser_max_range);

  //do the ray tracing
  double local_xyz[3];
  double hitPoint[2];
  double s, c;
  for (int i = 0; i < laser_msg->nranges; i++) {
    double * laser_xyz = laserFramePoints + 3 * i;
    bot_trans_apply_vec(curr_pose, laser_xyz, local_xyz);

    double borderInstersect[2];
    if (nearMapBorder && getMapBorderInstersection(curr_pose->trans_vec, local_xyz, borderInstersect)) {
      // crop beam to edge of the map
      local_xyz[0] = borderInstersect[0];
      local_xyz[1] = borderInstersect[1];
    }

    if (map->collisionCheck(curr_pose->trans_vec, local_xyz, occupancy_thresh, hitPoint)) {
      laser_msg->ranges[i] = bot_vector_dist_2d(curr_pose->trans_vec, hitPoint); //TODO: reasonable noise?
    }
    else {
      laser_msg->ranges[i] = laser_max_range;
    }
  }
  laser_msg->utime = utime;
  return laser_msg;

}

LaserSim2D::LaserSim2D(const occ_map::FloatPixelMap * _map, int nranges, float rad0, float radstep, float max_range)
{
  laser_msg = (bot_core_planar_lidar_t *) calloc(1, sizeof(bot_core_planar_lidar_t));
  laser_msg->nranges = nranges;
  laser_msg->ranges = (float *) calloc(nranges, sizeof(float));
  laser_msg->nintensities = 0;
  laser_msg->rad0 = rad0;
  laser_msg->radstep = radstep;
  laser_max_range = max_range;

  map = _map;
  occupancy_thresh = .85; //TODO:

  laserFramePoints = (double *) calloc(3 * laser_msg->nranges, sizeof(double));
  for (int i = 0; i < laser_msg->nranges; i++) {
    double * laser_xyz = laserFramePoints + 3 * i;
    double theta = laser_msg->rad0 + laser_msg->radstep * i;
    laser_xyz[0] = laser_max_range * cos(theta);
    laser_xyz[1] = laser_max_range * sin(theta);
    laser_xyz[2] = 0;
  }
}

LaserSim2D::~LaserSim2D()
{
  free(laserFramePoints);
  bot_core_planar_lidar_t_destroy(laser_msg);
}
