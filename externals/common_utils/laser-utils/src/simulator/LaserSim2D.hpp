#ifndef LASERSIM2D_H_
#define LASERSIM2D_H_

#include <occ_map/PixelMap.hpp>
#include <bot_core/bot_core.h>

namespace laser_util {

class LaserSim2D {
public:
  LaserSim2D(const occ_map::FloatPixelMap * map, int nranges, float rad0, float radstep, float max_range);
  ~LaserSim2D();
  bool isNearMapBorder(const double location[2], double range);
  bool getMapBorderInstersection(const double P0[2], const double P1[2], double intersect[2]);

  const occ_map::FloatPixelMap * map;
  float occupancy_thresh;

  double * laserFramePoints;
  bot_core_planar_lidar_t * laser_msg;
  float laser_max_range;

  const bot_core_planar_lidar_t * simulate(BotTrans * curr_pose, int64_t utime = 0);
  const bot_core_planar_lidar_t * simulate(double x, double y, double t, int64_t utime = 0)
  {
    BotTrans trans;
    trans.trans_vec[0] = x;
    trans.trans_vec[1] = y;
    trans.trans_vec[2] = 0;
    double rpy[3] = { 0, 0, t };
    bot_roll_pitch_yaw_to_quat(rpy, trans.rot_quat);
    return simulate(&trans, utime);
  }
  const bot_core_planar_lidar_t * simulate(const double xyt[3], int64_t utime = 0)
  {
    return simulate(xyt[0], xyt[1], xyt[2], utime);
  }

};

} /* namespace laser_util */
#endif /* LASERSIM_H_ */
