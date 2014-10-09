#ifndef LASERSIM3D_H_
#define LASERSIM3D_H_

#include <octomap/octomap.h>
#include <bot_core/bot_core.h>
#include <laser_utils/laser_util.h>
#include <bot_frames/bot_frames.h>

namespace laser_util {

class LaserSim3D {
public:
  LaserSim3D(const octomap::OcTree * ocTree,BotParam * param, BotFrames * frames, const std::string & laser_name);
  ~LaserSim3D();

  const octomap::OcTree * map;
  laser_projected_scan * body_frame_scan;
  bot_core_planar_lidar_t * laser_msg;
  float laser_max_range;

  int decimation_factor;

  const bot_core_planar_lidar_t * simulate(BotTrans * curr_pose, int64_t utime = 0);
};

} /* namespace laser_util */
#endif /* LASERSIM_H_ */
