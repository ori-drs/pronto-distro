#ifndef LASER_VOXMAPPER_H_
#define LASER_VOXMAPPER_H_
#include <occ_map/VoxelMap.hpp>
#include <occ_map/PixelMap.hpp>
#include <list>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <laser_utils/laser_util.h>


class LaserVoxmapper {
public:
  LaserVoxmapper(char * logFname);
  ~LaserVoxmapper();
  void addProjectedScan(laser_projected_scan * lscan);
  void processScansInQueue();
  void publish_map();
  void save_map();

  BotParam *param;
  BotFrames *frames;
  bool fromLog;
  char * logFName;
  lcm_t *lcm_pub; //two different ones for running from log
  lcm_t *lcm_recv; //will point to same place if running live

  occ_map::FloatVoxelMap * voxmap;

  GHashTable * laser_projectors;
  std::list<laser_projected_scan *> lscans_to_be_processed;

  int64_t last_publish_time;

};

#endif /* LASER_VOXMAPPER_H_ */
