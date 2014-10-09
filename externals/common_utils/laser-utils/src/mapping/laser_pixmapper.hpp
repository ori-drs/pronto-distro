#ifndef LASER_OCTOMAPPER_H_
#define LASER_OCTOMAPPER_H_
#include <list>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <laser_utils/laser_util.h>
#include <occ_map/PixelMap.hpp>


class LaserPixmapper {
public:
  LaserPixmapper(int argc, char ** argv);
  ~LaserPixmapper();
  void addProjectedScan(laser_projected_scan * lscan);
  void processScansInQueue();
  void publish_map();
  void save_map();

  BotParam *param;
  BotFrames *frames;
  bool fromLog;
  bool addFloor;
  double floor_height;
  bool rayTracing;
  char * logFName;
  lcm_t *lcm_pub; //two different ones for running from log
  lcm_t *lcm_recv; //will point to same place if running live

  double resolution;
  occ_map::FloatPixelMap * pixmap;

  GHashTable * laser_projectors;
  std::list<laser_projected_scan *> lscans_to_be_processed;

  std::string outFname;

  int64_t last_publish_time;

};

#endif /* LASER_OCTOMAPPER_H_ */
