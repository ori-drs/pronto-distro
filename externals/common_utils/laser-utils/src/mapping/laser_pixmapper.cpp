#include "laser_pixmapper.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <deque>
#include <vector>
#include <iostream>
#include <sstream>



#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_param/param_util.h>

#include <ConciseArgs>

using namespace std;
using namespace occ_map;
//TODO: make me a parameter
static int scan_skip = 0;
static int beam_skip = 0;
static double publish_interval = 5;

#define MISS_INC -0.1
#define HIT_INC 1.0

static void on_laser(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_planar_lidar_t *msg,
    void *user_data)
{
  LaserPixmapper * self = (LaserPixmapper *) user_data;
  Laser_projector * proj = (Laser_projector*) g_hash_table_lookup(self->laser_projectors, channel);

  static int counter = 0;
  if (counter++ % (scan_skip + 1) != 0)
    return;

  laser_projected_scan * lscan = laser_create_projected_scan_from_planar_lidar(proj, msg, bot_frames_get_root_name(
      self->frames));
  if (lscan != NULL)
    self->addProjectedScan(lscan);

  self->processScansInQueue();

  if (msg->utime - self->last_publish_time > publish_interval * 1e6 && !self->fromLog) {
    self->last_publish_time = msg->utime;
    self->publish_map();
  }

  static int num_laser_processed = 0;

  if (num_laser_processed % 1000 == 0)
    fprintf(stderr, "processed %d laser scans\n", num_laser_processed);
  num_laser_processed++;

}

void LaserPixmapper::addProjectedScan(laser_projected_scan * lscan)
{
  lscans_to_be_processed.push_back(lscan);
}

LaserPixmapper::LaserPixmapper(int argc, char ** argv) :
    last_publish_time(-1)
{

  string logFName;
  string paramName;
  outFname = "laser_pixmap.pxmp";
  rayTracing = false;

  ConciseArgs opt(argc, argv);
  opt.add(resolution, "r", "resolution");
  opt.add(logFName, "l", "log_name");
  opt.add(outFname, "o", "out_name");
  opt.add(paramName, "p", "param_file");
  opt.add(rayTracing, "t", "ray_trace");
  opt.parse();

  lcm_pub = bot_lcm_get_global(NULL);
  if (!logFName.empty()) {
    fromLog = true;
    char provider_buf[1024];
    sprintf(provider_buf, "file://%s?speed=0", logFName.c_str());
    lcm_recv = lcm_create(provider_buf);
  }
  else {
    printf("running mapping from LCM\n");
    lcm_recv = lcm_pub;

  }
  if (paramName.empty())
    param = bot_param_get_global(lcm_pub, 0);
  else
    param = bot_param_new_from_file(paramName.c_str());
  frames = bot_frames_get_global(lcm_recv, param);

  //TODO: these should be parameters
  double xy0[2] = { -200, -200 };
  double xy1[2] = { 200, 200 };

  pixmap = new FloatPixelMap(xy0, xy1, resolution, 0);

  laser_projectors = g_hash_table_new(g_str_hash, g_str_equal);

  char **planar_lidar_names = bot_param_get_all_planar_lidar_names(param);
  if (planar_lidar_names) {
    for (int pind = 0; planar_lidar_names[pind] != NULL; pind++) {
      char conf_path[1024];
      bot_param_get_planar_lidar_prefix(param, planar_lidar_names[pind], conf_path, sizeof(conf_path));
      char channel_path[1024];
      sprintf(channel_path, "%s.lcm_channel", conf_path);
      char * channel_name;
      int ret = bot_param_get_str(param, channel_path, &channel_name);
      if (ret < 0) {
        printf("No LCM Channel for lidar %s\n", planar_lidar_names[pind]);
        continue;
      }
      printf("subscribing to channel %s for laser %s\n", channel_name, planar_lidar_names[pind]);
      bot_core_planar_lidar_t_subscribe(lcm_recv, channel_name, on_laser, this);
      g_hash_table_insert(laser_projectors, strdup(channel_name), laser_projector_new(param, frames,
          planar_lidar_names[pind], 1));
      free(channel_name);
    }
    g_strfreev(planar_lidar_names);
  }
  else {
    fprintf(stderr, "["__FILE__":%d] Error: Could not"
    " get lidar names.\n", __LINE__);
    exit(1);
  }
}

LaserPixmapper::~LaserPixmapper()
{
  delete pixmap;
  //TODO: don't leak everything else?
}

inline float logOddsToML(float l)
{
  return l > 0.1 ? 1. : (l < -0.1 ? 0 : .5);
}

void LaserPixmapper::publish_map()
{
  fprintf(stderr, "Publishing map... ");
  FloatPixelMap * MLpixmap = new FloatPixelMap(pixmap, logOddsToML);
  const occ_map_pixel_map_t * msg = MLpixmap->get_pixel_map_t(bot_timestamp_now());
  occ_map_pixel_map_t_publish(lcm_pub, "VOXEL_MAP", msg);
  delete MLpixmap;
  fprintf(stderr, "done! \n");
}

void LaserPixmapper::save_map()
{
  fprintf(stderr, "Saving map... ");
  FloatPixelMap * MLpixmap = new FloatPixelMap(pixmap, logOddsToML);
  MLpixmap->saveToFile(outFname);

  delete MLpixmap;
  fprintf(stderr, "done! \n");
}

void LaserPixmapper::processScansInQueue()
{
  bot_tictoc("addLaser");
  list<laser_projected_scan *>::iterator it;
  for (it = lscans_to_be_processed.begin(); it != lscans_to_be_processed.end(); it++) {
    laser_projected_scan * lscan = *it;
    if (!lscan->projection_status) {
      laser_update_projected_scan(lscan->projector, lscan, bot_frames_get_root_name(this->frames));
    }
    if (!lscan->projection_status)
      break;
    else {
      bot_tictoc("processScan");
      for (int i = 0; i < lscan->npoints; i += (beam_skip + 1)) {
        double max_range = lscan->projector->max_range - 1.0;
        if (lscan->point_status[i] >= laser_min_range)
          continue;
        else if (lscan->point_status[i] == laser_max_range) { //maxrange
          if (rayTracing) {
//            octomap::point3d endpoint(lscan->points[i].x, lscan->points[i].y, lscan->points[i].z);
//            ocTree->insertRay(origin, endpoint, lscan->projector->max_range / 2.0);
          }
        }
        else {
          double endpoint[3] = { lscan->points[i].x, lscan->points[i].y, lscan->points[i].z };
          if (rayTracing) {
            pixmap->rayTrace(lscan->origin.trans_vec, endpoint, MISS_INC, HIT_INC);
          }
          else {
            pixmap->updateValue(endpoint, HIT_INC);
          }

        }

      }
      bot_tictoc("processScan");

      laser_destroy_projected_scan(lscan);
    }
  }
  if (it != lscans_to_be_processed.begin()) {
    lscans_to_be_processed.erase(lscans_to_be_processed.begin(), it); //destructor was already called in the loop
  }
  bot_tictoc("addLaser");
}

/**
 * Shutdown
 */
static LaserPixmapper * _map3d;
static void shutdown_module(int unused __attribute__((unused)))
{
  fprintf(stderr, "shutting down!\n");

  _map3d->save_map(); //TODO make an option?
  _map3d->publish_map();
  bot_tictoc_print_stats(BOT_TICTOC_AVG);
  exit(1);
}

/**
 * Main
 */
int main(int argc, char *argv[])
{
  signal(SIGINT, shutdown_module);

  LaserPixmapper *map3d = new LaserPixmapper(argc, argv);
  _map3d = map3d;
  while (true) {
    int ret = lcm_handle(map3d->lcm_recv);
    if (ret != 0)
      break; //log is done...
  }
  shutdown_module(1);

}
