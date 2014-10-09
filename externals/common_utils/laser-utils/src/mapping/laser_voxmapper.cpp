#include "laser_voxmapper.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <bot_core/bot_core.h>
#include <pthread.h>
#include <deque>
#include <bot_param/param_util.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <vector>

using namespace std;
using namespace occ_map;
//TODO: make me a parameter
static int scan_skip = 0;
static int beam_skip = 1;
static double publish_interval = 1;

#define MISS_INC -0.1
#define HIT_INC 1.0

static void on_laser(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_planar_lidar_t *msg,
    void *user_data)
{
  LaserVoxmapper * self = (LaserVoxmapper *) user_data;

  BotTrans pose;
  Laser_projector * proj = (Laser_projector*) g_hash_table_lookup(self->laser_projectors, channel);
  if (proj==NULL)
    return;
  bot_frames_get_trans(self->frames, proj->coord_frame, bot_frames_get_root_name(self->frames), &pose);

  //discard messages with poses that are outside the map
  if (!self->voxmap->isInMap(pose.trans_vec))
    return;

  static int counter = 0;
  if (counter++ % (scan_skip + 1) != 0)
    return;

  laser_projected_scan * lscan = laser_create_projected_scan_from_planar_lidar(proj, msg, bot_frames_get_root_name(
      self->frames));
  if (lscan != NULL)
    self->addProjectedScan(lscan);

  self->processScansInQueue();

  if (msg->utime - self->last_publish_time> publish_interval*1e6){
    self->last_publish_time = msg->utime;
    self->publish_map();
  }

}

void LaserVoxmapper::addProjectedScan(laser_projected_scan * lscan)
{
  lscans_to_be_processed.push_back(lscan);
}

LaserVoxmapper::LaserVoxmapper(char * logFname) :
  last_publish_time(-1)
{
  lcm_pub = bot_lcm_get_global(NULL);
  if (logFname != NULL) {
    fromLog = true;
    char provider_buf[1024];
    sprintf(provider_buf, "file://%s?speed=0", logFname);
    lcm_recv = lcm_create(provider_buf);
  }
  else {
    printf("running mapping from LCM\n");
    lcm_recv = lcm_pub;

  }
  param = bot_param_get_global(lcm_pub,0); //use pub cuz it's always from network
  frames = bot_frames_get_global(lcm_pub, param);

  //TODO: these should be parameters
  float res = .1;
  double xyz0[3] = { -30, -30, -5 };
  double xyz1[3] = { 30, 30, 10 };
  double mpp[3] = { res, res, res };

  voxmap = new FloatVoxelMap(xyz0, xyz1, mpp, 0);

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
      g_hash_table_insert(laser_projectors, strdup(channel_name), laser_projector_new(param, frames, planar_lidar_names[pind],
          1));
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

LaserVoxmapper::~LaserVoxmapper()
{
  delete voxmap;
  //TODO: don't leak everything else?
}

inline float logOddsToML(float l)
{
  return l > 0.1 ? 1. : (l < -0.1 ? 0 : .5);
}

void LaserVoxmapper::publish_map()
{
  fprintf(stderr, "Publishing map... ");
  FloatVoxelMap * MLvoxmap = new FloatVoxelMap(voxmap, logOddsToML);
  const occ_map_voxel_map_t * msg = MLvoxmap->get_voxel_map_t(bot_timestamp_now());
  occ_map_voxel_map_t_publish(lcm_pub, "VOXEL_MAP", msg);
  delete MLvoxmap;
  fprintf(stderr, "done! \n");
}

void LaserVoxmapper::save_map()
{
  fprintf(stderr, "Saving map... ");
  FloatVoxelMap * MLvoxmap = new FloatVoxelMap(voxmap, logOddsToML);
  MLvoxmap->saveToFile("voxmap.vxmp");

  delete MLvoxmap;
  fprintf(stderr, "done! \n");
}

void LaserVoxmapper::processScansInQueue()
{
  bot_tictoc("addLaser");
  list<laser_projected_scan *>::iterator it;
  for (it = lscans_to_be_processed.begin(); it != lscans_to_be_processed.end(); it++) {
    laser_projected_scan * lscan = *it;
    if (!lscan->projection_status) {
      laser_update_projected_scan(lscan->projector, lscan, "local");
    }
    if (!lscan->projection_status)
      break;
    else {
      bot_tictoc("processScan");
      double origin[3] = { lscan->origin.trans_vec[0], lscan->origin.trans_vec[1], lscan->origin.trans_vec[2] };
      for (int i = 0; i < lscan->npoints; i += (beam_skip + 1)) {
        double max_range = lscan->projector->max_range - 1.0;
        if (lscan->point_status[i] > laser_min_range)
          continue;
        else if (lscan->points[i].z < voxmap->xyz0[2]) { //discard rays below this to the map
          continue;
        }
        else if (lscan->points[i].z > voxmap->xyz1[2]) { //crop rays above this to the threshold
          double ray[3];
          bot_vector_subtract_3d(point3d_as_array(&lscan->points[i]), lscan->origin.trans_vec, ray);
          double s = (voxmap->xyz1[2] - lscan->origin.trans_vec[2]) / (lscan->points[i].z - lscan->origin.trans_vec[2]);
          bot_vector_scale_3d(ray, s);
          double endpoint[3];
          bot_vector_add_3d(lscan->origin.trans_vec, ray, endpoint);
          voxmap->raytrace(origin, endpoint, MISS_INC, MISS_INC);
        }
        else if (lscan->point_status[i] == laser_max_range) {
          double endpoint[3] = { lscan->points[i].x, lscan->points[i].y, lscan->points[i].z };
          voxmap->raytrace(origin, endpoint, MISS_INC, MISS_INC);
        }
        else {
          double endpoint[3] = { lscan->points[i].x, lscan->points[i].y, lscan->points[i].z };
          voxmap->raytrace(origin, endpoint, MISS_INC, HIT_INC);
        }

      }
      bot_tictoc("processScan");

      laser_destroy_projected_scan( lscan);
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
static LaserVoxmapper * _map3d;
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
  char * logFName = NULL;
  if (argc > 1)
    logFName = argv[1];

  LaserVoxmapper *map3d = new LaserVoxmapper(logFName);
  _map3d = map3d;
  while (true) {
    int ret = lcm_handle(map3d->lcm_recv);
    if (ret != 0)
      break;//log is done...
  }
  shutdown_module(1);

}
