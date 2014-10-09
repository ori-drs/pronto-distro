#include "LaserSim3D.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_util.h>
#include <geom_utils/geometry.h>

#include <string>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

typedef struct {
  lcm_t *lcm;
  BotParam *param;
  BotFrames *frames;

  int64_t last_print_time;

  float occupancy_thresh;

  int64_t last_publish_time;

  char * laser_name;
  char * laser_channel;

  laser_util::LaserSim3D * sim;

} app_t;

using namespace std;
using namespace octomap;

static gboolean pub_handler(gpointer userdata)
{
  app_t * self = (app_t *) userdata;

  bot_tictoc("publishLaser");
  //get the current location of the lidar
  BotTrans curr_pose;
  bot_frames_get_trans(self->frames, self->laser_name, bot_frames_get_root_name(self->frames), &curr_pose);

  int64_t utime;
  bot_frames_get_trans_latest_timestamp(self->frames, self->laser_name, bot_frames_get_root_name(self->frames),
      &utime);

  const bot_core_planar_lidar_t * laser_msg = self->sim->simulate(&curr_pose, utime);

  bot_core_planar_lidar_t_publish(self->lcm, self->laser_channel, laser_msg);
  bot_tictoc("publishLaser");

  int now = bot_timestamp_now();
  if (now - self->last_print_time > 1e6) {
    fprintf(stderr, ".");
    self->last_print_time = now;
  }

}

void usage(char * name)
{
  fprintf(stderr, "usage: %s octomap-filename [lidar-name]\n", name);
  fprintf(stderr, " assumes the coordinate frames are all set up in the bot-param-server,\n");
  fprintf(stderr, " and that the pose of the laser will be updated in BotFrames\n");
  exit(1);
}

int main(int argc, char *argv[])
{

  app_t app;
  memset(&app, 0, sizeof(app));

  if (argc < 2 || argc > 3)
    usage(argv[0]);
  char * map_fname = argv[1];
  if (argc > 2)
    app.laser_name = strdup(argv[2]);
  else
    app.laser_name = strdup("sim_laser");

  GMainLoop * mainloop = g_main_loop_new(NULL, FALSE);

  app.lcm = bot_lcm_get_global(NULL);
  bot_glib_mainloop_attach_lcm(app.lcm);
  bot_signal_pipe_glib_quit_on_kill(mainloop);
  app.param = bot_param_get_global(app.lcm, 0);
  app.frames = bot_frames_get_global(app.lcm, app.param);

  float publish_freq = bot_param_get_double_or_fail(app.param,
      ("planar_lidars." + string(app.laser_name) + ".frequency").c_str());

  app.laser_channel = bot_param_get_planar_lidar_lcm_channel(app.param, app.laser_name);

  //TODO: remove hardcoded parameters for a hokuyo UTM
  octomap::OcTree * ocTree = new OcTree(map_fname);
  app.sim = new laser_util::LaserSim3D(ocTree, app.param, app.frames, app.laser_name);

  g_timeout_add((guint) 1.0 / publish_freq * 1000.0, pub_handler, &app);

  fprintf(stderr, "Publishing the simulated lider '%s'\n", app.laser_name);

// run
  g_main_loop_run(mainloop);

//shutdown
  fprintf(stderr, "Shutting Down\n");
  bot_tictoc_print_stats(BOT_TICTOC_AVG);

}
