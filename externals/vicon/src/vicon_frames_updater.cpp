/*
 * vicon_frames_updater.cpp
 *
 *  Created on: Aug 31, 2011
 *      Author: abry
 */

#include <lcm/lcm.h>
#include <lcmtypes/vicon_body_t.h>
#include <bot_frames/bot_frames.h>

typedef struct {
  lcm_t * lcm;
  BotFrames * frames;
  BotParam * param;
  char * body_frame_name, *vicon_channel;
} app_t;

static void vicon_handler(const lcm_recv_buf_t *rbuf, const char *channel, const vicon_body_t *msg,
    void *user_data)
{
  app_t * app = (app_t *) user_data;
  BotTrans body_to_local;

  bot_trans_set_from_quat_trans(&body_to_local, msg->quat, msg->trans);
  bot_frames_update_frame(app->frames, app->body_frame_name, bot_frames_get_root_name(app->frames), &body_to_local,
      msg->utime);
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    fprintf(stderr, "usage: %s <full vicon channel name> [frame name to update]\n", argv[0]);
    exit(1);
  }
  app_t * app = (app_t *) calloc(1, sizeof(app_t));
  app->vicon_channel = strdup(argv[1]);
  app->body_frame_name = strdup("body");
  if (argc > 2) {
    app->body_frame_name = strdup(argv[2]);
  }

  app->lcm = lcm_create(NULL);
  app->param = bot_param_new_from_server(app->lcm, 0);
  app->frames = bot_frames_new(app->lcm, app->param);

  vicon_body_t_subscribe(app->lcm, app->vicon_channel, vicon_handler, (void *) app);

  while (true) {
    int ret = lcm_handle(app->lcm);
  }
}

