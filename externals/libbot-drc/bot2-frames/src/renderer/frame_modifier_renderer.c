#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <gtk/gtk.h>

#include <lcm/lcm.h>

#include "bot_frames_renderers.h"
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#define RENDERER_NAME "Frame Modifier"

#define PARAM_FRAME_SELECT "Frame"

#define PARAM_X         "X    "
#define PARAM_Y         "Y    "
#define PARAM_Z         "Z    "
#define PARAM_ROLL      "Roll "
#define PARAM_PITCH     "Pitch"
#define PARAM_YAW       "Yaw  "

#define PARAM_SAVE  "Save Calib"

typedef struct _RendererFrames {

  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  BotGtkParamWidget *pw;

  BotFrames * frames;

  const char * rootFrame;
  int numFrames;
  char ** frameNames;
  int * frameNums;

  int updating;

} RendererFrames;

static void destroy_renderer_frames(BotRenderer *super)
{
  RendererFrames *self = (RendererFrames*) super->user;

  free(self);
}

static void draw(BotViewer *viewer, BotRenderer *super)
{
  RendererFrames *self = (RendererFrames*) super->user;
  //nothing to draw

}

static void frames_update_handler(BotFrames *bot_frames, const char *frame, const char * relative_to, int64_t utime,
    void *user)
{
  RendererFrames *self = (RendererFrames *) user;
  BotTrans tran;
  int activeSensorNum = bot_gtk_param_widget_get_enum(self->pw, PARAM_FRAME_SELECT);
  if (activeSensorNum > 0) {
    const char * activeSensorName = self->frameNames[activeSensorNum];
    if (strcmp(frame, activeSensorName) == 0) {
      const char * relative_to = bot_frames_get_relative_to(self->frames, activeSensorName);
      bot_frames_get_trans(self->frames, activeSensorName, relative_to, &tran);
      double rpy[3];
      bot_quat_to_roll_pitch_yaw(tran.rot_quat, rpy);
      bot_gtk_param_widget_set_double(self->pw, PARAM_X, tran.trans_vec[0]);
      bot_gtk_param_widget_set_double(self->pw, PARAM_Y, tran.trans_vec[1]);
      bot_gtk_param_widget_set_double(self->pw, PARAM_Z, tran.trans_vec[2]);
      bot_gtk_param_widget_set_double(self->pw, PARAM_ROLL, bot_to_degrees(rpy[0]));
      bot_gtk_param_widget_set_double(self->pw, PARAM_PITCH, bot_to_degrees(rpy[1]));
      bot_gtk_param_widget_set_double(self->pw, PARAM_YAW, bot_to_degrees(rpy[2]));
      self->updating = 0;
    }
  }
  bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererFrames *self = (RendererFrames*) user;
  if (self->updating) {
    return;
  }
  BotViewer *viewer = self->viewer;
  int activeSensorNum = bot_gtk_param_widget_get_enum(pw, PARAM_FRAME_SELECT);
  if (!strcmp(name, PARAM_FRAME_SELECT)) {
    if (activeSensorNum > 0) {
      self->updating = 1;
      bot_viewer_set_status_bar_message(self->viewer, "Modify Calibration relative to %s", bot_frames_get_relative_to(
          self->frames, self->frameNames[activeSensorNum]));
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 1);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 1);
      frames_update_handler(self->frames, self->frameNames[activeSensorNum], bot_frames_get_relative_to(self->frames,
          self->frameNames[activeSensorNum]), bot_timestamp_now(), self);
    }
    else {
      bot_viewer_set_status_bar_message(self->viewer, "");
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 0);
      bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 0);
    }
  }
  else if (!strcmp(name, PARAM_SAVE) && activeSensorNum > 0) {
    char save_fname[1024];
    sprintf(save_fname, "manual_calib_%s.cfg", self->frameNames[activeSensorNum]);
    fprintf(stderr, "saving params to: %s\n", save_fname);
    FILE * f = fopen(save_fname, "w");
    double pos[3];
    pos[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X);
    pos[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_Y);
    pos[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_Z);
    double rpy[3];
    rpy[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_ROLL);
    rpy[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_PITCH);
    rpy[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_YAW);

    double rpy_rad[3];
    for (int i = 0; i < 3; i++)
      rpy_rad[i] = bot_to_radians(rpy[i]);
    double quat[4];
    bot_roll_pitch_yaw_to_quat(rpy_rad, quat);
    double rod[3];
    bot_quat_to_rodrigues(quat, rod);

    fprintf(f, ""
      "%s {\n"
      "position = [%f, %f, %f];\n"
      "rpy = [%f, %f, %f];\n"
      "relative_to = \"%s\";\n"
      "}", self->frameNames[activeSensorNum], pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2],
        bot_frames_get_relative_to(self->frames, self->frameNames[activeSensorNum]));
    fprintf(f, ""
      "\n"
      "%s {\n"
      "position = [%f, %f, %f];\n"
      "rodrigues = [%f, %f, %f];\n"
      "relative_to = \"%s\";\n"
      "}", self->frameNames[activeSensorNum], pos[0], pos[1], pos[2], rod[0], rod[1], rod[2],
        bot_frames_get_relative_to(self->frames, self->frameNames[activeSensorNum]));

    fclose(f);
    bot_viewer_set_status_bar_message(self->viewer, "Calibration saved to %s", save_fname);

  }
  else if (activeSensorNum > 0) {
    self->updating = 1;
    BotTrans curr;
    curr.trans_vec[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X);
    curr.trans_vec[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_Y);
    curr.trans_vec[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_Z);

    double rpy[3];
    rpy[0] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_ROLL));
    rpy[1] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_PITCH));
    rpy[2] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_YAW));
    bot_roll_pitch_yaw_to_quat(rpy, curr.rot_quat);
    if (fabs(rpy[0]) > M_PI || fabs(rpy[1]) > M_PI || fabs(rpy[2]) > M_PI) {
      bot_gtk_param_widget_set_double(self->pw, PARAM_ROLL, bot_to_degrees(bot_mod2pi(rpy[0])));
      bot_gtk_param_widget_set_double(self->pw, PARAM_PITCH, bot_to_degrees(bot_mod2pi(rpy[1])));
      bot_gtk_param_widget_set_double(self->pw, PARAM_YAW, bot_to_degrees(bot_mod2pi(rpy[2])));
    }

    //and update the link
    const char * frame_name = self->frameNames[activeSensorNum];
    const char * relative_to = bot_frames_get_relative_to(self->frames, frame_name);
    bot_frames_update_frame(self->frames, frame_name, relative_to, &curr, bot_timestamp_now());
  }

  bot_viewer_request_redraw(self->viewer);
}

void bot_frames_add_frame_modifier_to_viewer(BotViewer *viewer, int render_priority, BotFrames * frames)
{

  RendererFrames *self = (RendererFrames*) calloc(1, sizeof(RendererFrames));

  BotRenderer *renderer = &self->renderer;

  renderer->draw = draw;
  renderer->destroy = destroy_renderer_frames;

  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = strdup(RENDERER_NAME);
  renderer->user = self;
  renderer->enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = renderer->name;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = NULL;
  ehandler->mouse_release = NULL;
  ehandler->mouse_motion = NULL;
  ehandler->user = self;

  self->viewer = viewer;
  self->frames = frames;
  bot_frames_add_update_subscriber(self->frames, frames_update_handler, (void *) self);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  self->rootFrame = bot_frames_get_root_name(self->frames);
  int num_frames = bot_frames_get_num_frames(self->frames);

  char ** frame_names = bot_frames_get_frame_names(self->frames);

  self->numFrames = num_frames + 1; //need space for extra "empty" one
  self->frameNames = calloc(self->numFrames, sizeof(char *));
  self->frameNums = calloc(self->numFrames, sizeof(int));
  self->frameNums[0] = 0;
  self->frameNames[0] = "";
  for (int i = 1; i < self->numFrames; i++) {
    self->frameNums[i] = i;
    self->frameNames[i] = strdup(frame_names[i - 1]);
  }
  g_strfreev(frame_names);
  bot_gtk_param_widget_add_enumv(self->pw, PARAM_FRAME_SELECT, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numFrames,
      (const char **) self->frameNames, self->frameNums);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);

  bot_gtk_param_widget_add_double(self->pw, PARAM_X, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_Y, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_ROLL, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_PITCH, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_YAW, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);

  bot_gtk_param_widget_add_separator(self->pw, "");
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_SAVE, NULL);

  bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 0);
  bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 0);

  gtk_widget_show_all(renderer->widget);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
}
