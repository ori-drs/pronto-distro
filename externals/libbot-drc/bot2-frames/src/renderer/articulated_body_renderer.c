/*
 *
 * articulated_body_renderer.c
 *
 * renders an articulated object where each body in the object is drawn according to a bot_frames frame
 * bodies are defined in a param file:
 *
 * Example Param File Snippet (assumes laser and camera frames have been appropriately defined
articulated_body_name {
  body1 {
    frame = "laser";
    visualization = "cube"; #sphere, cylinder, model.rwx, model.obj
    scale  = [1, 2, 3];
    translation = [.5, 1, 1.5];
    rpy = [0,0,0];
    color = [0, 1, 0, 1];
  }

  body2 {
    frame = "camera";
    visualization = "cube";
    scale  = [3, 2, 1];
    translation = [1.5, 1, .5];
    rpy = [0,0,0];
    color = [1, 0, 0, 1];
  }
}

 * translation and rpy are constant draw offsets with respect to the frame.
 * the visualization field can either be a geometric object: cube, sphere, or cylinder, or an .rwx or .obj model file
 *
 * the path to the model files and the name of the articulated body in the param file are passed into bot_frames_add_articulated_body_renderer_to_viewer
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bot_frames_renderers.h"
#include <bot_param/param_util.h>
#include <bot_vis/bot_vis.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#define DEG2RAD(X) (X*180.0/M_PI)

typedef enum {
  cube, sphere, cylinder, rwx, wavefront, invalid
} vis_type_t;

typedef struct _BodyProperties BodyProperties;

struct _BodyProperties {
  char * frame_name;
  vis_type_t vis_type;
  double scale[3];
  BotTrans body_to_frame_trans;
  BotTrans cur_draw_trans;
  double color[4];
  GLuint draw_list;
  BotRwxModel * rwx_model;
  BotWavefrontModel * wave_model;
  int draw_list_ready;
};

typedef struct _RendererArticulated RendererArticulated;

struct _RendererArticulated {
  BotRenderer renderer;
//  BotGtkParamWidget *pw;
  BotViewer *viewer;
  BotParam * param;
  BotFrames * frames;

  char articulated_name[256];

  int num_bodies;
  BodyProperties * body_properties;
};

static void frames_update_handler(BotFrames *bot_frames, const char *frame, const char * relative_to, int64_t utime,
    void *user)
{
  RendererArticulated *self = (RendererArticulated *) user;
  int ii;
  //loop through all frames for bodies and request redraw if any are updated
  //FIXME this will cause redundant redrawing as frame updates trickle through, not sure how to handle
  for (ii = 0; ii < self->num_bodies; ii++) {
    if (strcmp(self->body_properties[ii].frame_name, frame) == 0) {
      bot_viewer_request_redraw(self->viewer);
      return;
    }
  }
}

static GLuint compile_rwx_display_list(BotRwxModel * model)
{
  GLuint dl = glGenLists(1);
  glNewList(dl, GL_COMPILE);

  glPushMatrix();

  glEnable(GL_BLEND);
  glEnable(GL_RESCALE_NORMAL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);

  glEnable(GL_LIGHTING);
  bot_rwx_model_gl_draw(model);
  glDisable(GL_LIGHTING);
  glDisable(GL_RESCALE_NORMAL);
  glDisable(GL_BLEND);

  glPopMatrix();
  glEndList();
  return dl;
}

static GLuint compile_wave_display_list(BotWavefrontModel * model)
{
  GLuint dl = glGenLists(1);
  glNewList(dl, GL_COMPILE);

  glEnable(GL_LIGHTING);


  bot_wavefront_model_gl_draw(model);
  glDisable(GL_LIGHTING);

  glEndList();
  return dl;
}

void draw_body(RendererArticulated * self, BodyProperties * body_properties)
{
  BotTrans frame_trans, draw_trans;
  bot_frames_get_trans(self->frames, body_properties->frame_name, bot_frames_get_root_name(self->frames), &frame_trans);
  draw_trans = body_properties->body_to_frame_trans;

  bot_trans_apply_trans(&draw_trans, &frame_trans);
  // rotate and translate the vehicle

  double curr_quat_m[16];
  bot_quat_pos_to_matrix(draw_trans.rot_quat, draw_trans.trans_vec, curr_quat_m);
  // opengl expects column-major matrices
  double curr_quat_m_opengl[16];
  bot_matrix_transpose_4x4d(curr_quat_m, curr_quat_m_opengl);

  glPushMatrix();
  //get to the vehicle CG at correct orientation
  glMultMatrixd(curr_quat_m_opengl);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glScaled(body_properties->scale[0], body_properties->scale[1], body_properties->scale[2]);

  glColor4d(body_properties->color[0], body_properties->color[1], body_properties->color[2], body_properties->color[3]);

  switch (body_properties->vis_type) {
  case cube:
    bot_gl_draw_cube();
    break;

  case sphere:
    {
      glPushAttrib(GL_ENABLE_BIT);
      glEnable(GL_DEPTH_TEST);
      glPushMatrix();
      GLUquadricObj *q = gluNewQuadric();
      double radius = 1;
      int slices = 20;
      int stacks = 20;
      gluSphere(q, radius, slices, stacks);
      gluDeleteQuadric(q);
      glPopMatrix();
      glPopAttrib();
      break;
    }

  case cylinder:
    {
      double r_base = 1;
      double r_top = 1;
      double height = 1;
      int slices = 20;
      int stacks = 20;

      glPushAttrib(GL_ENABLE_BIT);
      glEnable(GL_DEPTH_TEST);
      glPushMatrix();
      GLUquadricObj *q = gluNewQuadric();
      gluCylinder(q, r_base, r_top, height, slices, stacks);
      glPopMatrix();
      gluDeleteQuadric(q);
      glPopAttrib();
      break;
    }
  case rwx:
    if (!body_properties->draw_list_ready) {
      body_properties->draw_list = compile_rwx_display_list(body_properties->rwx_model);
      body_properties->draw_list_ready = 1;
    }
    glCallList(body_properties->draw_list);
    break;

  case wavefront:
    if (!body_properties->draw_list_ready) {
      body_properties->draw_list = compile_wave_display_list(body_properties->wave_model);
      body_properties->draw_list_ready = 1;
    }
    glPushMatrix();
    glEnable(GL_BLEND);
    glEnable(GL_RESCALE_NORMAL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glCallList(body_properties->draw_list);
    glPopMatrix();
    break;
  case invalid:
    //nothing to draw
    break;
  }

  //pop matrix from translation
  glPopMatrix();
}

void configure_body(RendererArticulated * self, BodyProperties * body_properties, const char * body_name,
    const char * models_dir)
{
  char key_name[256];
  sprintf(key_name, "%s.%s.frame", self->articulated_name, body_name);
  body_properties->frame_name = (char *) calloc(256, sizeof(char));
  if (bot_param_get_str(self->param, key_name, &body_properties->frame_name) == -1)
    goto fail; //FIXME no error checking currently to see if the frame exists

  sprintf(key_name, "%s.%s.visualization", self->articulated_name, body_name);
  char * vis_string = calloc(256, sizeof(char));
  if (bot_param_get_str(self->param, key_name, &vis_string) == -1)
    goto fail;

  char model_full_path[256];
  BotRwxModel * rwx_model;

  int vis_string_length = strlen(vis_string);
  if (strcmp(vis_string, "cube") == 0) {
    body_properties->vis_type = cube;
  }
  else if (strcmp(vis_string, "sphere") == 0) {
    body_properties->vis_type = sphere;
  }
  else if (strcmp(vis_string, "cylinder") == 0) {
    body_properties->vis_type = cylinder;
  }
  else if (strcmp(&vis_string[vis_string_length - 4], ".rwx") == 0) {
    snprintf(model_full_path, sizeof(model_full_path), "%s/%s", models_dir, vis_string);
    body_properties->rwx_model = bot_rwx_model_create(model_full_path);
    body_properties->vis_type = rwx;
  }
  else if (strcmp(&vis_string[vis_string_length - 4], ".obj") == 0) {
    snprintf(model_full_path, sizeof(model_full_path), "%s/%s", models_dir, vis_string);
    body_properties->wave_model = bot_wavefront_model_create(model_full_path);
    body_properties->vis_type = wavefront;
  }
  else {
    fprintf(
        stderr,
        "error: invalid display type, must either be \"cube\" or of type \".rwx\", nothing will be drawn for body %s\n",
        body_name);
    body_properties->vis_type = invalid;
  }

  sprintf(key_name, "%s.%s", self->articulated_name, body_name);
  bot_param_get_trans(self->param, key_name, &body_properties->body_to_frame_trans);

  sprintf(key_name, "%s.%s.color", self->articulated_name, body_name);
  if (bot_param_get_double_array(self->param, key_name, body_properties->color, 4) == -1) {
    //default color is blue
    body_properties->color[0] = 0;
    body_properties->color[1] = 0;
    body_properties->color[2] = 1;
    body_properties->color[3] = 1;
  }

  sprintf(key_name, "%s.%s.scale", self->articulated_name, body_name);
  if (bot_param_get_double_array(self->param, key_name, body_properties->scale, 3) == -1) {
    //default scale is 1
    body_properties->scale[0] = 1;
    body_properties->scale[1] = 1;
    body_properties->scale[2] = 1;
  }

  return;

  fail: fprintf(stderr, "error: failed to get %s, nothing will be drawn for this body", key_name);
  body_properties->vis_type = invalid;

}

static void articulated_body_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererArticulated *self = (RendererArticulated*) renderer;
  int ii;
  for (ii = 0; ii < self->num_bodies; ii++)
    draw_body(self, &self->body_properties[ii]);
}

static void articulated_body_free(BotRenderer *renderer)
{
  RendererArticulated *self = (RendererArticulated*) renderer;
  int ii;
  for (ii = 0; ii < self->num_bodies; ii++) {
    if (self->body_properties[ii].vis_type == rwx)
      bot_rwx_model_destroy(self->body_properties[ii].rwx_model);
    else if (self->body_properties[ii].vis_type == wavefront)
      bot_wavefront_model_destroy(self->body_properties[ii].wave_model);
  }
  free(self->body_properties);
  free(self);
}
//
//static void on_param_widget_changed(BotGtkParamWidget *pw, const char *param, void *user_data)
//{
//  RendererArticulated *self = (RendererArticulated*) user_data;
//  bot_viewer_request_redraw(self->viewer);
//}
//
//static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
//{
//  RendererArticulated *self = user_data;
//  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, self->articulated_name);
//}
//
//static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
//{
//  RendererArticulated *self = user_data;
//  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, self->articulated_name);
//}

void bot_frames_add_articulated_body_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param,
    BotFrames * frames, const char * model_path, const char * param_articulated_name)
{
  RendererArticulated *self = (RendererArticulated*) calloc(1, sizeof(RendererArticulated));
  BotRenderer *renderer = &self->renderer;
  self->param = param;
  self->frames = frames;
  self->viewer = viewer;
  strcpy(self->articulated_name, param_articulated_name);

  bot_frames_add_update_subscriber(self->frames, frames_update_handler, (void *) self);

  renderer->draw = articulated_body_draw;
  renderer->destroy = articulated_body_free;
  renderer->name = self->articulated_name;
  renderer->user = self;
  renderer->enabled = 1;

  //  renderer->widget = gtk_vbox_new(FALSE, 0);
  //  renderer->widget = gtk_alignment_new(0, 0.5, 1.0, 0);
  //
  //  g_signal_connect (G_OBJECT (self->pw), "changed",
  //      G_CALLBACK (on_param_widget_changed), self);
//  g_signal_connect (G_OBJECT (viewer), "load-preferences",
//      G_CALLBACK (on_load_preferences), self);
//  g_signal_connect (G_OBJECT (viewer), "save-preferences",
//      G_CALLBACK (on_save_preferences), self);

  self->num_bodies = bot_param_get_num_subkeys(self->param, self->articulated_name);
  if (self->num_bodies == -1) {
    fprintf(stderr, "error: %s not found in config\n", self->articulated_name);
  }
  else if (self->num_bodies == 0) {
    fprintf(stderr, "error: %s has no subkeys (bodies)\n", self->articulated_name);
  }

  char ** body_names = bot_param_get_subkeys(self->param, self->articulated_name);

  self->body_properties = (BodyProperties *) calloc(self->num_bodies, sizeof(BodyProperties));

  int ii;
  for (ii = 0; ii < self->num_bodies; ii++) {
    configure_body(self, &self->body_properties[ii], body_names[ii], model_path);
  }

  g_strfreev(body_names);

  bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
  return;

}

