/*
 * renders a VoxelMap
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gtk/gtk.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <occ_map/VoxelMap.hpp>

#include <lcmtypes/occ_map_voxel_map_t.h>

#include "occ_map_renderers.h"

#define RENDERER_NAME "VoxelMap"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_RENDER_MODE "Render Mode"
#define PARAM_Z_MAX_CUTOFF "Z Max Cutoff"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"
#define PARAM_SHOW_FREE "Show Free"
#define PARAM_SHOW_OCC  "Show Occ"
#define PARAM_SHOW_ALPHA "Show Alpha"
#define PARAM_CUTOFF "Occ Cutoff"
#define PARAM_POINT_SIZE "Point Size"

using namespace occ_map;

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_Z,
} color_mode_t;

enum {
  RENDER_MODE_POINTS, RENDER_MODE_BOXES
};

typedef struct _OccMapRendererVoxelMap OccMapRendererVoxelMap;

struct _OccMapRendererVoxelMap {
  BotRenderer renderer;
  BotEventHandler ehandler;
  lcm_t *lc;
  BotGtkParamWidget *pw;
  GtkWidget *label;
  BotViewer *viewer;
  FloatVoxelMap* voxmap;

  GLuint voxelmap_dl;

  //vertex buffer to draw all points at once
  int pointBuffSize;
  double * pointBuffer;
  float * colorBuffer;
  bool data_dirty;

  int numPointsToDraw;
};

static void update_vertex_buffers(OccMapRendererVoxelMap *self)
{
  bool show_free = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FREE);
  bool show_occ = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_OCC);
  bool show_alpha = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA);

  double cutoff = bot_gtk_param_widget_get_double(self->pw, PARAM_CUTOFF);

  double occ_thresh = 0.5 + cutoff;
  double free_thresh = 0.5 - cutoff;

  int color_mod = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);

  self->numPointsToDraw = 0;
  int ixyz[3];
  for (ixyz[2] = 0; ixyz[2] < self->voxmap->dimensions[2]; ixyz[2]++) {
    for (ixyz[1] = 0; ixyz[1] < self->voxmap->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < self->voxmap->dimensions[0]; ixyz[0]++) {
        float likelihood = self->voxmap->readValue(ixyz);
        if ((show_occ && likelihood <= 1.0 && likelihood > occ_thresh) || (show_free && likelihood >= 0 && likelihood
            < free_thresh)) {
          self->numPointsToDraw++;
        }

      }
    }
  }
  self->pointBuffSize = self->numPointsToDraw;

  int color_size = 3;
  if (show_alpha)
    color_size = 4;
  self->pointBuffer = (double *) realloc(self->pointBuffer, self->numPointsToDraw * 3 * sizeof(double));
  self->colorBuffer = (float *) realloc(self->colorBuffer, self->numPointsToDraw * color_size * sizeof(float));

  double max_z_cutoff = bot_gtk_param_widget_get_double(self->pw, PARAM_Z_MAX_CUTOFF);

  double min_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);
  double max_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);

  int numLoadedPoints = 0;
  for (ixyz[2] = 0; ixyz[2] < self->voxmap->dimensions[2]; ixyz[2]++) {
    for (ixyz[1] = 0; ixyz[1] < self->voxmap->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < self->voxmap->dimensions[0]; ixyz[0]++) {
        float likelihood = self->voxmap->readValue(ixyz);
        if ((show_occ && likelihood <= 1.0 && likelihood > occ_thresh) || (show_free && likelihood >= 0 && likelihood
            < free_thresh)) {

          double * pointP = self->pointBuffer + (3 * numLoadedPoints); //pointer into the vertex buffer
          self->voxmap->tableToWorld(ixyz, pointP);
          float * colorP = self->colorBuffer + (color_size * numLoadedPoints); //pointer into the color buffer
          switch (color_mod) {
          case COLOR_MODE_Z:
            {
              double z_norm = (pointP[2] - min_z) / (max_z - min_z);
              float * jetC = bot_color_util_jet(z_norm);
              memcpy(colorP, jetC, 3 * sizeof(float));
            }
            break;
          case COLOR_MODE_DRAB:
            {
              colorP[0] = 0.3;
              colorP[1] = 0.3;
              colorP[2] = 0.3;
            }
            break;
          }
          if (show_alpha)
            colorP[3] = (likelihood - cutoff) / (1 - cutoff);

          numLoadedPoints++;
        }
      }
    }
  }
  self->data_dirty = true;
}

static void voxmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const occ_map_voxel_map_t *msg, void *user)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;

  if (self->voxmap != NULL)
    delete self->voxmap;
  self->voxmap = new FloatVoxelMap(msg);
  update_vertex_buffers(self);
  bot_viewer_request_redraw(self->viewer);
}

static void VoxelMap_draw(BotViewer *viewer, BotRenderer *renderer)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) renderer;
  if (self->voxmap == NULL)
    return;

  // do we have new voxel data?
  if (self->data_dirty) {
    if (self->voxelmap_dl) {
      glDeleteLists(self->voxelmap_dl, 1);
    }
    self->voxelmap_dl = glGenLists(1);
    glNewList(self->voxelmap_dl, GL_COMPILE_AND_EXECUTE);

    //fprintf(stderr, "data dirty\n");
    glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    //z_buffering
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    int rmode = bot_gtk_param_widget_get_enum(self->pw, PARAM_RENDER_MODE);

    if (rmode == RENDER_MODE_POINTS) {
      glPointSize(bot_gtk_param_widget_get_int(self->pw, PARAM_POINT_SIZE));

#if 1
      // render using vertex arrays
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_COLOR_ARRAY);

      int color_size = 3;
      if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA))
        color_size = 4;

      glColorPointer(color_size, GL_FLOAT, 0, self->colorBuffer);
      glVertexPointer(3, GL_DOUBLE, 0, self->pointBuffer);

      // draw
      glDrawArrays(GL_POINTS, 0, self->numPointsToDraw);

      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_COLOR_ARRAY);
#else
      glBegin(GL_POINTS);
      if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA)) {
        for (int i = 0; i < self->numPointsToDraw; i++) {
          glColor4fv(self->colorBuffer + 4 * i);
          glVertex3dv(self->pointBuffer + 3 * i);
        }
      }
      else {
        for (int i = 0; i < self->numPointsToDraw; i++) {
          glColor3fv(self->colorBuffer + 3 * i);
          glVertex3dv(self->pointBuffer + 3 * i);
        }
      }
      glEnd();
    }
    else {
      int use_alpha = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA);
      double mpp[3] = {self->voxmap->metersPerPixel[0],
        self->voxmap->metersPerPixel[1],
        self->voxmap->metersPerPixel[2]};
      glEnable(GL_LIGHTING);
      //      glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
      glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
      glEnable(GL_COLOR_MATERIAL);
      glEnable(GL_RESCALE_NORMAL);
      for (int i = 0; i < self->numPointsToDraw; i++) {
        double* p = self->pointBuffer + 3 * i;
        if (use_alpha) {
          float* c = self->colorBuffer + 4 * i;
          glColor4fv(c);
        }
        else {
          float* c = self->colorBuffer + 3 * i;
          //          glColor3f(c[0] * 0.1, c[1] * 0.1, c[2] * 0.1);
          glColor3fv(c);
        }
        glPushMatrix();
        glTranslatef(p[0] + mpp[0] / 2, p[1] + mpp[1] / 2, p[2] + mpp[2] / 2);
        glScalef(mpp[0], mpp[1], mpp[2]);
        bot_gl_draw_cube();
        glPopMatrix();
      }
      glDisable(GL_RESCALE_NORMAL);
      glDisable(GL_COLOR_MATERIAL);
      glDisable(GL_LIGHTING);
#endif
    }

    glPopAttrib();

    glEndList();
    self->data_dirty = false;
  }
  else {
    glCallList(self->voxelmap_dl);
  }
}

static void VoxelMap_free(BotRenderer *renderer)
{
  OccMapRendererVoxelMap* self = (OccMapRendererVoxelMap*) renderer;
  free(self->pointBuffer);
  free(self->colorBuffer);
  free(renderer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;
  if (self->voxmap != NULL)
    update_vertex_buffers(self);
  if (!strcmp(name, PARAM_Z_MAX_CUTOFF))
    update_vertex_buffers(self);
  bot_viewer_request_redraw(self->viewer);
}
static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_clear_button(GtkWidget *button, OccMapRendererVoxelMap *self)
{
  if (self->voxmap != NULL) {
    delete self->voxmap;
    self->voxmap = NULL;
  }
  if (self->pointBuffer != NULL) {
    free(self->pointBuffer);
    self->pointBuffer = NULL;
    self->pointBuffSize = 0;
  }
  if (self->colorBuffer != NULL) {
    free(self->colorBuffer);
    self->colorBuffer = NULL;
  }
  if (self->voxelmap_dl) {
    glDeleteLists(self->voxelmap_dl, 1);
    self->voxelmap_dl = 0;
  }
  self->data_dirty = false;

  if (!self->viewer)
    return;
  bot_viewer_request_redraw(self->viewer);
}

static BotRenderer*
renderer_voxel_map_new(BotViewer *viewer, int render_priority, const char* lcm_channel)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) calloc(1, sizeof(OccMapRendererVoxelMap));
  BotRenderer *renderer = &self->renderer;
  self->viewer = viewer;
  self->lc = bot_lcm_get_global(NULL);
  self->pointBuffer = NULL;
  self->colorBuffer = NULL;

  self->voxelmap_dl = 0;
  self->data_dirty = false;

  renderer->draw = VoxelMap_draw;
  renderer->destroy = VoxelMap_free;
  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = (char *) RENDERER_NAME;
  renderer->user = self;
  renderer->enabled = 1;

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  renderer->widget = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, COLOR_MODE_Z, "Height",
      COLOR_MODE_Z, "Drab", COLOR_MODE_DRAB, NULL);

  bot_gtk_param_widget_add_enum(self->pw, PARAM_RENDER_MODE, BOT_GTK_PARAM_WIDGET_MENU, RENDER_MODE_BOXES, "Points",
      RENDER_MODE_POINTS, "Boxes", RENDER_MODE_BOXES, NULL);

  bot_gtk_param_widget_add_int(self->pw, PARAM_POINT_SIZE, BOT_GTK_PARAM_WIDGET_SLIDER, 1, 20, 1, 4);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_ALPHA, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_OCC, 1, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FREE, 0, NULL);

  bot_gtk_param_widget_add_double(self->pw, PARAM_CUTOFF, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, .01, .1);

  bot_gtk_param_widget_add_double(self->pw, PARAM_Z_MAX_CUTOFF, BOT_GTK_PARAM_WIDGET_SPINBOX, -2, 40, .5, 5);

  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SLIDER, -2, 40, .5, 15);
  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SLIDER, -2, 40, .5, 0);

  GtkWidget *clear_button = gtk_button_new_with_label("Clear memory");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

  gtk_widget_show_all(renderer->widget);
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);

  // pick a default channel if none specified
  if (!lcm_channel || !strlen(lcm_channel))
    lcm_channel = "VOXEL_MAP";
  occ_map_voxel_map_t_subscribe(self->lc, lcm_channel, voxmap_handler, self);
  return &self->renderer;
}

extern "C" void occ_map_voxel_map_add_renderer_to_viewer(BotViewer *viewer, int render_priority,
    const char* lcm_channel)
{
  BotRenderer* renderer = renderer_voxel_map_new(viewer, render_priority, lcm_channel);
  bot_viewer_add_renderer(viewer, renderer, render_priority);
}
