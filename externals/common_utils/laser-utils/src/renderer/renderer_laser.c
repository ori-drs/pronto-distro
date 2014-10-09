#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <laser_utils/laser_util.h>

#include "renderer_laser.h"

#define RENDERER_NAME "Laser"
#define PARAM_SCAN_MEMORY "Scan Memory"
#define PARAM_MAX_BUFFER_SIZE "Max Buffer Size"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_Z_BUFFER "Z Buffer"
#define PARAM_BIG_POINTS "Big Points"
#define PARAM_SPATIAL_DECIMATE "Spatial Decimation"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"
#define PARAM_Z_RELATIVE "Relative Z"
#define PARAM_ALPHA "Alpha"

#define MAX_SCAN_MEMORY 5000
#define OLD_HISTORY_THRESHOLD 3000000 /* microseconds */
#define MAX_SENSOR_RANGE_DEFAULT 30.0 /* meters */
#define MIN_SENSOR_RANGE_DEFAULT 0.15 /* meters */
#define COLOR_MODE_Z_MAX_Z 30
#define COLOR_MODE_Z_MIN_Z -30
#define COLOR_MODE_Z_DZ 0.1
#define PARAM_MAX_DRAW_Z "Max Draw Z"
#define PARAM_MIN_DRAW_Z "Min Draw Z"

#define PARAM_MAX_DRAW_RANGE "MaxDrawRng"
#define MAX_DRAW_RANGE 80.0 /* length of a sick range - 80.0m */

#define SPACIAL_DECIMATION_LIMIT 0.05 /* meters */
#define ANGULAR_DECIMATION_LIMIT .075 /* radians */

#ifndef DATA_FROM_LR3
#define DATA_FROM_LR3 0
#endif

#ifndef USE_LATEST_POSE
#define USE_LATEST_POSE 1
#endif

#if 1
#define ERR(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__);  \
        fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)
#else
#define ERR(...)
#endif

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#define DBG(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)
#endif

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_LASER, COLOR_MODE_INTENSITY, COLOR_MODE_Z, COLOR_MODE_WORKSPACE
} color_mode_t;

typedef struct _laser_channel {
  int enabled;
  char * name; /* lidar name (from bot_param file) */
  char * channel; /* channel name */
  float color[3]; /* point cloud color (from bot_param file) */

  Laser_projector *projector;

  BotPtrCircular *scans; /* circular buffer of laser_scan */

} laser_channel;

typedef struct _RendererLaser {
  BotRenderer renderer;

  BotViewer *viewer;
  BotParam *bot_param;
  BotFrames *bot_frames;
  lcm_t *lcm;

  /* user parameters */
  BotGtkParamWidget *pw;
  int param_scan_memory;
  int param_max_buffer_size;
  color_mode_t param_color_mode;
  gboolean param_z_buffer;
  gboolean param_big_points;
  gboolean param_spacial_decimate;
  double param_color_mode_z_max_z;
  double param_color_mode_z_min_z;
  double param_max_draw_z;
  double param_min_draw_z;
  double param_max_draw_range;
  double param_alpha;
  int z_relative;

  gchar *last_save_filename;

  GHashTable *channels_hash; /* hash of laser_channels, key: lidar
   configuration file name, value: pointer to
   laser_channel */
  GPtrArray *channels; /* array of pointers to all laser_channel */
  GList *lcm_hids;

  //vertex buffer to draw all points at once
  int pointBuffSize;
  double * pointBuffer;
  float * colorBuffer;

} RendererLaser;

static void laser_scan_destroy(void *user, void *p)
{
  if (p) {
    laser_projected_scan *lscan = (laser_projected_scan*) p;
    laser_destroy_projected_scan(lscan);
  }
}

static void renderer_laser_destroy(BotRenderer *renderer)
{
  if (!renderer)
    return;

  RendererLaser *self = (RendererLaser*) renderer->user;
  if (!self)
    return;

  /* stop listening to lcm */
  if (self->lcm) {
    for (GList *iter = self->lcm_hids; iter; iter = iter->next) {
      bot_core_planar_lidar_t_subscription_t *hid = (bot_core_planar_lidar_t_subscription_t *) iter->data;
      bot_core_planar_lidar_t_unsubscribe(self->lcm, hid);
    }
  }

  /* free channels hash table and array */
  if (self->channels) {
    for (int i = 0; i < self->channels->len; i++) {
      laser_channel *lchan = g_ptr_array_index(self->channels, i);
      if (lchan->scans)
        bot_ptr_circular_destroy(lchan->scans);
      if (lchan->projector)
        laser_projector_destroy(lchan->projector);
      if (lchan->name)
        free(lchan->name);
      free(lchan);
    }
    g_ptr_array_free(self->channels, FALSE);
  }

  if (self->channels_hash)
    g_hash_table_destroy(self->channels_hash);

  if (self->last_save_filename)
    g_free(self->last_save_filename);

  free(self);
}

static void renderer_laser_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererLaser *self = (RendererLaser*) renderer->user;
  g_assert(self);

  /* get local position in order to height-color points */
  color_mode_t color_mode = self->param_color_mode;
  int z_relative = self->z_relative;
  double z_norm_scale = 0;
  if (color_mode == COLOR_MODE_Z) {
    if (!bot_frames_have_trans(self->bot_frames, "body", bot_frames_get_root_name(self->bot_frames))) {

      color_mode = COLOR_MODE_DRAB; /* no position */
    }
    else
      z_norm_scale = 1 / (self->param_color_mode_z_max_z - self->param_color_mode_z_min_z);
  }

  //compute total number of points that need to get drawn
  int numPointsToDraw = 0;
  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);
    if (!lchan->enabled)
      continue;
    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);
      if (!lscan->projection_status) {
        laser_update_projected_scan(lchan->projector, lscan, bot_frames_get_root_name(self->bot_frames));
      }
      //count number of points we want to draw
      for (int i = 0; i < lscan->npoints; i++) {
        if ((lscan->point_status[i]<=laser_valid_projection && lscan->points[i].z >= self->param_min_draw_z&& lscan->points[i].z <= self->param_max_draw_z)
              &&  (lscan->rawScan->ranges[i] <= self->param_max_draw_range ))

          numPointsToDraw++;
      }
    }
  }
  //reallocate the vertex buffers if necessary
  if (numPointsToDraw > self->pointBuffSize) {
    self->pointBuffSize = numPointsToDraw;
    self->pointBuffer = (double *) realloc(self->pointBuffer, numPointsToDraw * 3 * sizeof(double));
    self->colorBuffer = (float *) realloc(self->colorBuffer, numPointsToDraw * 4 * sizeof(float));
  }

  int numLoadedPoints = 0;
  //load points into the vertex buffers
  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);
    if (!lchan->enabled)
      continue;
    //        printf("%s has %d points.\n", lchan->name, lchan->scans->size);

    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);

      for (int i = 0; i < lscan->npoints; i++) {
        if (lscan->point_status[i]>laser_valid_projection || lscan->points[i].z < self->param_min_draw_z|| 
               lscan->points[i].z > self->param_max_draw_z || lscan->rawScan->ranges[i] > self->param_max_draw_range ){
          continue;
        }
        else {
          float * colorV = self->colorBuffer + (4 * numLoadedPoints); //pointer into the color buffer
          colorV[3] = self->param_alpha;
          switch (color_mode) {
          case COLOR_MODE_Z:
            {
              double z;
              if (z_relative)
                z = lscan->points[i].z - lscan->origin.trans_vec[2];
              else
                z = lscan->points[i].z;

              double z_norm = (z - self->param_color_mode_z_min_z) * z_norm_scale;
              float * jetC = bot_color_util_jet(z_norm);
              memcpy(colorV, jetC, 3 * sizeof(float));
            }
            break;
          case COLOR_MODE_WORKSPACE:
            {
              double range_mapping = (lscan->rawScan->ranges[i] -0.5 /1.5); // map 0.5->2 range to be 0->1
              double z_norm=0;
              if (range_mapping >1.0){
                range_mapping = 1.0;
              }else if(range_mapping < 0.0){
                range_mapping = 0.0;
              }
              float * jetC = bot_color_util_jet(range_mapping);
              memcpy(colorV, jetC, 3 * sizeof(float));
            }
            break;
          case COLOR_MODE_INTENSITY:
            {
              if (lscan->rawScan->nintensities == lscan->rawScan->nranges) {
                double v = lscan->rawScan->intensities[i];
                if (v > 1) {
                  // SICK intensity ranges: (rarely used now)
                  // double minv = 7000, maxv = 12000; 
                  // Hokuyo Typical Ranges:
                  double minv = 300, maxv = 4000;
                  
                  v = fmax(300.0 , fmin(v,4000.0) );
                  v = (v - minv) / (maxv - minv);
                }
                float * jetC = bot_color_util_jet(v);
                memcpy(colorV, jetC, 3 * sizeof(float));
                break;
              }
            }
            // no intensity data? fall through!
          case COLOR_MODE_LASER:
            memcpy(colorV, lchan->color, 3 * sizeof(float));
            break;
          case COLOR_MODE_DRAB:/* COLOR_MODE_DRAB */
          default:
            {
              colorV[0] = 0.3;
              colorV[1] = 0.3;
              colorV[2] = 0.3;
            }
            break;
          }
          double * pointP = self->pointBuffer + (3 * numLoadedPoints);
          memcpy(pointP, point3d_as_array(&lscan->points[i]), 3 * sizeof(double));
          numLoadedPoints++;
        }
      }

    }
  }

  if (numLoadedPoints != numPointsToDraw) {
    fprintf(stderr, "ERROR: numLoadedPoints=%d, numPointsToDraw=%d \n", numLoadedPoints, numPointsToDraw);
  }

  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
  if (self->param_z_buffer) {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
  }

  if (self->param_big_points)
    glPointSize(4.0f);
  else
    glPointSize(2.0f);

 if (self->param_alpha < 1) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable(GL_BLEND);
  }

  //render using vertex buffers
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glColorPointer(4, GL_FLOAT, 0, self->colorBuffer);
  glVertexPointer(3, GL_DOUBLE, 0, self->pointBuffer);
  //draw
  glDrawArrays(GL_POINTS, 0, numPointsToDraw);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);

  if (self->param_alpha < 1)
    glDisable(GL_BLEND);
  if (self->param_z_buffer)
    glDisable(GL_DEPTH_TEST);

  glPopAttrib();
}

static void on_laser(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_planar_lidar_t *msg, void *user)
{
  RendererLaser *self = (RendererLaser*) user;
  g_assert(self);

  /* get the laser channel object based on channel name */
  laser_channel *lchan = g_hash_table_lookup(self->channels_hash, channel);
  if (lchan == NULL) {
    /* allocate and initialize laser channel structure */
    lchan = (laser_channel*) calloc(1, sizeof(laser_channel));
    g_assert(lchan);
    lchan->enabled = 1;
    lchan->name = bot_param_get_planar_lidar_name_from_lcm_channel(self->bot_param, channel);
    lchan->channel = strdup(channel);
    lchan->projector = laser_projector_new(self->bot_param, self->bot_frames, lchan->name, 1);

    char param_prefix[1024];
    bot_param_get_planar_lidar_prefix(NULL, lchan->name, param_prefix, sizeof(param_prefix));

    char color_key[1024];
    sprintf(color_key, "%s.viewer_color", param_prefix);
    double color[3];
    int color_size = bot_param_get_double_array(self->bot_param, color_key, color, 3);
    if (color_size != 3) {
      ERR("Error: Missing or funny color for planar LIDAR "
      "configuration key: '%s'\n", color_key);
      lchan->color[0] = 1;
      lchan->color[1] = 1;
      lchan->color[2] = 1;
    }
    else {
      lchan->color[0] = color[0];
      lchan->color[1] = color[1];
      lchan->color[2] = color[2];
    }

    lchan->scans = bot_ptr_circular_new(self->param_max_buffer_size, laser_scan_destroy, NULL);
    g_assert(lchan->scans);

    /* add laser channel to hash table and array */
    g_hash_table_insert(self->channels_hash, lchan->channel, lchan);
    g_ptr_array_add(self->channels, lchan);

    /* add check box */
    if (self->viewer)
      bot_gtk_param_widget_add_booleans(self->pw, 0, lchan->name, lchan->enabled, NULL);
  }

  /* TODO: Optimization - allocate space for local points from a
   circular buffer instead of calling calloc for each scan */

  laser_projected_scan *lscan = laser_create_projected_scan_from_planar_lidar(lchan->projector, msg,
      bot_frames_get_root_name(self->bot_frames));
  if (lscan == NULL)
    return; //probably didn't have a pose message yet...
  
  if(lchan->scans->capacity != self->param_max_buffer_size){
      bot_ptr_circular_resize(lchan->scans, self->param_max_buffer_size);
  }
  
  if (bot_ptr_circular_size(lchan->scans) > 0) {
    laser_projected_scan *last_scan = bot_ptr_circular_index(lchan->scans, 0);

    /* check for a large time difference between scans (typical when
     jumping around an LCM log file) */
    gboolean time_jump = FALSE;
    int64_t dt = msg->utime - last_scan->utime;
    if (dt < -OLD_HISTORY_THRESHOLD || dt > OLD_HISTORY_THRESHOLD)
      time_jump = TRUE;

    /* spacial decimation */
    gboolean stationary = FALSE;
    BotTrans delta;
    if (self->param_spacial_decimate && self->param_scan_memory > 10) {
      bot_trans_invert_and_compose(&lscan->origin, &last_scan->origin, &delta);
      double dist = bot_vector_magnitude_3d(delta.trans_vec);
      double rot;
      double axis[3];
      bot_quat_to_angle_axis(delta.rot_quat, &rot, axis);
      if (dist < SPACIAL_DECIMATION_LIMIT && rot < ANGULAR_DECIMATION_LIMIT) {
        stationary = TRUE;
      }
    }

    if (stationary) {
      laser_scan_destroy(NULL, lscan);
      return;
    }
  }

  bot_ptr_circular_add(lchan->scans, lscan);

  if (self->viewer)
    bot_viewer_request_redraw(self->viewer);

  return;
}

static void on_clear_button(GtkWidget *button, RendererLaser *self)
{
  if (!self->viewer)
    return;

  for (int i = 0; i < self->channels->len; i++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, i);
    bot_ptr_circular_clear(lchan->scans);
  }
  bot_viewer_request_redraw(self->viewer);
}

static int save_points_to_file(RendererLaser *self, FILE *file)
{
  /* first line: number of columns */
  fprintf(file, "%%%% %d\n", 10);
  /* second line: column titles */
  fprintf(file, "%%%% POINT_X POINT_Y POINT_Z ORIGIN_X ORIGIN_Y ORIGIN_Z"
      " POINT_STATUS LASER_NUMER SCAN_NUMER POINT_NUMBER\n");

  int count = 0;
  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);

    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);

      for (int i = 0; i < lscan->npoints; i++) {
        fprintf(file, "%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %d %d %d "
            "%d\n", lscan->points[i].x, lscan->points[i].y, lscan->points[i].z, lscan->origin.trans_vec[0],
            lscan->origin.trans_vec[1], lscan->origin.trans_vec[2], lscan->point_status[i], chan_idx, scan_idx, i);
        count++;
      }
    }
  }
  return count;
}

static void on_save_button(GtkWidget *button, RendererLaser *self)
{
  GtkWidget *dialog;
  dialog = gtk_file_chooser_dialog_new("Save Data File", NULL, GTK_FILE_CHOOSER_ACTION_SAVE, GTK_STOCK_CANCEL,
      GTK_RESPONSE_CANCEL, GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT, NULL);
  g_assert(dialog);
  gtk_file_chooser_set_do_overwrite_confirmation(GTK_FILE_CHOOSER(dialog), TRUE);
  if (self->last_save_filename)
    gtk_file_chooser_set_filename(GTK_FILE_CHOOSER(dialog), self->last_save_filename);

  if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
    char *filename;
    filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
    if (NULL != filename) {
      FILE *file = fopen(filename, "w");
      if (NULL != file) {
        /* store selected file into self->last_save_filename */
        if (self->last_save_filename)
          g_free(self->last_save_filename);
        self->last_save_filename = g_strdup(filename);

        int count = save_points_to_file(self, file);

        DBG("Wrote %d points to file '%s'.\n", count, filename);
        fclose(file);
      }
      else {
        ERR("Error: Failed to open file: '%s', error is: '%s'\n",
            filename, g_strerror(errno));
      }
      g_free(filename);
    }
  }
  gtk_widget_destroy(dialog);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererLaser *self = (RendererLaser*) user;

  if (!self->viewer)
    return;

  self->param_scan_memory = bot_gtk_param_widget_get_int(self->pw, PARAM_SCAN_MEMORY);
  self->param_color_mode = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);
  self->param_z_buffer = bot_gtk_param_widget_get_bool(self->pw, PARAM_Z_BUFFER);
  self->param_big_points = bot_gtk_param_widget_get_bool(self->pw, PARAM_BIG_POINTS);
  self->param_spacial_decimate = bot_gtk_param_widget_get_bool(self->pw, PARAM_SPATIAL_DECIMATE);
  self->z_relative = bot_gtk_param_widget_get_bool(self->pw, PARAM_Z_RELATIVE);
  self->param_color_mode_z_max_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);
  self->param_color_mode_z_min_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);
  self->param_max_draw_z = bot_gtk_param_widget_get_double(self->pw, PARAM_MAX_DRAW_Z);
  self->param_min_draw_z = bot_gtk_param_widget_get_double(self->pw, PARAM_MIN_DRAW_Z);
  self->param_max_draw_range = bot_gtk_param_widget_get_double(self->pw, PARAM_MAX_DRAW_RANGE);
  self->param_alpha = bot_gtk_param_widget_get_double(self->pw, PARAM_ALPHA);
  self->param_max_buffer_size = bot_gtk_param_widget_get_int(self->pw, PARAM_MAX_BUFFER_SIZE);

  for (int i = 0; i < self->channels->len; i++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, i);
    lchan->enabled = bot_gtk_param_widget_get_bool(pw, lchan->name);
  }

  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererLaser *self = (RendererLaser*) user;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererLaser *self = (RendererLaser*) user;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

static BotRenderer* renderer_laser_new(BotViewer *viewer, lcm_t * lcm, BotParam * param, BotFrames * frames)
{
  RendererLaser *self = (RendererLaser*) calloc(1, sizeof(RendererLaser));
  g_assert(self);

  self->viewer = viewer;

  BotRenderer *renderer = &self->renderer;
  renderer->draw = renderer_laser_draw;
  renderer->destroy = renderer_laser_destroy;
  renderer->user = self;
  renderer->name = RENDERER_NAME;
  renderer->enabled = 1;

  self->lcm = lcm;
  self->bot_param = param;
  self->bot_frames = frames;

  if (!self->lcm) {
    ERR("Error: setup_renderer_laser() failed no LCM provided "
    "object\n");
    renderer_laser_destroy(renderer);
    return NULL;
  }

  self->param_scan_memory = 50;
  self->param_color_mode = COLOR_MODE_LASER;
  self->param_max_buffer_size = MAX_SCAN_MEMORY;
  self->param_z_buffer = TRUE;
  self->param_big_points = FALSE;
  self->param_spacial_decimate = FALSE;
  self->param_color_mode_z_max_z = COLOR_MODE_Z_MAX_Z;
  self->param_color_mode_z_min_z = COLOR_MODE_Z_MIN_Z;
  self->param_max_draw_z = COLOR_MODE_Z_MAX_Z;
  self->param_min_draw_z = COLOR_MODE_Z_MIN_Z;
  self->param_max_draw_range = MAX_DRAW_RANGE;
  self->param_alpha = 1;
  self->param_max_buffer_size = MAX_SCAN_MEMORY; 

  if (viewer) {
    /* setup parameter widget */
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = gtk_vbox_new(FALSE, 0);
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int(self->pw, PARAM_SCAN_MEMORY, BOT_GTK_PARAM_WIDGET_SPINBOX, 1, MAX_SCAN_MEMORY, 1,
        self->param_scan_memory);
    bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->param_color_mode,
        "Laser", COLOR_MODE_LASER, "Drab", COLOR_MODE_DRAB, "Intensity", COLOR_MODE_INTENSITY, "Height", COLOR_MODE_Z, "Workspace", COLOR_MODE_WORKSPACE,
        NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_Z_BUFFER, self->param_z_buffer, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_BIG_POINTS, self->param_big_points, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_SPATIAL_DECIMATE, self->param_spacial_decimate, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_Z_RELATIVE, 0, NULL);

    bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER,
        0, 1, .01, self->param_alpha);

    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SPINBOX,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_max_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SPINBOX,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_min_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_MIN_DRAW_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, COLOR_MODE_Z_MIN_Z,
        COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_min_draw_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_DRAW_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, COLOR_MODE_Z_MIN_Z,
        COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_max_draw_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_DRAW_RANGE, BOT_GTK_PARAM_WIDGET_SPINBOX, 0,
        MAX_DRAW_RANGE, 0.1, MAX_DRAW_RANGE); // from short to sick range

    bot_gtk_param_widget_add_int(self->pw, PARAM_MAX_BUFFER_SIZE, BOT_GTK_PARAM_WIDGET_SPINBOX, 1, MAX_SCAN_MEMORY, 1,
        self->param_max_buffer_size);

    GtkWidget *clear_button = gtk_button_new_with_label("Clear memory");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);
    GtkWidget *save_button = gtk_button_new_with_label("Save To Points File");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_button), "clicked", G_CALLBACK(on_save_button), self);
    gtk_widget_show_all(renderer->widget);

    /* setup signal callbacks */
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
  }

  // iterate through planar lidars, subscribing to thier LCM and
  // initializing the channels' hash table and array
  self->channels_hash = g_hash_table_new(g_str_hash, g_str_equal);
  self->channels = g_ptr_array_new();
  bot_core_planar_lidar_t_subscription_t *hid;

  char **planar_lidar_names = bot_param_get_all_planar_lidar_names(self->bot_param);
  if (planar_lidar_names) {
    for (int pind = 0; planar_lidar_names[pind] != NULL; pind++) {
      char conf_path[1024];
      bot_param_get_planar_lidar_prefix(self->bot_param, planar_lidar_names[pind], conf_path, sizeof(conf_path));
      char channel_path[1024];
      sprintf(channel_path, "%s.lcm_channel", conf_path);
      char * channel_name;
      int ret = bot_param_get_str(self->bot_param, channel_path, &channel_name);
      if (ret < 0) {
        printf("No LCM Channel for lidar %s\n", planar_lidar_names[pind]);
        continue;
      }
      printf("subscribing to channel %s for laser %s\n", channel_name, planar_lidar_names[pind]);
      hid = bot_core_planar_lidar_t_subscribe(self->lcm, channel_name, on_laser, self);
      self->lcm_hids = g_list_append(self->lcm_hids, hid);
      free(channel_name);
    }
    g_strfreev(planar_lidar_names);
  }
  else {
    fprintf(stderr, "["__FILE__":%d] Error: Could not"
    " get lidar names.\n", __LINE__);
  }

  return &self->renderer;
}

void laser_util_add_renderer_to_viewer(BotViewer *viewer, int priority, lcm_t * lcm, BotParam * param,
    BotFrames * frames)
{
  BotRenderer *renderer = renderer_laser_new(viewer, lcm, param, frames);
  if (viewer && renderer) {
    bot_viewer_add_renderer(viewer, renderer, priority);
  }
}

