#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include <zlib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <bot_core/bot_core.h>

#include "gtk_util.h"
#include "viewer.h"
#include "default_view_handler.h"

//#define dbg(args...) fprintf (stderr, args)
#define dbg(args...) 
#define err(args...) fprintf (stderr, args)

#define TYPE_BOT_VIEWER  bot_viewer_get_type()
#define BOT_VIEWER_BOT_VIEWER_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST ((klass), \
            TYPE_BOT_VIEWER, BotBotViewerClass ))
#define IS_BOT_VIEWER(obj) (G_TYPE_CHECK_INSTANCE_TYPE ((obj), TYPE_BOT_VIEWER))
#define IS_BOT_VIEWER_CLASS(klass)   (G_TYPE_CHECK_CLASS_TYPE((klass), TYPE_BOT_VIEWER))

#define MAX_REDRAW_HZ 30

static int g_draws = 0;


// a structure for bookmarked viewpoints
typedef struct _bookmark_persp bookmark_persp_t;
struct _bookmark_persp {
    BotViewer* viewer;
    double eye[3];
    double lookat[3];
    double up[3];
    BotProjectionMode projection_mode;
    int saved;
    int index;
} bmtemp;

enum {
    LOAD_PREFERENCES_SIGNAL,
    SAVE_PREFERENCES_SIGNAL,
    RENDER_BEGIN_SIGNAL,
    RENDER_END_SIGNAL,
    LAST_SIGNAL
};

typedef struct _BotViewerPriv BotViewerPriv;
struct _BotViewerPriv {
    /*< private >*/
    int64_t next_render_utime;
    int64_t render_interval_usec;

    bookmark_persp_t* bookmarks;
    int num_bookmarks;

    GtkAccelGroup* key_accel_group;
};
#define BOT_VIEWER_GET_PRIVATE(o) (G_TYPE_INSTANCE_GET_PRIVATE((o), TYPE_BOT_VIEWER, BotViewerPriv))

static guint bot_viewer_signals[LAST_SIGNAL] = { 0 };

void bot_viewer_request_redraw (BotViewer *self)
{
    bot_gtk_gl_drawing_area_invalidate (self->gl_area);
}

static gboolean
on_render_timer (BotViewer * self)
{
    BotViewerPriv *priv = BOT_VIEWER_GET_PRIVATE(self);
    // this timer is called more often than we actually want to render.  
    // check if we want to render.
    int64_t now = bot_timestamp_now();
    if(priv->next_render_utime < now) {
        self->movie_draw_pending = 1;
        bot_viewer_request_redraw(self);

        while(priv->next_render_utime < now) 
            priv->next_render_utime += priv->render_interval_usec;
    }

    return TRUE;
}

void 
bot_viewer_start_recording (BotViewer *self)
{
    if (self->is_recording) {
        err ("viewer: already recording!!\n");
        return;
    }

    assert (self->movie_buffer == NULL);

    int window_width = GTK_WIDGET (self->gl_area)->allocation.width;
    int window_height = GTK_WIDGET (self->gl_area)->allocation.height;

    self->movie_width = window_width - (window_width % 4);
    self->movie_height = window_height;
    self->movie_stride = self->movie_width * 3;
    self->movie_buffer = (uint8_t*) malloc (self->movie_stride * self->movie_height);
    self->movie_desired_fps = 1000.0 / gtk_spin_button_get_value (GTK_SPIN_BUTTON (self->fps_spin));
    self->movie_actual_fps = self->movie_desired_fps;
    self->movie_frame_last_utime = 0;

/*
    int wwidth, wheight;
    gtk_window_get_size(GTK_WINDOW(self->window), &wwidth, &wheight);
    printf("%5d, %5d\n", self->movie_width, self->movie_height);
    gtk_widget_set_size_request(GTK_WIDGET(self->gl_area), self->movie_width, self->movie_height);
    gtk_window_set_default_size(GTK_WINDOW(self->window), wwidth, wheight);
    gtk_widget_set_size_request(GTK_WIDGET(self->gl_area), self->movie_width, self->movie_height);
    gtk_window_set_resizable(GTK_WINDOW(self->window), FALSE);
*/

    self->movie_path = bot_fileutils_get_unique_filename(NULL, "viewer", 1, 
            "ppms.gz");
    self->movie_gzf = gzopen(self->movie_path, "w");
    gzsetparams(self->movie_gzf, Z_BEST_SPEED, Z_DEFAULT_STRATEGY);

    if (self->movie_gzf == NULL)
        goto abort_recording;
    
    bot_viewer_set_status_bar_message (self, "Recording to: %s", self->movie_path);

    BotViewerPriv *priv = BOT_VIEWER_GET_PRIVATE(self);
    priv->next_render_utime = bot_timestamp_now();
    double render_interval_sec = 1 / gtk_spin_button_get_value(GTK_SPIN_BUTTON(self->fps_spin));
    priv->render_interval_usec = (int64_t)(render_interval_sec * 1000000);

    // start a GLib timer that gets called 3 times as frequently as the video recording framerate
    int timer_interval_ms = MAX(1, (render_interval_sec * 1000) / 3);

    self->render_timer_id = g_timeout_add(timer_interval_ms, (GSourceFunc) on_render_timer, self);
    self->is_recording = 1;

    return;

abort_recording:
    gtk_toggle_tool_button_set_active (
            GTK_TOGGLE_TOOL_BUTTON (self->record_button),
            FALSE);
    free (self->mov_bgr_buf);
}

void 
bot_viewer_stop_recording (BotViewer *self)
{
    free(self->movie_buffer);
    self->movie_buffer = NULL;

    printf("\nRecording stopped\n");
    bot_viewer_set_status_bar_message (self, "Recording stopped");

    gzclose(self->movie_gzf);
    self->movie_gzf = NULL;
    free(self->movie_path);
    self->movie_draw_pending = 0;

    g_source_remove (self->render_timer_id);

    self->is_recording = 0;
    gtk_toggle_tool_button_set_active (GTK_TOGGLE_TOOL_BUTTON (self->record_button),
                                       FALSE);
//    gtk_window_set_resizable(GTK_WINDOW(self->window), TRUE);
}

static void update_status_bar(BotViewer *viewer)
{
    char buf[1024];
    
    if (viewer->picking_handler && !viewer->picking_handler->picking)
        viewer->picking_handler = NULL;

    int vp[4] = {0,0,0,0};
    if (viewer->gl_area &&
            bot_gtk_gl_drawing_area_set_context (viewer->gl_area) == 0) {
        glGetIntegerv(GL_VIEWPORT, vp);
    }
    int width = vp[2], height = vp[3];

    if (viewer->picking_handler)
        snprintf(buf, sizeof (buf), "%d x %d [%s]  %s", width, height, 
                 viewer->picking_handler->name,  viewer->status_bar_message);
    else
        snprintf(buf, sizeof (buf), "%d x %d [Idle] %s", width, height,
                 viewer->status_bar_message);

    gtk_statusbar_push(GTK_STATUSBAR(viewer->status_bar), 
                       gtk_statusbar_get_context_id(
                           GTK_STATUSBAR(viewer->status_bar),"info"), buf);
}

void bot_viewer_set_status_bar_message (BotViewer *viewer, const char *fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start (ap, fmt);
    vsnprintf (buf, sizeof (buf), fmt, ap);
    va_end (ap);

    if (viewer->status_bar_message)
        free(viewer->status_bar_message);

    viewer->status_bar_message = strdup(buf);

    update_status_bar(viewer);
}

void
bot_viewer_load_preferences (BotViewer *viewer, const char *fname)
{

    BotViewerPriv* priv = BOT_VIEWER_GET_PRIVATE(viewer);

    GKeyFile *preferences = g_key_file_new ();
    int loaded = 0;
    if (g_file_test (fname, G_FILE_TEST_IS_REGULAR)) {
        loaded = g_key_file_load_from_file (preferences, fname, 0, NULL);
    }

    if (!loaded) goto done;
    dbg ("loading viewer settings from %s\n", fname);
        
    for (int ridx = 0; ridx < viewer->renderers->len; ridx++) {
        BotRenderer *renderer = g_ptr_array_index (viewer->renderers, ridx);
        GtkCheckMenuItem *cmi = GTK_CHECK_MENU_ITEM (renderer->cmi);

        GError *err = NULL;
        int enabled = g_key_file_get_boolean (preferences,
                "__libviewer_enabled_renderers", renderer->name, &err);

        if (err) {
            err ("%s\n", err->message);
            g_error_free (err);
        } else {
            gtk_check_menu_item_set_active (cmi, enabled);
        }

        if (renderer->widget) {
            if (g_key_file_has_key (preferences, "__libviewer_show_renderers", 
                        renderer->name, NULL)) {
                renderer->expanded = g_key_file_get_boolean (preferences, 
                        "__libviewer_show_renderers", renderer->name, NULL);
                gtk_expander_set_expanded (GTK_EXPANDER (renderer->expander),
                        renderer->expanded);
            }
        }
    }
    
    // load bookmarks to preferences
    if(g_key_file_has_group(preferences, "__libviewer_bookmarks"))
    {
        for (int bm_indx=0; bm_indx < priv->num_bookmarks; bm_indx++) {
            GError *err = NULL;
            char *str_key;   
            str_key = g_strdup_printf("bookmark_%d", bm_indx);

            int bmlist_len = 11;      
            if(!g_key_file_has_key(preferences, "__libviewer_bookmarks", str_key, NULL))
                break;
            double* pointtolist = 
                g_key_file_get_double_list(preferences, "__libviewer_bookmarks", 
                        str_key, (gsize*) &bmlist_len, &err);
            if(err) {
                g_error_free(err);
                continue;
            }
            for (int i=0; i<3; i++) {
                priv->bookmarks[bm_indx].eye[i] = pointtolist[i];
                priv->bookmarks[bm_indx].lookat[i] = pointtolist[3+i];
                priv->bookmarks[bm_indx].up[i] = pointtolist[6+i];
            }
            priv->bookmarks[bm_indx].projection_mode =(int) pointtolist[9];
            priv->bookmarks[bm_indx].saved = pointtolist[10];
            free(pointtolist);
            g_free(str_key);

        }
    }
    g_signal_emit (G_OBJECT(viewer), bot_viewer_signals[LOAD_PREFERENCES_SIGNAL], 0, preferences);

done:
    g_key_file_free (preferences);
}

void
bot_viewer_save_preferences (BotViewer *viewer, const char *fname)
{
    GKeyFile *preferences = g_key_file_new ();
    BotViewerPriv* priv = BOT_VIEWER_GET_PRIVATE(viewer);

    dbg ("saving viewer settings...\n");

    for (int ridx = 0; ridx < viewer->renderers->len; ridx++) {
        BotRenderer *renderer = g_ptr_array_index (viewer->renderers, ridx);

        g_key_file_set_boolean (preferences, "__libviewer_enabled_renderers",
                renderer->name, renderer->enabled);
        if (renderer->widget) {
            g_key_file_set_boolean (preferences, "__libviewer_show_renderers",
                    renderer->name, renderer->expanded);
        }
    }

    // save bookmarks to prefs
    int bmlist_len = 11;
    char *str_key;
    for (int bm_indx=0; bm_indx < priv->num_bookmarks; bm_indx++) { 
        str_key = g_strdup_printf("bookmark_%d", bm_indx);
        double bmlist[bmlist_len]; 
        for(int i=0; i<3; i++) {
            bmlist[i] = priv->bookmarks[bm_indx].eye[i];
            bmlist[i+3] = priv->bookmarks[bm_indx].lookat[i];
            bmlist[i+6] = priv->bookmarks[bm_indx].up[i];
        }
        bmlist[9] = (double)priv->bookmarks[bm_indx].projection_mode;
        bmlist[10] = priv->bookmarks[bm_indx].saved;
        g_key_file_set_double_list(preferences, "__libviewer_bookmarks", str_key, bmlist, bmlist_len);
    }
    g_free(str_key);

    FILE *fp = fopen (fname, "w");
    if (!fp) {
        perror ("error saving preferences");
        return;
    }

    g_signal_emit (G_OBJECT (viewer), bot_viewer_signals[SAVE_PREFERENCES_SIGNAL], 
            0, preferences);

    gsize len = 0;
    char *data = g_key_file_to_data (preferences, &len, NULL);
    fwrite (data, len, 1, fp);
    free (data);

    fclose (fp);
    g_key_file_free (preferences);
}

// ============== event handlers ===========

static gboolean
on_gl_configure (GtkWidget *widget, GdkEventConfigure *event, void* user_data)
{
    BotViewer * self = (BotViewer *) user_data;

    update_status_bar(self); // redraw our window size
    bot_viewer_request_redraw (self);
    return TRUE;
}

static gboolean on_redraw_timer(void *user_data)
{
    BotViewer * self = (BotViewer *) user_data;
    self->redraw_timer_pending = 0;

    bot_viewer_request_redraw (self);
    return FALSE; // this is a one-shot event.
}

static void 
check_gl_errors (const char *label) {
    GLenum errCode = glGetError ();
    const GLubyte *errStr;
    
    while (errCode != GL_NO_ERROR) {
        errStr = gluErrorString(errCode);
        fprintf (stderr, "OpenGL Error on draw #%d (%s)\n", g_draws, label);
        fprintf (stderr, "%s\n", (char*)errStr);
        errCode = glGetError ();
    }
}

static void
render_scene (BotViewer *self)
{
//    glClearColor (0.0, 0.0, 0.0, 1.0);
    glClearColor(self->backgroundColor[0], self->backgroundColor[1], 
                 self->backgroundColor[2], self->backgroundColor[3]);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (self->view_handler)
        self->view_handler->update_gl_matrices(self, self->view_handler);

    glMatrixMode (GL_MODELVIEW);

    /* give the ambient light a blue tint to match the blue sky */
    float light0_amb[] = { 0.8, 0.8, 1.0, 1 };
    float light0_dif[] = { 1, 1, 1, 1 };
    float light0_spe[] = { .5, .5, .5, 1 };
    float light0_pos[] = { 100, 100, 100, 0 };
    glLightfv (GL_LIGHT0, GL_AMBIENT, light0_amb);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, light0_dif);
    glLightfv (GL_LIGHT0, GL_SPECULAR, light0_spe);
    glLightfv (GL_LIGHT0, GL_POSITION, light0_pos);
    glEnable (GL_LIGHT0);

    if (self->prettier_flag) {
        glEnable(GL_LINE_STIPPLE);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    }

    for (unsigned int ridx = 0; ridx < self->renderers->len; ridx++) {
        BotRenderer *renderer = g_ptr_array_index(self->renderers, ridx);

        if (renderer->enabled) {
            
            glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_POLYGON_STIPPLE_BIT | 
                         GL_POLYGON_BIT | GL_LINE_BIT | GL_FOG_BIT | GL_LIGHTING_BIT );
            glPushMatrix();

            if (renderer->draw)
                renderer->draw (self, renderer);

            check_gl_errors (renderer->name);

            glPopMatrix();
            glPopAttrib();
        }
    }
}

static gboolean
on_gl_expose (GtkWidget *widget, GdkEventExpose *event, void *user_data)
{
    BotViewer * self = (BotViewer *) user_data;

    // if not enough time has elapsed since our last redraw, we
    // schedule a redraw in the future (if one isn't already pending).
    int64_t now = bot_timestamp_now();
    double dt = (now - self->last_draw_utime) / 1000000.0;
    if (dt < (1.0/MAX_REDRAW_HZ)) {
        if (!self->redraw_timer_pending) {
            int delay_ms = (now - self->last_draw_utime)/1000 + 1;
            g_timeout_add(delay_ms, on_redraw_timer, self);
            self->redraw_timer_pending = 1;
        }
        return TRUE;
    }
    self->last_draw_utime = now;

    // If we're making a movie, don't draw any faster than the
    // requested movie FPS rate.
    if (self->movie_gzf && !self->movie_draw_pending)
        return TRUE;

    // set this to 1 in order to cause viewer to exit cleanly after a
    // few hundred frames: useful for generating gprof output.
    g_draws++;
    if (0) {
        int thresh = 300;

        // for profiling PROFILE
        if (g_draws%50 == 0)
            printf("draws: %5i / %i\n", g_draws, thresh);
        if (g_draws == thresh) {
            printf("Profiling is enabled: exiting now\n");
            exit(0);
        }
    }
    
    // we're going to actually draw.
    
    bot_gtk_gl_drawing_area_set_context (self->gl_area);
    g_signal_emit(G_OBJECT(self), bot_viewer_signals[RENDER_BEGIN_SIGNAL], 0);
    render_scene (self);
    g_signal_emit(G_OBJECT(self), bot_viewer_signals[RENDER_END_SIGNAL], 0);
    bot_gtk_gl_drawing_area_swap_buffers (self->gl_area);
    
    // write a movie frame?
    if (self->movie_draw_pending) {
        assert(self->movie_gzf);

        glReadPixels (0, 0, self->movie_width, self->movie_height, GL_RGB, GL_UNSIGNED_BYTE, self->movie_buffer); 
        
        gzprintf(self->movie_gzf, "P6 %d %d %d\n", self->movie_width, self->movie_height, 255);
        
        for (int h = self->movie_height - 1; h >= 0; h--) {
            int offset = self->movie_stride * h;
            gzwrite(self->movie_gzf, &self->movie_buffer[offset], self->movie_stride);
        }

        self->movie_draw_pending = 0;
        int64_t now = bot_timestamp_now();
        double dt;
        if (self->movie_frame_last_utime == 0)
            dt = 1.0 / self->movie_desired_fps;
        else
            dt = (now - self->movie_frame_last_utime)/1000000.0;
        double fps = 1.0 / dt;
        self->movie_frame_last_utime = now;
        double alpha = 0.8; // higher = lower-pass
        self->movie_actual_fps = alpha * self->movie_actual_fps + (1 - alpha) * fps;
        self->movie_frames++;

        printf("%20s %6d (%5.2f fps)\r", self->movie_path, self->movie_frames, self->movie_actual_fps);
        fflush(NULL);
    }

    return TRUE;
}

static GLint
mygluUnProject (double winx, double winy, double winz,
        const double model[16], const double proj[16], const int viewport[4],
        double * objx, double * objy, double * objz)
{
    double p[16], m[16];
    bot_matrix_transpose_4x4d (proj, p);
    bot_matrix_transpose_4x4d (model, m);
    double t[16];
    bot_matrix_multiply_4x4_4x4 (p, m, t);
    if (bot_matrix_inverse_4x4d (t, m) < 0)
        return GL_FALSE;
    if (viewport[2] == 0 || viewport[3] == 0)
        return GL_FALSE;
    double v[4] = {
        2 * (winx - viewport[0]) / viewport[2] - 1,
        2 * (winy - viewport[1]) / viewport[3] - 1,
        2 * winz - 1,
        1,
    };
    double v2[4];
    bot_matrix_vector_multiply_4x4_4d (m, v, v2);
    if (v2[3] == 0)
        return GL_FALSE;
    *objx = v2[0] / v2[3];
    *objy = v2[1] / v2[3];
    *objz = v2[2] / v2[3];
    return GL_TRUE;
}

static int
_window_coord_to_ray (double x, double y, double ray_start[3], double ray_dir[3])
{
    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    if (mygluUnProject (x, y, 0,
                      model_matrix, proj_matrix, viewport, 
                      &ray_start[0], &ray_start[1], &ray_start[2]) == GL_FALSE) 
        return -1;
    
    double ray_end[3];

    if (mygluUnProject (x, y, 1,
                      model_matrix, proj_matrix, viewport, 
                      &ray_end[0], &ray_end[1], &ray_end[2]) == GL_FALSE) 
        return -1;

    ray_dir[0] = ray_end[0] - ray_start[0];
    ray_dir[1] = ray_end[1] - ray_start[1];
    ray_dir[2] = ray_end[2] - ray_start[2];
            
    bot_vector_normalize_3d(ray_dir);

    return 0;
}

static gboolean 
on_button_press (GtkWidget *widget, GdkEventButton *event, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    bot_gtk_gl_drawing_area_set_context (self->gl_area);
    double ray_start[3];
    double ray_dir[3];
    _window_coord_to_ray (event->x, widget->allocation.height - event->y, ray_start, ray_dir);

    // find a new picker?
    double best_distance = HUGE;
    BotEventHandler *best_handler = NULL;

    if (self->picking_handler == NULL || self->picking_handler->picking==0) {
        for (unsigned int eidx = 0; eidx < self->event_handlers->len; eidx++) {
            BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
            
            if (handler->enabled && handler->pick_query) {
                double this_distance = handler->pick_query(self, handler, ray_start, ray_dir);
                if (this_distance < best_distance && this_distance >= 0) {
                    best_distance = this_distance;
                    best_handler = handler;
                }
            }
        }

        // notify the new handler
        if (best_handler)
            bot_viewer_request_pick(self, best_handler);
    }

    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && !self->picking_handler->picking)
        self->picking_handler = NULL;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->mouse_press) {
        consumed = self->picking_handler->mouse_press(self, self->picking_handler, ray_start, ray_dir, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; 
            !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->mouse_press)
            if (handler->mouse_press(self, handler, ray_start, ray_dir, event))
                break;
    }

    return TRUE;
}

static gboolean
on_button_release (GtkWidget *widget, GdkEventButton *event, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    bot_gtk_gl_drawing_area_set_context (self->gl_area);
    double ray_start[3];
    double ray_dir[3];
    _window_coord_to_ray (event->x, widget->allocation.height - event->y, ray_start, ray_dir);

    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && !self->picking_handler->picking)
        self->picking_handler = NULL;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->mouse_release) {
        consumed = self->picking_handler->mouse_release(self, self->picking_handler, ray_start, ray_dir, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->mouse_release)
            if (handler->mouse_release(self, handler, ray_start, ray_dir, event))
                break;
    }

    return TRUE;
}

static gboolean 
on_motion_notify (GtkWidget *widget, GdkEventMotion *event, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    bot_gtk_gl_drawing_area_set_context (self->gl_area);
    double ray_start[3];
    double ray_dir[3];
    _window_coord_to_ray (event->x, widget->allocation.height - event->y, ray_start, ray_dir);
    
    // is anyone hovering?
    if (self->picking_handler == NULL || !self->picking_handler->picking) {

        // find a new hover?
        double best_distance = HUGE;
        BotEventHandler *best_handler = NULL;
        
        for (unsigned int eidx = 0; eidx < self->event_handlers->len; eidx++) {
            BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
            
            handler->hovering = 0;

            if (handler->enabled && handler->hover_query) {
                double this_distance = handler->hover_query(self, handler, ray_start, ray_dir);
                if (this_distance < best_distance && this_distance >= 0) {
                    best_distance = this_distance;
                    best_handler = handler;
                }
            }
        }

        // notify the new handler
        if (best_handler)
            best_handler->hovering = 1;

        bot_viewer_request_redraw(self);
    }

    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && !self->picking_handler->picking)
        self->picking_handler = NULL;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->mouse_motion) {
        consumed = self->picking_handler->mouse_motion(self, self->picking_handler, ray_start, ray_dir, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->mouse_motion)
            if (handler->mouse_motion(self, handler, ray_start, ray_dir, event))
                break;
    }

    return TRUE;
}

static gboolean 
on_scroll_notify (GtkWidget *widget, GdkEventScroll *event, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    bot_gtk_gl_drawing_area_set_context (self->gl_area);
    double ray_start[3];
    double ray_dir[3];
    _window_coord_to_ray (event->x, widget->allocation.height - event->y, ray_start, ray_dir);
    
    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && !self->picking_handler->picking)
        self->picking_handler = NULL;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->mouse_scroll) {
        consumed = self->picking_handler->mouse_scroll(self, self->picking_handler, ray_start, ray_dir, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->mouse_scroll)
            if (handler->mouse_scroll(self, handler, ray_start, ray_dir, event))
                break;
    }

    return TRUE;
}

static gint 
on_main_window_key_press_event (GtkWidget *widget, GdkEventKey *event, 
                                void *user)
{
    BotViewer *self = (BotViewer*) user;

    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->picking &&
        self->picking_handler->key_press) {
        consumed = self->picking_handler->key_press(self, self->picking_handler, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->key_press) {
            consumed = handler->key_press(self, handler, event);
            if (consumed)
                break;
        }
    }

    return consumed;
}

static gint 
on_main_window_key_release_event (GtkWidget *widget, GdkEventKey *event, 
                                void *user)
{
    BotViewer *self = (BotViewer*) user;

    // give picking handler first dibs
    int consumed = 0;
    if (self->picking_handler && self->picking_handler->enabled && self->picking_handler->picking &&
        self->picking_handler->key_release) {
        consumed = self->picking_handler->key_release(self, self->picking_handler, event);
        update_status_bar(self);
    }

    // try all the other handlers in order of priority
    for (unsigned int eidx = 0; !consumed && eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *handler = g_ptr_array_index(self->event_handlers, eidx);
        
        if (handler != self->picking_handler && handler->enabled && handler->key_release) {
            consumed = handler->key_release(self, handler, event);
            if (consumed)
                break;
        }
    }

    return consumed;
}

static int 
_pixel_convert_8u_bgra_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j*3 + 0] = srow[j*4 + 2];
            drow[j*3 + 1] = srow[j*4 + 1];
            drow[j*3 + 2] = srow[j*4 + 0];
        }
    }
    return 0;
}

static gboolean
take_screenshot (void *user_data, char *fname)
{
 
    BotViewer *self = (BotViewer*) user_data;
    BotViewHandler *vhandler = self->view_handler;

    int w = GTK_WIDGET (self->gl_area)->allocation.width;
    int h = GTK_WIDGET (self->gl_area)->allocation.height;
    uint8_t *bgra = (uint8_t*)malloc (w*h*4);
    uint8_t *rgb = (uint8_t*)malloc (w*h*3);
    glReadPixels (0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, bgra); 
    FILE *fp = fopen (fname, "wb");
    if (! fp) {
        perror ("fopen");
        err ("couldn't take screenshot\n");
    return FALSE;
    } else {
        _pixel_convert_8u_bgra_to_8u_rgb (rgb, w*3, w, h, bgra, w*4);
        bot_ppm_write_bottom_up (fp, rgb, w, h, w*3);
        fclose (fp);

        dbg ("screenshot saved to %s\n", fname);
        bot_viewer_set_status_bar_message (self, "screenshot saved to %s", fname);
    }
    free (bgra);
    free (rgb);
    return TRUE;
}

static void
on_screenshot_clicked (GtkToolButton *ssbt, void *user_data)
{
    char * fname = bot_fileutils_get_unique_filename (NULL, "viewer", 1, "ppm");
    take_screenshot (user_data, fname);
    free (fname);
}


/*
static gboolean
on_screenshot_timer (BotViewer * self)
{
    return take_screenshot (self, "viewer.ppm");
}
*/

static void
on_record_toggled (GtkToggleToolButton *tb, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;
    int record = gtk_toggle_tool_button_get_active (tb);

    if (record && ! self->is_recording) {
        bot_viewer_start_recording (self);
    } else {
        bot_viewer_stop_recording (self);
    }
}


static void
on_renderer_enabled_toggled (GtkCheckMenuItem *cmi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;
    BotRenderer *r = (BotRenderer*) g_object_get_data (G_OBJECT (cmi), 
                                                 "BotViewer:plugin");
    r->enabled = gtk_check_menu_item_get_active (cmi);

    if (r->widget) {
        GtkWidget *frame = g_object_get_data (G_OBJECT (r->widget), 
                                              "BotViewer:frame");
        if (frame) {
            if (r->enabled) {
                gtk_widget_show (frame);
            } else {
                gtk_widget_hide (frame);
            }
        }
    }
    bot_viewer_request_redraw (self);
}

static void
on_event_handler_enabled_toggled (GtkCheckMenuItem *cmi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;
    BotEventHandler *ehandler = (BotEventHandler*) g_object_get_data (G_OBJECT(cmi), "BotViewer:plugin");

    ehandler->enabled = gtk_check_menu_item_get_active (cmi);

    bot_viewer_request_redraw (self);
}

static void
on_select_all_event_handlers_activate (GtkMenuItem *mi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    for (unsigned int eidx = 0; eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *ehandler = g_ptr_array_index(self->event_handlers, eidx);
        ehandler->enabled = 1;
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(ehandler->cmi), ehandler->enabled);
    }
}

static void
on_select_no_event_handlers_activate (GtkMenuItem *mi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    for (unsigned int eidx = 0; eidx < self->event_handlers->len; eidx++) {
        BotEventHandler *ehandler = g_ptr_array_index(self->event_handlers, eidx);
        ehandler->enabled = 0;
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(ehandler->cmi), ehandler->enabled);
    }
}

static void
on_select_all_renderers_activate (GtkMenuItem *mi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    for (unsigned int ridx = 0; ridx < self->renderers->len; ridx++) {
        BotRenderer *renderer = g_ptr_array_index(self->renderers, ridx);

        renderer->enabled = 1;
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);
    }
}

static void
on_select_no_renderers_activate (GtkMenuItem *mi, void *user_data)
{
    BotViewer *self = (BotViewer*) user_data;

    for (unsigned int ridx = 0; ridx < self->renderers->len; ridx++) {
        BotRenderer *renderer = g_ptr_array_index(self->renderers, ridx);

        renderer->enabled = 0;
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);
    }
}

static void
on_renderer_widget_expander_notify (GObject *object, GParamSpec *param_spec, 
        void *user_data)
{
    GtkExpander *expander = GTK_EXPANDER (object);
    BotRenderer *renderer = 
        (BotRenderer*) g_object_get_data (G_OBJECT (expander), "BotViewer:plugin");

    renderer->expanded = gtk_expander_get_expanded (expander);
    if (renderer->expanded)
        gtk_widget_show (renderer->widget);
    else
        gtk_widget_hide (renderer->widget);
}

static void on_select_bookmark_save_view(GtkMenuItem *mi, void *user)
{
  bookmark_persp_t *views = user;
  BotViewer* viewer = views->viewer;
  BotViewHandler *vhandler = viewer->view_handler;

  BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

  // figure out projection mode  
  views->projection_mode = vhandler->get_projection_mode(vhandler);

  views->saved = 1;
  vhandler->get_eye_look(vhandler, views->eye, views->lookat, views->up);

  bot_viewer_set_status_bar_message(viewer, "Saved viewpoint %d", views->index);
}



static void on_select_bookmark_load_view(GtkMenuItem *mi, void *user)
{
 
  bookmark_persp_t *views = user;
  BotViewer* viewer = views->viewer;
  BotViewHandler *vhandler = viewer->view_handler;

  //HARDCODE: duration of smooth transition
  int duration_ms = 500;  //milliseconds

  if(views->saved) {
    // get projection type
    if (views->projection_mode == BOT_VIEW_ORTHOGRAPHIC)
      vhandler->set_camera_orthographic(vhandler);
    if (views->projection_mode == BOT_VIEW_PERSPECTIVE)
      vhandler->set_camera_perspective(vhandler, vhandler->get_perspective_fov(vhandler));

    vhandler->set_look_at_smooth(vhandler, views->eye, views->lookat, views->up, duration_ms);
    bot_viewer_request_redraw(viewer);

    bot_viewer_set_status_bar_message(viewer, "Loaded viewpoint %d", views->index);
  }
}

static void on_select_perspective_item(GtkMenuItem *mi, void *user)
{
  BotViewer* viewer = (BotViewer*) user;
  BotViewHandler *vhandler = viewer->view_handler;
  if (vhandler) {
    vhandler->set_camera_perspective(vhandler, 60);
  }
}

static void on_select_orthographic_item(GtkMenuItem *mi, void *user)
{
  BotViewer* viewer = (BotViewer*) user;
  BotViewHandler *vhandler = viewer->view_handler;
  if (vhandler) {
    vhandler->set_camera_orthographic(vhandler);
  }
}



// ================

static gint renderer_name_compare_function(gconstpointer _a, gconstpointer _b)
{
    BotRenderer *ra = *(BotRenderer**) _a;
    BotRenderer *rb = *(BotRenderer**) _b;

    return strcmp(ra->name, rb->name);
}

// sort in decending order of priority
static gint sort_renderers_priority_decreasing(gconstpointer _a, gconstpointer _b)
{
    BotRenderer *a = *(BotRenderer**) _a;
    BotRenderer *b = *(BotRenderer**) _b;

    return (a->priority < b->priority) ? 1 : -1;
}

static int _g_ptr_array_find_index(GPtrArray *a, gconstpointer v)
{
    for (unsigned int i = 0; i < a->len; i++) {
        if (g_ptr_array_index(a, i) == v)
            return i;
    }
    return -1;
}



void bot_viewer_add_renderer_on_side (BotViewer *self, BotRenderer *renderer, int priority, int which_side)
{
    renderer->priority = priority;
    renderer->cmi      = gtk_check_menu_item_new_with_label (renderer->name);
    gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (renderer->cmi), renderer->enabled);

    g_object_set_data (G_OBJECT (renderer->cmi), "BotViewer:plugin", renderer);
    g_signal_connect (G_OBJECT (renderer->cmi), "toggled", 
                      G_CALLBACK (on_renderer_enabled_toggled), self);
    
    g_ptr_array_add(self->renderers, renderer);
    g_ptr_array_add(self->renderers_sorted, renderer);
    g_ptr_array_sort(self->renderers_sorted, renderer_name_compare_function);

    // What position in the sorted array is this item?
    unsigned int menu_idx = _g_ptr_array_find_index(self->renderers_sorted, renderer);

    // add the menu item, accounting for the tear-off tab at index 0
    gtk_menu_shell_insert (GTK_MENU_SHELL (self->renderers_menu), renderer->cmi, menu_idx + 1);
    gtk_widget_show (renderer->cmi);

    // create a control widget
    if (renderer->widget) {
        g_ptr_array_add(self->renderers_sorted_with_controls, renderer);
        g_ptr_array_sort(self->renderers_sorted_with_controls, renderer_name_compare_function);

        unsigned int control_idx = _g_ptr_array_find_index(self->renderers_sorted_with_controls, renderer);

        renderer->expander = gtk_expander_new (renderer->name);
        gtk_expander_set_expanded (GTK_EXPANDER (renderer->expander), TRUE);
        renderer->control_frame = gtk_frame_new (NULL);
        
        if (renderer->enabled) {
            gtk_widget_show (renderer->control_frame);
        } else {
            gtk_widget_hide (renderer->control_frame);
        }
        
        gtk_frame_set_label_widget (GTK_FRAME (renderer->control_frame), renderer->expander);
        gtk_container_add (GTK_CONTAINER (renderer->control_frame), renderer->widget);
        
        if (which_side==0){
           gtk_box_pack_start (GTK_BOX (self->controls_box_left), renderer->control_frame,
                            FALSE, TRUE, 0);
           gtk_box_reorder_child (GTK_BOX (self->controls_box_left), renderer->control_frame, control_idx);
        }else{ // new dec 2012:
           // Add your controls to the left hand side of the viewer - instead of the default:
           gtk_box_pack_start (GTK_BOX (self->controls_box), renderer->control_frame,
                            FALSE, TRUE, 0);
           gtk_box_reorder_child (GTK_BOX (self->controls_box), renderer->control_frame, control_idx);
        }


        gtk_widget_show (renderer->expander);
        gtk_widget_show (renderer->widget);
        
        g_signal_connect (G_OBJECT (renderer->expander), "notify::expanded",
                          G_CALLBACK (on_renderer_widget_expander_notify), self);
        g_object_set_data (G_OBJECT (renderer->expander), 
                           "BotViewer:plugin", renderer);
        g_object_set_data (G_OBJECT (renderer->widget), 
                           "BotViewer:expander", renderer->expander);
        g_object_set_data (G_OBJECT (renderer->widget), 
                           "BotViewer:frame", renderer->control_frame);
    }

    g_ptr_array_sort(self->renderers, sort_renderers_priority_decreasing);
}


void bot_viewer_add_renderer (BotViewer *self, BotRenderer *renderer, int priority){
    return bot_viewer_add_renderer_on_side (self, renderer, priority, 1);
}




/*
static void
destroy_plugins (BotViewer *self)
{
    for (unsigned int ridx = 0; ridx < g_ptr_array_size(self->renderers); ridx++) {
        BotRenderer *renderer = g_ptr_array_index(self->renderers, ridx);

        if (renderer->destroy)
            renderer->destroy(renderer);
    }

    g_ptr_array_free(self->plugins, TRUE);
}
*/

static void 
make_menus(BotViewer *viewer, GtkWidget *parent)
{
    BotViewerPriv* priv = BOT_VIEWER_GET_PRIVATE(viewer);

    GtkWidget *menubar = gtk_menu_bar_new();
    viewer->menu_bar = menubar;

    gtk_box_pack_start(GTK_BOX(parent), menubar, FALSE, FALSE, 0);
    
    GtkWidget *file_menuitem = gtk_menu_item_new_with_mnemonic("_File");
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), file_menuitem);

    viewer->file_menu = gtk_menu_new();
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(file_menuitem), viewer->file_menu);

    /////////////////////
    GtkWidget *renderers_menuitem = gtk_menu_item_new_with_mnemonic("_BotRenderers");
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), renderers_menuitem);

    viewer->renderers_menu = gtk_menu_new();
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(renderers_menuitem), viewer->renderers_menu);

    if (1) {
        // tearoff
        GtkWidget *tearoff = gtk_tearoff_menu_item_new();
        gtk_menu_append (GTK_MENU(viewer->renderers_menu), tearoff);
        gtk_widget_show (tearoff);

        // separator
        GtkWidget *sep = gtk_separator_menu_item_new ();
        gtk_menu_append (GTK_MENU(viewer->renderers_menu), sep);
        gtk_widget_show (sep);

        // select all item
        GtkWidget *select_all_mi = gtk_menu_item_new_with_mnemonic ("Select _All");
        gtk_menu_append (GTK_MENU(viewer->renderers_menu), select_all_mi);
        gtk_widget_show (select_all_mi);
        
        // remove all item
        GtkWidget *select_none_mi = 
            gtk_menu_item_new_with_mnemonic ("Select _None");
        gtk_menu_append (GTK_MENU(viewer->renderers_menu), select_none_mi);
        gtk_widget_show (select_none_mi);
        g_signal_connect (G_OBJECT (select_all_mi), "activate", 
                          G_CALLBACK (on_select_all_renderers_activate), viewer);
        g_signal_connect (G_OBJECT (select_none_mi), "activate",
                          G_CALLBACK (on_select_no_renderers_activate), viewer);
    }

    /////////////////////
    GtkWidget *event_handlers_menuitem = gtk_menu_item_new_with_mnemonic("_Input");
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), event_handlers_menuitem);

    viewer->event_handlers_menu = gtk_menu_new();
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(event_handlers_menuitem), viewer->event_handlers_menu);

    if (1) {
        // tearoff
        GtkWidget *tearoff = gtk_tearoff_menu_item_new();
        gtk_menu_append (GTK_MENU(viewer->event_handlers_menu), tearoff);
        gtk_widget_show (tearoff);

        // separator
        GtkWidget *sep = gtk_separator_menu_item_new ();
        gtk_menu_append (GTK_MENU(viewer->event_handlers_menu), sep);
        gtk_widget_show (sep);

        // select all item
        GtkWidget *select_all_mi = gtk_menu_item_new_with_mnemonic ("Select _All");
        gtk_menu_append (GTK_MENU(viewer->event_handlers_menu), select_all_mi);
        gtk_widget_show (select_all_mi);
        
        // remove all item
        GtkWidget *select_none_mi = 
            gtk_menu_item_new_with_mnemonic ("Select _None");
        gtk_menu_append (GTK_MENU(viewer->event_handlers_menu), select_none_mi);
        gtk_widget_show (select_none_mi);
        g_signal_connect (G_OBJECT (select_all_mi), "activate", 
                          G_CALLBACK (on_select_all_event_handlers_activate), viewer);
        g_signal_connect (G_OBJECT (select_none_mi), "activate",
                          G_CALLBACK (on_select_no_event_handlers_activate), viewer);
    }


    //add perspective and orthographic controls...
    GtkWidget *view_menuitem = gtk_menu_item_new_with_mnemonic("_View");
    gtk_menu_bar_append(GTK_MENU_BAR(viewer->menu_bar), view_menuitem);

    viewer->view_menu = gtk_menu_new();
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(view_menuitem), viewer->view_menu);

    GSList *view_list = NULL;
    GtkWidget *perspective_item = gtk_radio_menu_item_new_with_label(view_list, "Perspective");
    view_list = gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(perspective_item));
    gtk_menu_append(GTK_MENU(viewer->view_menu), perspective_item);
    g_signal_connect(G_OBJECT(perspective_item), "activate", G_CALLBACK(on_select_perspective_item), viewer);

    GtkWidget *orthographic_item = gtk_radio_menu_item_new_with_label(view_list, "Orthographic");
    view_list = gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(orthographic_item));
    gtk_menu_append(GTK_MENU(viewer->view_menu), orthographic_item);
    g_signal_connect(G_OBJECT(orthographic_item), "activate", G_CALLBACK(on_select_orthographic_item), viewer);

    // create labels for loading and saving bookmarked views
    guint keys[] = {
        GDK_F1,
        GDK_F2,
        GDK_F3,
        GDK_F4,
        GDK_F5,
        GDK_F6,
    };

    for(int i=0; i<priv->num_bookmarks; i++) {
      char* smi_label = g_strdup_printf("Viewpoint %d", i + 1);
      GtkWidget* submenu_item = gtk_menu_item_new_with_label(smi_label);
      gtk_menu_append(GTK_MENU(viewer->view_menu), submenu_item);
      g_free(smi_label);

      GtkWidget* submenu = gtk_menu_new();
      gtk_menu_item_set_submenu(GTK_MENU_ITEM(submenu_item), submenu);
      GtkWidget* save_smi = gtk_menu_item_new_with_label("Save");
      GtkWidget* load_smi = gtk_menu_item_new_with_label("Load");
      gtk_menu_append(GTK_MENU(submenu), save_smi);
      gtk_menu_append(GTK_MENU(submenu), load_smi);
      g_signal_connect(G_OBJECT(save_smi), "activate", 
              G_CALLBACK(on_select_bookmark_save_view), &priv->bookmarks[i]);
      g_signal_connect(G_OBJECT(load_smi), "activate", 
              G_CALLBACK(on_select_bookmark_load_view), &priv->bookmarks[i]);

      gtk_widget_add_accelerator(save_smi, "activate", priv->key_accel_group,
              keys[i], GDK_CONTROL_MASK, GTK_ACCEL_VISIBLE);
      gtk_widget_add_accelerator(load_smi, "activate", priv->key_accel_group,
              keys[i], 0, GTK_ACCEL_VISIBLE);
    }

    gtk_widget_show_all(view_menuitem);


}

static void 
make_toolbar(BotViewer *viewer, GtkWidget *parent)
{
    GtkWidget *toolbar = gtk_toolbar_new();
    gtk_box_pack_start(GTK_BOX(parent), toolbar, FALSE, FALSE, 0);

    // add a recording button to the toolbar
    viewer->record_button = GTK_WIDGET(
        gtk_toggle_tool_button_new_from_stock (GTK_STOCK_MEDIA_RECORD));
    gtk_tool_item_set_is_important (GTK_TOOL_ITEM (viewer->record_button), 
            TRUE);
    gtk_tool_item_set_tooltip (GTK_TOOL_ITEM (viewer->record_button), 
            viewer->tips,
            "Record an AVI of the viewport, saved in the current directory",
            NULL);

    gtk_toolbar_insert (GTK_TOOLBAR (toolbar), 
            GTK_TOOL_ITEM (viewer->record_button), 0);
    gtk_widget_show (viewer->record_button);
    g_signal_connect (G_OBJECT (viewer->record_button), "toggled", 
            G_CALLBACK (on_record_toggled), viewer);

    // screenshot button
    GtkToolItem *ssbt = gtk_tool_button_new_from_stock (GTK_STOCK_FLOPPY);
    gtk_tool_button_set_label (GTK_TOOL_BUTTON (ssbt), "Screenshot");
    gtk_tool_item_set_is_important (GTK_TOOL_ITEM (ssbt), TRUE);
    gtk_tool_item_set_tooltip (GTK_TOOL_ITEM (ssbt), viewer->tips,
            "Save a PPM screenshot of the viewport to the current directory",
            NULL);
    gtk_toolbar_insert (GTK_TOOLBAR (toolbar), ssbt, 1);
    gtk_widget_show (GTK_WIDGET (ssbt));
    g_signal_connect (G_OBJECT (ssbt), "clicked", 
            G_CALLBACK (on_screenshot_clicked), viewer);

    // quit button
    GtkToolItem *quitbt = gtk_tool_button_new_from_stock (GTK_STOCK_QUIT);
    gtk_tool_item_set_tooltip (GTK_TOOL_ITEM (quitbt), viewer->tips,
            "Quit", NULL);
    gtk_toolbar_insert (GTK_TOOLBAR (toolbar), quitbt, 2);
    gtk_widget_show (GTK_WIDGET (quitbt));
    g_signal_connect (G_OBJECT (quitbt), "clicked", gtk_main_quit, NULL);

    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_widget_show (GTK_WIDGET (sep));
    gtk_toolbar_insert (GTK_TOOLBAR (toolbar), sep, 3);

    GtkWidget * hbox = gtk_hbox_new (FALSE, 5);
    GtkWidget * label = gtk_label_new ("Record FPS");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE, 0);
    viewer->fps_spin = gtk_spin_button_new_with_range (0.1, 120.0, 1.0);
    gtk_spin_button_set_digits (GTK_SPIN_BUTTON (viewer->fps_spin), 1);
    gtk_spin_button_set_value (GTK_SPIN_BUTTON (viewer->fps_spin), 30);
    gtk_box_pack_start (GTK_BOX (hbox), viewer->fps_spin, FALSE, FALSE, 0);

    GtkToolItem * toolitem = gtk_tool_item_new ();
    gtk_container_add (GTK_CONTAINER (toolitem), hbox);
    gtk_widget_show_all (GTK_WIDGET (toolitem));
    gtk_toolbar_insert (GTK_TOOLBAR (toolbar), toolitem, 4);
    viewer->toolbar = GTK_TOOLBAR (toolbar);
}

static void bot_viewer_class_init (BotViewerClass *klass);

G_DEFINE_TYPE (BotViewer, bot_viewer, G_TYPE_OBJECT);

static void
bot_viewer_finalize (GObject *obj)
{
    BotViewer *self = BOT_VIEWER (obj);
    dbg ("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__);

    BotViewerPriv* priv = BOT_VIEWER_GET_PRIVATE(self);
    free(priv->bookmarks);

    gtk_widget_destroy (GTK_WIDGET (self->window));

    G_OBJECT_CLASS (bot_viewer_parent_class)->finalize(obj);
}

static void
bot_viewer_class_init (BotViewerClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->finalize = bot_viewer_finalize;
    /**
     * BotViewer::load-preferences
     * @keyfile: a GKeyFile* to be used for loading saved preferences
     *
     * The load-preferences signal is emitted when bot_viewer_load_preferences is
     * called on the viewer.  Objects can subscribe to this signal to load data
     * from the GKeyFile.
     */
    bot_viewer_signals[LOAD_PREFERENCES_SIGNAL] = g_signal_new("load-preferences",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST,
            0, NULL, NULL, g_cclosure_marshal_VOID__POINTER,
            G_TYPE_NONE, 1, G_TYPE_POINTER);
    /**
     * BotViewer::save-preferences
     * @keyfile: a GKeyFile* to be used for saving preferences
     *
     * The save-preferences signal is emitted when bot_viewer_save_preferences is
     * called on the viewer.  Objects can subscribe to this signal to save data
     * to the GKeyFile.
     */
    bot_viewer_signals[SAVE_PREFERENCES_SIGNAL] = g_signal_new("save-preferences",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST,
            0, NULL, NULL, g_cclosure_marshal_VOID__POINTER,
            G_TYPE_NONE, 1, G_TYPE_POINTER);
    /**
     * BotViewer::render-begin
     *
     * The render-begin signal is emitted during every render cycle, immediately
     * before the draw method is invoked on the first enabled renderer.  It is
     * emitted after the view handler update method is called.
     */
    bot_viewer_signals[RENDER_BEGIN_SIGNAL] = g_signal_new("render-begin",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST,
            0, NULL, NULL, g_cclosure_marshal_VOID__VOID,
            G_TYPE_NONE, 0);
    /**
     * BotViewer::render-end
     *
     * The render-end signal is emitted during every render cycle, after all
     * enabled renderers have been invoked.
     */
    bot_viewer_signals[RENDER_END_SIGNAL] = g_signal_new("render-end",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST,
            0, NULL, NULL, g_cclosure_marshal_VOID__VOID,
            G_TYPE_NONE, 0);

    g_type_class_add_private (gobject_class, sizeof (BotViewerPriv));
}

static void
bot_viewer_init (BotViewer *viewer)
{
    BotViewerPriv* priv = BOT_VIEWER_GET_PRIVATE(viewer);

    viewer->renderers = g_ptr_array_new();
    viewer->renderers_sorted = g_ptr_array_new();
    viewer->renderers_sorted_with_controls = g_ptr_array_new();
    viewer->event_handlers = g_ptr_array_new();
    viewer->event_handlers_sorted = g_ptr_array_new();

    // default to a white background
    viewer->backgroundColor[0] = 1;
    viewer->backgroundColor[1] = 1;
    viewer->backgroundColor[2] = 1;
    viewer->backgroundColor[3] = 1;

    // allocate memory for saved bookmarks
    priv->num_bookmarks = 6;
    priv->bookmarks = (bookmark_persp_t*) calloc(priv->num_bookmarks, 
            sizeof(bookmark_persp_t));
    for(int bk_ind=0; bk_ind<priv->num_bookmarks; bk_ind++) {
        priv->bookmarks[bk_ind].viewer = viewer;
        priv->bookmarks[bk_ind].index = bk_ind + 1;
        //default views
        //for (int i=0; i<3; i++) {
        //  priv->bookmarks[bk_ind].eye[i] = *(defaultbmview_eye+i);
        //  priv->bookmarks[bk_ind].lookat[i] = *(defaultbmview_lookat+i);
        //  priv->bookmarks[bk_ind].up[i] = *(defaultbmview_up+i);
        // }
    }

    viewer->prettier_flag = (getenv("BOT_VIEWER_PRETTIER") != NULL && 
                             atoi(getenv("BOT_VIEWER_PRETTIER"))>0);;
    printf("BOT_VIEWER_PRETTIER: %d\n", viewer->prettier_flag);

    // viewer main window
    viewer->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(viewer->window), "BotViewer");
    gtk_window_set_resizable(GTK_WINDOW(viewer->window), TRUE);
    gtk_window_set_default_size(GTK_WINDOW(viewer->window), 1000, 675);

    GtkWidget *vbox = gtk_vbox_new(FALSE, 0);
    gtk_container_add(GTK_CONTAINER(viewer->window), vbox);

    // keyboard accelerators for viewer
    priv->key_accel_group = gtk_accel_group_new();
    gtk_window_add_accel_group(GTK_WINDOW(viewer->window), priv->key_accel_group);

    make_menus(viewer, vbox);

    viewer->tips = gtk_tooltips_new ();
    make_toolbar(viewer, vbox);

    // Embed the original viewing pane inside a second hpaned so as to add controls on the right:
    // GTK pan is then split like this:  [   left ctrl box        |[    gl pan  |right ctrl box]]
    GtkWidget *hpaned_main = gtk_hpaned_new();

    // Newer control box on the left:
    GtkWidget *controls_align1 = gtk_alignment_new(.5, .5, 1, 1);
    gtk_paned_pack1(GTK_PANED(hpaned_main), controls_align1, FALSE, TRUE);

    GtkWidget *controls_scroll1 = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(controls_align1), controls_scroll1);

    GtkWidget *controls_view1 = gtk_viewport_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(controls_scroll1), controls_view1);

    viewer->controls_box_left = gtk_vbox_new(FALSE, 0);
    gtk_container_add(GTK_CONTAINER(controls_view1), viewer->controls_box_left);

    // Original hpan on the right:
    GtkWidget *hpaned_right = gtk_hpaned_new();
    gtk_paned_pack2(GTK_PANED(hpaned_main), hpaned_right, FALSE, TRUE);

    // within original put GL pan on left:
    GtkWidget *gl_box = gtk_event_box_new();
    gtk_paned_pack1(GTK_PANED(hpaned_right), gl_box, TRUE, TRUE);

    // and older controls on the right:
    GtkWidget *controls_align = gtk_alignment_new(.5, .5, 1, 1);
    gtk_paned_pack2(GTK_PANED(hpaned_right), controls_align, FALSE, TRUE);


    GtkWidget *controls_scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(controls_align), controls_scroll);

    GtkWidget *controls_view = gtk_viewport_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(controls_scroll), controls_view);

    viewer->controls_box = gtk_vbox_new(FALSE, 0);
    gtk_container_add(GTK_CONTAINER(controls_view), viewer->controls_box);


    gtk_paned_set_position(GTK_PANED(hpaned_main), 200); // initial width of bar
    gtk_paned_set_position(GTK_PANED(hpaned_right), 460); // initial width of bar
    gtk_box_pack_start(GTK_BOX(vbox), hpaned_main, TRUE, TRUE, 0);



    viewer->status_bar = gtk_statusbar_new();
    gtk_box_pack_start(GTK_BOX(vbox), viewer->status_bar, FALSE, FALSE, 0);

    bot_viewer_set_status_bar_message(viewer, "Ready");

    // create the aspect area to maintain a 1:1 aspect ratio
    viewer->gl_area = BOT_GTK_GL_DRAWING_AREA (bot_gtk_gl_drawing_area_new (FALSE));
    gtk_widget_set_events (GTK_WIDGET (viewer->gl_area), 
            GDK_LEAVE_NOTIFY_MASK |
            GDK_BUTTON_PRESS_MASK | 
            GDK_BUTTON_RELEASE_MASK | 
            GDK_POINTER_MOTION_MASK);

    gtk_container_add (GTK_CONTAINER (gl_box), GTK_WIDGET (viewer->gl_area));

    gtk_widget_show (GTK_WIDGET (viewer->gl_area));

    g_signal_connect (G_OBJECT (viewer->gl_area), "configure-event",
            G_CALLBACK (on_gl_configure), viewer);
    g_signal_connect (G_OBJECT (viewer->gl_area), "expose-event",
            G_CALLBACK (on_gl_expose), viewer);
    g_signal_connect (G_OBJECT (viewer->gl_area), "button-press-event",
            G_CALLBACK (on_button_press), viewer);
    g_signal_connect (G_OBJECT (viewer->gl_area), "button-release-event",
            G_CALLBACK (on_button_release), viewer);
    g_signal_connect (G_OBJECT (viewer->gl_area), "motion-notify-event",
            G_CALLBACK (on_motion_notify), viewer);
    g_signal_connect (G_OBJECT (viewer->gl_area), "scroll-event",
            G_CALLBACK (on_scroll_notify), viewer);

    g_signal_connect (G_OBJECT (viewer->window), "key_press_event",
		      G_CALLBACK (on_main_window_key_press_event), viewer);
    g_signal_connect (G_OBJECT (viewer->window), "key_release_event",
		      G_CALLBACK (on_main_window_key_release_event), viewer);

    g_signal_connect (G_OBJECT (viewer->window), "delete_event",
		      G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect (G_OBJECT (viewer->window), "delete_event",
            G_CALLBACK (gtk_widget_hide_on_delete), NULL);

    g_signal_connect (G_OBJECT (viewer->window), "destroy_event",
		      G_CALLBACK(gtk_main_quit), NULL);

    ////////////////////////////////////////////////////////////////////
    // Create plugins menu


    // plugins will be inserted here as add_plugin is called
 
    gtk_widget_show_all(viewer->window);

    BotDefaultViewHandler *dvh = bot_default_view_handler_new(viewer);
    viewer->default_view_handler = &dvh->vhandler;

    bot_viewer_request_redraw(viewer);
}

BotViewer *bot_viewer_new (const char *window_title)
{
    BotViewer *viewer = BOT_VIEWER (g_object_new (TYPE_BOT_VIEWER, NULL));
    if (viewer!=NULL)
      bot_viewer_set_window_title (viewer, window_title);
    return viewer;
}

void bot_viewer_unref(BotViewer *viewer)
{
    g_object_unref (viewer);
}

void bot_viewer_set_window_title (BotViewer *viewer, const char *window_title)
{
    gtk_window_set_title(GTK_WINDOW(viewer->window), window_title);
}

void bot_viewer_set_view_handler(BotViewer *viewer, BotViewHandler *vhandler)
{
    if (vhandler) {
        viewer->view_handler = vhandler;
    } else {
        viewer->view_handler = viewer->default_view_handler;
    }
}

// sort in decending order of priority
static gint sort_ehandler_priority_decreasing(gconstpointer _a, gconstpointer _b)
{
    BotEventHandler *a = *(BotEventHandler**) _a;
    BotEventHandler *b = *(BotEventHandler**) _b;

    return (a->priority < b->priority) ? 1 : -1;
}

// sort in decending order of priority
static gint sort_ehandler_alphabetical(gconstpointer _a, gconstpointer _b)
{
    BotEventHandler *a = *(BotEventHandler**) _a;
    BotEventHandler *b = *(BotEventHandler**) _b;

    return strcmp(a->name, b->name);
}

void bot_viewer_add_event_handler(BotViewer *viewer, BotEventHandler *ehandler, int priority)
{
    g_ptr_array_add(viewer->event_handlers, ehandler);
    bot_viewer_event_handler_set_priority(viewer, ehandler, priority);

    g_ptr_array_add(viewer->event_handlers_sorted, ehandler);
    g_ptr_array_sort(viewer->event_handlers_sorted, sort_ehandler_alphabetical);

    ehandler->cmi = gtk_check_menu_item_new_with_label(ehandler->name);
    g_object_set_data (G_OBJECT (ehandler->cmi), "BotViewer:plugin", ehandler);
    gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (ehandler->cmi), ehandler->enabled);

    g_signal_connect(G_OBJECT(ehandler->cmi), "toggled",
                     G_CALLBACK(on_event_handler_enabled_toggled), viewer);

    // What position in the sorted array is this item?
    unsigned int menu_idx = _g_ptr_array_find_index(viewer->event_handlers_sorted, ehandler);

    // add the menu item, accounting for the tear-off tab at index 0
    gtk_menu_shell_insert (GTK_MENU_SHELL (viewer->event_handlers_menu), ehandler->cmi, menu_idx + 1);
    gtk_widget_show (ehandler->cmi);
}

void bot_viewer_event_handler_set_priority(BotViewer *viewer, BotEventHandler *ehandler, int priority)
{
    ehandler->priority = priority;
    g_ptr_array_sort(viewer->event_handlers, sort_ehandler_priority_decreasing);
}

int bot_viewer_picking(BotViewer *viewer)
{
    return viewer->picking_handler && viewer->picking_handler->picking;
}

int bot_viewer_request_pick(BotViewer *viewer, BotEventHandler *ehandler)
{
    if (bot_viewer_picking(viewer) && viewer->picking_handler != ehandler)
        return -1;

    ehandler->picking = 1;
    viewer->picking_handler = ehandler;
    update_status_bar(viewer);
    return 0;
}

void bot_viewer_add_stock_renderer_grid(BotViewer* viewer, int priority);

void 
bot_viewer_add_stock_renderer(BotViewer* viewer, int stock_renderer_id, int priority)
{
    switch(stock_renderer_id) {
        case BOT_VIEWER_STOCK_RENDERER_GRID:
            bot_viewer_add_stock_renderer_grid(viewer, priority);
            break;
        default:
            fprintf(stderr, "%s: Unrecognized stock renderer id %d.\n", __FUNCTION__, stock_renderer_id);
            break;
    }
}
