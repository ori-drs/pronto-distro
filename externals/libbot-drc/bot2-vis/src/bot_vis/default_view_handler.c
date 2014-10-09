#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gdk/gdkkeysyms.h>

#include <bot_core/bot_core.h>

#include "viewer.h"
#include "default_view_handler.h"

#define EYE_MIN_DIST 0.1
#define EYE_MAX_DIST 10000
#define EYE_ZOOM_INC (EYE_MAX_DIST - EYE_MIN_DIST) / 100
#define MAX_MOTION_MAGNITUDE 2.0
#define MIN_MOTION_MAGNITUDE 0.0001
BotProjectionMode projection_mode;

// column-major (opengl compatible) order.  We need this because we
// need to be able to recompute the model matrix without requiring the
// correct GL context to be current.
static void look_at_to_matrix(const double eye[3], const double lookat[3], const double _up[3], double M[16])
{
    double up[3];
    memcpy(up, _up, 3 * sizeof(double));
    bot_vector_normalize_3d(up);

    double f[3];
    bot_vector_subtract_3d(lookat, eye, f);
    bot_vector_normalize_3d(f);

    double s[3], u[3];
    bot_vector_cross_3d(f, up, s);
    bot_vector_cross_3d(s, f, u);

    // row-major
    double R[16];
    memset(R, 0, sizeof(R));
    R[0] = s[0];
    R[1] = s[1];
    R[2] = s[2];
    R[4] = u[0];
    R[5] = u[1];
    R[6] = u[2];
    R[8] = -f[0];
    R[9] = -f[1];
    R[10] = -f[2];
    R[15] = 1;
    
    double T[16];
    memset(T, 0, sizeof(T));
    T[0] = 1;
    T[3] = -eye[0];
    T[5] = 1;
    T[7] = -eye[1];
    T[10] = 1;
    T[11] = -eye[2];
    T[15] = 1;

    double MT[16];
    bot_matrix_multiply_4x4_4x4(R, T, MT);
    bot_matrix_transpose_4x4d(MT, M);
}

// must call this after any manual change to the eye/lookat/up fields
// in order to recompute the model_matrix!
static void look_at_changed(BotDefaultViewHandler *dvh)
{
    look_at_to_matrix(dvh->eye, dvh->lookat, dvh->up, dvh->model_matrix);
}

static void build_pan_jacobian(BotDefaultViewHandler *dvh, double dq[3], double up[3], double left[3], double A[4])
{
    //////////////////////////////////////////
    // How fast do the screen coordinates for dq change when we
    // translate dq in the up and left directions? We compute
    // derivatives numerically with step size eps.
    double eps = 0.00001;

    double PM_dq[3];
    gluProject(dq[0], dq[1], dq[2], 
               dvh->model_matrix, dvh->projection_matrix, dvh->viewport,
               &PM_dq[0], &PM_dq[1], &PM_dq[2]);

    double PM_dq_up[3];
    gluProject(dq[0] + up[0]*eps, dq[1] + up[1]*eps, dq[2] + up[2]*eps, 
               dvh->model_matrix, dvh->projection_matrix, dvh->viewport,
               &PM_dq_up[0], &PM_dq_up[1], &PM_dq_up[2]);

    double PM_dq_left[3];
    gluProject(dq[0] + left[0]*eps, dq[1] + left[1]*eps, dq[2] + left[2]*eps, 
               dvh->model_matrix, dvh->projection_matrix, dvh->viewport,
               &PM_dq_left[0], &PM_dq_left[1], &PM_dq_left[2]);

    //////////////////////////////////////////
    // Compute the Jacobian of screen coordinate changes with respect
    // to translation.
    A[0] = (PM_dq_up[0]   - PM_dq[0]) / eps;
    A[1] = (PM_dq_left[0] - PM_dq[0]) / eps;
    A[2] = (PM_dq_up[1]   - PM_dq[1]) / eps;
    A[3] = (PM_dq_left[1] - PM_dq[1]) / eps;
}

/** Given a coordinate in scene coordinates dq, modify the camera
	such that the screen projection of point dq moves (x,y) pixels.
**/
static void window_space_pan(BotDefaultViewHandler *dvh, double dq[], double x, double y, int preserveZ)
{
    double orig_eye[3], orig_lookat[3];
    memcpy(orig_eye, dvh->eye, 3 * sizeof(double));
    memcpy(orig_lookat, dvh->lookat, 3 * sizeof(double));

    y *= -1;   // y is upside down...

    double left[3], up[3];

    // compute left and up vectors
    if (!preserveZ) {
        // constraint translation such that the distance between the
        // eye and the lookat does not change; i.e., that the motion
        // is upwards and leftwards only.
        double look_vector[3];
        bot_vector_subtract_3d(dvh->lookat, dvh->eye, look_vector);
        bot_vector_normalize_3d(look_vector);
        bot_vector_cross_3d(dvh->up, look_vector, left);
        
        memcpy(up, dvh->up, 3 * sizeof(double));
    } else {
        // abuse the left and up vectors: change them to xhat, yhat... this ensures
        // that pan does not affect the camera Z height.
        left[0] = 1;
        left[1] = 0;
        left[2] = 0;
        up[0] = 0;
        up[1] = 1;
        up[2] = 0;
    }

    double A[4];
    double B[2] = { x, y };

    build_pan_jacobian(dvh, dq, up, left, A);
    double detOriginal = A[0]*A[3] - A[1]*A[2];

	// Ax = b, where x = how much we should move in the up and left directions.
    double Ainverse[4] = { 0, 0, 0, 0 };
    bot_matrix_inverse_2x2d(A, Ainverse);

    double sol[2];
    bot_matrix_vector_multiply_2x2_2d(Ainverse, B, sol);

    double motionup[3], motionleft[3], motion[3];
    memcpy(motionup, up, 3 * sizeof(double));
    bot_vector_scale_3d(motionup, sol[0]);

    memcpy(motionleft, left, 3 * sizeof(double));
    bot_vector_scale_3d(motionleft, sol[1]);
   
    bot_vector_add_3d(motionup, motionleft, motion);
    

    double magnitude = bot_vector_magnitude_3d(motion);
    double new_magnitude = fmax(fmin(magnitude,MAX_MOTION_MAGNITUDE),MIN_MOTION_MAGNITUDE);
    //bot_vector_normalize_3d(motion); // if magnitude is zero it will return nan's
    bot_vector_scale_3d(motion,new_magnitude/fmax(magnitude,MIN_MOTION_MAGNITUDE));
    
    double neweye[3], newlookat[3];
    bot_vector_subtract_3d(dvh->eye, motion, neweye);
    bot_vector_subtract_3d(dvh->lookat, motion, newlookat);


    memcpy(dvh->eye, neweye, sizeof(double)*3);
    memcpy(dvh->lookat, newlookat, sizeof(double)*3);

    look_at_changed(dvh);
    build_pan_jacobian(dvh, dq, up, left, A);
    // if the projection is getting sketchy, and it's getting worse,
    // then just reject it.  this is better than letting the
    // projection become singular! (This only happens with preserveZ?)
    double detNew = A[0]*A[3] - A[1]*A[2];
    //printf(" %15f %15f\n", detOriginal, detNew);
    //if (fabs(detNew) < 0.01 && fabs(detNew) <= fabs(detOriginal)) {
    if ((fabs(detNew) < 25 )||(fabs(detOriginal) < 25 )) { 
        memcpy(dvh->eye, orig_eye, 3 * sizeof(double));
        memcpy(dvh->lookat, orig_lookat, 3 * sizeof(double));
        look_at_changed(dvh);
        printf("skipping pan: %15f %15f\n", detOriginal, detNew);
    }
    
    
}

static int mouse_press   (BotViewer *viewer, BotEventHandler *ehandler, 
                           const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) ehandler->user;
    
    dvh->last_mouse_x = event->x;
    dvh->last_mouse_y = event->y;

    double dist = (ray_start[2] - dvh->lastpos[2])/ray_dir[2];
    for (int i = 0; i < 3; i++)
        dvh->manipulation_point[i] = ray_start[i] - dist * ray_dir[i];
    dvh->manipulation_point[3] = 1;

    return 1;
}

static int mouse_release (BotViewer *viewer, BotEventHandler *ehandler,
                           const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
//    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) ehandler->user;

    return 1;
}

static int mouse_motion  (BotViewer *viewer, BotEventHandler *ehandler, 
                          const double ray_start[3], const double ray_dir[3], const GdkEventMotion *event)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) ehandler->user;

    double dy = event->y - dvh->last_mouse_y;
    double dx = event->x - dvh->last_mouse_x;
    double look[3];
    bot_vector_subtract_3d (dvh->lookat, dvh->eye, look);

    if (event->state & GDK_BUTTON3_MASK) {
        double xy_len = bot_vector_magnitude_2d (look);
        double init_elevation = atan2 (look[2], xy_len);

        double left[3];
        bot_vector_cross_3d (dvh->up, look, left);
        bot_vector_normalize_3d (left);

        double delevation = -dy * 0.005;
        if (delevation + init_elevation < -M_PI/2) {
            delevation = -M_PI/2 - init_elevation;
        }
        if (delevation + init_elevation > M_PI/8) {
            delevation = M_PI/8 - init_elevation;
        }

        double q[4];
        bot_angle_axis_to_quat(-delevation, left, q);
        double newlook[3]; 
        memcpy (newlook, look, sizeof (newlook));
        bot_quat_rotate (q, newlook);

        double dazimuth = -dx * 0.01;
        double zaxis[] = { 0, 0, 1 };
        bot_angle_axis_to_quat (dazimuth, zaxis, q);
        bot_quat_rotate (q, newlook);
        bot_quat_rotate (q, left);

        bot_vector_subtract_3d (dvh->lookat, newlook, dvh->eye);
        bot_vector_cross_3d (newlook, left, dvh->up);
        look_at_changed(dvh);
    }
    else if (event->state & GDK_BUTTON2_MASK) {
        double init_eye_dist = bot_vector_magnitude_3d (look);

        double ded = pow (10, dy * 0.01);
        double eye_dist = init_eye_dist * ded;
        if (eye_dist > EYE_MAX_DIST) 
            eye_dist = EYE_MAX_DIST;
        else if (eye_dist < EYE_MIN_DIST) 
            eye_dist = EYE_MIN_DIST;
        
        double le[3];
        memcpy (le, look, sizeof (le));
        bot_vector_normalize_3d (le);
        bot_vector_scale_3d (le, eye_dist);

        bot_vector_subtract_3d (dvh->lookat, le, dvh->eye);
        look_at_changed(dvh);
    }
    else if (event->state & GDK_BUTTON1_MASK) {

        double dx = event->x - dvh->last_mouse_x;
        double dy = event->y - dvh->last_mouse_y;

        window_space_pan(dvh, dvh->manipulation_point, dx, dy, 1);
    }
    dvh->last_mouse_x = event->x;
    dvh->last_mouse_y = event->y;

    bot_viewer_request_redraw(viewer);
    return 1;
}

static void zoom_ratio(BotDefaultViewHandler *dvh, double ratio)
{
    double le[3];
    bot_vector_subtract_3d (dvh->eye, dvh->lookat, le);
    double eye_dist = bot_vector_magnitude_3d (le);

    eye_dist *= ratio;
    if (eye_dist < EYE_MIN_DIST) return;
    if (eye_dist > EYE_MAX_DIST) return;

    bot_vector_normalize_3d (le);
    bot_vector_scale_3d (le, eye_dist);

    bot_vector_add_3d (le, dvh->lookat, dvh->eye);

    look_at_changed(dvh);
}

static int mouse_scroll  (BotViewer *viewer, BotEventHandler *ehandler, 
                          const double ray_start[3], const double ray_dir[3], const GdkEventScroll *event)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) ehandler->user;
 
    // scrolling = zoom in and out
    if ( event->direction == GDK_SCROLL_UP) {
        zoom_ratio (dvh, 0.9);
    } else if (event->direction == GDK_SCROLL_DOWN) {
        zoom_ratio (dvh, 1.1);
    }
    
    // how far from XY plane?
    double dist = (ray_start[2] - dvh->lastpos[2]) / ray_dir[2];
    
    double dq[] = {ray_start[0] - ray_dir[0]*dist,
                   ray_start[1] - ray_dir[1]*dist,
                   ray_start[2] - ray_dir[2]*dist,
                   1 };
    
    double PM_dq[3];
    gluProject(dq[0], dq[1], dq[2], 
               dvh->model_matrix, dvh->projection_matrix, dvh->viewport,
               &PM_dq[0], &PM_dq[1], &PM_dq[2]);
    
    window_space_pan(dvh, dq, event->x - PM_dq[0], -((dvh->viewport[3] - event->y) - PM_dq[1]), 1);

    bot_viewer_request_redraw(viewer);
    return 1;
}

static int key_press     (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey    *event)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) ehandler->user;

    int result = TRUE;
    switch (event->keyval) {
        case GDK_Page_Up:
            zoom_ratio (dvh, 0.9);
            break;
        case GDK_Page_Down:
            zoom_ratio (dvh, 1.1);
            break;
        case GDK_leftarrow:
        case GDK_rightarrow:
        case GDK_uparrow:
        case GDK_downarrow:
        default:
            result = FALSE;
            break;
    } 

    bot_viewer_request_redraw(viewer);
    return result;
}

static void set_camera_perspective (BotViewHandler *vhandler, double fov_degrees)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    dvh->projection_type = BOT_VIEW_PERSPECTIVE;
    dvh->fov_degrees = fov_degrees;
}

static void set_camera_orthographic (BotViewHandler *vhandler)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    dvh->projection_type = BOT_VIEW_ORTHOGRAPHIC;
}

static BotProjectionMode get_projection_mode (BotViewHandler *vhandler)
{
  BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

  projection_mode = dvh->projection_type;
  return projection_mode;
}

static double get_perspective_fov (BotViewHandler *vhandler)
{
  BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

  return dvh->fov_degrees;
}

static void update_gl_matrices(BotViewer *viewer, BotViewHandler *vhandler)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    glGetIntegerv(GL_VIEWPORT, dvh->viewport);
    dvh->width = dvh->viewport[2];
    dvh->height = dvh->viewport[3];
    dvh->aspect_ratio = ((double) dvh->width) / dvh->height;

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    if (dvh->projection_type == BOT_VIEW_ORTHOGRAPHIC) {
        double le[3];
        bot_vector_subtract_3d (dvh->eye, dvh->lookat, le);
        double dist = bot_vector_magnitude_3d (le) *
            tan (dvh->fov_degrees * M_PI / 180 / 2);
        glOrtho (-dist*dvh->aspect_ratio, dist*dvh->aspect_ratio,
                 -dist, dist, -10*dist, EYE_MAX_DIST * 2);
    } else {
        gluPerspective (dvh->fov_degrees, dvh->aspect_ratio, 0.1, EYE_MAX_DIST * 2);
    }
    
    glGetDoublev(GL_PROJECTION_MATRIX, dvh->projection_matrix);

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    glMultMatrixd(dvh->model_matrix);
}

static void get_eye_look (BotViewHandler *vhandler, double eye[3], double look[3], double up[3])
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    memcpy(eye, dvh->eye, 3 * sizeof(double));
    memcpy(look, dvh->lookat, 3 * sizeof(double));
    memcpy(up, dvh->up, 3 * sizeof(double));
}

static void set_look_at (BotViewHandler *vhandler, const double eye[3], const double lookat[3], const double up[3])
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    memcpy(dvh->eye, eye, 3 * sizeof(double));
    memcpy(dvh->lookat, lookat, 3 * sizeof(double));
    memcpy(dvh->up, up, 3 * sizeof(double));

    look_at_changed(dvh);
}

static gboolean 
set_look_at_smooth_func (BotViewHandler *vhandler)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;  

    gboolean end_timeout = TRUE;
    
    double elapsed_ms = (bot_timestamp_now() - dvh->viewpath_timer_start) * 1e-3;
    double time_param = elapsed_ms / dvh->viewpath_duration_ms;
    
    if (time_param >= 1) {
      time_param = 1;
      end_timeout = FALSE;
    }
    
    for (int i=0; i<3; i++) {
      dvh->eye[i]    = dvh->origin_eye[i]    + time_param * (dvh->goal_eye[i]    - dvh->origin_eye[i]);
      dvh->lookat[i] = dvh->origin_lookat[i] + time_param * (dvh->goal_lookat[i] - dvh->origin_lookat[i]);
      dvh->up[i]     = dvh->origin_up[i]     + time_param * (dvh->goal_up[i]     - dvh->origin_up[i]);
    }

    look_at_changed(dvh);
    bot_viewer_request_redraw(dvh->viewer);

    return end_timeout;
}

static void set_look_at_smooth (BotViewHandler *vhandler, const double eye[3], const double lookat[3], const double up[3], double duration_ms)
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;
    dvh->viewpath_duration_ms = duration_ms;

    dvh->viewpath_timer_start = bot_timestamp_now();

    // get the origin for the view path, and set the goal
    vhandler->get_eye_look(vhandler, dvh->origin_eye, dvh->origin_lookat, dvh->origin_up);
    memcpy(dvh->goal_eye, eye, 3 * sizeof(double));
    memcpy(dvh->goal_lookat, lookat, 3 * sizeof(double));
    double up_norm = sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
    dvh->goal_up[0] = up[0] / up_norm;
    dvh->goal_up[1] = up[1] / up_norm;
    dvh->goal_up[2] = up[2] / up_norm;
//    memcpy(dvh->goal_up, up, 3 * sizeof(double));

    // periodically update the viewpoint until the goal viewpoint is reached
    g_timeout_add(30, (GSourceFunc)set_look_at_smooth_func, vhandler); 
}
   
static void update_follow_target(BotViewHandler *vhandler, const double pos[3], const double quat[4])
{
    BotDefaultViewHandler *dvh = (BotDefaultViewHandler*) vhandler->user;

    if (dvh->have_last && (vhandler->follow_mode & BOT_FOLLOW_YAW)) {

        // compute the vectors from the vehicle to the lookat and eye
        // point and then project them given the new position of the car/
        double v2eye[3];
        bot_vector_subtract_3d(dvh->lastpos, dvh->eye, v2eye);
        double v2look[3];
        bot_vector_subtract_3d(dvh->lastpos, dvh->lookat, v2look);

        double vxy[3] = { 1, 0, 0 };
        bot_quat_rotate (quat, vxy);
        vxy[2] = 0;

        // where was the car pointing last time?
        double oxy[3] = { 1, 0, 0 };
        bot_quat_rotate (dvh->lastquat, oxy);
        oxy[2] = 0;
        double theta = bot_vector_angle_2d (oxy, vxy);

        double zaxis[3] = { 0, 0, 1 };
        double q[4];
        bot_angle_axis_to_quat (theta, zaxis, q);

        bot_quat_rotate(q, v2look);
        bot_vector_subtract_3d(pos, v2look, dvh->lookat);

        bot_quat_rotate(q, v2eye);
        bot_vector_subtract_3d(pos, v2eye, dvh->eye);
        bot_quat_rotate (q, dvh->up);

        // the above algorithm "builds in" a BOT_FOLLOW_POS behavior.

    } else if (dvh->have_last && (vhandler->follow_mode & BOT_FOLLOW_POS)) {

        double dpos[3];
        for (int i = 0; i < 3; i++)
            dpos[i] = pos[i] - dvh->lastpos[i];
        
        for (int i = 0; i < 3; i++) {
            dvh->eye[i] += dpos[i];
            dvh->lookat[i] += dpos[i];
        }
    } else {

        // when the follow target moves, we want rotations to still behave
        // correctly. Rotations use the lookat point as the center of rotation,
        // so adjust the lookat point so that it is on the same plane as the
        // follow target.

        double look_dir[3];
        bot_vector_subtract_3d(dvh->lookat, dvh->eye, look_dir);
        bot_vector_normalize_3d(look_dir);

        if (fabs(look_dir[2]) > 0.0001) {
            double dist = (dvh->lastpos[2] - dvh->lookat[2]) / look_dir[2];
            for (int i = 0; i < 3; i++)
                dvh->lookat[i] += dist * look_dir[i];
        }
    }

    look_at_changed(dvh);

    dvh->have_last = 1;
    memcpy(dvh->lastpos, pos, 3 * sizeof(double));
    memcpy(dvh->lastquat, quat, 4 * sizeof(double));
}

BotDefaultViewHandler *bot_default_view_handler_new(BotViewer *viewer)
{
    BotDefaultViewHandler *dvh = 
        (BotDefaultViewHandler*) calloc(1, sizeof(BotDefaultViewHandler));

    dvh->fov_degrees = 60;
    dvh->aspect_ratio = 1;
    dvh->projection_type = BOT_VIEW_PERSPECTIVE;

    dvh->lookat[0] = 0;
    dvh->lookat[1] = 0;
    dvh->lookat[2] = 0;

    dvh->eye[0] = 0;
    dvh->eye[1] = 0;
    dvh->eye[2] = 50;

    dvh->up[0] = 1;
    dvh->up[1] = 0;
    dvh->up[2] = 0;

    look_at_changed(dvh);

    dvh->vhandler.update_gl_matrices = update_gl_matrices;
    dvh->vhandler.get_eye_look = get_eye_look;
    dvh->vhandler.set_look_at = set_look_at;
    dvh->vhandler.set_look_at_smooth = set_look_at_smooth;
    dvh->vhandler.update_follow_target = update_follow_target;
    dvh->vhandler.set_camera_perspective = set_camera_perspective;
    dvh->vhandler.set_camera_orthographic = set_camera_orthographic;
    dvh->vhandler.get_projection_mode = get_projection_mode;
    dvh->vhandler.get_perspective_fov = get_perspective_fov;
    dvh->vhandler.user = dvh;

    dvh->ehandler.name = "Camera Control";
    dvh->ehandler.mouse_press = mouse_press;
    dvh->ehandler.mouse_release = mouse_release;
    dvh->ehandler.mouse_motion = mouse_motion;
    dvh->ehandler.mouse_scroll = mouse_scroll;
    dvh->ehandler.key_press = key_press;
    dvh->ehandler.enabled = 1;
    dvh->ehandler.user = dvh;

    dvh->viewer = viewer;
    
    bot_viewer_set_view_handler(viewer, &dvh->vhandler);
    bot_viewer_add_event_handler(viewer, &dvh->ehandler, -10000);
    
    return dvh;
}
