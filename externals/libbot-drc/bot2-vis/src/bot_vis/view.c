#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>

#include <bot_core/small_linalg.h>
#include <bot_core/rotations.h>

#include "gl_util.h"
#include "view.h"

//#define dbg(args...) fprintf(stderr, args)
#define dbg(args...)
#define err(args...) fprintf(stderr, args)

#define to_radians(deg) ((deg)*M_PI/180)

struct _BotGlView {
    double perspective_vertical_fov_degrees;

    double perspectiveness; 

    double lookAt[3];
    double eye[3];
    double up[3];

    double projectionMatrix[16];
    double modelMatrix[16];
    int viewport[4];

    double width;
    double height;
};

BotGlView * 
bot_gl_view_new(void)
{
    BotGlView * self = (BotGlView*) calloc(1, sizeof(BotGlView));

    self->perspectiveness = 1.0;
    self->perspective_vertical_fov_degrees = 60;

    self->lookAt[0] = 0;
    self->lookAt[1] = 0;
    self->lookAt[2] = 0;

    self->eye[0] = 0;
    self->eye[1] = 0;
    self->eye[2] = 100;

    self->up[0] = 0;
    self->up[1] = 1;
    self->up[2] = 0;

    // setup a reasonable default viewport
    self->viewport[0] = 0;
    self->viewport[1] = 0;
    self->viewport[2] = 640;
    self->viewport[3] = 480;

    return self;
}

void 
bot_gl_view_unref(BotGlView *self)
{
    free(self);
}

static void
_gluPerspective(double fovy, double aspect, double znear, double zfar,
        double M[16])
{
    memset(M, 0, 16*sizeof(double));
    double sine, cotangent, deltaZ;
    double radians = fovy / 2 * M_PI / 180;

    deltaZ = zfar - znear;
    sine = sin(radians);
    if ((deltaZ == 0) || (sine == 0) || (aspect == 0)) {
        return;
    }
    cotangent = cos(radians) / sine;

    M[0*4+0] = cotangent / aspect;
    M[1*4+1] = cotangent;
    M[2*4+2] = -(zfar + znear) / deltaZ;
    M[3*4+2] = -1;
    M[2*4+3] = -2 * znear * zfar / deltaZ;
    M[3*4+3] = 0;
}

static void
_glOrtho(double left, double right, double bottom, double top, 
        double znear, double zfar, double M[16])
{
    memset(M, 0, 16*sizeof(double));
    M[0*4+0] = 2 / (right - left);
    M[0*4+3] = -(right+left)/(right-left);
    M[1*4+1] = 2 / (top-bottom);
    M[1*4+3] = -(top+bottom)/(top-bottom);
    M[2*4+2] = -2 / (zfar - znear);
    M[2*4+3] = -(zfar+znear)/(zfar-znear);
    M[3*4+3] = 1;
}

static void
_lookAt(const double eye_[3], const double c_[3], const double up_[3], 
        double M[16])
{
    double eye[3]; memcpy(eye, eye_, sizeof(eye));
    double c[3];   memcpy(c, c_, sizeof(c));
    double up[3];  memcpy(up, up_, sizeof(up));

    bot_vector_normalize_3d(up);
    double f[3] = {
        c[0] - eye[0],
        c[1] - eye[1],
        c[2] - eye[2],
    };
    bot_vector_normalize_3d(f);

    double s[3];
    double u[3];
    bot_vector_cross_3d(f, up, s);
    bot_vector_cross_3d(s, f, u);

    double A[16] = {  s[0],  s[1],  s[2], 0,
                      u[0],  u[1],  u[2], 0,
                     -f[0], -f[1], -f[2], 0,
                     0, 0, 0, 1
    };

    double T[16] = { 1, 0, 0, -eye[0],
                     0, 1, 0, -eye[1],
                     0, 0, 1, -eye[2],
                     0, 0, 0, 1 };

    bot_matrix_multiply_4x4_4x4(A, T, M);
}

static void
_recompute(BotGlView *self)
{
    self->width = self->viewport[2]; 
    self->height = self->viewport[3];

	double aspect = ((double) self->width) / self->height;
	double dist = bot_vector_dist_3d(self->eye, self->lookAt);

	double pM[16];
    _gluPerspective(self->perspective_vertical_fov_degrees, aspect, 0.1, 
            100000, pM);

	double oM[16];
    _glOrtho(-dist * aspect / 2, dist*aspect / 2, -dist/2, dist/2, -100000, 
            1000000, oM);

    for(int i=0; i<16; i++) {
        oM[i] *= (1-self->perspectiveness);
        pM[i] *= self->perspectiveness;
        self->projectionMatrix[i] = oM[i] + pM[i];
    }

    _lookAt(self->eye, self->lookAt, self->up, self->modelMatrix);
}

void 
bot_gl_view_setup_camera(BotGlView *self)
{
    dbg("%s:%d\n", __FILE__, __LINE__);
    glGetIntegerv(GL_VIEWPORT, self->viewport);
    _recompute(self);

	/////////// PROJECTION MATRIX ////////////////
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    double proj[16];
    bot_matrix_transpose_4x4d(self->projectionMatrix, proj);
    glMultMatrixd(proj);

	/////////// MODEL MATRIX ////////////////
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    double model[16];
    bot_matrix_transpose_4x4d(self->modelMatrix, model);
	glMultMatrixd(model);
}

void 
bot_gl_view_look_at(BotGlView *self,
        const double eye[3], const double lookAt[3], const double up[3])
{
    memcpy(self->eye, eye, 3*sizeof(double));
    memcpy(self->lookAt, lookAt, 3*sizeof(double));
    memcpy(self->up, up, 3*sizeof(double));

    _recompute(self);
}

//void
//    public void fit2D(double xy0[], double xy1[])
//    {
//	this.lookAt = new double[] {(xy0[0]+xy1[0])/2.0,
//				    (xy0[1]+xy1[1])/2.0,
//				    0};
//	this.up = new double[] {0, 1, 0};
//	double dist = Math.sqrt(Math.pow(xy0[0]-xy1[0],2) + Math.pow(xy0[1]-xy1[1],2));
//	this.eye = new double[] {lookAt[0], lookAt[1], dist};
//
//	recompute();
//    }

void 
bot_gl_view_follow(BotGlView *self, 
        double lastPos[3], 
        double lastQuat[4], 
        double newPos[3], 
        double newQuat[4], 
        gboolean followYaw)
{
    if (followYaw) {
        // follow X,Y, and orientation.

        // our strategy is to compute the eye,lookAt relative to
        // the vehicle position, then use the new vehicle position
        // to recompute new eye/lookAt. We'll keep 'up' as it is.

        double v2eye[3] = {
            lastPos[0] - self->eye[0],
            lastPos[1] - self->eye[1],
            lastPos[2] - self->eye[2]
        };
        double v2look[3] = {
            lastPos[0] - self->lookAt[0],
            lastPos[1] - self->lookAt[1],
            lastPos[2] - self->lookAt[2],
        };

        // this is the vector that the robot is newly pointing in
        double vxy[3] = { 1, 0, 0};
        bot_quat_rotate(newQuat, vxy);
        vxy[2] = 0;

        // where were we pointing last time?
        double rpy_new[3], rpy_last[3];
        bot_quat_to_roll_pitch_yaw(newQuat, rpy_new);
        bot_quat_to_roll_pitch_yaw(newQuat, rpy_last);
        double theta = rpy_new[2] - rpy_last[2];

        double zaxis[3] = {0,0,1};
        double q[4];
        bot_angle_axis_to_quat(theta, zaxis, q);

        bot_quat_rotate(q, v2look);
        double newLookAt[3] = {
            newPos[0] - v2look[0],
            newPos[1] - v2look[1],
            newPos[2] - v2look[2],
        };

        bot_quat_rotate(q, v2eye);
        double newEye[3] = {
            newPos[0] - v2eye[0],
            newPos[1] - v2eye[1],
            newPos[2] - v2eye[2],
        };
        double newUp[3] = { self->up[0], self->up[1], self->up[2] };
        bot_quat_rotate(q, newUp);

        bot_gl_view_look_at(self, newEye, newLookAt, newUp);
    } else {
        // follow in X/Y (but not yaw)

        double dpos[3] = {
            newPos[0] - lastPos[0],
            newPos[1] - lastPos[1],
            newPos[2] - lastPos[2],
        };

        double newEye[3] = {
            self->eye[0] + dpos[0],
            self->eye[1] + dpos[1],
            self->eye[2] + dpos[2],
        };
        double newLookAt[3] = {
            self->lookAt[0] + dpos[0],
            self->lookAt[1] + dpos[1],
            self->lookAt[2] + dpos[2],
        };

        bot_gl_view_look_at(self, newEye, newLookAt, self->up);
    }
}

void 
bot_gl_view_set_perspectiveness(BotGlView *self, double perspectiveness)
{
    self->perspectiveness = perspectiveness;
}

double 
bot_gl_view_get_perspectiveness(const BotGlView * self)
{
    return self->perspectiveness;
}

void 
bot_gl_view_get_projection_matrix(const BotGlView *self, double m[16])
{
    bot_matrix_transpose_4x4d(self->projectionMatrix, m);
}

void 
bot_gl_view_get_modelview_matrix(const BotGlView *self, double m[16])
{
    bot_matrix_transpose_4x4d(self->modelMatrix, m);
}

void
bot_gl_view_get_viewport(const BotGlView *self, int viewport[4])
{
    memcpy(viewport, self->viewport, 4*sizeof(int));
}

void 
bot_gl_view_get_look_at(const BotGlView *self, double eye[3], 
        double lookat[3], double up[3])
{
    if(eye)
        memcpy(eye, self->eye, 3*sizeof(double));
    if(lookat)
        memcpy(lookat, self->lookAt, 3*sizeof(double));
    if(up)
        memcpy(up, self->up, 3*sizeof(double));
}
