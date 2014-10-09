#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include "fasttrig.h"
#include "small_linalg.h"

#include "camtrans.h"

#if 1
#define ERR(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__);	\
        fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)
#else
#define ERR(...) 
#endif

#if 1
#define DBG(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...) 
#endif

#define CAMERA_EPSILON 1e-10

// Structure for spherical distortion model
typedef struct {
    double a;
} SphericalDistortionParams;

static void
spherical_distortion_destroy (void *dist)
{
    free((SphericalDistortionParams*)dist);
}

static int
spherical_undistort_func(const void *data, const double x, const double y,
                         double ray[3])
{
    SphericalDistortionParams *dist = (SphericalDistortionParams*)data;

    ray[0] = x;
    ray[1] = y;

    double r2 = ray[0]*ray[0] + ray[1]*ray[1];
    double denom = 1 - r2*dist->a;
    if (denom < CAMERA_EPSILON) return -1;
    double new_r2 = r2/denom;

    double ratio;
    if (fabs (r2) < CAMERA_EPSILON) ratio = 0;
    else ratio = new_r2/r2;
    ratio = sqrt (ratio);
    ray[0] = ray[0] * ratio;
    ray[1] = ray[1] * ratio;
    ray[2] = 1;

    return 0;
}

static int
spherical_distort_func(const void *data, const double ray[3],
                       double *x, double *y)
{
    SphericalDistortionParams *dist = (SphericalDistortionParams*)data;

    if (ray[2] < CAMERA_EPSILON) return -1;
    double nx = ray[0] / ray[2];
    double ny = ray[1] / ray[2];
    double r2 = nx*nx + ny*ny;
    double new_r2 = r2/(1 + r2*dist->a);

    double ratio;
    if (fabs (r2) < CAMERA_EPSILON) ratio = 0;
    else ratio = new_r2/r2;

    ratio = sqrt (ratio);
    *x = ratio*nx;
    *y = ratio*ny;

    return 0;
}

BotDistortionObj*
bot_spherical_distortion_create (const double a)
{
    BotDistortionObj *dist = (BotDistortionObj*)calloc(1, sizeof(BotDistortionObj));
    assert (NULL != dist);

    SphericalDistortionParams *params = (SphericalDistortionParams*)calloc(1, sizeof(SphericalDistortionParams));
    params->a = a*a*a*a;
    dist->params = (void*)params;

    BotDistortionFuncs *funcs = (BotDistortionFuncs*)calloc(1, sizeof(BotDistortionFuncs));
    funcs->dist_func = spherical_distort_func;
    funcs->undist_func = spherical_undistort_func;
    funcs->destroy_func = spherical_distortion_destroy;
    dist->funcs = funcs;

    return dist;
}

typedef struct {
    int num_dist;
    double *dist_vals;
    double dist_max;
    double dist_step;
    int num_undist;
    double *undist_vals;
    double undist_max;
    double undist_step;
} AngularLookupDistortionParams;

static void
angular_lookup_distortion_destroy (void *dist)
{
    AngularLookupDistortionParams* params = (AngularLookupDistortionParams*)dist;
    free(params->dist_vals);
    free(params->undist_vals);
    free(params);
}

static int
angular_lookup_undistort_func(const void *data, const double x, const double y,
                              double ray[3])
{
    AngularLookupDistortionParams *dist = (AngularLookupDistortionParams*)data;
    double r_in = sqrt(x*x + y*y);
    double theta_in = bot_fasttrig_atan2(r_in, 1);
    if (fabs(theta_in) > dist->dist_max) return -1;
    
    // TODO
    return 0;
}

static int
angular_lookup_distort_func(const void *data, const double ray[3],
                            double *x, double *y)
{
/*     AngularLookupDistortionParams *dist = (AngularLookupDistortionParams*)data; */
    //TODO
    return 0;
}

BotDistortionObj*
bot_angular_lookup_distortion_create (const int num_dist,
                                      const double* dist_vals,
                                      const double dist_step,
                                      const int num_undist,
                                      const double* undist_vals,
                                      const double undist_step)
{
    BotDistortionObj *dist = (BotDistortionObj*)calloc(1, sizeof(BotDistortionObj));
    assert (NULL != dist);

    AngularLookupDistortionParams *params = (AngularLookupDistortionParams*)
        calloc(1, sizeof(AngularLookupDistortionParams));
    params->num_dist = num_dist;
    params->dist_vals = (double*)malloc(num_dist*sizeof(double));
    memcpy(params->dist_vals, dist_vals, num_dist*sizeof(double));
    params->dist_step = dist_step;
    params->dist_max = dist_step * num_dist;
    params->num_undist = num_undist;
    params->undist_vals = (double*)malloc(num_undist*sizeof(double));
    memcpy(params->undist_vals, undist_vals, num_undist*sizeof(double));
    params->undist_step = undist_step;
    params->undist_max = undist_step * num_undist;

    dist->params = (void*)params;

    BotDistortionFuncs *funcs = (BotDistortionFuncs*)calloc(1, sizeof(BotDistortionFuncs));
    funcs->dist_func = angular_lookup_distort_func;
    funcs->undist_func = angular_lookup_undistort_func;
    funcs->destroy_func = angular_lookup_distortion_destroy;
    dist->funcs = funcs;

    return dist;
}


typedef struct {
    int num_coeffs;
    double *coeffs;
} AngularPolyDistortionParams;

static void
angular_poly_distortion_destroy (void *dist)
{
    AngularPolyDistortionParams* params = (AngularPolyDistortionParams*)dist;
    free(params->coeffs);
    free(params);
}

static double
angular_poly_evaluate(const AngularPolyDistortionParams *dist, const double in)
{
    double in2 = in*in;
    double out = in;
    double accum = in;
    for (int k = 0; k < dist->num_coeffs; ++k) {
        accum *= in2;
        out += accum * dist->coeffs[k];
    }
    return out;
}

static int
angular_poly_distort_func(const void *data, const double ray[3],
                          double *x, double *y)
{
    AngularPolyDistortionParams *dist = (AngularPolyDistortionParams*)data;
    double phi_u = acos(ray[2]/sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]));
    double phi_d = phi_u;
    for (int iter = 0; iter < 10; ++iter) {
        double f = angular_poly_evaluate(dist, phi_d) - phi_u;
        if (f < 1e-6)
            break;

        double df = 1;
        double c = 1;
        double phi_d2 = phi_d*phi_d;
        double phi_d_accum = 1;
        for (int k = 0; k < dist->num_coeffs; ++k) {
            c += 2;
            phi_d_accum *= phi_d2;
            df += dist->coeffs[k] * c * phi_d_accum;
        }
        phi_d -= f/df;
    }
    double theta = atan2(ray[1],ray[0]);
    *x = sin(phi_d)*cos(theta);
    *y = sin(phi_d)*sin(theta);
    double z = cos(phi_d);
    if (z < CAMERA_EPSILON)
        return -1;
    *x /= z;
    *y /= z;
    return 0;
}

static int
angular_poly_undistort_func(const void *data, const double x, const double y,
                            double ray[3])
{
    AngularPolyDistortionParams *dist = (AngularPolyDistortionParams*)data;
    double phi_d = acos(1/sqrt(x*x+y*y+1));
    double phi_u = angular_poly_evaluate(dist, phi_d);
    double theta = atan2(y,x);
    ray[0] = sin(phi_u)*cos(theta);
    ray[1] = sin(phi_u)*sin(theta);
    ray[2] = cos(phi_u);
    return 0;
}

BotDistortionObj*
bot_angular_poly_distortion_create (const double *coeffs, const int num_coeffs)
{
    BotDistortionObj *dist = (BotDistortionObj*)calloc(1, sizeof(BotDistortionObj));
    assert (NULL != dist);

    AngularPolyDistortionParams *params = (AngularPolyDistortionParams*)calloc(1, sizeof(AngularPolyDistortionParams));
    params->num_coeffs = num_coeffs;
    params->coeffs = calloc(num_coeffs, sizeof(double));
    memcpy(params->coeffs, coeffs, num_coeffs*sizeof(double));
    dist->params = (void*)params;

    BotDistortionFuncs *funcs = (BotDistortionFuncs*)calloc(1, sizeof(BotDistortionFuncs));
    funcs->dist_func = angular_poly_distort_func;
    funcs->undist_func = angular_poly_undistort_func;
    funcs->destroy_func = angular_poly_distortion_destroy;
    dist->funcs = funcs;

    return dist;
}



static int
null_distort_func(const void *data, const double ray[3],
                  double *x, double *y)
{

    if (ray[2] < CAMERA_EPSILON) return -1;
    *x = ray[0] / ray[2];
    *y = ray[1] / ray[2];
    return 0;
}


static int
null_undistort_func(const void *data, const double x, const double y,
                    double ray[3])
{
    ray[0] = x;
    ray[1] = y;
    ray[2] = 1;
    return 0;
}

static void
null_distortion_destroy (void *dist) {}


BotDistortionObj*
bot_null_distortion_create(void)
{
    BotDistortionObj *dist = (BotDistortionObj*)calloc(1, sizeof(BotDistortionObj));
    assert (NULL != dist);

    dist->params = NULL;

    BotDistortionFuncs *funcs = (BotDistortionFuncs*)calloc(1, sizeof(BotDistortionFuncs));
    funcs->dist_func = null_distort_func;
    funcs->undist_func = null_undistort_func;
    funcs->destroy_func = null_distortion_destroy;
    dist->funcs = funcs;

    return dist;
}

// Structure for spherical distortion model
typedef struct {
    double k1; //radial distortion coeffs
    double k2;
    double k3;
    double p1; //tangential distortion coeffs
    double p2;

} OpenCvDistortionParams;


static void
plumb_bob_distortion_destroy (void *dist)
{
    free((OpenCvDistortionParams*)dist);
}

// Undistort according to plumb bob model
static int
plumb_bob_undistort_func(const void *data, const double x, const double y,
                         double ray[3])
{
    OpenCvDistortionParams *dist = (OpenCvDistortionParams*)data;
    //modifed from opencv function cvUndistortPoints
    //k contains distortion params... we don't have k3
    double k[5]={dist->k1,dist->k2,dist->k3,dist->p1,dist->p2};

    double x1, y1, x0, y0;
    x0 = x1 =x;
    y0 = y1 =y;

    // compensate distortion iteratively
    for(int j = 0; j < 5; j++ ) //opencv uses 5
    {
        double r2 = x1*x1 + y1*y1;
        double icdist = 1./(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
        double deltaX = 2*k[2]*x1*y1 + k[3]*(r2 + 2*x1*x1);
        double deltaY = k[2]*(r2 + 2*y1*y1) + 2*k[3]*x1*y1;
        x1 = (x0 - deltaX)*icdist;
        y1 = (y0 - deltaY)*icdist;
    }

    ray[0] = x1;
    ray[1] = y1;
    ray[2] = 1;

    return 0;
}

// Distort according to plumb bob model
static int
plumb_bob_distort_func(const void *data, const double ray[3],
                       double *x, double *y)
{
    OpenCvDistortionParams *dist = (OpenCvDistortionParams*)data;
    //hopefully it is correct
    if (ray[2] < CAMERA_EPSILON) return -1;
    double xn = ray[0] / ray[2];
    double yn = ray[1] / ray[2];
    double r2 = xn*xn + yn * yn;
    double r4 = r2*r2;
    double r6 = r4*r2;
    double cdist = 1 + dist->k1*r2+dist->k2*r4 + dist->k3*r6;
    double xr = xn*cdist;
    double yr = yn*cdist;
    double a1 = 2*xn*yn;
    double a2 = r2+2*xn*xn;
    double a3 = r2+2*yn*yn;
    double dx = a1*dist->p1+a2*dist->p2;
    double dy = a3*dist->p1+a1*dist->p2;
    double xd = xr+dx;
    double yd = yr+dy;
    *x = xd;
    *y = yd;

    return 0;
}

BotDistortionObj*
bot_plumb_bob_distortion_create (const double k1, const double k2,
                                 const double k3,const double p1,
                                 const double p2)
{
    BotDistortionObj *dist = (BotDistortionObj*)calloc(1, sizeof(BotDistortionObj));
    assert (NULL != dist);

    OpenCvDistortionParams *params =
        (OpenCvDistortionParams*)calloc(1, sizeof(OpenCvDistortionParams));
    assert (NULL != params);
    params->k1 = k1;
    params->k2 = k2;
    params->k3 = k3;
    params->p1 = p1;
    params->p2 = p2;

    dist->params = (void*)params;

    BotDistortionFuncs *funcs = (BotDistortionFuncs*)calloc(1, sizeof(BotDistortionFuncs));
    funcs->dist_func = plumb_bob_distort_func;
    funcs->undist_func = plumb_bob_undistort_func;
    funcs->destroy_func = plumb_bob_distortion_destroy;
    dist->funcs = funcs;

    return dist;
}

struct _BotCamTrans {
    char *name;
    double width;             // Image width in pixels
    double height;            // Image height in pixels

    double fx;
    double fy;
    double cx;
    double cy;
    double skew;

    double matx[9];           // Camera projection matrix
    double inv_matx[9];       // Inverse camera projection matrix

    BotDistortionObj *obj; // Distortion/undistortion functions
};

static void bot_camtrans_compute_matrices (BotCamTrans *self);

BotCamTrans*
bot_camtrans_new (const char *name,
                  double width, double height,
                  double fx, double fy, double cx, double cy, double skew,
                  BotDistortionObj * distortion_obj)
{
    BotCamTrans *self = (BotCamTrans*)calloc(1, sizeof(BotCamTrans));
    assert (NULL != self);

    if(name) 
        self->name = strdup(name);
    self->width = width;
    self->height = height;

    self->fx = fx;
    self->fy = fy;
    self->cx = cx;
    self->cy = cy;
    self->skew = skew;

    bot_camtrans_compute_matrices (self);

    // Create distortion/undistortion objects
    self->obj = distortion_obj;
    
    return self;
}

static void 
bot_camtrans_compute_matrices (BotCamTrans *self)
{
    double pinhole[] = {
        self->fx, self->skew, self->cx,
        0,        self->fy,   self->cy,
        0,        0,          1 
    };
    memcpy(self->matx, pinhole, 9*sizeof(double));
    int status = bot_matrix_inverse_3x3d (self->matx, self->inv_matx);
    if (0 != status) {
        fprintf (stderr, "WARNING: camera matrix is singular (%s:%d)\n",
                 __FILE__, __LINE__);
    }
/*     printf("inv_matx = [%lf %lf %lf\n" */
/*            "            %lf %lf %lf\n" */
/*            "            %lf %lf %lf\n\n", */
/*            self->inv_matx[0], self->inv_matx[1], self->inv_matx[2], */
/*            self->inv_matx[3], self->inv_matx[4], self->inv_matx[5], */
/*            self->inv_matx[6], self->inv_matx[7], self->inv_matx[8]); */
           
}


void
bot_camtrans_destroy (BotCamTrans *self)
{
    if (NULL == self)
        return;

    if (NULL != self->obj->funcs->dist_func) {
        self->obj->funcs->destroy_func(self->obj->params);
    }
    free(self->obj->funcs);
    free(self->obj);
    free(self->name);
    free(self);
}

const char *
bot_camtrans_get_name(const BotCamTrans *self)
{
    return self->name;
}

double
bot_camtrans_get_skew (const BotCamTrans *self)
{
    return self->skew;
}

double
bot_camtrans_get_focal_length_x (const BotCamTrans *self)
{
    return self->fx;
}
double
bot_camtrans_get_focal_length_y (const BotCamTrans *self)
{
    return self->fy;
}
double
bot_camtrans_get_image_width (const BotCamTrans *self)
{
    return self->width;
}
double
bot_camtrans_get_image_height (const BotCamTrans *self)
{
    return self->height;
}
double
bot_camtrans_get_principal_x (const BotCamTrans *self)
{
    return self->cx;
}
double
bot_camtrans_get_principal_y (const BotCamTrans *self)
{
    return self->cy;
}
double
bot_camtrans_get_width (const BotCamTrans *self)
{
    return self->width;
}
double
bot_camtrans_get_height (const BotCamTrans *self)
{
    return self->height;
}

int
bot_camtrans_unproject_pixel(const BotCamTrans *self, double im_x, double im_y, 
                             double ray[3])
{
    double temp[3];
    double pixel_v[3] = {im_x, im_y, 1.};
    bot_matrix_vector_multiply_3x3_3d (self->inv_matx, pixel_v, temp);
    return self->obj->funcs->undist_func (self->obj->params, temp[0], temp[1], ray);
}

int
bot_camtrans_project_point(const BotCamTrans *self, const double p[3],
                           double im_xyz[3])
{
    double uxyz[3];
    int rval = self->obj->funcs->dist_func(self->obj->params, p, &uxyz[0], &uxyz[1]);
    if (rval == 0) {
        uxyz[2] = 1;
        bot_matrix_vector_multiply_3x3_3d (self->matx, uxyz, im_xyz);
        im_xyz[2] = p[2];
        return 0;
    }
    return -1;
}

void
bot_camtrans_scale_image (BotCamTrans *self,
                          const double scale_factor)
{
    // Image size
    self->width *= scale_factor;
    self->height *= scale_factor;

    // Pinhole parameters
    self->cx *= scale_factor;
    self->cy *= scale_factor;
    self->fx *= scale_factor;
    self->fy *= scale_factor;
    self->skew *= scale_factor;

    // Projection matrix
    int i;
    for (i = 0; i < 8; ++i) {
        self->matx[i] *= scale_factor;
    }

    // Inverse projection matrix
    double inv_scale_factor = 1/scale_factor;
    for (i = 0; i < 3; ++i) {
        self->inv_matx[3*i+0] *= inv_scale_factor;
        self->inv_matx[3*i+1] *= inv_scale_factor;
    }
}
