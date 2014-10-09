#ifndef __dgc_geometry_h__
#define __dgc_geometry_h__

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GEOM_EPSILON 1e-9

// ===== 2 dimensional structures =====

#ifndef _point2d_t_h // XXX hack... _point2d_t_h is defined in
                     // lcmtypes/point2d_t.h
// double
typedef struct _point2d {
    double x;
    double y;
} point2d_t;
#endif

#define point2d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT2D() macro to convert a
 * double[2] to a point2d_t.  gcc is smart -- if you try to cast anything
 * other than a point2d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point2d_any_t {
    point2d_t point;
    double array[2];
};
#define POINT2D(p) (&(((union _point2d_any_t *)(p))->point))

typedef point2d_t vec2d_t;

typedef struct _pointlist2d_t {
    int npoints;
    point2d_t *points;
} pointlist2d_t;

/**
 * polygon2d_t:
 *
 * represents a general polygon, which can be self-intersecting and have holes.
 * It's not always necessary to use this.  For a convex polygon, just use a
 * pointlist2d_t
 */
typedef struct _polygon2d_t {
    int nlists;
    pointlist2d_t *pointlists;
} polygon2d_t;

typedef polygon2d_t pointlistlist2d_t;

// int
typedef struct _point2i {
    int x;
    int y;
} point2i_t;

#define point2i_as_array(p) ((int*)p)

typedef point2i_t vec2i_t;

typedef struct _pointlist2i_t {
    int npoints;
    point2i_t *points;
} pointlist2i_t;

typedef struct _polygon2i_t {
    int nlists;
    pointlist2i_t *pointlists;
} polygon2i_t;


// ===== 3 dimensional strucutres =====

// double 
typedef struct _point3d {
    double x;
    double y;
    double z;
} point3d_t;

#define point3d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT3D() macro to convert a
 * double[3] to a point3d_t.  gcc is smart -- if you try to cast anything
 * other than a point3d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point3d_any_t {
    point3d_t point;
    double array[3];
};
#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))

typedef point3d_t vec3d_t;

typedef struct _pointlist3d_t {
    int npoints;
    point3d_t *points;
} pointlist3d_t;

// int

#ifdef __cplusplus
}
#endif

#include "geometry_2i.h"
#include "geometry_2d.h"
#include "geometry_3d.h"

#endif
