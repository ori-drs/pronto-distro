#ifndef __geometry_2i_h__
#define __geometry_2i_h__

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

// do not #include this file directly, instead #include geometry.h

// =========== constructors, destructors ==========

static inline point2i_t *
point2i_new (int x, int y) 
{
    point2i_t *self = (point2i_t*) g_slice_new (point2i_t);
    self->x = x;
    self->y = y;
    return self;
}

static inline point2i_t *
point2i_new_copy (const point2i_t *point) {
    point2i_t *self = (point2i_t*) g_slice_new (point2i_t);
    self->x = point->x;
    self->y = point->y;
    return (self);
}

static inline void
point2i_copy (const point2i_t *src, point2i_t *dst) {
    dst->x = src->x; dst->y = src->y;
}

static inline void point2i_free (point2i_t *self) { 
    g_slice_free (point2i_t, self); 
}

static inline pointlist2i_t *
pointlist2i_new (int npoints)
{
    pointlist2i_t *self = (pointlist2i_t*) g_slice_new (pointlist2i_t);
    self->npoints = npoints;
    self->points = (point2i_t*) g_slice_alloc0 (npoints * sizeof(point2i_t));
    return self;
}

static inline pointlist2i_t *
pointlist2i_new_from_array (const point2i_t *points, int npoints)
{
    pointlist2i_t *self = (pointlist2i_t*) g_slice_new (pointlist2i_t);
    self->npoints = npoints;
    self->points = (point2i_t*) g_slice_alloc (npoints * sizeof(point2i_t));
    memcpy (self->points, points, npoints * sizeof (point2i_t));
    return self;
}

static inline pointlist2i_t *
pointlist2i_new_from_glist (const GList *points)
{
    pointlist2i_t *self = (pointlist2i_t*) g_slice_new (pointlist2i_t);
    self->npoints = g_list_length ( (GList*) points);
    self->points = 
        (point2i_t*) g_slice_alloc (self->npoints * sizeof (point2i_t));
    const GList *piter;
    int i = 0;
    for (piter=points; piter; piter=piter->next) {
        point2i_t *p = (point2i_t*) piter->data;
        self->points[i].x = p->x;
        self->points[i].y = p->y;
        i++;
    }
    return self;
}

static inline pointlist2i_t *
pointlist2i_new_from_gqueue (const GQueue *points)
{
    return pointlist2i_new_from_glist (g_queue_peek_head_link((GQueue*)points));
}

static inline pointlist2i_t *
pointlist2i_new_copy (const pointlist2i_t *poly)
{
    pointlist2i_t *self = (pointlist2i_t*) g_slice_new (pointlist2i_t);
    self->npoints = poly->npoints;
    self->points = 
        (point2i_t*) g_slice_alloc (self->npoints * sizeof (point2i_t));
    memcpy (self->points, poly->points, self->npoints * sizeof (point2i_t));
    return self;
}

static inline void
pointlist2i_copy (const pointlist2i_t *src, pointlist2i_t *dest)
{
    if (dest->points) free (dest->points);
    dest->npoints = src->npoints;
    dest->points = 
        (point2i_t*) g_slice_alloc (src->npoints * sizeof (point2i_t));
    int i;
    for (i=0; i<src->npoints; i++) { 
        point2i_copy (&src->points[i], &dest->points[i]);
    }
}

static inline pointlist2i_t *
pointlist2i_new_from_pointlist2d (const pointlist2d_t *points)
{
    pointlist2i_t *self = (pointlist2i_t*) g_slice_new (pointlist2i_t);
    self->npoints = points->npoints;
    self->points = 
        (point2i_t*) g_slice_alloc (self->npoints * sizeof (point2i_t));
    int i;
    for (i=0; i<points->npoints; i++) {
        self->points[i].x = (int) round (points->points[i].x);
        self->points[i].y = (int) round (points->points[i].y);
    }
    return self;
}

static inline void
pointlist2i_free (pointlist2i_t *self) { 
    g_slice_free1 (self->npoints * sizeof (point2i_t), self->points);
    g_slice_free (pointlist2i_t, self);
}

static inline polygon2i_t *
polygon2i_new (void)
{
    polygon2i_t *self = g_slice_new (polygon2i_t);
    self->nlists = 0;
    self->pointlists = NULL;
    return self;
}

static inline void
polygon2i_add_pointlist (polygon2i_t * self,
        const pointlist2i_t * points)
{
    int n = self->nlists + 1;
    pointlist2i_t * newpolygons = 
        (pointlist2i_t*) g_slice_alloc (n * sizeof (pointlist2i_t));
    if (self->pointlists) {
        memcpy (newpolygons, self->pointlists,
                self->nlists * sizeof (pointlist2i_t));
        g_slice_free1 (self->nlists * sizeof (pointlist2i_t),
                self->pointlists);
    }
    self->pointlists = newpolygons;
    self->nlists = n;
    memset (self->pointlists + n - 1, 0, sizeof (pointlist2i_t));
    pointlist2i_copy (points, self->pointlists + n - 1);
}

static inline polygon2i_t *
polygon2i_new_copy (const polygon2i_t *list)
{
    polygon2i_t *self = g_slice_new (polygon2i_t);
    self->nlists = list->nlists;
    self->pointlists = (pointlist2i_t*) g_slice_alloc (self->nlists *
            sizeof (pointlist2i_t));

    memset (self->pointlists, 0, self->nlists * sizeof (pointlist2i_t));
    int i;
    for (i = 0; i < self->nlists; i++)
        pointlist2i_copy (list->pointlists + i, self->pointlists + i);
    return self;
}

static inline void
polygon2i_copy (const polygon2i_t * src, polygon2i_t * dest)
{
    int i;
    if (!dest->pointlists) {
        dest->nlists = src->nlists;
        dest->pointlists = 
            (pointlist2i_t*) g_slice_alloc(src->nlists *
                    sizeof (pointlist2i_t));
        memset (dest->pointlists, 0, dest->nlists * sizeof (pointlist2i_t));
    } else if (dest->nlists != src->nlists) {
        for (i = 0; i < dest->nlists; i++)
            g_slice_free1 (dest->pointlists[i].npoints * sizeof (point2i_t),
                    dest->pointlists[i].points);
        g_slice_free1 (dest->nlists * sizeof (pointlist2i_t),
                dest->pointlists);
        dest->nlists = src->nlists;
        dest->pointlists = 
            (pointlist2i_t*) g_slice_alloc(src->nlists *
                    sizeof (pointlist2i_t));
        memset (dest->pointlists, 0, dest->nlists * sizeof (pointlist2i_t));
    }

    for (i = 0; i < src->nlists; i++)
        pointlist2i_copy (src->pointlists + i, dest->pointlists + i);
}

static inline void
polygon2i_free (polygon2i_t * self)
{
    int i;
    for (i = 0; i < self->nlists; i++)
        g_slice_free1 (self->pointlists[i].npoints * sizeof (point2i_t),
                self->pointlists[i].points);
    g_slice_free1 (self->nlists * sizeof (pointlist2i_t), self->pointlists);
    g_slice_free (polygon2i_t, self);
}

// ===========

static inline int
point2i_equals (const point2i_t *a, const point2i_t *b)
{
    if (a->x == b->x && a->y == b->y) return 1;
    return 0;
}

// =========== functions ===========

/**
 * geom_handedness_2i:
 *
 * Computes the handedness (clockwise or counterclockwise) of the line segments
 * (%p0 - %p1) - (%p1 - %p2).
 *
 * Can also be considered a half-plane test.  In this case, the result is -1 if
 * %p2 is in the half-plane to the left of the line segment {%p0, %p1}, 1 if it
 * is in the half-plane to the right, and 0 if it is on the divider.
 *
 * Returns: -1 if the segments are CCW (left handed)
 *           1 if CW (right handed)
 *           0 if colinear.
 */
static inline int
geom_handedness_2i (const point2i_t *p0, const point2i_t *p1, 
        const point2i_t *p2)
{
    point2i_t v0 = { p1->x - p0->x, p1->y - p0->y };
    point2i_t v1 = { p2->x - p1->x, p2->y - p1->y };
    int det = v1.x * v0.y - v1.y * v0.x;
    if(det < 0) return -1;
    if(det > 0) return 1;
    return 0;
}

/**
 * geom_line_seg_line_seg_intersect_test_2i:
 *
 * Returns: 1 if the line segments (p0 - p1) and (p2 - p3) intersect, and 0
 * otherwise
 */
static inline int
geom_line_seg_line_seg_intersect_test_2i (const point2i_t *p0, 
                                 const point2i_t *p1, const point2i_t *p2, 
                                 const point2i_t *p3)
{
    if ((geom_handedness_2i(p0, p1, p2) != geom_handedness_2i(p0, p1, p3)) &&
        (geom_handedness_2i(p2, p3, p0) != geom_handedness_2i(p2, p3, p1)))
        return 1;
    return 0;
}

static inline int
geom_point_point_distance_squared_2i (const point2i_t *a, const point2i_t *b)
{
    return (a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y);
}

static inline double
geom_point_point_distance_2i (const point2i_t *a, const point2i_t *b)
{
    return sqrt ( (double) (a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y));
}

static inline int
geom_polygon_npoints_2i (const polygon2i_t *p) {
    int npoints = 0;
    for (int i=0; i<p->nlists; i++) npoints += p->pointlists[i].npoints;
    return npoints;
}

pointlist2i_t *
geom_line_seg_covered_points_2i (const point2i_t *a, const point2i_t *b);

/**
 * geom_point_inside_polygon_2i:
 *
 * Returns: 1 if the point @p is completely inside the polygon and 0
 * if it is completely outside.  If the point is exactly on the
 * polygon's border, the results are undefined.
 */
int geom_point_inside_polygon_2i (const point2i_t *p, 
        const polygon2i_t *poly);

pointlist2i_t * 
geom_compute_polygon_edge_points_2i (const polygon2i_t *poly);

/**
 * geom_compute_polygon_covered_points_2i:
 *
 * Computes a list of all points lying on the edge of and inside the specified
 * polygon. 
 *
 * Returns: a newly allocated pointlist2i_t.
 */
pointlist2i_t *
geom_compute_polygon_covered_points_2i (const polygon2i_t *poly);

/**
 * geom_compute_convex_polygon_covered_points_2i:
 *
 * Computes a list of all points lying on the edge of and inside the specified
 * convex polygon.  If you know your polygon is convex, use this function
 * instead of geom_compute_convex_polygon_covered_points_2i, as this is 
 * faster.
 *
 * Returns: a newly allocated pointlist2i_t.
 */
pointlist2i_t *
geom_compute_convex_polygon_covered_points_2i (const pointlist2i_t *p);

// ==== conversion functions ====

static inline int
pointlist2i_to_array (const pointlist2i_t *points, int *array, int max_points)
{
    int i;
    for (i=0; i<points->npoints && i<max_points; i++) {
        array[i*2+0] = points->points[i].x;
        array[i*2+1] = points->points[i].y;
    }
    return i;
}

static inline int *
pointlist2i_to_new_array (const pointlist2i_t *points)
{
    int *result = (int*) malloc (2 * points->npoints * sizeof (int));
    pointlist2i_to_array (points, result, points->npoints);
    return result;
}

#ifdef __cplusplus
}
#endif

#endif
