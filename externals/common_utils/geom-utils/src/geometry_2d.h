#ifndef __geometry_2d_h__
#define __geometry_2d_h__

// do not #include this file directly, instead #include geometry.h

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========== constructors, destructors ==========

static inline point2d_t *
point2d_new (double x, double y) 
{
    point2d_t *self = g_slice_new (point2d_t);
    self->x = x;
    self->y = y;
    return self;
}

static inline point2d_t *
point2d_new_copy (const point2d_t *point) {
    point2d_t *self = g_slice_new (point2d_t);
    self->x = point->x;
    self->y = point->y;
    return (self);
}

static inline void
point2d_copy (const point2d_t *src, point2d_t *dst) {
    dst->x = src->x; dst->y = src->y;
}

static inline void point2d_free (point2d_t *self) { 
    g_slice_free (point2d_t, self);
}

static inline pointlist2d_t *
pointlist2d_new (int npoints)
{
    pointlist2d_t *self = g_slice_new (pointlist2d_t);
    self->npoints = npoints;
    self->points = (point2d_t*) g_slice_alloc (npoints * sizeof(point2d_t));
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_from_array (const point2d_t *points, int npoints)
{
    pointlist2d_t *self = g_slice_new (pointlist2d_t);
    self->npoints = npoints;
    self->points = (point2d_t*) g_slice_alloc (npoints * sizeof(point2d_t));
    memcpy (self->points, points, npoints * sizeof (point2d_t));
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_from_double_array (const double *vals, int npoints)
{
    pointlist2d_t *self = g_slice_new (pointlist2d_t);
    self->npoints = npoints;
    self->points = (point2d_t*) g_slice_alloc (npoints * sizeof(point2d_t));
    int i = 0;
    for (i=0; i<npoints; i++) {
        self->points[i].x = vals[i*2+0];
        self->points[i].y = vals[i*2+1];
    }
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_from_glist (const GList *points)
{
    pointlist2d_t *self = g_slice_new (pointlist2d_t);
    self->npoints = g_list_length ( (GList*) points);
    self->points = 
        (point2d_t*) g_slice_alloc (self->npoints * sizeof (point2d_t));
    const GList *piter;
    int i = 0;
    for (piter=points; piter; piter=piter->next) {
        point2d_t *p = (point2d_t*) piter->data;
        self->points[i].x = p->x;
        self->points[i].y = p->y;
        i++;
    }
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_from_gqueue (const GQueue *points)
{
    return pointlist2d_new_from_glist (g_queue_peek_head_link((GQueue*)points));
}

static inline pointlist2d_t *
pointlist2d_new_from_garray (const GArray *points)
{
    return pointlist2d_new_from_array ((point2d_t*) points->data, points->len);
}

static inline pointlist2d_t *
pointlist2d_new_from_xy_arrays (const double *x, const double *y, int n)
{
    pointlist2d_t *self = pointlist2d_new (n);
    for (int i=0; i<n; i++) { 
        self->points[i].x = x[i];
        self->points[i].y = y[i];
    }
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_copy (const pointlist2d_t *poly)
{
    pointlist2d_t *self = g_slice_new (pointlist2d_t);
    self->npoints = poly->npoints;
    self->points = 
        (point2d_t*) g_slice_alloc (self->npoints * sizeof (point2d_t));
    memcpy (self->points, poly->points, self->npoints * sizeof (point2d_t));
    return self;
}

static inline pointlist2d_t *
pointlist2d_new_copy_reversed (const pointlist2d_t *poly)
{
    pointlist2d_t *copy = g_slice_new (pointlist2d_t);
    copy->npoints = poly->npoints;
    copy->points = 
        (point2d_t*) g_slice_alloc (copy->npoints * sizeof (point2d_t));
    for (int i=0; i<poly->npoints; i++) {
        copy->points[i] = poly->points[poly->npoints-i-1];
    }
    return copy;
}

#if 0
static inline pointlist2d_t *
pointlist2d_new_copy_section (const pointlist2d_t *poly, int start, 
        int npoints)
{
    if (start + npoints > poly->npoints) return NULL;
    pointlist2d_t *self = pointlist2d_new (npoints);
    for (int i=0; i<npoints; i++) {
        self->points[i] = poly->points[start + i];
    }
    return self;
}
#endif

/**
 * 
 */
pointlist2d_t *
pointlist2d_new_copy_subsection (const pointlist2d_t *line, int start_ind,
        double start_alpha, int end_ind, double end_alpha);

static inline void
pointlist2d_copy (const pointlist2d_t *src, pointlist2d_t *dest)
{
    if (!dest->points) {
        dest->npoints = src->npoints;
        dest->points = 
            (point2d_t*) g_slice_alloc(src->npoints * sizeof (point2d_t));
    } else if (dest->npoints != src->npoints) {
        g_slice_free1 (dest->npoints * sizeof (point2d_t), dest->points);
        dest->npoints = src->npoints;
        dest->points = 
            (point2d_t*) g_slice_alloc(src->npoints * sizeof (point2d_t));
    }
    memcpy (dest->points, src->points, src->npoints * sizeof (point2d_t));
}

static inline void
pointlist2d_free (pointlist2d_t *self) { 
    g_slice_free1 (self->npoints * sizeof (point2d_t), self->points); 
    g_slice_free (pointlist2d_t, self);
}

static inline polygon2d_t *
polygon2d_new (void)
{
    polygon2d_t *self = g_slice_new (polygon2d_t);
    self->nlists = 0;
    self->pointlists = NULL;
    return self;
}

/**
 * polygon_2d_new_circ
 *
 * Generates a 2D polygon representation of a circle centered at %cx and %cy 
 * of radius %radius. The resolution %res specifies the number of points 
 * that are used to generate the polygon.
 *
 * Returns a polygon2d_t representing the polygon approximation of the circle.
 * The calling function is responsible for freeing memory via polygon2d_free().
 */
static inline polygon2d_t* 
polygon2d_new_circle(double cx, double cy, double radius, int res) {
 
    polygon2d_t *self = g_slice_new(polygon2d_t);
    self->nlists = 1;
    
    pointlist2d_t *plist  = pointlist2d_new (res);
 
    double theta = 2 * M_PI / res;
    for(int i=0; i < res; i ++) {
        plist->points[i].x= cx + radius * cos(i * theta);
        plist->points[i].y= cy + radius * sin(i * theta);
    }
    
    self->pointlists = plist;
  
    return self;
}

static inline void
polygon2d_add_pointlist (polygon2d_t * self,
        const pointlist2d_t * points)
{
    int n = self->nlists + 1;
    pointlist2d_t * newpolygons = 
        (pointlist2d_t*) g_slice_alloc (n * sizeof (pointlist2d_t));
    if (self->pointlists) {
        memcpy (newpolygons, self->pointlists,
                self->nlists * sizeof (pointlist2d_t));
        g_slice_free1 (self->nlists * sizeof (pointlist2d_t),
                self->pointlists);
    }
    self->pointlists = newpolygons;
    self->nlists = n;
    memset (self->pointlists + n - 1, 0, sizeof (pointlist2d_t));
    pointlist2d_copy (points, self->pointlists + n - 1);
}

static inline polygon2d_t *
polygon2d_new_copy (const polygon2d_t *list)
{
    polygon2d_t *self = g_slice_new (polygon2d_t);
    self->nlists = list->nlists;
    self->pointlists = (pointlist2d_t*) g_slice_alloc (self->nlists *
            sizeof (pointlist2d_t));

    memset (self->pointlists, 0, self->nlists * sizeof (pointlist2d_t));
    int i;
    for (i = 0; i < self->nlists; i++)
        pointlist2d_copy (list->pointlists + i, self->pointlists + i);
    return self;
}

static inline void
polygon2d_copy (const polygon2d_t * src, polygon2d_t * dest)
{
    int i;
    if (!dest->pointlists) {
        dest->nlists = src->nlists;
        dest->pointlists = 
            (pointlist2d_t*) g_slice_alloc(src->nlists *
                    sizeof (pointlist2d_t));
        memset (dest->pointlists, 0, dest->nlists * sizeof (pointlist2d_t));
    } else if (dest->nlists != src->nlists) {
        for (i = 0; i < dest->nlists; i++)
            g_slice_free1 (dest->pointlists[i].npoints * sizeof (point2d_t),
                    dest->pointlists[i].points);
        g_slice_free1 (dest->nlists * sizeof (pointlist2d_t),
                dest->pointlists);
        dest->nlists = src->nlists;
        dest->pointlists = 
            (pointlist2d_t*) g_slice_alloc(src->nlists *
                    sizeof (pointlist2d_t));
        memset (dest->pointlists, 0, dest->nlists * sizeof (pointlist2d_t));
    }

    for (i = 0; i < src->nlists; i++)
        pointlist2d_copy (src->pointlists + i, dest->pointlists + i);
}

static inline void
polygon2d_free (polygon2d_t * self)
{
    int i;
    for (i = 0; i < self->nlists; i++)
        g_slice_free1 (self->pointlists[i].npoints * sizeof (point2d_t),
                self->pointlists[i].points);
    g_slice_free1 (self->nlists * sizeof (pointlist2d_t), self->pointlists);
    g_slice_free (polygon2d_t, self);
}

/**
 * pointlist2d_resize: 
 *
 * adjust the number of points in a pointlist.  If the # of points increases,
 * then new points are set to zero.  If the # of points decreases, then the
 * later points in the list are truncated.
 */
static inline void
pointlist2d_resize (pointlist2d_t *poly, int npoints) {
    if (poly->npoints != npoints) {
        point2d_t *newpoints = 
            (point2d_t*) g_slice_alloc (npoints * sizeof (point2d_t));
        int min_npoints = MIN (npoints, poly->npoints);
        memcpy (newpoints, poly->points, min_npoints * sizeof (point2d_t));
        if (npoints > poly->npoints) {
            memset (newpoints + min_npoints, 0, 
                    (npoints - min_npoints) * sizeof (point2d_t));
        }
        g_slice_free1 (poly->npoints * sizeof (point2d_t), poly->points);
        poly->points = newpoints;
        poly->npoints = npoints;
    }
}

// ===========

static inline int
point2d_equals (const point2d_t *a, const point2d_t *b)
{
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return (fabs (dx) < GEOM_EPSILON && fabs (dy) < GEOM_EPSILON);
}

static inline int
pointlist2d_equals (const pointlist2d_t *a, const pointlist2d_t *b)
{
    if (a->npoints != b->npoints) return 0;
    for (int i=0; i<a->npoints; i++) {
        if (! point2d_equals (&a->points[i], &b->points[i])) return 0;
    }
    return 1;
}

static inline int
pointlist2d_bounding_box (const pointlist2d_t * a, point2d_t * min,
        point2d_t * max)
{
    max->x = -INFINITY;
    max->y = -INFINITY;
    min->x = INFINITY;
    min->y = INFINITY;
    for (int i = 0; i < a->npoints; i++) {
        point2d_t * p = a->points + i;
        if (p->x > max->x) max->x = p->x;
        if (p->y > max->y) max->y = p->y;
        if (p->x < min->x) min->x = p->x;
        if (p->y < min->y) min->y = p->y;
    }
    return 0;
}

static inline int
polygon2d_bounding_box (const polygon2d_t * a, point2d_t * min,
        point2d_t * max)
{
    max->x = -INFINITY;
    max->y = -INFINITY;
    min->x = INFINITY;
    min->y = INFINITY;
    for (int j = 0; j < a->nlists; j++) {
        pointlist2d_t * list = a->pointlists + j;
        for (int i = 0; i < list->npoints; i++) {
            point2d_t * p = list->points + i;
            if (p->x > max->x) max->x = p->x;
            if (p->y > max->y) max->y = p->y;
            if (p->x < min->x) min->x = p->x;
            if (p->y < min->y) min->y = p->y;
        }
    }
    return 0;
}

// =========== functions ===========

/**
 * geom_handedness_2d:
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
geom_handedness_2d (const point2d_t *p0, const point2d_t *p1, 
        const point2d_t *p2)
{
    point2d_t v0 = { p1->x - p0->x, p1->y - p0->y };
    point2d_t v1 = { p2->x - p1->x, p2->y - p1->y };
    double det = v1.x * v0.y - v1.y * v0.x;
    if(det < 0) return -1;
    if(det > 0) return 1;
    return 0;
}

/**
 * geom_line_seg_line_seg_intersect_test_2d:
 *
 * Returns: 1 if the line segments (p0 - p1) and (p2 - p3) intersect, and 0
 * otherwise
 */
static inline int
geom_line_seg_line_seg_intersect_test_2d (const point2d_t *p0, 
                                 const point2d_t *p1, const point2d_t *p2, 
                                 const point2d_t *p3)
{
#if 0
    if ((geom_handedness_2d(p0, p1, p2) != geom_handedness_2d(p0, p1, p3)) &&
        (geom_handedness_2d(p2, p3, p0) != geom_handedness_2d(p2, p3, p1)))
        return 1;
    return 0;
#else
    double d = (p3->y-p2->y)*(p1->x-p0->x) - (p3->x-p2->x)*(p1->y-p0->y);
    if (fabs (d) < GEOM_EPSILON) return 0;
    double ua_n = (p3->x-p2->x)*(p0->y-p2->y) - (p3->y-p2->y)*(p0->x-p2->x);
    double ua = ua_n / d;
    if (ua < 0 || ua > 1) return 0;
    double ub_n = (p1->x-p0->x)*(p0->y-p2->y) - (p1->y-p0->y)*(p0->x-p2->x);
    double ub = ub_n / d;
    if (ub < 0 || ub > 1) return 0;
    return 1;
#endif
}

/**
 * geom_line_seg_line_seg_intersect_2d:
 * Computes the intersection point of two line segments.  Coincident line
 * segments (i.e. parallel and overlapping) are considered to not intersect.
 *
 * If the two segments intersect, stores the intersection point in %result.
 *
 * Returns: 1 if the two line segments intersect, and 0 if not.
 */
static inline int
geom_line_seg_line_seg_intersect_2d (const point2d_t *p0, const point2d_t *p1,
                                     const point2d_t *p2, const point2d_t *p3,
                                     point2d_t *result)
{
    double d = (p3->y-p2->y)*(p1->x-p0->x) - (p3->x-p2->x)*(p1->y-p0->y);
    if (fabs (d) < GEOM_EPSILON) return 0;
    double ua_n = (p3->x-p2->x)*(p0->y-p2->y) - (p3->y-p2->y)*(p0->x-p2->x);
    double ua = ua_n / d;
    if (ua < 0 || ua > 1) return 0;
    double ub_n = (p1->x-p0->x)*(p0->y-p2->y) - (p1->y-p0->y)*(p0->x-p2->x);
    double ub = ub_n / d;
    if (ub < 0 || ub > 1) return 0;
    result->x = p0->x + ua * (p1->x - p0->x);
    result->y = p0->y + ua * (p1->y - p0->y);
    return 1;
}

/**
 * geom_line_line_intersect_2d:
 * Computes the intersection point of two lines.  Each line is specified
 * by two points on that line.
 *
 * If the two segments intersect, stores the intersection point in %result.
 *
 * Returns: 1 if the two line segments intersect, and 0 if not.
 */
static inline int
geom_line_line_intersect_2d (const point2d_t *p0, const point2d_t *p1,
                                     const point2d_t *p2, const point2d_t *p3,
                                     point2d_t *result)
{
    double d = (p3->y-p2->y)*(p1->x-p0->x) - (p3->x-p2->x)*(p1->y-p0->y);
    if (fabs (d) < GEOM_EPSILON) return 0;
    double ua_n = (p3->x-p2->x)*(p0->y-p2->y) - (p3->y-p2->y)*(p0->x-p2->x);
    double ua = ua_n / d;
    result->x = p0->x + ua * (p1->x - p0->x);
    result->y = p0->y + ua * (p1->y - p0->y);
    return 1;
}

/**
 * geom_ray_line_seg_intersect_2d:
 * Computes the intersection of a ray with a line segment.
 *
 * Returns: 1 if the line segments intersect, and 0 if not.
 */
static inline int
geom_ray_line_seg_intersect_2d(const point2d_t *ray_start, const vec2d_t *ray_dir,
        const point2d_t *p2, const point2d_t *p3, point2d_t *result, double *t)
{
    const point2d_t *p0 = ray_start;
    double d = (p3->y-p2->y)*ray_dir->x - (p3->x-p2->x)*ray_dir->y;
    if (fabs (d) < GEOM_EPSILON) return 0;
    double ub_n = ray_dir->x*(p0->y-p2->y) - ray_dir->y*(p0->x-p2->x);
    double ub = ub_n / d;
    if (ub < 0 || ub > 1) return 0;
    double ua_n = (p3->x-p2->x)*(p0->y-p2->y) - (p3->y-p2->y)*(p0->x-p2->x);
    double ua = ua_n / d;
    if(result) {
        result->x = p0->x + ua * ray_dir->x;
        result->y = p0->y + ua * ray_dir->y;
    }
    *t = ua;
    return 1;
}

/**
 * geom_point_inside_polygon_2d:
 *
 * Returns: 1 if the point @p is completely inside the polygon and 0
 * if it is completely outside.  If the point is exactly on the
 * polygon's border, the results are undefined.
 */
int geom_point_inside_polygon_2d (const point2d_t *p, const polygon2d_t *poly);

static inline int
geom_point_inside_or_on_edge_of_convex_polygon_2d (const point2d_t *p,
        const pointlist2d_t *poly)
{
    if (poly->npoints < 3) return 0;
    int h = geom_handedness_2d (p, &poly->points[0], &poly->points[1]);
    for (int i=1; i<poly->npoints - 1; i++) {
        int ph = geom_handedness_2d (p, &poly->points[i], &poly->points[i+1]);
        if (ph && ph != h) return 0;
    }
    int ph = geom_handedness_2d (p, &poly->points[poly->npoints-1], 
            &poly->points[0]);
    return (ph == 0 || ph == h);
}

/**
 * geom_line_seg_polygon_intersect_test_2d:
 *
 * Runtime O(n) where n is the number of points in the polygon
 *
 * Returns: 1 if the line segment [%p0, %p1] intersects the polygon %poly, and
 * 0 otherwise
 */
static inline int
geom_line_seg_polygon_intersect_test_2d (const point2d_t *p0,
        const point2d_t *p1, const pointlist2d_t *poly)
{
    if (poly->npoints < 2) return 0;
    for (int i=0; i<poly->npoints-1; i++) {
        if (geom_line_seg_line_seg_intersect_test_2d (p0, p1, 
                    &poly->points[i], &poly->points[i+1])) return 1;
    }
    return geom_line_seg_line_seg_intersect_test_2d (p0, p1,
            &poly->points[poly->npoints-1], &poly->points[0]);
}

static inline int
geom_line_seg_line_intersect_test_2d (const point2d_t * p0,
        const point2d_t * p1, const pointlist2d_t * line)
{
    if (line->npoints < 2) return 0;
    for (int i=0; i<line->npoints-1; i++) {
        if (geom_line_seg_line_seg_intersect_test_2d (p0, p1, 
                    &line->points[i], &line->points[i+1])) return 1;
    }
    return 0;
}

/**
 * geom_convex_polygon_convex_polygon_intersect_2d:
 *
 * Computes the intersection of two convex polygons.
 *
 * Runtime O(n + m), where n and m are the number of points in %a and %b
 *
 * Returns: a newly allocated pointlist2d_t, or NULL if there is no 
 * intersection
 */
pointlist2d_t * geom_convex_polygon_convex_polygon_intersect_2d (
        const pointlist2d_t *a, const pointlist2d_t *b);

/**
 * geom_polygon_intersect_2d:
 *
 * Computes the intersection of two polygons %a and %b.
 *
 * Returns: a newly allocated list of polygons representing the
 * intersection of the two polygons.  Note that the intersection of
 * two polygons can often result in more than one polygon.
 */
polygon2d_t * geom_polygon_intersect_2d (const polygon2d_t * a,
                                         const polygon2d_t * b);

/**
 * geom_polygon_diff_2d:
 *
 * Computes the difference of two polygons %a and %b.  (%a minus
 * the intersecting regions of %a and %b).
 *
 * Returns: a newly allocated list of polygons representing the
 * difference of the two polygons.  Note that the difference of
 * two polygons can often result in more than one polygon.
 */
polygon2d_t * geom_polygon_diff_2d (const polygon2d_t * a,
                                    const polygon2d_t * b);

/**
 * geom_polygon_area_2d:
 *
 * Computes the area of a polygon.
 */
double
geom_polygon_area_2d (const polygon2d_t * a);

/**
 * geom_polygon_union_list_2d:
 *
 * Computes the union of a list of pointlists.
 *
 * Returns: a newly allocated polygon representing the
 * union of the pointlists in the input list.  Note that the
 * resulting polygon can have holes, represented by additional
 * contours in the resulting polygon.
 */
polygon2d_t * geom_polygon_union_list_2d (pointlist2d_t ** list,
                                          int num);

/**
 * geom_polygon_union_2d:
 *
 * Computes the union of a list of polygons.
 *
 * Returns: a newly allocated polygon representing the
 * union of the polygons in the input list.
 */
polygon2d_t * geom_polygon_union_2d (polygon2d_t ** list, int num);


/**
 * geom_rotate_point_2d:
 *
 * rotates a point %p0 by %theta radians counterclockwise about the origin.
 * The result is stored in ouput parameter %result, which may be the same as
 * %p0
 */
static inline void
geom_rotate_point_2d (const point2d_t *p0, double theta, point2d_t *result)
{
    double costheta = cos (theta);
    double sintheta = sin (theta);
    double newx = p0->x * costheta - p0->y * sintheta;
    double newy = p0->x * sintheta + p0->y * costheta;
    result->x = newx;
    result->y = newy;
}

static inline double
geom_vec_magnitude_squared_2d (const vec2d_t *v) 
{ return v->x*v->x + v->y*v->y; }

static inline double
geom_vec_magnitude_2d (const vec2d_t *v) 
{ return sqrt (v->x*v->x + v->y*v->y); }

static inline void
geom_point_point_subtract_2d (const point2d_t *a, const point2d_t *b,
        vec2d_t *result)
{ result->x = a->x - b->x; result->y = a->y - b->y; }

static inline double
geom_point_point_distance_squared_2d (const point2d_t *a, const point2d_t *b)
{
    return (a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y);
}

static inline double
geom_point_point_distance_2d (const point2d_t *a, const point2d_t *b)
{
    return sqrt ((a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y));
}

static inline void
geom_line_seg_midpoint_2d (const point2d_t *a, const point2d_t *b, 
        point2d_t *result)
{
    result->x = (a->x + b->x) / 2;
    result->y = (a->y + b->y) / 2;
}

/**
 * geom_point_line_closest_point_2d:
 *
 * computes the point on a line closest to a target point.  The resulting point
 * does not necessarily lie on the segment used to define the line.  The result
 * point can be described as:
 *     result = seg0 + u * (seg1 - seg0)
 */
static inline int
geom_point_line_closest_point_2d (const point2d_t *a,
        const point2d_t *seg0, const point2d_t *seg1, 
        point2d_t *result, double *out_u)
{
    point2d_t v = { seg1->x - seg0->x, seg1->y - seg0->y };
    double vmag_sq = v.x * v.x + v.y * v.y;
    if (vmag_sq < GEOM_EPSILON) {
        if (result) *result = *seg0; 
        if (out_u) *out_u = 0; 
        return -1; 
    }
    double u = ((a->x-seg0->x)*v.x + (a->y-seg0->y)*v.y) / vmag_sq;
    if (result) {
        result->x = seg0->x + u * v.x;
        result->y = seg0->y + u * v.y;
    }
    if (out_u) *out_u = u;
    return 0;
}

/**
 * geom_point_line_seg_closest_point_2d:
 *
 * computes the point on a line segment closest to a target point.  The
 * resulting point will always lie on the line segment.
 */
static inline int
geom_point_line_seg_closest_point_2d (const point2d_t *a, 
        const point2d_t *seg0, const point2d_t *seg1, point2d_t *result,
        double *out_u)
{
    double u = 0;
    int status = geom_point_line_closest_point_2d (a, seg0, seg1, result, 
            &u);
    if (u < 0) { u = 0; if (result) *result = *seg0; }
    if (u > 1) { u = 1; if (result) *result = *seg1; }
    if (0 == status && out_u) *out_u = u;
    return 0;
}

static inline double
geom_point_line_seg_distance_2d (const point2d_t *a, 
        const point2d_t *seg0, const point2d_t *seg1) 
{
    point2d_t closest_point;
    geom_point_line_seg_closest_point_2d (a, seg0, seg1, &closest_point, NULL);
    return geom_point_point_distance_2d (a, &closest_point);
}

static inline double
geom_triangle_area_2d (const point2d_t *p0, const point2d_t *p1, 
        const point2d_t *p2)
{
    double a = geom_point_point_distance_2d (p0, p1);
    double b = geom_point_point_distance_2d (p1, p2);
    double c = geom_point_point_distance_2d (p2, p0);
    return sqrt((a+(b+c))*(c-(a-b))*(c+(a-b))*(a+(b-c))) / 4;
}

static inline void
geom_vec_scale_2d (const vec2d_t *v, double sf, vec2d_t *result) { 
    result->x = v->x * sf; 
    result->y = v->y * sf; 
}

static inline void
geom_vec_normalize_2d (vec2d_t *v)
{
    double mag = geom_vec_magnitude_2d (v);
    v->x /= mag;
    v->y /= mag;
}

static inline double
geom_vec_vec_dot_2d (const vec2d_t *v1, const vec2d_t *v2) 
{ return v1->x*v2->x + v1->y*v2->y; }

/**
 * Computes the angle theta such that rotation v1 about the origin by theta
 * radians causes v1 to point in the same direction as v2.  positive theta 
 * results in counterclockwise turns.
 */
static inline double
geom_vec_vec_angle_2d (const vec2d_t *v1, const vec2d_t *v2) 
{
    double mag1 = geom_vec_magnitude_2d (v1);
    double mag2 = geom_vec_magnitude_2d (v2);
    double dot = geom_vec_vec_dot_2d (v1, v2);

    double costheta = dot / (mag1 * mag2);
    if (costheta > 1) return 0;
    double theta = acos (costheta);

    if ((v1->x*v2->y - v1->y*v2->x) < 0) return -theta;
    return theta;
}

static inline void
geom_saxpy_2d (double a, const vec2d_t *x, const vec2d_t *y, 
        vec2d_t *result)
{
    point2d_t tmp = { a * x->x + y->x, a * x->y + y->y };
    result->x = tmp.x; result->y = tmp.y;
}

static inline double
geom_polyline_length_2d (const pointlist2d_t *polyline)
{
    double len = 0;
    for (int i=0; i<polyline->npoints-1; i++) {
        len += geom_point_point_distance_2d (&polyline->points[i],
                &polyline->points[i+1]);
    }
    return len;
}

static inline double
geom_simple_polygon_area_2d (const pointlist2d_t *polygon)
{
    double A = 0;
    for (int i=0; i<polygon->npoints - 1; i++) {
        A += polygon->points[i].x * polygon->points[i+1].y - 
             polygon->points[i+1].x * polygon->points[i].y;
    }
    A += polygon->points[0].y * polygon->points[polygon->npoints-1].x - 
         polygon->points[polygon->npoints-1].y * polygon->points[0].x;
    A /= 2;
    return fabs (A);
}

static inline int
geom_simple_polygon_centroid_2d (const pointlist2d_t *polygon, point2d_t *c)
{
    double A = 0;
    for (int i=0; i<polygon->npoints - 1; i++) {
        A += polygon->points[i].x * polygon->points[i+1].y - 
             polygon->points[i+1].x * polygon->points[i].y;
    }
    A += polygon->points[0].y * polygon->points[polygon->npoints-1].x - 
         polygon->points[polygon->npoints-1].y * polygon->points[0].x;
    A /= 2;
    if (fabs (A) < GEOM_EPSILON) return -1;

    double x = 0;
    double y = 0;
    for (int i=0; i<polygon->npoints - 1; i++) {
        point2d_t p1 = polygon->points[i];
        point2d_t p2 = polygon->points[i+1];
        x += (p1.x + p2.x) * (p1.x * p2.y - p2.x * p1.y);
        y += (p1.y + p2.y) * (p1.x * p2.y - p2.x * p1.y);
    }
    point2d_t p1 = polygon->points[polygon->npoints - 1];
    point2d_t p2 = polygon->points[0];
    x += (p1.x + p2.x) * (p1.x * p2.y - p2.x * p1.y);
    y += (p1.y + p2.y) * (p1.x * p2.y - p2.x * p1.y);

    c->x = x / (6 * A);
    c->y = y / (6 * A);
    return 0;
}

static inline void
geom_convex_polygon_dilate_2d (pointlist2d_t *polygon, double dilation)
{
    point2d_t centroid = { 0, 0 };
    if (0 != geom_simple_polygon_centroid_2d (polygon, &centroid)) return;
    for (int i=0; i<polygon->npoints; i++) {
        vec2d_t v = { polygon->points[i].x - centroid.x,
                      polygon->points[i].y - centroid.y };
        double vmag = geom_vec_magnitude_2d (&v);
        if (vmag < GEOM_EPSILON) continue;
        vec2d_t dv = { dilation * v.x / vmag, dilation * v.y / vmag };
        polygon->points[i].x += dv.x;
        polygon->points[i].y += dv.y;
    }
}

/**
 * geom_circle_circle_intersect_2d:
 *
 * Computes the intersection points of two circles centered at %p0 and %p1 with
 * radii %r0 and %r1, respectively.  If successful, %result_left and
 * %result_right are set to the intersection points that lie to the left and
 * right, respectively, of the line segment from %p0 to %p1.
 *
 * Returns: 0 if there is no intersection
 *          1 if there is a single intersection point
 *          2 if there are two points of intersection
 */
static inline int 
geom_circle_circle_intersect_2d (const point2d_t *p0, double r0,
        const point2d_t *p1, double r1,
        point2d_t *result_left, point2d_t *result_right)
{
    vec2d_t p0p1 = { p1->x - p0->x, p1->y - p0->y };
    double d = geom_vec_magnitude_2d (&p0p1);
    if (d > r0 + r1 || d < fabs (r1 - r0)) return 0;

    if (fabs (d - (r0 + r1)) < GEOM_EPSILON) {
        result_left->x = p0->x + p0p1.x * r0 / d;
        result_left->y = p0->y + p0p1.y * r0 / d;
        result_right->x = result_left->x;
        result_right->y = result_right->y;
        return 1;
    }

    double r0_sq = r0*r0;
    double a = (r0_sq - r1*r1 + d*d) / (2*d);
    double h = sqrt (r0_sq - a*a);

    double a_over_d = a / d;
    point2d_t p2 = { 
        p0->x + a_over_d * p0p1.x,
        p0->y + a_over_d * p0p1.y
    };

    double h_over_d = h / d;
    result_left->x = p2.x - h_over_d * p0p1.y;
    result_left->y = p2.y + h_over_d * p0p1.x;

    result_right->x = p2.x + h_over_d * p0p1.y;
    result_right->y = p2.y - h_over_d * p0p1.x;
    return 2;
}

/**
 * geom_circle_point_tangent_2d:
 * @p: a point
 * @circle_center: the center point of a circle
 * @radius: the radius of the circle
 * @result_left: output parameter
 * @result_right: output parameter
 *
 * Computes the points %result_left and %result_right on a circle such that the
 * line segments { %p to %result_left } and { %p to %result_right } are tangent
 * to the circle.  If successful, %result_left and %result_right are positioned 
 * to the left and to the right of the line segment { %p to %circle_center }.
 * In the special case where %p lies on the circle, the output points are
 * equal.
 *
 * Returns: 0 if there are no tangent lines
 *          1 if %p lies on the circle,
 *          2 if there are two tangent lines.
 */
static inline int
geom_point_circle_tangent_2d (const point2d_t *p,
        const point2d_t *circle_center, double radius,
       point2d_t *result_left, point2d_t *result_right)
{
    double distance = geom_point_point_distance_2d (p, circle_center);
    if (fabs (distance - radius) < GEOM_EPSILON) {
        result_left->x = result_right->x = p->x;
        result_left->y = result_right->y = p->y;
        return 1;
    }
    if (distance < radius || radius < 0) return 0;
    if (radius < GEOM_EPSILON) {
        result_left->x = result_right->x = circle_center->x;
        result_left->y = result_right->y = circle_center->y;
    }

    point2d_t midpoint = { 
        (circle_center->x + p->x) / 2, 
        (circle_center->y + p->y) / 2 
    };

    geom_circle_circle_intersect_2d (&midpoint, distance / 2,
            circle_center, radius, result_left, result_right);

    return 2;
}

/**
 * geom_circle_circle_tangents_2d:
 *
 * Computes the set of line segments that are tangent to two circles and each
 * endpoint of each line segment lies on one of the circles.  There can be
 * anywhere from zero to four of these tangents.
 *
 * Returns: a newly allocated pointlist2d_t containing the tangents, or NULL if
 *          there are no tangents.  Successive points in the pointlist
 *          correspond to endpoints of a tangent (e.g. points 0 & 1 are one
 *          line segment; 2 & 3 are another; etc) The ordering of the returned
 *          points is meaningful.
 *
 *          When there are two or four tangents:
 *           - Points lying on p0 will always have even index (i.e. all line
 *             segments given with the p0 point first)
 *           - The inner pair of tangents (that intersect  between the circles) 
 *             will be given first if they exist.
 *           - Within a pair of line segments, the segment where the endpoint
 *             lying on p1 that is to the left of the line segment from p0 to
 *             p1 will be first.
 *          
 *          When there is one tangent:
 *           - The points on the line segment will not lie on the circles, as
 *             they would then form a degenerate line segment
 *
 *          When there are three tangents:
 *           - The inner tangent (perpendicular to the line segment joining the
 *             circles) is given first, and its endpoints will not lie on the
 *             circles.
 *           - The outer pair of tangents (that intersect between the circles) 
 *             is ordered as specified above
 */
pointlist2d_t *
geom_circle_circle_tangents_2d (const point2d_t *p0, double r0,
        const point2d_t *p1, double r1);

/**
 * shifts a polyline sideways by the specified amount.  negative offset
 * corresponds to a right shift.  positive corresponds to a left shift
 *
 * Returns: a newly allocated pointlist2d_t
 */
pointlist2d_t *
geom_polyline_shift_sideways_2d (const pointlist2d_t * line, double offset);

/**
 * geom_polyline_shift_sideways_labeled_2d:
 *
 * Same as geom_polyline_shift_sideways_2d, but provides data association
 * between the original polyline and the newly generated polyline.
 *
 * %associations must point to a NULL pointer.  On return, %associations points
 * to a newly allocated array the same length as the returned result.  For each
 * point i in the returned result, (*associations)[i] references the edge
 * generating that point.
 *
 */
pointlist2d_t *
geom_polyline_shift_sideways_labeled_2d (const pointlist2d_t *line, 
        double offset, int **associations);

/**
 * geom_polyline_advance_point_by_dist:
 * @start_index: specifies the initial point
 * @start_alpha: specifies the initial point 
 * @advance_distance: the distance, positive or negative, to advance the point
 * @result_index: output parameter
 * @result_alpha: output parameter
 * @result_point: output parameter
 *
 * Given a point on a polyline, represented by the index of its corresponding
 * line segment and the distance along that segment, computes a point on the
 * polyline that is %advance_distance from that point.
 *
 * A point is specified by two parameters: the line segment of the polyline on
 * which the point lies, and the normalized distance along that segment.  So a
 * point given by [i, a] is:
 *  pt = points[i] * (1-a) + points[i+1] * a
 *
 * Returns: 0 if the resulting point has distance %advance_distance along the
 *          polyline from the initial point.  If the polyline is too short
 *          (i.e. the requested %advance_distance lies off the end of the
 *          polyline) then -1 is returned and the result is either the first or
 *          last point in the polyline, depending on the sign of
 *          %advance_distance
 */
int
geom_polyline_advance_point_by_dist (const pointlist2d_t *line, 
        int start_index, double start_alpha,
        double advance_distance,
        int *result_index, double *result_alpha, point2d_t *result_point);

#if 0
pointlist2d_t *
geom_polylines_compute_centerline (const pointlist2d_t *left,
        const pointlist2d_t *right);
#endif

/**
 * geom_polyline_resample_at_regular_intervals:
 *
 * resamples a polyline so that the points are roughly equally spaced.  Note
 * that if the spacing is not fine enough, corners will get cut.  The last
 * segment of the resulting polyline may be shorter than the rest.
 */
pointlist2d_t *
geom_polyline_resample_at_regular_intervals (const pointlist2d_t *line,
        double interval);

pointlist2d_t *
geom_polyline_resample_uniform_npoints (const pointlist2d_t *line,
        int npoints);

/**
 * geom_compute_ellipse_points:
 * @cx:        center of the ellipse
 * @cy:        center of the ellipse
 * @semimajor: length of the semimajor axis
 * @semiminor: length of the semiminor axis
 * @angle:     the angle such that rotating a vector pointing along the X axis
 *             by this amount results in the vector pointing along the semimajor
 *             axis.  i.e. the angle between the semimajor axis and the X axis
 *
 * Computes points lying on an ellipse.
 */
pointlist2d_t *
geom_compute_ellipse_points (double cx, double cy, 
        double semimajor, double semiminor, double angle, int npoints);

static inline int
geom_polyline_ind_alpha_point_2d (const pointlist2d_t *line, 
        int ind, double alpha, point2d_t *result)
{
    if (ind < 0 || ind >= line->npoints || alpha < 0 || alpha > 1) return -1;
    if (ind == line->npoints-1) {
        if (0 == alpha) *result = line->points[line->npoints-1];
        else return -1;
    } else {
        result->x = 
            line->points[ind].x * (1-alpha) + line->points[ind+1].x * alpha;
        result->y = 
            line->points[ind].y * (1-alpha) + line->points[ind+1].y * alpha;
    }
    return 0;
}

// ==== conversion functions ====

static inline int
pointlist2d_to_array (const pointlist2d_t *points, double *array, 
        int max_points)
{
    int i;
    for (i=0; i<points->npoints && i<max_points; i++) {
        array[i*2+0] = points->points[i].x;
        array[i*2+1] = points->points[i].y;
    }
    return i;
}

static inline double *
pointlist2d_to_new_array (const pointlist2d_t *points)
{
    double *result = (double*) malloc (2 * points->npoints * sizeof (double));
    pointlist2d_to_array (points, result, points->npoints);
    return result;
}

/**
 * geom_point_polyline_closest_point_2d:
 * @line: specifies the polyline
 * @p: specifies the test point 
 * @result_index: output parameter, may be NULL
 * @result_alpha: output parameter, may be NULL
 * @result_point: output parameter, may be NULL
 *
 * Given a point p and a polyline, find the point on the polyline closest to p. 
 * Represent this closest point as a line segment index and a fraction along
 * the segment.  If the closest point is an endpoint of the polyline, then
 * the resulting index/alpha will be either (0, 0) or (line->npoints-1, 1).
 *
 * Output postconditions: 0 <= result_index < line->npoints
 *                        0 <= result_alpha <= 1
 *
 * Returns: 0
 */
int geom_point_polyline_closest_point_2d (const point2d_t *p,
        const pointlist2d_t *line,
        int *result_index, double *result_alpha, 
        point2d_t *result_point);

int geom_polyline_estimate_tangent_at (const pointlist2d_t *curve, int ind,
        double alpha, vec2d_t *result);

double geom_polyline_estimate_curvature_at (const pointlist2d_t *curve, int i);

#ifdef __cplusplus
}
#endif

#endif
