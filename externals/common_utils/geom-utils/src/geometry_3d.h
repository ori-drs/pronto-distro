#ifndef __geometry_3d_h__
#define __geometry_3d_h__

// do not #include this file directly, instead #include geometry.h

#ifdef __cplusplus
extern "C" {
#endif

// =========== constructors, destructors ==========

static inline point3d_t *
point3d_new (double x, double y, double z) 
{
    point3d_t *self = g_slice_new (point3d_t);
    self->x = x;
    self->y = y;
    self->z = z;
    return self;
}

static inline point3d_t *
point3d_copy (const point3d_t *point) {
    point3d_t *self = g_slice_new (point3d_t);
    self->x = point->x;
    self->y = point->y;
    self->z = point->z;
    return (self);
}

static inline void point3d_free (point3d_t *self) { 
    g_slice_free (point3d_t, self);
}

static inline pointlist3d_t *
pointlist3d_new (int npoints)
{
    pointlist3d_t *self = g_slice_new (pointlist3d_t);
    self->npoints = npoints;
    self->points = (point3d_t*) g_slice_alloc (npoints * sizeof(point3d_t));
    return self;
}

static inline pointlist3d_t *
pointlist3d_new_from_array (const point3d_t *points, int npoints)
{
    pointlist3d_t *self = g_slice_new (pointlist3d_t);
    self->npoints = npoints;
    self->points = (point3d_t*) g_slice_alloc (npoints * sizeof(point3d_t));
    memcpy (self->points, points, npoints * sizeof (point3d_t));
    return self;
}

static inline pointlist3d_t *
pointlist3d_new_from_double_array (const double *vals, int npoints)
{
    pointlist3d_t *self = g_slice_new (pointlist3d_t);
    self->npoints = npoints;
    self->points = (point3d_t*) g_slice_alloc (npoints * sizeof(point3d_t));
    int i = 0;
    for (i=0; i<npoints; i++) {
        self->points[i].x = vals[i*3+0];
        self->points[i].y = vals[i*3+1];
        self->points[i].z = vals[i*3+2];
    }
    return self;
}

static inline pointlist3d_t *
pointlist3d_new_from_glist (const GList *points)
{
    pointlist3d_t *self = g_slice_new (pointlist3d_t);
    self->npoints = g_list_length ((GList*) points);
    self->points = 
        (point3d_t*) g_slice_alloc (self->npoints * sizeof (point3d_t));
    const GList *piter;
    int i = 0;
    for (piter=points; piter; piter=piter->next) {
        point3d_t *p = (point3d_t*) piter->data;
        self->points[i].x = p->x;
        self->points[i].y = p->y;
        self->points[i].z = p->z;
        i++;
    }
    return self;
}

static inline pointlist3d_t *
pointlist3d_new_copy (const pointlist3d_t *poly)
{
    pointlist3d_t *self = g_slice_new (pointlist3d_t);
    self->npoints = poly->npoints;
    self->points = 
        (point3d_t*) g_slice_alloc (self->npoints * sizeof (point3d_t));
    memcpy (self->points, poly->points, self->npoints * sizeof (point3d_t));
    return self;
}

static inline void
pointlist3d_copy (const pointlist3d_t *src, pointlist3d_t *dest)
{
    if (!dest->points) {
        dest->npoints = src->npoints;
        dest->points = 
            (point3d_t*) g_slice_alloc(src->npoints * sizeof (point3d_t));
    } else if (dest->npoints != src->npoints) {
        g_slice_free1 (dest->npoints * sizeof (point3d_t), dest->points);
        dest->npoints = src->npoints;
        dest->points = 
            (point3d_t*) g_slice_alloc(src->npoints * sizeof (point3d_t));
    }
    memcpy (dest->points, src->points, src->npoints * sizeof (point3d_t));
}

static inline void
pointlist3d_free (pointlist3d_t *self) { 
    g_slice_free1 (self->npoints * sizeof (point3d_t), self->points); 
    g_slice_free (pointlist3d_t, self);
}

// ===========

static inline int
point3d_equals (const point3d_t *a, const point3d_t *b)
{
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    if (fabs (dx) < GEOM_EPSILON && 
        fabs (dy) < GEOM_EPSILON &&
        fabs (dz) < GEOM_EPSILON) return 1;
    return 0;
}

static inline double
geom_vec_vec_dot_3d (const vec3d_t *a, const vec3d_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

static inline void
geom_vec_cross_3d (const vec3d_t *a, const vec3d_t *b, vec3d_t *result)
{
    result->x = a->y*b->z - a->z*b->y;
    result->y = a->z*b->x - a->x*b->z;
    result->z = a->x*b->y - a->y*b->x;
}

static inline double
geom_vec_magnitude_3d (const vec3d_t *v)
{
    return sqrt (v->x*v->x + v->y*v->y + v->z*v->z);
}

static inline void
geom_vec_normalize_3d (vec3d_t *v)
{
    double mag = geom_vec_magnitude_3d (v);
    v->x /= mag;
    v->y /= mag;
    v->z /= mag;
}

static inline void
geom_saxpy_3d (double a, const vec3d_t *x, const vec3d_t *y, 
        vec3d_t *result)
{
    point3d_t tmp = { a * x->x + y->x, a * x->y + y->y, a * x->z + y->z };
    result->x = tmp.x; result->y = tmp.y; result->z = tmp.z;
}

static inline double
geom_point_point_distance_squared_3d (const point3d_t *a, const point3d_t *b)
{
    return (a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y) + 
        (a->z-b->z)*(a->z-b->z);
}

static inline double
geom_point_point_distance_3d (const point3d_t *a, const point3d_t *b)
{
    return sqrt (geom_point_point_distance_squared_3d (a, b));
}

/**
 * geom_point_plane_distance_3d:
 * @p: a point
 * @point_in_plane: a point on a plane
 * @n: a vector normal to the plane, not necessarily unit length
 *
 * Returns: the distance from a point to a plane
 */
static inline double
geom_point_plane_distance_3d (const point3d_t *p,
        const point3d_t *point_in_plane, const vec3d_t *n)
{
    double d = - geom_vec_vec_dot_3d (point_in_plane, n);
    return fabs (geom_vec_vec_dot_3d (n, p) - d) / geom_vec_magnitude_3d (n);
}

/**
 * geom_ray_plane_intersect_3d:
 * @ray_point: a point on the ray
 * @ray_dir: a vector parallel to the ray, not necessarily unit length
 * @plane_point: a point on the plane
 * @plane_normal: a normal vector of the plane, not necessarily unit length
 * @result: output parameter.
 * @u: output parameter.
 *
 * computes the intersection of a ray with a plane.  The intersection point
 * will be stored in %result, and if %u is not NULL, it will be set to the
 * value such that %result = %ray_point + u * %ray_dir.  If the two do not
 * intersect, then %result and %u are unmodified
 *
 * Returns: 1 if the two intersect, and 0 if not.
 */
static inline int
geom_ray_plane_intersect_3d (const point3d_t *ray_point, const vec3d_t *ray_dir,
        const point3d_t *plane_point, const vec3d_t *plane_normal,
        point3d_t *result, double *u)
{
    double lambda1 = ray_dir->x * plane_normal->x + 
                     ray_dir->y * plane_normal->y + 
                     ray_dir->z * plane_normal->z;

    // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < GEOM_EPSILON) return 0;

    double lambda2 = (plane_point->x - ray_point->x) * plane_normal->x +
        (plane_point->y - ray_point->y) * plane_normal->y +
        (plane_point->z - ray_point->z) * plane_normal->z;
    double v = lambda2 / lambda1;
    result->x = ray_point->x + v * ray_dir->x;
    result->y = ray_point->y + v * ray_dir->y;
    result->z = ray_point->z + v * ray_dir->z;
    if (u) *u = v;
    return 1;
}

/**
 * geom_ray_plane_intersect_3d:
 * @ray_start: the start of the ray
 * @ray_dir: the direction of the ray
 * @triangle_A: a triangle point
 * @triangle_B: a triangle point
 * @triangle_C: a triangle point
 * @result: output parameter, stores the intersection point
 * @result_t: output parameter.  %result = %ray_start + %result_t * %ray_dir
 *
 * Returns: 1 if the ray intersects the triangle, 0 if not.
 */
int geom_ray_triangle_intersect_3d (const point3d_t *ray_start, 
        const vec3d_t *ray_dir, const point3d_t *triangle_A,
        const point3d_t *triangle_B, const point3d_t *trianlge_C, 
        point3d_t *result, double *result_t);

static inline int
geom_ray_z_plane_intersect_3d(const point3d_t *ray_point, 
        const point3d_t *ray_dir, double plane_z, point2d_t *result_xy)
{
    point3d_t plane_pt = { 0, 0, plane_z};
    point3d_t plane_normal = { 0, 0, 1};
    point3d_t plane_isect_point;
    double plane_point_dist;
    if (!geom_ray_plane_intersect_3d (ray_point, ray_dir, &plane_pt, 
                &plane_normal, &plane_isect_point, &plane_point_dist) ||
        plane_point_dist <= 0) {
        return -1;
    }
    result_xy->x = plane_isect_point.x;
    result_xy->y = plane_isect_point.y;
    return 0;
}

/**
 * geom_ray_axis_aligned_box_intersect_3d:
 * @ray_point:  a point on the ray
 * @ray_dir:    the direction of the ray
 * @box_center: the centroid of the box
 * @box_size:   the size of the box along the x, y, and z axes.
 * @N:          output parameter.
 *
 * determines whether or not a ray intersects an axis-aligned box.
 *
 * Returns: INFINITY if the ray does not intersect the box.  If the ray does
 *          intersect the box, then the value t is returned such that 
 *               ray_point + ray_dir * t
 *          is the first point of intersection.  Additionally, if the output
 *          parameter N is not null, then it is set to the normal vector of the
 *          box at the point of intersection.
 */
double geom_ray_axis_aligned_box_intersect_3d (const point3d_t *ray_point, 
        const vec3d_t *ray_dir,
        const point3d_t *box_center, const point3d_t *box_size,
        vec3d_t *N);

#ifdef __cplusplus
}
#endif

#endif
