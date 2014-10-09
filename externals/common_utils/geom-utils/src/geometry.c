#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include <math.h>

#if 0
#include <GL/gl.h>
#include <GL/glu.h>

static inline void draw_disk (double x, double y, double r)
{
    GLUquadricObj *q = gluNewQuadric ();
    glPushMatrix ();
    glTranslatef (x, y, 0);
    gluDisk (q, 0, r, 20, 1);
    glPopMatrix ();
    gluDeleteQuadric (q);
}
#endif

#include <glib.h>

#include "geometry.h"
#include "convexhull.h"
#include "gpc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338
#endif


//#define dbg(...) fprintf (stderr, __VA_ARGS__)
#define dbg(...)
#define errl(...) do { \
    fprintf (stderr, "%s:%d ", __FILE__, __LINE__); \
    fprintf (stderr, __VA_ARGS__); \
} while (0)

static inline int 
areasign (const point2d_t *a, const point2d_t *b,
        const point2d_t *c)
{
    double area2 = (b->x - a->x)*(c->y - a->y) - (c->x - a->x)*(b->y - a->y);
    if (area2 > GEOM_EPSILON) return 1;
    else if (area2 < -GEOM_EPSILON) return -1;
    else return 0;
}

// implementation of 
//     J. O'Rourke and C.B. Chien and T. Olson and D. Naddor. "A New Linear
//     Algorithm for Intersecting Convex Polygons". Computer Graphics and Image
//     Processing, vol 19, pp 384--391, 1982.
//
// O (m + n) algorithm for computing the intersection of two convex polygons,
// where m and n are the number of vertices in the polygons
pointlist2d_t * geom_convex_polygon_convex_polygon_intersect_2d (
        const pointlist2d_t *a, const pointlist2d_t *b)
{
    // reject degenerate polygons
    if (a->npoints < 3 || b->npoints < 3) return NULL;

    int nchecks = 0;
    int npoints_total = a->npoints + b->npoints;
    int a_ind = 0;
    int b_ind = 0;

    // must traverse the polygons in counterclockwise order, so determine the
    // handedness of the polygons and choose whether to advance in forward or
    // reverse
    int a_step = 1;
    int b_step = 1;
    if (geom_handedness_2d (&a->points[0], &a->points[1], &a->points[2]) > 0) {
        a_step = -1;
    }
    if (geom_handedness_2d (&b->points[0], &b->points[1], &b->points[2]) > 0) {
        b_step = -1;
    }
    dbg ("a step %d (%d) b step %d (%d)\n", a_step, 
            geom_handedness_2d (&a->points[0], &a->points[1], &a->points[2]),
            b_step,
            geom_handedness_2d (&a->points[0], &a->points[1], &a->points[2]));

    GQueue *isect_points = g_queue_new ();

    const pointlist2d_t *inside = NULL;
    point2d_t last_isect_point = { 0, 0 };

    for (nchecks = 0; nchecks < 2 * npoints_total; nchecks++) {
        // pick out line segments
        point2d_t a_prev = a->points[a_ind];
        point2d_t b_prev = b->points[b_ind];
        int a_next_ind = (a_ind + a_step + a->npoints) % a->npoints;
        int b_next_ind = (b_ind + b_step + b->npoints) % b->npoints;
        point2d_t a_cur = a->points[a_next_ind];
        point2d_t b_cur = b->points[b_next_ind];
        point2d_t origin = { 0, 0 };
        vec2d_t a_dir = { a_cur.x - a_prev.x, a_cur.y - a_prev.y };
        vec2d_t b_dir = { b_cur.x - b_prev.x, b_cur.y - b_prev.y };

        dbg ("a: %d b: %d (%d, %d - %d, %d) (%d, %d - %d, %d)\n", 
                a_ind, b_ind,
                (int)a_prev.x, (int)a_prev.y, (int)a_cur.x, (int)a_cur.y, 
                (int)b_prev.x, (int)b_prev.y, (int)b_cur.x, (int)b_cur.y);
        int axb = areasign (&origin, &a_dir, &b_dir);
        int ahb = areasign (&b_prev, &b_cur, &a_cur);
        int bha = areasign (&a_prev, &a_cur, &b_cur);
        dbg ("cross = %d ahb = %d bha=%d\n", axb, ahb, bha);

        // check to see if line segments intersect
        point2d_t isect = { 0, 0 };
        if (geom_line_seg_line_seg_intersect_2d (&a_prev, &a_cur, 
                        &b_prev, &b_cur, &isect)) {

            dbg ("<%f, %f> - <%f, %f>    <%f, %f> <%f, %f>\n",
                    a_prev.x, a_prev.y, a_cur.x, a_cur.y, 
                    b_prev.x, b_prev.y, b_cur.x, b_cur.y);

            int skip_point = (!g_queue_is_empty (isect_points)) &&
                point2d_equals (&isect, &last_isect_point);

            if ( (! g_queue_is_empty (isect_points)) && !skip_point &&
                point2d_equals (&isect, 
                    (point2d_t*)g_queue_peek_head (isect_points))) {
                dbg ("repeated point <%d, %d>, finishing\n",
                        (int)isect.x, (int)isect.y);
                pointlist2d_t *result = 
                    pointlist2d_new_from_gqueue (isect_points);
                while (! g_queue_is_empty (isect_points)) {
                    point2d_free ((point2d_t*) g_queue_pop_head (isect_points));
                }
                g_queue_free (isect_points);
                // refuse to return a degenerate polygon
                if (result->npoints < 3) {
                    pointlist2d_free (result);
                    return NULL;
                }
                return result;
            }

            if (!skip_point) {
                g_queue_push_tail (isect_points, point2d_new_copy (&isect));

                last_isect_point.x = isect.x;
                last_isect_point.y = isect.y;
                dbg ("append <%f, %f>\n", isect.x, isect.y);
            }

            if (ahb > 0) {
                inside = a;
            } else if (bha > 0) { 
                inside = b;
            }
        }

        // spceical case co-linear points
        if (axb == 0 && ahb == 0 && bha == 0) {
            if (inside == a) {
                b_ind = b_next_ind;
            } else {
                a_ind = a_next_ind;
            }
        // advance either p or q
        } else if (axb >= 0) {
            if (bha > 0) {
                if (inside == a) {
                    g_queue_push_tail (isect_points, point2d_new_copy (&a_cur));
                }
                a_ind = a_next_ind;
                dbg ("   advance a %d\n", __LINE__);
            } else {
                if (inside == b) {
                    g_queue_push_tail (isect_points, point2d_new_copy (&b_cur));
                }
                b_ind = b_next_ind;
                dbg ("   advance b %d\n", __LINE__);
            }
        } else {
            if (ahb > 0) {
                if (inside == b) {
                    g_queue_push_tail (isect_points, point2d_new_copy (&b_cur));
                }
                b_ind = b_next_ind;
                dbg ("   advance b %d\n", __LINE__);
            } else {
                if (inside == a) {
                    g_queue_push_tail (isect_points, point2d_new_copy (&a_cur));
                }
                a_ind = a_next_ind;
                dbg ("   advance a %d\n", __LINE__);
            }
        }
    }

    polygon2d_t bpoly = { 1, (pointlist2d_t*)b };
    polygon2d_t apoly = { 1, (pointlist2d_t*)a };
    if (! g_queue_is_empty (isect_points)) {
        while (! g_queue_is_empty (isect_points)) {
            point2d_free ((point2d_t*) g_queue_pop_head (isect_points));
        }
        g_queue_free (isect_points);
        for (int i=0; i<3; i++) {
            if (geom_point_inside_polygon_2d (&a->points[i], &bpoly)) {
                dbg ("  a is inside b\n");
                return pointlist2d_new_copy (a);
            }
            if (geom_point_inside_polygon_2d (&b->points[i], &apoly)) {
                dbg ("   b is inside a\n");
                return pointlist2d_new_copy (b);
            }
        }
    } else {
        g_queue_free (isect_points);
        if (geom_point_inside_polygon_2d (&a->points[0], &bpoly)) {
            dbg ("  a is inside b\n");
            return pointlist2d_new_copy (a);
        }
        if (geom_point_inside_polygon_2d (&b->points[0], &apoly)) {
            dbg ("   b is inside a\n");
            return pointlist2d_new_copy (b);
        }
    }

    return NULL;
}

polygon2d_t * 
geom_polygon_intersect_2d (const polygon2d_t * a,
                           const polygon2d_t * b)
{
    gpc_polygon pa, pb, pres;

    pa.num_contours = a->nlists;
    pa.hole = NULL;
    pa.contour = a->pointlists;

    pb.num_contours = b->nlists;
    pb.hole = NULL;
    pb.contour = b->pointlists;
    
    gpc_polygon_clip (GPC_INT, &pa, &pb, &pres);

    polygon2d_t plist;
    plist.nlists = pres.num_contours;
    plist.pointlists = pres.contour;

    polygon2d_t * pout = polygon2d_new_copy (&plist);
    gpc_free_polygon (&pres);

    return pout;
}

polygon2d_t *
geom_polygon_diff_2d (const polygon2d_t * a, const polygon2d_t * b)
{
    gpc_polygon pa, pb, pres;

    pa.num_contours = a->nlists;
    pa.hole = NULL;
    pa.contour = a->pointlists;

    pb.num_contours = b->nlists;
    pb.hole = NULL;
    pb.contour = b->pointlists;
    
    gpc_polygon_clip (GPC_DIFF, &pa, &pb, &pres);

    polygon2d_t plist;
    plist.nlists = pres.num_contours;
    plist.pointlists = pres.contour;

    polygon2d_t * pout = polygon2d_new_copy (&plist);
    gpc_free_polygon (&pres);

    return pout;
}

double
geom_polygon_area_2d (const polygon2d_t * a)
{
    gpc_polygon pa, pb, pres;

    pa.num_contours = a->nlists;
    pa.hole = NULL;
    pa.contour = a->pointlists;

    pb.num_contours = 0;
    pb.hole = NULL;
    pb.contour = NULL;
    
    gpc_polygon_clip (GPC_UNION, &pa, &pb, &pres);

    double area = 0;
    int i;
    for (i = 0; i < pres.num_contours; i++) {
        double a = geom_simple_polygon_area_2d (pres.contour + i);
        area += pres.hole[i] ? -a : a;
    }
    gpc_free_polygon (&pres);

    return area;
}

polygon2d_t *
geom_polygon_union_list_2d (pointlist2d_t ** list, int num)
{
    gpc_polygon pa;

    int i;
    pa.num_contours = 0;
    pa.hole = NULL;
    pa.contour = NULL;

    for (i = 0; i < num; i++) {
        gpc_polygon pb, pres;
        pb.num_contours = 1;
        pb.hole = NULL;
        pb.contour = (pointlist2d_t *) list[i];
        gpc_polygon_clip (GPC_UNION, &pa, &pb, &pres);
        gpc_free_polygon (&pa);
        memcpy (&pa, &pres, sizeof (gpc_polygon));
    }

    polygon2d_t plist;
    plist.nlists = pa.num_contours;
    plist.pointlists = pa.contour;
    polygon2d_t * pout = polygon2d_new_copy (&plist);
    gpc_free_polygon (&pa);

    return pout;
}

polygon2d_t *
geom_polygon_union_2d (polygon2d_t ** list, int num)
{
    gpc_polygon pa;

    int i;
    pa.num_contours = 0;
    pa.hole = NULL;
    pa.contour = NULL;

    for (i = 0; i < num; i++) {
        gpc_polygon pb, pres;
        pb.num_contours = list[i]->nlists;
        pb.hole = NULL;
        pb.contour = list[i]->pointlists;
        gpc_polygon_clip (GPC_UNION, &pa, &pb, &pres);
        gpc_free_polygon (&pa);
        memcpy (&pa, &pres, sizeof (gpc_polygon));
    }

    polygon2d_t plist;
    plist.nlists = pa.num_contours;
    plist.pointlists = pa.contour;
    polygon2d_t * pout = polygon2d_new_copy (&plist);
    gpc_free_polygon (&pa);

    return pout;
}


pointlist2i_t *
geom_compute_convex_polygon_covered_points_2i (const pointlist2i_t *p)
{
    int min_pt_ind = -1;
    int max_pt_ind = -1;
    const point2i_t *min_pt = NULL;
    const point2i_t *max_pt = NULL;

    // find the points with extremal y-coordinates in the polygon, breaking
    // ties with x-coordinate
    for (int i=0; i<p->npoints; i++) {
        if (min_pt_ind < 0 || 
            (p->points[i].y <  min_pt->y ||
             (p->points[i].y == min_pt->y &&
              p->points[i].x <  min_pt->x))) {
            min_pt_ind = i;
            min_pt = &p->points[i];
        }
        if (max_pt_ind < 0 ||
            (p->points[i].y >  max_pt->y ||
             (p->points[i].y == max_pt->y &&
              p->points[i].x >  max_pt->x))) {
            max_pt_ind = i;
            max_pt = &p->points[i];
        }
    }

    // since the polygon is convex, we're guaranteed that by traversing the
    // polygon from the point with minimal y coordinate to the point with
    // maximal y coordinate, (along either the left or right chain), the
    // y-coordinates of subsequent points will be monotonically non-decreasing.

    int a_ind      = min_pt_ind;
    int a_ind_next = (a_ind + 1) % p->npoints;
    int b_ind      = min_pt_ind;
    int b_ind_next = (p->npoints + b_ind - 1) % p->npoints;

    const point2i_t *a0 = &p->points[a_ind];
    const point2i_t *a1 = &p->points[a_ind_next];
    const point2i_t *b0 = &p->points[b_ind];
    const point2i_t *b1 = &p->points[b_ind_next];

    GArray *ptarray = g_array_new (0, 0, sizeof (point2i_t));

    for (int y=min_pt->y; y<=max_pt->y; y++) {
        // advance?
        if (y > a1->y) {
            a_ind = a_ind_next;
            a_ind_next = (a_ind + 1) % p->npoints;
            a0 = &p->points[a_ind];
            a1 = &p->points[a_ind_next];
        }
        if (y > b1->y) {
            b_ind = b_ind_next;
            b_ind_next = (p->npoints + b_ind - 1) % p->npoints;
            b0 = &p->points[b_ind];
            b1 = &p->points[b_ind_next];
        }

        assert (a1->y >= y && b1->y >= y && a0->y <= y && b0->y <= y);

        point2i_t toadd = { 0, 0 };
        int min_x, max_x;

        if (a0->y == a1->y) {
            // segment a is horizontal
            if (a0->x > a1->x) {
                min_x = a1->x;
                max_x = a0->x;
            } else {
                min_x = a0->x;
                max_x = a1->x;
            }
        } else if (b0->y == b1->y) {
            // segment b is horizontal
            if (b0->x > b1->x) {
                min_x = b1->x;
                max_x = b0->x;
            } else {
                min_x = b0->x;
                max_x = b1->x;
            }
        } else {
            // both a and b have nonzero slope

            // determine the x-coordinates where a and b intersect the
            // horizontal line at y

            // the intersection P of a line segment [P1, P2] with the line y=y0
            // where P1_y != P2_y can be given as
            //
            // P = P1 + u (P2 - P1)
            //
            //            y0 - P1_y
            // where u = -----------
            //           P2_y - P1_y

            double u_a = (y - a0->y) / (double)(a1->y - a0->y);
            double u_b = (y - b0->y) / (double)(b1->y - b0->y);
            double x_a = a0->x + u_a * (a1->x - a0->x);
            double x_b = b0->x + u_b * (b1->x - b0->x);

            if (x_a < x_b) {
                min_x = (int) round (x_a);
                max_x = (int) round (x_b);
            } else {
                min_x = (int) round (x_b);
                max_x = (int) round (x_a);
            }
        }

        for (int x=min_x; x<=max_x; x++) {
            toadd.x = x;
            toadd.y = y;
            g_array_append_val (ptarray, toadd);
        }
    }

    pointlist2i_t *result = 
        pointlist2i_new_from_array ((point2i_t*) ptarray->data, ptarray->len);

    g_array_free (ptarray, TRUE);

    return result;
}

static double
point_side_of_line (const point2d_t * l0, const point2d_t * l1,
        const point2d_t * p, double offset)
{
    point2d_t v0 = { l1->x - l0->x, l1->y - l0->y };
    point2d_t v1 = { p->x - l1->x, p->y - l1->y };
    double det = v1.x * v0.y - v1.y * v0.x;
    return -det * offset;
}

static int
_polyline_shift_incorporate_segment (GArray * line, GArray *assoc,
        point2d_t * p0, point2d_t * p1, double offset, 
        point2d_t * cap_p0, point2d_t * cap_p1, int assoc_ind)
{
    int j;
    //printf ("inc ");
    for (j = line->len - 1; j >= 1; j--) {
        point2d_t * pp0 = &g_array_index (line, point2d_t, j-1);
        point2d_t * pp1 = &g_array_index (line, point2d_t, j);
        point2d_t res;
        if (geom_line_seg_line_seg_intersect_2d (pp0, pp1, p0, p1, &res) &&
                point_side_of_line (pp0, pp1, p1, offset) >= 0) {
            if (assoc) g_array_remove_range (assoc, j, line->len -j );
            g_array_remove_range (line, j, line->len - j);
            g_array_append_val (line, res);
            g_array_append_val (line, *p1);
            if (assoc) g_array_append_val (assoc, assoc_ind);
            if (assoc) g_array_append_val (assoc, assoc_ind);
            //printf ("added 2\n");
            return 1;
        }

        if (j == 1 && geom_line_seg_line_seg_intersect_2d (cap_p0, cap_p1,
                    p0, p1, &res)) {
            if (assoc) g_array_remove_range (assoc, 0, line->len);
            g_array_remove_range (line, 0, line->len);
            g_array_append_val (line, res);
            g_array_append_val (line, *p1);
            if (assoc) g_array_append_val (assoc, assoc_ind);
            if (assoc) g_array_append_val (assoc, assoc_ind);
            //printf ("hit cap\n");
            return 1;
        }
    }

    //printf ("none\n");
    return 0;
}

static void
generate_arc (point2d_t origin, const double v[2], double theta, double offset,
        point2d_t * p, int ndivs)
{
    int j;
    for (j = 0; j < ndivs; j++) {
        double th = theta - theta / (2*ndivs) - j * theta / ndivs;
        if (offset < 0)
            th = -th;
        double s = sin(th), c = cos(th);
        p[j].x = origin.x + v[0] * c - v[1] * s;
        p[j].y = origin.y + v[0] * s + v[1] * c;
    }
}

pointlist2d_t *
geom_polyline_shift_sideways_labeled_2d (const pointlist2d_t * line, 
        double offset, int **associations)
{
    if (fabs (offset) < 1e-10)
        return pointlist2d_new_copy (line);

    GArray *assoc = NULL;

    // TODO
    if (associations) {
        assoc = g_array_sized_new (FALSE, FALSE, sizeof (int), line->npoints);
    }

    point2d_t * pts = line->points;
    GArray * out = g_array_new (FALSE, FALSE, sizeof (point2d_t));
    int i0=0;
    int skipped_last = 0;
    point2d_t p0, p1;
    point2d_t cap_p0, cap_p1;
    for (int i = 1; i < line->npoints; i++) {
        point2d_t pp1 = p1;
        double dvec[2] = { pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y };
        double d = sqrt(dvec[0]*dvec[0] + dvec[1]+dvec[1]);
        if (d < 1e-10)
            continue;
        double offvec[2] = { -dvec[1] * offset / d, dvec[0] * offset / d };
        int ip = i0;
        i0 = i - 1;
        p0.x = pts[i-1].x + offvec[0];
        p0.y = pts[i-1].y + offvec[1];
        p1.x = pts[i].x   + offvec[0];
        p1.y = pts[i].y   + offvec[1];

        if (out->len == 0) {
            cap_p0 = pts[0];
            cap_p1.x = pts[0].x + 3*offvec[0];
            cap_p1.y = pts[0].y + 3*offvec[1];
            g_array_append_val (out, p0);
            g_array_append_val (out, p1);
            if (assoc) g_array_append_val (assoc, i0);
            if (assoc) g_array_append_val (assoc, i);
            skipped_last = 0;
            continue;
        }

        double v = point_side_of_line (pts+ip, pts+i-1, pts+i, offset);
        /* Bend is nearly straight */
        if (fabs (v) < 1e-4) {
//            printf ("%d. %f straight %d\n", i, d, skipped_last);
            if (skipped_last) {
                skipped_last = !_polyline_shift_incorporate_segment (out, assoc,
                        &p0, &p1, offset, &cap_p0, &cap_p1, i0);
                if (!skipped_last && assoc) {
                    g_array_index (assoc, int, assoc->len - 1) = i;
                }
            } else {
                g_array_append_val (out, p1);
                if (assoc) g_array_append_val (assoc, i0);
            }
            continue;
        }

        /* We are on the inside of a bend (less than 180 degrees) */
        if (v >= 0) {
//            printf ("%d. %f inside %d\n", i, d, skipped_last);
            /* Look for intersection with existing contour and add it. */
            skipped_last = !_polyline_shift_incorporate_segment (out, assoc,
                    &p0, &p1, offset, &cap_p0, &cap_p1, i0);
            if (!skipped_last && assoc) {
                g_array_index (assoc, int, assoc->len - 1) = i;
            }
            continue;
        }

//        printf ("%d. %f outside %d\n", i, d, skipped_last);
        /* We are on the outside of a bend (greater than 180 degrees) */
        double pdvec[2] = { pts[i-1].x - pts[ip].x, pts[i-1].y - pts[ip].y };
        double theta = fabs(geom_vec_vec_angle_2d (POINT2D(dvec), 
                    POINT2D(pdvec)));
        int ndivs = ceil  (theta / (2*M_PI / 20));
        double r = offset / cos (theta / (2*ndivs));
        double rvec[2] = { offvec[0] * r / offset, offvec[1] * r / offset };
        point2d_t arc[ndivs+2];
        generate_arc (pts[i-1], rvec, theta, offset, arc+1, ndivs);
        arc[0] = pp1;
        arc[ndivs+1] = p1;

        int j;
        if (skipped_last) {
//            printf ("%d. skipped last\n", i);
            for (j = 1; j < ndivs+2; j++) {
                if (_polyline_shift_incorporate_segment (out, assoc,
                            arc+j-1, arc+j, offset, &cap_p0, &cap_p1, i0)) {
                    j++;
                    skipped_last = 0;
                    break;
                }
            }
        }
        else {
//            printf ("%d. didn't skip last\n", i);
            point2d_t * pp = &g_array_index (out, point2d_t, out->len - 1);
            *pp = arc[1];
            if (assoc) g_array_index (assoc, int, out->len - 1) = i0;
            j = 2;
        }

        int added = 0;
        for ( ; j < ndivs+2; j++) {
            g_array_append_val (out, arc[j]);
            if (assoc) g_array_append_val (assoc, i0);
            // TODO
            skipped_last = 0;
            added = 1;
        }
        if (added && assoc) {
            g_array_index (assoc, int, assoc->len - 1) = i;
        }
    }

    pointlist2d_t * res =
        pointlist2d_new_from_array ((point2d_t *)out->data, out->len);
    g_array_free (out, TRUE);
    if (associations) {
        assert (assoc);
        assert (assoc->len == res->npoints);
        *associations = (int*)assoc->data;
        g_array_free (assoc, FALSE);
    }
    return res;
}

pointlist2d_t *
geom_polyline_shift_sideways_2d (const pointlist2d_t * line, double offset)
{
    return geom_polyline_shift_sideways_labeled_2d (line, offset, NULL);
}

pointlist2d_t *
geom_circle_circle_tangents_2d (const point2d_t *p0, double r0,
        const point2d_t *p1, double r1)
{
    vec2d_t p0p1 = { p1->x - p0->x, p1->y - p0->y };
    double d = geom_vec_magnitude_2d (&p0p1);
    double dr = fabs (r1 - r0);
    if (d < dr) return 0;

    if (point2d_equals (p0, p1) && dr < GEOM_EPSILON) return 0;

    double rbig, rsmall;
    if (r0 > r1) { rbig = r0; rsmall = r1; } 
    else { rbig = r1; rsmall = r0; }

    if (fabs (rbig - (d + rsmall)) < GEOM_EPSILON) {
        // one circle inside the other with a single interesction - one tangent

        pointlist2d_t *result = pointlist2d_new (2);
        point2d_t pt;
        if (r1 > r0) {
            pt.x = p1->x - r1 * p0p1.x / d;
            pt.y = p1->y - r1 * p0p1.y / d;
        } else {
            pt.x = p0->x + r0 * p0p1.x / d;
            pt.y = p0->y + r0 * p0p1.y / d;
        }

        result->points[0].x = pt.x - p0p1.y * r1 / d;
        result->points[0].y = pt.y + p0p1.x * r1 / d;
        result->points[1].x = pt.x + p0p1.y * r1 / d;
        result->points[1].y = pt.y - p0p1.x * r1 / d;

        return result;
    } 

    point2d_t points[8];
    int ntangents = 0;

    if (d < r0 + r1) {
        // circles intersect at two points - two tangents
    } else if (fabs (d - (rsmall + rbig)) < GEOM_EPSILON) {
        // circles intersect at a single point - three tangents
        point2d_t pt = {
            p0->x + r0 * p0p1.x / d,
            p0->y + r0 * p0p1.y / d
        };
        points[0].x = pt.x + p0p1.y * rbig / d;
        points[0].y = pt.y - p0p1.x * rbig / d;
        points[1].x = pt.x - p0p1.y * rbig / d;
        points[1].y = pt.y + p0p1.x * rbig / d;

        ntangents ++;
    } else {
        // circles do not intersect - four tangents
        double a = (r0 * d) / (r1 + r0);
        point2d_t m = { 
            p0->x + p0p1.x * a / d,
            p0->y + p0p1.y * a / d
        };

        geom_point_circle_tangent_2d (&m, p0, r0, &points[0], &points[2]);
        geom_point_circle_tangent_2d (&m, p1, r1, &points[1], &points[3]);

        ntangents +=2;
    }

    // compute the outer pair of tangents
    if (fabs (r1 - r0) < GEOM_EPSILON) {
        // outer pair of tangents are parallel
        vec2d_t n = {
            - p0p1.y * r0 / d,
            p0p1.x * r0 / d,
        };

        points[ntangents*2+0].x = p0->x + n.x;
        points[ntangents*2+0].y = p0->y + n.y;
        points[ntangents*2+1].x = p1->x + n.x;
        points[ntangents*2+1].y = p1->y + n.y;

        points[ntangents*2+2].x = p0->x - n.x;
        points[ntangents*2+2].y = p0->y - n.y;
        points[ntangents*2+3].x = p1->x - n.x;
        points[ntangents*2+3].y = p1->y - n.y;

        ntangents += 2;
    } else {
        int i = ntangents * 2;
        // XXX this may not be numerically stable...

        if (r0 > r1) {
            double b = (r1 * d) / (r0 - r1);
            point2d_t m = {
                p0->x + p0p1.x * (b + d) / d,
                p0->y + p0p1.y * (b + d) / d,
            };
            geom_point_circle_tangent_2d (&m, p0, r0, 
                    &points[i+2], &points[i+0]);
            geom_point_circle_tangent_2d (&m, p1, r1, 
                    &points[i+3], &points[i+1]);
        } else {
            double b = (r0 * d) / (r1 - r0);
            point2d_t m = {
                p1->x - p0p1.x * (b + d) / d,
                p1->y - p0p1.y * (b + d) / d,
            };
            geom_point_circle_tangent_2d (&m, p0, r0, 
                    &points[i+0], &points[i+2]);
            geom_point_circle_tangent_2d (&m, p1, r1, 
                    &points[i+1], &points[i+3]);
        }
        ntangents += 2;
    }

    pointlist2d_t *result = pointlist2d_new (ntangents * 2);
    for (int i=0; i<ntangents; i++) {
        result->points[i*2+0].x = points[i*2+0].x;
        result->points[i*2+0].y = points[i*2+0].y;
        result->points[i*2+1].x = points[i*2+1].x;
        result->points[i*2+1].y = points[i*2+1].y;
    }

    return result;
}

#define DEFINE_GEOM_POINT_INSIDE_POLYGON(basic_type, point_type, pointlist_type, polygon_type, fn_suffix) \
int \
geom_point_inside_polygon_2 ## fn_suffix (const point_type *p, \
        const polygon_type *poly)\
{\
    basic_type x = p->x;\
    basic_type y = p->y;\
    int crossings = 0;\
\
    int i;\
    for (i = 0; i < poly->nlists; i++) {\
        pointlist_type * list = poly->pointlists + i;\
        if (!list->npoints)\
            continue;\
        point_type * pn = list->points + list->npoints - 1;\
        int is_above = pn->y > y;\
\
        int j;\
        for (j = 0; j < list->npoints; j++) {\
            point_type * pp = pn;\
            int was_above = is_above;\
\
            pn = list->points + j;\
            is_above = pn->y > y;\
            if (is_above == was_above)\
                continue;\
\
            if (pn->x > x && pp->x > x)\
                continue;\
            if (pn->x <= x && pp->x <= x) {\
                crossings++;\
                continue;\
            }\
\
            double xc = pp->x + \
                (y - pp->y)*(pn->x - pp->x)/(double)(pn->y - pp->y);\
            if (xc <= x)\
                crossings++;\
        }\
    }\
\
    if (crossings % 2 == 0)\
        return 0;\
    else\
        return 1;\
}

DEFINE_GEOM_POINT_INSIDE_POLYGON (double, point2d_t, pointlist2d_t, polygon2d_t, d)
DEFINE_GEOM_POINT_INSIDE_POLYGON (int, point2i_t, pointlist2i_t, polygon2i_t, i)

/**
 * This function adapted from the Python Imaging Library
 */
pointlist2i_t *
geom_line_seg_covered_points_2i (const point2i_t *a, const point2i_t *b)
{
    int x0 = a->x;
    int y0 = a->y;
    int x1 = b->x;
    int y1 = b->y;

    // normalize
    int xs = 1;
    int ys = 1;
    int dx = x1 - x0;
    if (dx < 0) { dx = -dx; xs = -1; }
    int dy = y1 - y0;
    if (dy < 0) { dy = -dy; ys = -1; }

    GArray *points = g_array_new (FALSE, FALSE, sizeof (point2i_t));

    point2i_t toadd = { 0, 0 };

    if (dx == 0) {
        // vertical 
        for (int i=0; i<=dy; i++) {
            toadd.x = x0;
            toadd.y = y0;
            g_array_append_val (points, toadd);
            y0 += ys;
        }
    } else if (dy == 0) {
        // horizontal
        for (int i=0; i<=dx; i++) {
            toadd.x = x0;
            toadd.y = y0;
            g_array_append_val (points, toadd);
            x0 += xs;
        }
    } else if (dx > dy) {
        // bresenham, horizontal slope
        int n = dx;
        dy += dy;
        int e = dy - dx;
        dx += dx;

        for (int i = 0; i <= n; i++) {
            toadd.x = x0;
            toadd.y = y0;
            g_array_append_val (points, toadd);
            if (e >= 0) {
                y0 += ys;
                e -= dx;
            }
            e += dy;
            x0 += xs;
        }
    } else {
        // bresenham, vertical slope
        int n = dy;
        dx += dx;
        int e = dx - dy;
        dy += dy;

        for (int i = 0; i <= n; i++) {
            toadd.x = x0;
            toadd.y = y0;
            g_array_append_val (points, toadd);
            if (e >= 0) {
                x0 += xs;
                e -= dy;
            }
            e += dx;
            y0 += ys;
        }
    }
    pointlist2i_t * result = 
        pointlist2i_new_from_array ((point2i_t*)points->data, points->len);
    g_array_free (points, TRUE);
    return result;
}

static GPtrArray *
_compute_polygon_edge_pointlists_2i (const polygon2i_t *poly, 
        int *npoints_total)
{
    GPtrArray *edge_pointlists = 
        g_ptr_array_sized_new (geom_polygon_npoints_2i (poly));
    int npoints_total_ = 0;

    for (int list_ind = 0; list_ind < poly->nlists; list_ind++) {
        const pointlist2i_t *plist = &poly->pointlists[list_ind];
        if (plist->npoints <= 0) continue;

        int npoints = plist->npoints;

        for (int pt_ind = 0; pt_ind < npoints - 1; pt_ind++) {
            pointlist2i_t *edge_points = 
                geom_line_seg_covered_points_2i (&plist->points[pt_ind],
                        &plist->points[pt_ind+1]);
            g_ptr_array_add (edge_pointlists, edge_points);
            npoints_total_ += edge_points->npoints;
        }
        pointlist2i_t *edge_points = 
            geom_line_seg_covered_points_2i (&plist->points[npoints-1],
                    &plist->points[0]);
        g_ptr_array_add (edge_pointlists, edge_points);
        npoints_total_ += edge_points->npoints;
    }
    if (npoints_total) *npoints_total = npoints_total_;
    return edge_pointlists;
}

pointlist2i_t * 
geom_compute_polygon_edge_points_2i (const polygon2i_t *poly)
{
    int npoints_total;
    GPtrArray *edge_pointlists =
        _compute_polygon_edge_pointlists_2i (poly, &npoints_total);

    pointlist2i_t *result = pointlist2i_new (npoints_total);
    int result_ind = 0;
    for (int epl_ind = 0; epl_ind < edge_pointlists->len; epl_ind++) {
        pointlist2i_t *edge_points = (pointlist2i_t*)
            g_ptr_array_index (edge_pointlists, epl_ind);
        for (int pt_ind = 0; pt_ind < edge_points->npoints; pt_ind++) {
            point2i_t pt = edge_points->points[pt_ind];
            result->points[result_ind] = pt;
            result_ind ++;
        }
        pointlist2i_free (edge_points);
    }
    g_ptr_array_free (edge_pointlists, TRUE);
    return result;
}

pointlist2i_t *
geom_compute_polygon_covered_points_2i (const polygon2i_t *poly)
{
    if (geom_polygon_npoints_2i (poly) < 2) return NULL;

    point2i_t min_xy = { INT_MAX, INT_MAX };
    point2i_t max_xy = { INT_MIN, INT_MIN };

    // find a bounding box
    for (int list_ind = 0; list_ind < poly->nlists; list_ind++) {
        const pointlist2i_t *plist = &poly->pointlists[list_ind];
        for (int pt_ind = 0; pt_ind < plist->npoints; pt_ind++) {
            const point2i_t *pt = &plist->points[pt_ind];
            if (pt->x > max_xy.x) max_xy.x = pt->x;
            if (pt->y > max_xy.y) max_xy.y = pt->y;
            if (pt->x < min_xy.x) min_xy.x = pt->x;
            if (pt->y < min_xy.y) min_xy.y = pt->y;
        }
    }

    // allocate a temporary workspace
    int box_width = max_xy.x - min_xy.x + 1;
    int box_height = max_xy.y - min_xy.y + 1;
    uint8_t *workspace = (uint8_t*) calloc (1, box_width * box_height);

    if (!workspace) {
        errl ("not enough memory to compute polygon covered points\n");
        return NULL;
    }

    for (int list_ind = 0; list_ind < poly->nlists; list_ind++) {
        const pointlist2i_t *plist = &poly->pointlists[list_ind];

        int prev_dy = 0;
        for (int pt_ind = plist->npoints-1; pt_ind >= 0 && !prev_dy; pt_ind--) {
            prev_dy = plist->points[pt_ind].y - 
                plist->points[(pt_ind-1+plist->npoints) % plist->npoints].y;
        }

        for (int pt_ind = 0; pt_ind < plist->npoints; pt_ind++) {
            point2i_t p1 = plist->points[pt_ind];
            point2i_t p2 = plist->points[(pt_ind + 1) % plist->npoints];

            pointlist2i_t *edge_points = 
                geom_line_seg_covered_points_2i (&p1, &p2);

            for (int ept_ind = 0; ept_ind < edge_points->npoints; ept_ind++) {
                point2i_t ep = edge_points->points[ept_ind];
                int col = ep.x - min_xy.x;
                int row = ep.y - min_xy.y;
                workspace[row * box_width + col] ++;

                // inside a horizontal segment, don't change parity
                if (ept_ind > 0 && ep.y == edge_points->points[ept_ind-1].y)
                    workspace[row * box_width + col] ++;
            }

            // 
            int cur_dy = p2.y - p1.y;
            if (cur_dy * prev_dy >= 0) {
                int col = p1.x - min_xy.x;
                int row = p1.y - min_xy.y;
                workspace[row * box_width + col] ++;
            }
            if (cur_dy) prev_dy = cur_dy;

            pointlist2i_free (edge_points);
        }
    }

    GArray *points_inside = g_array_new (FALSE, FALSE, sizeof (point2i_t));

    for (int row = 0; row < box_height; row++) {
        int inside = 0;
        for (int col = 0; col < box_width; col++) {
            int wk_ind = row * box_width + col;
            inside ^= (workspace[wk_ind] & 0x1);
            if (inside) {
                workspace[wk_ind] = 1;
            }
            if (workspace[wk_ind]) {
                point2i_t pt = { col + min_xy.x, row + min_xy.y };
                g_array_append_val (points_inside, pt);
            }
        }
    }

    pointlist2i_t * result = 
        pointlist2i_new_from_array ((point2i_t*)points_inside->data, 
                points_inside->len);
    g_array_free (points_inside, TRUE);
    free (workspace);
    return result;
}

int 
geom_point_polyline_closest_point_2d (const point2d_t *p, 
        const pointlist2d_t *line, int *result_index, double *result_alpha, 
        point2d_t *result_point)
{
    if (line->npoints < 2) return -1;

    double min_dist_sq = INFINITY;
    double min_u = 0;
    point2d_t min_pt = { 0, 0 };
    int min_ind = 0;
    for (int i=0; i<line->npoints - 1; i++) {
        const point2d_t *seg1 = &line->points[i];
        const point2d_t *seg2 = &line->points[i+1];

        point2d_t cp;
        double cp_u = 0;
        geom_point_line_seg_closest_point_2d (p, seg1, seg2, &cp, &cp_u);
        double cp_dist_sq = geom_point_point_distance_squared_2d (&cp, p);

        if (isless (cp_dist_sq, min_dist_sq)) {
            min_dist_sq = cp_dist_sq;
            min_u = cp_u;
            min_pt = cp;
            min_ind = i;
        }
    }
    assert (min_u >= 0);
    if (result_index) *result_index = min_ind;
    if (result_alpha) *result_alpha = min_u;
    if (result_point) *result_point = min_pt;
    return 0;
}

static point2d_t
geom_polyline_interp_point (const pointlist2d_t *line,
        int index, double alpha)
{
    point2d_t result = {
        line->points[index].x * (1-alpha) + line->points[index+1].x * alpha,
        line->points[index].y * (1-alpha) + line->points[index+1].y * alpha
    };
    return result;
}

int
geom_polyline_advance_point_by_dist (const pointlist2d_t *line, 
        int start_index, double start_alpha,
        double advance_distance,
        int *result_index, double *result_alpha, point2d_t *result_point)
{
    assert (start_alpha >= 0 && start_alpha <= 1);
    assert (line->npoints > 1);
    assert (start_index >= 0 && 
            ((start_index < line->npoints - 1) ||
             (start_index == line->npoints - 1 && start_alpha == 0)));

    int ind = start_index;
    double alpha = start_alpha;

    double to_advance = fabs (advance_distance);

    const point2d_t *p0 = &line->points[ind];
    const point2d_t *p1 = &line->points[ind + 1];
    assert (isfinite (p0->x) && isfinite (p0->y));

    double dist = 0;
    if (start_index != line->npoints - 1) {
        dist = geom_point_point_distance_2d (p0, p1);
    }

    double dist_to_next_point;
    int inc;
    if (advance_distance > 0) { 
        dist_to_next_point = dist * (1-alpha); 
        inc = 1;
    } else { 
        dist_to_next_point = dist * alpha; 
        inc = -1;
    }

    while (to_advance >= dist_to_next_point) {
        to_advance -= dist_to_next_point;
        ind += inc;
        if (ind >= line->npoints - 1 || ind < 0) goto too_far;

        p0 = &line->points[ind];
        p1 = &line->points[ind + 1];
        assert (isfinite (p1->x) && isfinite (p1->y));

        dist = geom_point_point_distance_2d (p0, p1);
        dist_to_next_point = dist;
        if (advance_distance > 0) alpha = 0; else alpha = 1;
    }

    if (advance_distance > 0) {
        *result_alpha = (dist * alpha + to_advance) / dist;
    } else {
        *result_alpha = (dist * alpha - to_advance) / dist;
    }

    *result_index = ind;
    *result_point = geom_polyline_interp_point (line, ind, *result_alpha);

    assert (*result_alpha >= 0 && *result_alpha <= 1);
    return 0;
too_far:
    if (advance_distance > 0) { *result_index = line->npoints - 1; } 
    else { *result_index = 0; }
    *result_alpha = 0;
    result_point->x = line->points[*result_index].x;
    result_point->y = line->points[*result_index].y;
    return -1;
}

#if 0
pointlist2d_t *
geom_polylines_compute_centerline (const pointlist2d_t *left,
        const pointlist2d_t *right)
{
    if (left->npoints < 2 || right->npoints < 2) return NULL;

    int left_ind = 0;
    int right_ind = 0;
    double left_alpha = 0;
    double right_alpha = 0;
    int left_can_advance = 1;
    int right_can_advance = 1;

    point2d_t left_pt = left->points[0];
    point2d_t right_pt = right->points[0];
    vec2d_t left_dir = { left->points[1].x - left_pt.x,
        left->points[1].y - left_pt.y };
    vec2d_t right_dir = { right->points[1].x - right_pt.x,
        right->points[1].y - right_pt.y };

    GArray *center_points = g_array_new (0, 0, sizeof (point2d_t));

    while (left_can_advance || right_can_advance) {
        enum {
            ADVANCE_LEFT = 0x1,
            ADVANCE_RIGHT = 0x2,
        };

        int advance = 0;

        double advance_distance = 5;

        vec2d_t left_right = { right_pt.x - left_pt.x, right_pt.y - left_pt.y };
        vec2d_t right_left = { -left_right.x, -left_right.y };

        double la = fabs(geom_vec_vec_angle_2d (&left_dir, &left_right));
        double ra = fabs(geom_vec_vec_angle_2d (&right_dir, &right_left));

        // if the crossbeam is roughly perpendicular to the lane boundaries at
        // this point, then lay down a centerpoint.
        double cp_angle_thresh = 10;
        if (fabs (M_PI/2 - fabs (la)) < cp_angle_thresh * M_PI/180 && 
            fabs (M_PI/2 - fabs (ra)) < cp_angle_thresh * M_PI/180) {
            point2d_t cp;
            geom_saxpy_2d (0.5, &left_right, &left_pt, &cp);
            g_array_append_val (center_points, cp);
        }

        if (left_can_advance && right_can_advance) {

            double left_right_mag = geom_vec_magnitude_2d (&left_right);

            if (la > ra) {
                // advance right
                advance = ADVANCE_RIGHT;
                advance_distance = MAX (1, left_right_mag * cos (ra));
            } else {
                // advance left
                advance = ADVANCE_LEFT;
                advance_distance = MAX (1, left_right_mag * cos (la));
            }
        } else if (left_can_advance) {
            advance = ADVANCE_LEFT;
        } else {
            advance = ADVANCE_RIGHT;
        }

        if (advance & ADVANCE_LEFT) {
            left_can_advance = 
                ! geom_polyline_advance_point_by_dist (left, 
                        left_ind, left_alpha, advance_distance, 
                        &left_ind, &left_alpha, &left_pt);
            if (left_ind == left->npoints - 1) {
                geom_point_point_subtract_2d (&left->points[left->npoints-1],
                        &left->points[left->npoints-2], &left_dir);
            } else {
                geom_point_point_subtract_2d (&left->points[left_ind],
                        &left->points[left_ind+1], &left_dir);
            }
        }
        if (advance & ADVANCE_RIGHT) {
            right_can_advance = 
                ! geom_polyline_advance_point_by_dist (right, 
                        right_ind, right_alpha, advance_distance, 
                        &right_ind, &right_alpha, &right_pt);
            if (right_ind == right->npoints - 1) {
                geom_point_point_subtract_2d (&right->points[right->npoints-1],
                        &right->points[right->npoints-2], &right_dir);
            } else {
                geom_point_point_subtract_2d (&right->points[right_ind],
                        &right->points[right_ind+1], &right_dir);
            }
        }
    }

    pointlist2d_t *result = 
        pointlist2d_new_from_array ((point2d_t*) center_points->data,
                center_points->len);
    g_array_free (center_points, TRUE);
    return result;
}
#endif

pointlist2d_t *
geom_polyline_resample_at_regular_intervals (const pointlist2d_t *line,
        double interval)
{
    double total_len = geom_polyline_length_2d (line);
    int expected_npoints = (int) (total_len / interval) + 1;
    GArray *new_points = g_array_sized_new (FALSE, FALSE, sizeof (point2d_t),
            expected_npoints);

    if (line->npoints == 0) return pointlist2d_new (0);

    int ind = 0;
    double alpha = 0;
    point2d_t p = line->points[0];
    g_array_append_val (new_points, p);

    while (1) {
        int status = geom_polyline_advance_point_by_dist (line, ind, alpha,
                interval, &ind, &alpha, &p);
        g_array_append_val (new_points, p);
        if (0 != status) break;
    }

    pointlist2d_t *result = pointlist2d_new_from_garray (new_points);

    g_array_free (new_points, TRUE);
    return result;
}

pointlist2d_t *
geom_polyline_resample_uniform_npoints (const pointlist2d_t *line,
        int npoints)
{
    double total_len = geom_polyline_length_2d (line);
    pointlist2d_t *result = pointlist2d_new (npoints);
    double spacing = total_len / (npoints - 1);
    result->points[0] = line->points[0];
    int ind = 0;
    double alpha = 0;
    for (int i=1; i<npoints; i++) {
        geom_polyline_advance_point_by_dist (line, ind, alpha,
                spacing, &ind, &alpha, &result->points[i]);
    }
    return result;
}

int
geom_ray_triangle_intersect_3d (const point3d_t *ray_start, 
        const vec3d_t *ray_dir, const point3d_t *triangle_A,
        const point3d_t *triangle_B, const point3d_t *triangle_C, 
        point3d_t *result, double *result_t)
{
    // compute the triangle normal vector
    vec3d_t ab = { 
        triangle_B->x - triangle_A->x,
        triangle_B->y - triangle_A->y,
        triangle_B->z - triangle_A->z
    };
    vec3d_t ac = { 
        triangle_C->x - triangle_A->x,
        triangle_C->y - triangle_A->y,
        triangle_C->z - triangle_A->z
    };
    vec3d_t n;
    geom_vec_cross_3d (&ab, &ac, &n);

    // intersect the ray with the triangle's plane
    point3d_t isect;
    double t;
    int intersects_plane = geom_ray_plane_intersect_3d(ray_start, ray_dir,
            triangle_A, &n, &isect, &t);
    if(!intersects_plane || t < 0) {
        return 0;
    }

#if 0
    glColor3f(0, 1, 0);
    glBegin(GL_LINES);
    glVertex3f(triangle_A->x, triangle_A->y, triangle_A->z);
    glVertex3f(triangle_B->x, triangle_B->y, triangle_B->z);
    glVertex3f(triangle_A->x, triangle_A->y, triangle_A->z);
    glVertex3f(triangle_C->x, triangle_C->y, triangle_C->z);

    glVertex3f(triangle_A->x, triangle_A->y, triangle_A->z);
    glVertex3f(triangle_A->x + n.x,
            triangle_A->y + n.y,
            triangle_A->z + n.z);
    glVertex3f(triangle_A->x, triangle_A->y, triangle_A->z);
    glVertex3f(isect.x, isect.y, isect.z);
    glEnd();
#endif

    point3d_t P = { 
        isect.x - triangle_A->x,
        isect.y - triangle_A->y,
        isect.z - triangle_A->z
    };

    // project the triangle and the intersected point onto an axis-aligned
    // plane.  Choose the plane based on the normal vector of the triangle.
    point2d_t b, c, p;
    if(n.x > n.y && n.x > n.z) {
        b.x = ab.y;           b.y = ab.z;
        c.x = ac.y;           c.y = ac.z;
        p.x = P.y;            p.y = P.z;
    } else if(n.y > n.x && n.y > n.z) {
        b.x = ab.x;           b.y = ab.z;
        c.x = ac.x;           c.y = ac.z;
        p.x = P.x;            p.y = P.z;
    } else {
        b.x = ab.x;           b.y = ab.y;
        c.x = ac.x;           c.y = ac.y;
        p.x = P.x;            p.y = P.y;
    }

    // now test if the 2d point lies in the 2d triangle
    double bycx = b.y * c.x;
    double bxcy = b.x * c.y;
    double u = (p.y * c.x - p.x * c.y) / (bycx - bxcy);
    double v = (p.y * b.x - p.x * b.y) / (bxcy - bycx);
#if 0
    glPointSize(10);
    glColor3f(1, 0, 0);
    glBegin(GL_POINTS);
    glVertex3d(triangle_A->x + u * ab.x + v * ac.x,
               triangle_A->y + u * ab.y + v * ac.y,
               triangle_A->z + u * ab.z + v * ac.z);
    glEnd();
#endif
    if(u < 0)
        return 0;
    if(v < 0 || u+v > 1)
        return 0;

    if(result_t) {
        *result_t = t;
    }
    if(result) {
        result->x = ray_start->x + t * ray_dir->x;
        result->y = ray_start->y + t * ray_dir->y;
        result->z = ray_start->z + t * ray_dir->z;
    }
    return 1;
}
double
geom_ray_axis_aligned_box_intersect_3d (const point3d_t *ray_point, 
        const vec3d_t *ray_dir,
        const point3d_t *box_center, const point3d_t *box_size,
        vec3d_t *N)
{
    double t_min = INFINITY;
    double t;

    double half_x = box_size->x / 2;
    double half_y = box_size->y / 2;
    double half_z = box_size->z / 2;

    if (N) { N->x = N->y = N->z = 0; }

    // X planes
    if (ray_dir->x != 0) {
        t = (box_center->x+half_x - ray_point->x) / ray_dir->x;
        double oy = ray_point->y + ray_dir->y * t - box_center->y;
        double oz = ray_point->z + ray_dir->z * t - box_center->z;
        if (t > 0 && fabs (oy) <= half_y && fabs (oz) <= half_z &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = 1; N->y = 0; N->z = 0; }
        }

        t = (box_center->x-half_x - ray_point->x) / ray_dir->x;
        oy = ray_point->y + ray_dir->y * t - box_center->y;
        oz = ray_point->z + ray_dir->z * t - box_center->z;
        if (t > 0 && fabs (oy) <= half_y && fabs (oz) <= half_z &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = -1; N->y = 0; N->z = 0; }
        }
    }

    // Y planes
    if (ray_dir->y != 0) {
        t = (box_center->y+half_y - ray_point->y) / ray_dir->y;
        double ox = ray_point->x + ray_dir->x * t - box_center->x;
        double oz = ray_point->z + ray_dir->z * t - box_center->z;
        if (t > 0 && fabs (ox) <= half_x && fabs (oz) <= half_z &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = 0; N->y = 1; N->z = 0; }
        }

        t = (box_center->y-half_y - ray_point->y) / ray_dir->y;
        ox = ray_point->x + ray_dir->x * t - box_center->x;
        oz = ray_point->z + ray_dir->z * t - box_center->z;
        if (t > 0 && fabs (ox) <= half_x && fabs (oz) <= half_z &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = 0; N->y = -1; N->z = 0; }
        }
    }

    // Z planes
    if (ray_dir->z != 0) {
        t = (box_center->z+half_z - ray_point->z) / ray_dir->z;
        double ox = ray_point->x + ray_dir->x * t - box_center->x;
        double oy = ray_point->y + ray_dir->y * t - box_center->y;
        if (t > 0 && fabs (ox) <= half_x && fabs (oy) <= half_y &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = 0; N->y = 0; N->z = 1; }
        }

        t = (box_center->z-half_z - ray_point->z) / ray_dir->z;
        ox = ray_point->x + ray_dir->x * t - box_center->x;
        oy = ray_point->y + ray_dir->y * t - box_center->y;
        if (t > 0 && fabs (ox) <= half_x && fabs (oy) <= half_y &&
                isless (t, t_min)) {
            t_min = t; 
            if (N) { N->x = 0; N->y = 0; N->z = -1; }
        }
    }

    return t_min;
}


pointlist2d_t * 
geom_compute_ellipse_points (double cx, double cy, 
        double a, double b, double angle, int npoints)
{
    pointlist2d_t *result = pointlist2d_new (npoints);
    
    double beta = angle;
    double sinbeta = sin(beta), cosbeta = cos(beta);

    for (int i=0; i<npoints; i++) {
        double alpha = i * 2 * M_PI / npoints;
        double cosalpha = cos(alpha), sinalpha = sin(alpha);

        double x = cx + b * cosalpha * cosbeta - a * sinalpha * sinbeta;
        double y = cy + b * cosalpha * sinbeta + a * sinalpha * cosbeta;

        result->points[i].x = x;
        result->points[i].y = y;
    }
    return result;
}

int 
geom_polyline_estimate_tangent_at (const pointlist2d_t *curve, int ind,
        double alpha, vec2d_t *result)
{
    if (curve->npoints < 2) {
        result->x = NAN;
        result->y = NAN;
        return -1;
    }
    if (0 == alpha) {
        if (ind == 0) {
            result->x = curve->points[1].x - curve->points[0].x;
            result->y = curve->points[1].y - curve->points[0].y;
        } else if (ind == curve->npoints - 1) {
            int n = curve->npoints;
            result->x = curve->points[n-1].x - curve->points[n-2].x;
            result->y = curve->points[n-1].y - curve->points[n-2].y;
        } else {
            vec2d_t tn = {
                curve->points[ind+1].x - curve->points[ind].x,
                curve->points[ind+1].y - curve->points[ind].y
            };
            vec2d_t tp = {
                curve->points[ind].x - curve->points[ind-1].x,
                curve->points[ind].y - curve->points[ind-1].y
            };
            result->x = (tn.x + tp.x) / 2;
            result->y = (tn.y + tp.y) / 2;
        }
    } else {
        result->x = curve->points[ind+1].x - curve->points[ind].x;
        result->y = curve->points[ind+1].y - curve->points[ind].y;
    }
    geom_vec_normalize_2d (result);
    return 0;
}

double
geom_polyline_estimate_curvature_at (const pointlist2d_t *curve, int i)
{
    if (curve->npoints < 2) return NAN;
    vec2d_t tan_i, tan_n, tan_p;
    geom_polyline_estimate_tangent_at (curve, i, 0, &tan_i);
    int have_prev_tan = 
        !geom_polyline_estimate_tangent_at (curve, i-1, 0, &tan_p);
    int have_next_tan = 
        !geom_polyline_estimate_tangent_at (curve, i+1, 0, &tan_n);

    double dtheta = 0;
    double ddist = 0;
    if (have_prev_tan) {
        dtheta = geom_vec_vec_angle_2d (&tan_p, &tan_i);
        ddist = geom_point_point_distance_2d (&curve->points[i],
                &curve->points[i-1]);
    } 
    if (have_next_tan) {
        dtheta += geom_vec_vec_angle_2d (&tan_i, &tan_n);
        ddist += geom_point_point_distance_2d (&curve->points[i],
                &curve->points[i+1]);
    }
    return dtheta / ddist;
}

pointlist2d_t *
pointlist2d_new_copy_subsection (const pointlist2d_t *line, int start_ind,
        double start_alpha, int end_ind, double end_alpha)
{
    if (start_alpha == 1) { start_ind++; start_alpha = 0; }
    if (start_ind < 0 || end_ind >= line->npoints ||
            (end_ind == line->npoints-1 && end_alpha >0) ||
            start_alpha < 0 || start_alpha > 1 || 
            end_alpha < 0 || end_alpha > 1 ||
            start_ind > end_ind ||
            (start_ind == end_ind && start_alpha >= end_alpha)) return NULL;

    point2d_t start_pt = { 0, 0 };
    geom_polyline_ind_alpha_point_2d (line, start_ind, start_alpha, &start_pt);
    int npoints = end_ind - start_ind + 1;
    if (end_alpha > 0) npoints++;
    pointlist2d_t *result = pointlist2d_new (npoints);
    result->points[0] = start_pt;
    for (int i=1; i<end_ind - start_ind + 1; i++) {
        result->points[i] = line->points[i+start_ind];
    }
    if (end_alpha > 0) {
        geom_polyline_ind_alpha_point_2d (line, end_ind, end_alpha, 
                &result->points[npoints-1]);
    }
    return result;
}
