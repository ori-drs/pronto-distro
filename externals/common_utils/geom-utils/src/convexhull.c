#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <glib.h>

#include "convexhull.h"

//#define dbg(...) fprintf (stderr, __VA_ARGS__)
#define dbg(...)

/**
 * implementation of A. Melkman algorithm for computing the convex hull of a
 * simple polygon
 *
 * Avraham A. Melkman.  "On-line Construction of the Convex Hull of a Simple
 * Polyline" Information Processing Letters, vol 25. pp 11 -- 12, 1987.
 */
pointlist2d_t *
convexhull_simple_polygon_2d (const pointlist2d_t *input)
{
    dbg ("computing convex hull of %d points\n", input->npoints);
    for (int i=0; i<input->npoints; i++) {
        dbg ("  <%5.0f, %5.0f>\n", input->points[i].x, input->points[i].y);
    }
    GQueue *q = g_queue_new ();

    if (input->npoints < 3) return NULL;

    const point2d_t *v1, *v2, *v3;
    v1 = &input->points[0];
    v2 = &input->points[1];
    v3 = &input->points[2];

    if (geom_handedness_2d (v1, v2, v3) > 0) {
        g_queue_push_tail (q, (void*) v1);
        g_queue_push_tail (q, (void*) v2);
    } else {
        g_queue_push_tail (q, (void*) v2);
        g_queue_push_tail (q, (void*) v1);
    }
    g_queue_push_tail (q, (void*) v3);
    g_queue_push_head (q, (void*) v3);

    for (int curpoint = 3; curpoint < input->npoints; curpoint++) {
        dbg ("considering %d (%5.0f, %5.0f)... ", curpoint, 
                input->points[curpoint].x,
                input->points[curpoint].y);
        GList *head_link = g_queue_peek_head_link (q);
        GList *tail_link = g_queue_peek_tail_link (q);

        const point2d_t *db0 = (const point2d_t*) head_link->data;
        const point2d_t *db1 = (const point2d_t*) head_link->next->data;
        const point2d_t *dt0 = (const point2d_t*) tail_link->data;
        const point2d_t *dt1 = (const point2d_t*) tail_link->prev->data;
        const point2d_t *v = &input->points[curpoint];
        while ( curpoint < input->npoints && !(geom_handedness_2d (v, db0, db1) < 0 ||
                  geom_handedness_2d (dt1, dt0, v) < 0)) {
            dbg ("nope\n");
            curpoint++;
            v = &input->points[curpoint];

            if (curpoint < input->npoints) {
                dbg ("considering %d (%5.0f, %5.0f)... ", curpoint, 
                        input->points[curpoint].x,
                        input->points[curpoint].y);
            }
        }
        if (curpoint >= input->npoints) break;
        dbg ("okay\n");

        while ( !(geom_handedness_2d (dt1, dt0, v) > 0)) {
            dbg ("popping tail (%5.0f, %5.0f)\n", dt0->x, dt0->y);
            g_queue_pop_tail (q);

            tail_link = g_queue_peek_tail_link (q);
            dt0 = (const point2d_t*) tail_link->data;
            dt1 = (const point2d_t*) tail_link->prev->data;
        }
        g_queue_push_tail (q, (void*) v);

        while ( !(geom_handedness_2d (v, db0, db1) > 0)) {
            dbg ("popping head (%5.0f, %5.0f)\n", db0->x, db0->y);
            g_queue_pop_head (q);

            head_link = g_queue_peek_head_link (q);
            db0 = (const point2d_t*) head_link->data;
            db1 = (const point2d_t*) head_link->next->data;
        }
        g_queue_push_head (q, (void*) v);
    }

    g_queue_pop_tail (q);
    pointlist2d_t *result = pointlist2d_new_from_glist (g_queue_peek_head_link (q));
    g_queue_free (q);
    return result;
}

int
angle_sort_func_2d (const void *a_ptr, const void *b_ptr, void *user_data) 
{
    const point2d_t *a = *(const point2d_t**) a_ptr;
    const point2d_t *b = *(const point2d_t**) b_ptr;
    const point2d_t *pivot = (const point2d_t*) user_data;
    if (a == pivot) return -1;
    if (b == pivot) return 1;
    int h = geom_handedness_2d (pivot, a, b);
    if (h) return h;
    double da = geom_point_point_distance_squared_2d (pivot, a);
    double db = geom_point_point_distance_squared_2d (pivot, b);
    if (da < db) return -1;
    if (db < da) return 1;
    return 0;
}

/**
 * implementation of the graham scan method for computing the convex hull of a
 * set of points in general position in the 2D plane.
 */
pointlist2d_t * 
convexhull_graham_scan_2d (const pointlist2d_t *input)
{
    if (input->npoints < 3) return NULL;

    // find the point with smallest y (and then x) coordinate
    const point2d_t *pivot = NULL;
    for (int i=0; i<input->npoints; i++) {
        if (! pivot ||
            pivot->y > input->points[i].y ||
            (pivot->y == input->points[i].y && 
             pivot->x  < input->points[i].x)) {
            pivot = &input->points[i];
        }
    }

    GPtrArray *sorted = g_ptr_array_sized_new (input->npoints);
    for (int i=0; i<input->npoints; i++) {
        g_ptr_array_add (sorted, (void*) &input->points[i]);
    }
    g_ptr_array_sort_with_data (sorted, angle_sort_func_2d, (void*) pivot);
    int siter = 0;

    GQueue *hull = g_queue_new ();
    g_queue_push_tail (hull, g_ptr_array_index (sorted, siter)); siter++;
    g_queue_push_tail (hull, g_ptr_array_index (sorted, siter)); siter++;
    int hull_size = 2;

    for (; siter<sorted->len; siter++) {
        GList *tail_link = g_queue_peek_tail_link (hull);
        const point2d_t *candidate = 
            (const point2d_t*) g_ptr_array_index (sorted, siter);
        const point2d_t *top = (const point2d_t*) tail_link->data;
        const point2d_t *second = (const point2d_t*) tail_link->prev->data;
        int h = geom_handedness_2d (second, top, candidate);

        if (h < 0) {
            g_queue_push_tail (hull, (void*) candidate);
            hull_size++;
        } else if (h == 0) {
            g_queue_pop_tail (hull);
            g_queue_push_tail (hull, (void*) candidate);
        } else {
            while (h >= 0 && hull_size > 2) {
                g_queue_pop_tail (hull);
                hull_size--;
                tail_link = g_queue_peek_tail_link (hull);
                top = (const point2d_t*) tail_link->data;
                second = (const point2d_t*) tail_link->prev->data;
                h = geom_handedness_2d (second, top, candidate);
            }
            g_queue_push_tail (hull, (void*) candidate);
            hull_size++;
        }
    }

    pointlist2d_t *result = 
        pointlist2d_new_from_glist (g_queue_peek_head_link (hull));
    g_ptr_array_free (sorted, TRUE);
    g_queue_free (hull);
    return result;
}

int
angle_sort_func_2i (const void *a_ptr, const void *b_ptr, void *user_data) 
{
    const point2i_t *a = *(const point2i_t**) a_ptr;
    const point2i_t *b = *(const point2i_t**) b_ptr;
    const point2i_t *pivot = (const point2i_t*) user_data;
    if (a == pivot) return -1;
    if (b == pivot) return 1;
    int h = geom_handedness_2i (pivot, a, b);
    if (h) return h;
    int da = geom_point_point_distance_squared_2i (pivot, a);
    int db = geom_point_point_distance_squared_2i (pivot, b);
    if (da < db) return -1;
    if (db < da) return 1;
    return 0;
}

/**
 * implementation of the graham scan method for computing the convex hull of a
 * set of points in general position in the 2D plane.
 */
pointlist2i_t * 
convexhull_graham_scan_2i (const pointlist2i_t *input)
{
    if (input->npoints < 3) return NULL;

    // find the point with smallest y (and then x) coordinate
    const point2i_t *pivot = NULL;
    for (int i=0; i<input->npoints; i++) {
        if (! pivot ||
            pivot->y > input->points[i].y ||
            (pivot->y == input->points[i].y && 
             pivot->x  < input->points[i].x)) {
            pivot = &input->points[i];
        }
    }

    GPtrArray *sorted = g_ptr_array_sized_new (input->npoints);
    for (int i=0; i<input->npoints; i++) {
        g_ptr_array_add (sorted, (void*) &input->points[i]);
    }
    g_ptr_array_sort_with_data (sorted, angle_sort_func_2i, (void*) pivot);
    int siter = 0;

    GQueue *hull = g_queue_new ();
    g_queue_push_tail (hull, g_ptr_array_index (sorted, siter)); siter++;
    g_queue_push_tail (hull, g_ptr_array_index (sorted, siter)); siter++;
    int hull_size = 2;

    for (; siter<sorted->len; siter++) {
        GList *tail_link = g_queue_peek_tail_link (hull);
        const point2i_t *candidate = 
            (const point2i_t*) g_ptr_array_index (sorted, siter);
        const point2i_t *top = (const point2i_t*) tail_link->data;
        const point2i_t *second = (const point2i_t*) tail_link->prev->data;
        int h = geom_handedness_2i (second, top, candidate);

        if (h < 0) {
            g_queue_push_tail (hull, (void*) candidate);
            hull_size++;
        } else if (h == 0) {
            g_queue_pop_tail (hull);
            g_queue_push_tail (hull, (void*) candidate);
        } else {
            while (h >= 0 && hull_size > 2) {
                g_queue_pop_tail (hull);
                hull_size--;
                tail_link = g_queue_peek_tail_link (hull);
                top = (const point2i_t*) tail_link->data;
                second = (const point2i_t*) tail_link->prev->data;
                h = geom_handedness_2i (second, top, candidate);
            }
            g_queue_push_tail (hull, (void*) candidate);
            hull_size++;
        }
    }

    pointlist2i_t *result = 
        pointlist2i_new_from_glist (g_queue_peek_head_link (hull));
    g_ptr_array_free (sorted, TRUE);
    g_queue_free (hull);
    return result;
}
