#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "gl_util.h"
#include "scrollplot2d.h"

typedef struct __plot2d_point {
    double x;
    double y;
} _plot2d_point_t;

typedef struct __plot2d {
    GQueue *points;
    double rgba[4];
    char *name;
    int max_points;
} _plot2d_t;

struct _BotGlScrollPlot2d {
    GHashTable *plots;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double bg_rgba[4];
    double border_rgba[4];
    double text_rgba[4];

    char *title;
    int show_title;

    int show_ylim;
    BotGlScrollPlot2dLegendLocation show_legend;

    void *text_font;
    int text_height;
};

static inline int _set_color (double rgba[4], double r, double g, double b,
        double a) {
    if (r < 0 || g < 0 || b < 0 || r > 1 || g > 1 || b > 1 || a < 0 || a > 1) {
        g_warning ("BotGlScrollPlot2d refusing to set invalid color "
                "%f, %f, %f, %f\n",
                r, g, b, a);
        return -1;
    }
    rgba[0] = r;
    rgba[1] = g;
    rgba[2] = b;
    rgba[3] = a;
    return 0;
}

static inline void _gl_setcolor (double rgba[4])
{
    glColor4f (rgba[0], rgba[1], rgba[2], rgba[3]);
}

static void
__get_all_vals_helper (gpointer key, gpointer value, gpointer user_data)
{
    GPtrArray *vals = (GPtrArray*) user_data;
    g_ptr_array_add(vals, value);    
}

static GPtrArray * 
__hash_table_get_vals (GHashTable *hash_table)
{
    GPtrArray *vals = g_ptr_array_sized_new(g_hash_table_size(hash_table));
    g_hash_table_foreach (hash_table, __get_all_vals_helper, vals);
    return vals;
}

static _plot2d_t * _plot2d_new (const char *name)
{
    _plot2d_t *self = g_slice_new0 (_plot2d_t);
    self->name = strdup (name);
    self->points = g_queue_new ();
    self->rgba[0] = 1;
    self->rgba[1] = 1;
    self->rgba[2] = 1;
    self->rgba[3] = 1;
    self->max_points = 1000;
    return self;
}

static void
_plot2d_free (_plot2d_t *self)
{
    if (self->points) {
        while (! g_queue_is_empty (self->points)) {
            g_slice_free (_plot2d_point_t, g_queue_pop_head (self->points));
        }
        g_queue_free (self->points);
        self->points = NULL;
    }
    if (self->name) free (self->name);
    g_slice_free (_plot2d_t, self);
}

BotGlScrollPlot2d * 
bot_gl_scrollplot2d_new ()
{
    BotGlScrollPlot2d *self = g_slice_new0 (BotGlScrollPlot2d);
    self->x_min = 0;
    self->x_max = 1;
    self->bg_rgba[0] = 0;
    self->bg_rgba[1] = 0;
    self->bg_rgba[2] = 0;
    self->bg_rgba[3] = 1;

    self->border_rgba[0] = 0;
    self->border_rgba[1] = 0;
    self->border_rgba[2] = 0;
    self->border_rgba[3] = 0;

    self->text_rgba[0] = 1;
    self->text_rgba[1] = 1;
    self->text_rgba[2] = 1;
    self->text_rgba[3] = 1;

    self->title = NULL;
    self->show_title = 1;

    self->show_ylim = 1;

    self->show_legend = BOT_GL_SCROLLPLOT2D_HIDDEN;

    self->text_font = GLUT_BITMAP_8_BY_13;
    self->text_height = 13;

    self->plots = g_hash_table_new_full (g_str_hash, g_str_equal,
            NULL, (GDestroyNotify) _plot2d_free);
    return self;
}

void
bot_gl_scrollplot2d_free (BotGlScrollPlot2d *self)
{
    memset (self, 0, sizeof (self));
    g_slice_free (BotGlScrollPlot2d, self);
}

int 
bot_gl_scrollplot2d_set_title (BotGlScrollPlot2d *self, const char *title)
{
    if (self->title) free (self->title);
    self->title = strdup (title);
    return 0;
}

void
bot_gl_scrollplot2d_set_show_title (BotGlScrollPlot2d *self, int val)
{
    self->show_title = val;
}

int 
bot_gl_scrollplot2d_add_plot (BotGlScrollPlot2d *self, const char *name,
        int max_points)
{
    if (max_points <= 0) {
        g_warning ("BotGlScrollPlot2d: bad max_points (%d), forcing to 1000\n",
                max_points);
        max_points = 1000;
    }

    _plot2d_t *existing = g_hash_table_lookup (self->plots, name);
    if (existing) {
        g_warning ("BotGlScrollPlot2d: replacing existing plot %s\n", name);
    }

    _plot2d_t *newplot = _plot2d_new (name);
    newplot->max_points = max_points;
    g_hash_table_replace (self->plots, newplot->name, newplot);
    return 0;
}

int 
bot_gl_scrollplot2d_remove_plot (BotGlScrollPlot2d *self, const char *name)
{
    gboolean result = g_hash_table_remove (self->plots, name);
    if (result) return 0;
    return -1;
}

int
bot_gl_scrollplot2d_add_point (BotGlScrollPlot2d *self, const char *name,
        double x, double y)
{
    _plot2d_t *plot = g_hash_table_lookup (self->plots, name);
    if (!plot) return -1;

    while (! g_queue_is_empty (plot->points)) {
        _plot2d_point_t *last_pt = 
            (_plot2d_point_t*) g_queue_peek_tail (plot->points);
        if (last_pt->x > x) {
            g_slice_free (_plot2d_point_t, last_pt);
            g_queue_pop_tail (plot->points);
        } else break;
    }

    _plot2d_point_t *pt = g_slice_new (_plot2d_point_t);
    pt->x = x;
    pt->y = y;
    g_queue_push_tail (plot->points, pt);

    while (g_queue_get_length (plot->points) > plot->max_points) {
        g_slice_free (_plot2d_point_t, g_queue_pop_head (plot->points));
    }
    return 0;
}

int 
bot_gl_scrollplot2d_set_color (BotGlScrollPlot2d *self, const char *name, 
        double r, double g, double b, double a)
{
    _plot2d_t *plot = g_hash_table_lookup (self->plots, name);
    if (!plot) return -1;
    return _set_color (plot->rgba, r, g, b, a);
}

int bot_gl_scrollplot2d_set_text_color (BotGlScrollPlot2d *self, 
        double r, double g, double b, double a)
{ return _set_color (self->text_rgba, r, g, b, a); }

int bot_gl_scrollplot2d_set_bgcolor (BotGlScrollPlot2d *self, 
        double r, double g, double b, double a)
{ return _set_color (self->bg_rgba, r, g, b, a); }

int bot_gl_scrollplot2d_set_border_color (BotGlScrollPlot2d *self, 
        double r, double g, double b, double a)
{ return _set_color (self->border_rgba, r, g, b, a); }

int 
bot_gl_scrollplot2d_set_show_legend (BotGlScrollPlot2d *self,
        BotGlScrollPlot2dLegendLocation where)
{ self->show_legend = where; return 0; }

int
bot_gl_scrollplot2d_set_xlim (BotGlScrollPlot2d *self, double xmin, double xmax)
{
    if (xmin >= xmax) {
        g_warning ("BotGlScrollPlot2d refusing to set xmin, max to %f, %f\n",
                xmin, xmax);
        return -1;
    }

    self->x_min = xmin;
    self->x_max = xmax;
    return 0;
}

int
bot_gl_scrollplot2d_set_ylim (BotGlScrollPlot2d *self, double ymin, double ymax)
{
    if (ymin >= ymax) {
        g_warning ("BotGlScrollPlot2d refusing to set ymin, max to %f, %f\n",
                ymin, ymax);
        return -1;
    }

    self->y_min = ymin;
    self->y_max = ymax;
    return 0;
}

static void
_plot2d_setup_gl_line_style (_plot2d_t *plot)
{
    glColor4f (plot->rgba[0], plot->rgba[1], plot->rgba[2], plot->rgba[3]);
}

static void
_plot2d_render_window (void *key, void *value, void *user_data)
{
    _plot2d_t *plot = (_plot2d_t*) value;
    BotGlScrollPlot2d *sp = (BotGlScrollPlot2d*) user_data;

    // remove all points that aren't visible
    while (! g_queue_is_empty (plot->points)) {
        _plot2d_point_t *pt = 
            (_plot2d_point_t*)g_queue_peek_head (plot->points);
        if (pt->x < sp->x_min) {
            g_queue_pop_head (plot->points);
            g_slice_free (_plot2d_point_t, pt);
        } else {
            break;
        }
    }

    double size_x = sp->x_max - sp->x_min;
    double size_y = sp->y_max - sp->y_min;

    glBegin (GL_LINES);
    _plot2d_setup_gl_line_style (plot);

    GList *piter = g_queue_peek_head_link (plot->points);
    for (; piter && piter->next; piter=piter->next) {
        _plot2d_point_t *pt = (_plot2d_point_t*) piter->data;
        _plot2d_point_t *next_pt = (_plot2d_point_t*) piter->next->data;

        double py = MIN (sp->y_max, MAX (sp->y_min, pt->y));
        double npy = MIN (sp->y_max, MAX (sp->y_min, next_pt->y));

        double sx = (pt->x - sp->x_min) / size_x;
        double sy = 1 - (py - sp->y_min) / size_y;
        double snx = (next_pt->x - sp->x_min) / size_x;
        double sny = 1 - (npy - sp->y_min) / size_y;

        glVertex2f (sx, sy);
        glVertex2f (snx, sny);
    }
    glEnd ();
}

static void
_draw_text (BotGlScrollPlot2d *self, const char *text)
{
    for (int i=0; i<strlen(text); i++) {
        glutBitmapCharacter (self->text_font, text[i]);
    }
}

static void
draw_title (BotGlScrollPlot2d *self, int x, int y, int width, int height)
{
    if (!self->title || !self->show_title) return;

    _gl_setcolor (self->text_rgba);
    int title_width_pixels = glutBitmapLength(self->text_font, 
            (unsigned char *) self->title);
    glRasterPos2f(x + width / 2 - title_width_pixels / 2, 
            y + self->text_height - 1);
    _draw_text (self, self->title);
}

static void
draw_axis_labels (BotGlScrollPlot2d *self, int x, int y, int width, int height)
{
    if (!self->show_ylim) return;

    _gl_setcolor (self->text_rgba);
    glRasterPos2f(x + 1, y + self->text_height - 1);
    char text[20];
    snprintf(text, sizeof(text), "%.2g", self->y_max);
    _draw_text (self, text);

    glRasterPos2f (x + 1, y + height - 1);
    snprintf(text, sizeof(text), "%.2g", self->y_min);
    _draw_text (self, text);
}

static void
draw_legend (BotGlScrollPlot2d *self, int x, int y, int width, int height)
{
    if (self->show_legend == BOT_GL_SCROLLPLOT2D_HIDDEN) return;

    // compute the width and height of the legend box
    int max_label_width = 0;
    GPtrArray * all_plots = __hash_table_get_vals (self->plots);
    for (int i=0; i<all_plots->len; i++) {
        _plot2d_t *plot = (_plot2d_t*) g_ptr_array_index (all_plots, i);
        int label_width = glutBitmapLength (self->text_font, 
                (unsigned char*) plot->name);
        if (label_width > max_label_width) max_label_width = label_width;
    }
    int smallbox_width = 20;
    int smallbox_height = 15;
    int sb_label_padding = 5;
    int row_height = MAX (self->text_height, smallbox_height) + 2;
    int row_width = smallbox_width + max_label_width + sb_label_padding;
    int legend_width = row_width + 2;
    int legend_height = row_height * all_plots->len;

    // now where does the legend go?
    int legend_x = 0;
    int legend_y = 0;
    switch (self->show_legend) {
        case BOT_GL_SCROLLPLOT2D_TOP_LEFT:
            legend_x = x;
            legend_y = y;
            break;
        case BOT_GL_SCROLLPLOT2D_TOP_RIGHT:
            legend_x = x + width - legend_width;
            legend_y = y;
            break;
        case BOT_GL_SCROLLPLOT2D_BOTTOM_LEFT:
            legend_x = x;
            legend_y = y + height - legend_height;
            break;
        case BOT_GL_SCROLLPLOT2D_BOTTOM_RIGHT:
            legend_x = x + width - legend_y;
            legend_y = y + height - legend_height;
            break;
        default:
            g_warning ("BotGlScrollPlot2d: invalid legend location %d\n", 
                    self->show_legend);
            goto done;
    }

    if (legend_x < x || legend_width > width) goto done;
    if (legend_y < y) legend_y = y;

    int row_x = legend_x;
    int row_y = legend_y;
    for (int i=0; i<all_plots->len; i++) {
        _plot2d_t *plot = (_plot2d_t*) g_ptr_array_index (all_plots, i);

        if (row_y + row_height > y + height) break;
        _plot2d_setup_gl_line_style (plot);

        glBegin (GL_LINES);
        glVertex2f (legend_x + 2, row_y + smallbox_height / 2);
        glVertex2f (legend_x + smallbox_width, row_y + smallbox_height / 2);
        glEnd ();

        _gl_setcolor (plot->rgba);
        glRasterPos2f (row_x + smallbox_width + sb_label_padding, 
                row_y + self->text_height - 1);
        _draw_text (self, plot->name);
        row_y += row_height;
    }

    glBegin (GL_LINE_LOOP);
    glColor3f (1, 1, 1);
    glVertex2f (legend_x, legend_y);
    glVertex2f (legend_x + legend_width, legend_y);
    glVertex2f (legend_x + legend_width, row_y);
    glVertex2f (legend_x, row_y);
    glEnd ();

done:
    g_ptr_array_free (all_plots, TRUE);
}

void 
bot_gl_scrollplot2d_gl_render_at_window_pos (BotGlScrollPlot2d *self,
        int x, int y, int width, int height)
{
    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glDisable (GL_DEPTH_TEST);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // figure out how big the drawing window (viewport) is
    GLint viewport[4];
    glGetIntegerv (GL_VIEWPORT, viewport);

    // transform into window coordinates, where <0, 0> is the top left corner
    // of the window and <viewport[2], viewport[3]> is the bottom right corner
    // of the window
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, viewport[2], 0, viewport[3]);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, viewport[3], 0);
    glScalef(1, -1, 1);

    // transform into normalized plot coordinates where <0, 0> is the top left
    // corner of the plot, and <1, 1> is the bottom right corner of the plot.
    glPushMatrix ();
    glTranslatef (x, y, 1);
    glScalef (width, height, 1);

    // draw background
    glColor4f (self->bg_rgba[0], self->bg_rgba[1], self->bg_rgba[2], 
            self->bg_rgba[3]);
    glBegin(GL_QUADS);
    glVertex2f (0, 0);
    glVertex2f (1, 0);
    glVertex2f (1, 1);
    glVertex2f (0, 1);
    glEnd();        

    // draw plots
    g_hash_table_foreach (self->plots, _plot2d_render_window, self);

    // draw border
    glColor4f (self->border_rgba[0], self->border_rgba[1], 
            self->border_rgba[2], self->border_rgba[3]);
    glBegin (GL_LINE_LOOP);
    glVertex2f (0, 0);
    glVertex2f (1, 0);
    glVertex2f (1, 1);
    glVertex2f (0, 1);
    glEnd ();

    // return to window coordinates
    glPopMatrix ();

    draw_title (self, x, y, width, height);
    draw_axis_labels (self, x, y, width, height);
    draw_legend (self, x, y, width, height);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib ();
}
