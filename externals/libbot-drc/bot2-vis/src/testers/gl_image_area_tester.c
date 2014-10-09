#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <ctype.h>
#include <stdint.h>

#include <glib.h>

#include <bot_vis/bot_vis.h>

typedef struct _app_params_t
{ 
    int foo;
    char *bar;
    char *filename;
} app_params_t;

typedef struct _app_t 
{
    int foo;

    uint8_t *img;
    int img_width;
    int img_height;
    int img_stride;

    BotGtkGlImageArea *gl_area;
    BotGtkParamWidget *param_widget;
} app_t;

static void app_destroy (app_t *self);
static void setup_gui (app_t *self);
static int load_ppm (app_t *self, const char *filename);

static gboolean
on_gl_area_expose (GtkWidget * widget, GdkEventExpose * event, void* user_data)
{
    app_t *self = (app_t*) user_data;
    if (bot_gtk_param_widget_get_bool (self->param_widget, "Draw X")) {
        glColor3f (0, 1, 0);
        glBegin (GL_LINES);
        glVertex2f (0, 0);
        glVertex2f (self->img_width, self->img_height);
        glVertex2f (self->img_width, 0);
        glVertex2f (0, self->img_height);
        glEnd ();
    }
    return FALSE;
}

static void
on_param_changed (BotGtkParamWidget *pw, const char *name, void *user_data)
{
    app_t *self = (app_t*) user_data;
    bot_gtk_gl_drawing_area_invalidate (BOT_GTK_GL_DRAWING_AREA (self->gl_area));
}

static app_t *
app_create (const app_params_t *params)
{
    app_t *self = (app_t*) calloc (1, sizeof (app_t));

    self->foo = params->foo;

    if (0 != load_ppm (self, params->filename)) goto fail;

    setup_gui (self);

    bot_gtk_gl_image_area_set_image_format (self->gl_area,
            self->img_width, self->img_height, GL_RGB);
    bot_gtk_gl_image_area_upload_image (self->gl_area, self->img, 
            self->img_stride);

    return self;
fail:
    app_destroy (self);
    return NULL;
}

static void
app_destroy (app_t *self)
{
    if (self->img) free (self->img);

    free (self);
}

static void
setup_gui (app_t *self)
{
    GtkWidget *main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    g_signal_connect (G_OBJECT (main_window), "delete_event", gtk_main_quit,
            NULL);
    g_signal_connect (G_OBJECT (main_window), "destroy", gtk_main_quit,
            NULL);

    GtkWidget *hbox = gtk_hbox_new (FALSE, FALSE);
    gtk_container_add (GTK_CONTAINER (main_window), hbox);

    self->gl_area = BOT_GTK_GL_IMAGE_AREA (bot_gtk_gl_image_area_new ());
    g_signal_connect (G_OBJECT (self->gl_area), "expose-event", 
            G_CALLBACK (on_gl_area_expose), self);
    gtk_widget_set_size_request (GTK_WIDGET (self->gl_area), 
            self->img_width, self->img_height);

    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (self->gl_area), TRUE, TRUE,
            0);

    self->param_widget = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (self->param_widget), 
            FALSE, TRUE, 0);
    bot_gtk_param_widget_add_booleans (self->param_widget, 0, 
            "Draw X", 1, NULL);
    g_signal_connect (G_OBJECT (self->param_widget), "changed", 
                G_CALLBACK (on_param_changed), self);

    gtk_widget_show_all (main_window);
}

static int
skip_whitespace_and_comments (FILE *fp)
{
    int c = fgetc(fp);
    while (isspace(c) && c != EOF && c == '#') {
        do {
            c = fgetc (fp);
        } while (isspace(c) && c != EOF);

        if (c == EOF) {
            fprintf(stderr, "unexpected EOF\n"); return EOF;
        }
        // ignore comments
        if (c == '#') {
            do {
                c = fgetc(fp);
            } while (c != EOF && c != '\n');
            if (c == EOF) {
                fprintf(stderr, "unexpected EOF\n"); return EOF;
            }
            c = fgetc(fp);
        }
    }
    ungetc(c, fp);
    return 0;
}

static int
read_header (FILE *fp, const char *magic, int *width, int *height, 
        int *maxval)
{
    char m[3] = { 0 };
    int ignored = fread (m, sizeof(m)-1, 1, fp);
    if (strcmp(m, magic)) {
        fprintf(stderr, "bad magic [%s]\n", m); return -1;
    }
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", width)) return -1;
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", height)) return -1;
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", maxval)) return -1;
    if (EOF == fgetc(fp)) return -1;
    return 0;
}

static int ppm_read (FILE *fp, uint8_t **pixels, 
        int *width, int *height, 
        int *rowstride)
{
    int maxval;
    int i;
    int nread;
    int w, h, rs;
    
    if (0 != read_header (fp, "P6", &w, &h, &maxval)) {
        fprintf(stderr, "that doesn't look like a PPM file!\n");
        return -1;
    }

    rs = w * 3;
    rs += rs % 4; // align each row on a 32-bit boundary

    *pixels = (unsigned char*) calloc (h, rs);
    for (i=0; i<h; i++) {
        nread = fread (*pixels + i*rs, w * 3, 1, fp);
        if (1 != nread) {
            perror("fread ppm");
            return -1;
        }
    }

    *height = h;
    *width = w;
    *rowstride = rs;

    return 0;
}

static int
load_ppm (app_t *self, const char *filename) 
{
    FILE *fp = fopen (filename, "rb");
    if (!fp) { perror ("fopen"); return -1; }
    int status = ppm_read (fp, &self->img, &self->img_width, &self->img_height,
            &self->img_stride);
    fclose (fp);
    return status;
}

static void
app_run (app_t *self)
{
    gtk_main ();
}

int main (int argc, char **argv)
{
    gtk_init (&argc, &argv);

    app_params_t params = {
        .foo = 1,
        .bar = NULL,
        .filename = NULL
    };

    GOptionEntry options[] = 
    {
        { "foo", 'f', 0, G_OPTION_ARG_INT, &params.foo, 
            "Example int parameter", "F" },
        { "bar", 'b', 0, G_OPTION_ARG_STRING, &params.filename, 
            "Example filename parameter", "B" },
        { NULL }
    };

    GError *opt_error = NULL;
    GOptionContext *opt_parser;

    opt_parser = g_option_context_new (" - BotGtkGlImageArea tester");
    g_option_context_add_main_entries (opt_parser, options, NULL);
    g_option_context_parse (opt_parser, &argc, &argv, &opt_error);
    g_option_context_free (opt_parser);

    if (argc < 2) {
        fprintf (stderr, "must specify filename\n");
        return 0;
    }
    params.filename = strdup (argv[1]);

    app_t *app = app_create (&params);
    if (app) {
        app_run (app);
        app_destroy (app);
    }

    return 0;
}
