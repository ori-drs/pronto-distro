#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define GL_GLEXT_PROTOTYPES 1
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#include "gl_image_area.h"

#define err(args...) fprintf (stderr, args)
#define errl(args...) fprintf (stderr, args)

static void bot_gtk_gl_image_area_finalize (GObject *obj);

G_DEFINE_TYPE (BotGtkGlImageArea, bot_gtk_gl_image_area, BOT_GTK_TYPE_GL_DRAWING_AREA);

static gboolean on_gl_expose (GtkWidget * widget, GdkEventExpose * event, 
        void* user_data);
static gboolean on_gl_expose_after (GtkWidget * widget, GdkEventExpose * event, 
        void* user_data);

static void
bot_gtk_gl_image_area_init (BotGtkGlImageArea *self)
{
    self->target = 0;
    self->int_format = 0;
    self->format = 0;
    self->texname = 0;
    self->width = 0;
    self->height = 0;

    self->texc_width = 0;
    self->texc_height = 0;

    self->pbo = 0;
    self->use_pbo = 0;
    self->max_data_size = 0;

    g_signal_connect (G_OBJECT (self), "expose-event",
            G_CALLBACK (on_gl_expose), self);
    g_signal_connect_after (G_OBJECT (self), "expose-event",
            G_CALLBACK (on_gl_expose_after), self);
}

static void
bot_gtk_gl_image_area_class_init (BotGtkGlImageAreaClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
    // add a class-specific destructor
    gobject_class->finalize = bot_gtk_gl_image_area_finalize;
}

// destructor (more or less)
static void
bot_gtk_gl_image_area_finalize (GObject *obj)
{
    BotGtkGlImageArea *self = BOT_GTK_GL_IMAGE_AREA (obj);
    if (self->texname) {
        glDeleteTextures (1, &self->texname);
    }
    if (self->pbo) {
        glDeleteBuffersARB (1, &self->pbo);
    }

    G_OBJECT_CLASS (bot_gtk_gl_image_area_parent_class)->finalize (obj);
}

GtkWidget *
bot_gtk_gl_image_area_new ()
{
    return GTK_WIDGET (g_object_new (BOT_GTK_TYPE_GL_IMAGE_AREA, NULL));
}

static gboolean 
on_gl_expose (GtkWidget * widget, GdkEventExpose * event, void* user_data)
{
    BotGtkGlImageArea *self = BOT_GTK_GL_IMAGE_AREA (user_data);

    // activate the opengl context
    bot_gtk_gl_drawing_area_set_context (BOT_GTK_GL_DRAWING_AREA (self));

    // setup opengl view and model matrices, and clear the opengl buffers
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho (0, self->width, self->height, 0, -1, 1);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    // abort if nothing to draw
    if (self->width <= 0 || self->height <= 0 || !self->texname) return FALSE;

    // draw the image
    glEnable (self->target);
    glBindTexture (self->target, self->texname);
    glBegin (GL_QUADS);
    glColor3f (1.0, 1.0, 1.0);
    glTexCoord2i (0, 0);
    glVertex2i (0, 0);
    glTexCoord2i (0, self->texc_height);
    glVertex2i (0, self->height);
    glTexCoord2i (self->texc_width, self->texc_height);
    glVertex2i (self->width, self->height);
    glTexCoord2i (self->texc_width, 0);
    glVertex2i (self->width, 0);
    glEnd ();
    glBindTexture (self->target, 0);
    glDisable (self->target);

    return FALSE;
}

static gboolean 
on_gl_expose_after (GtkWidget * widget, GdkEventExpose * event, void* user_data)
{
    BotGtkGlImageArea *self = BOT_GTK_GL_IMAGE_AREA (user_data);
    bot_gtk_gl_drawing_area_swap_buffers (BOT_GTK_GL_DRAWING_AREA (self));
    return FALSE;
}

static const char * _gl_format_str (GLenum format) {
    switch (format) {
        case GL_LUMINANCE: return "GL_LUMINANCE";
        case GL_RGB: return "GL_RGB";
        case GL_BGR: return "GL_BGR";
        case GL_RGBA: return "GL_RGBA";
        case GL_BGRA: return "GL_BGRA";
        default: return "UNKNOWN";
    }
}

static int _pixel_format_bpp (GLenum format) {
    switch (format) {
        case GL_LUMINANCE: return 8;
        case GL_RGB: return 24;
        case GL_BGR: return 24;
        case GL_RGBA: return 32;
        case GL_BGRA: return 32;
        default: return -1;
    }
}

int
bot_gtk_gl_image_area_set_image_format (BotGtkGlImageArea *self,
        int width, int height, GLenum format)
{
    if (format != GL_LUMINANCE && 
        format != GL_RGB && 
        format != GL_BGR && 
        format != GL_RGBA && 
        format != GL_BGRA)
    {
        errl ("Error: BotGtkGlImageArea does not support GL format %s\n",
                _gl_format_str (format));
        return -1;
    }

    int has_non_power_of_two = 0;
    int has_texture_rectangle = 0;
    int has_pbo = 0;

    const char * extstr = (const char *) glGetString (GL_EXTENSIONS);
    gchar ** exts = g_strsplit (extstr, " ", 0);
    int i;
    for (i = 0; exts[i]; i++) {
        gchar * ext = exts[i];
        if (!strcmp (ext, "GL_ARB_texture_non_power_of_two"))
            has_non_power_of_two = 1;
        if (!strcmp (ext, "GL_ARB_texture_rectangle"))
            has_texture_rectangle = 1;
        if (!strcmp (ext, "GL_ARB_pixel_buffer_object"))
            has_pbo = 1;
    }
    g_strfreev (exts);

    self->use_pbo = has_pbo;
    self->int_format = GL_RGBA8;
    self->width = width;
    self->height = height;

    if (has_non_power_of_two) {
        self->target = GL_TEXTURE_2D;
        self->texc_width = 1;
        self->texc_height = 1;
    }
    else if (has_texture_rectangle) {
        self->target = GL_TEXTURE_RECTANGLE_ARB;
        self->texc_width = width;
        self->texc_height = height;
    }
    else {
        fprintf (stderr, "Error: GL supports neither non-power-of-two nor "
                "texture-rectangle\n");
        return -1;
    }

    if (self->texname) {
        glDeleteTextures (1, &self->texname);
    }

    glGenTextures (1, &self->texname);

    self->max_data_size = width * height * 4;
    if (self->use_pbo) {
        if (self->pbo) {
            glDeleteBuffersARB (1, &self->pbo);
        }
        glGenBuffersARB (1, &self->pbo);
        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, self->pbo);
        glBufferDataARB (GL_PIXEL_UNPACK_BUFFER_ARB, self->max_data_size, NULL, 
                GL_STREAM_DRAW);
        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, 0);
    }

    self->format = format;

    return 0;
}

int
bot_gtk_gl_image_area_upload_image (BotGtkGlImageArea * self,
        const void *data, int row_stride)
{
    if (self->use_pbo && (row_stride * self->height) > self->max_data_size) {
        fprintf (stderr, "Error: gl_texture buffer (%d bytes) too small for "
                "texture (%d bytes)\n", self->max_data_size, 
                row_stride * self->height);
        return -1;
    }
    if (!data && !self->use_pbo) {
        fprintf (stderr, "Error: gl_texture data is NULL\n");
        return -1;
    }
    GLenum type = GL_UNSIGNED_BYTE;

    glBindTexture (self->target, self->texname);

    glTexParameterf (self->target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf (self->target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    if (row_stride % 2) {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    } else if (row_stride % 4) {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 2);
    } else {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 4);
    }

    glPixelStorei (GL_UNPACK_ROW_LENGTH, 
            row_stride * 8 / _pixel_format_bpp (self->format));
    if (self->use_pbo) {
        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, self->pbo);

        /* By setting data to NULL, we skip the memcpy and just re-upload
         * from the buffer object.  This can be useful to re-upload with
         * different PixelTransfer settings. */
        if (data) {
            uint8_t *buffer_data = 
                (uint8_t*) glMapBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB,
                        GL_WRITE_ONLY);
            memcpy (buffer_data, data, row_stride * self->height);
            glUnmapBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB);
        }

        glTexImage2D (self->target, 0, self->int_format, 
                self->width, self->height, 0,
                self->format, type, 0);

        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, 0);
    } else {
        glTexImage2D (self->target, 0, self->int_format, 
                self->width, self->height, 0,
                self->format, type, data);
    }
    glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture (self->target, 0);
    return 0;
}
