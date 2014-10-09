#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#define GL_GLEXT_PROTOTYPES 1
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#include <GL/glx.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
#endif


#include <glib-object.h>

#include "fbgl_drawing_area.h"

enum {
    BUFFER_READY_SIGNAL,
    LAST_SIGNAL
};

static guint signals[LAST_SIGNAL] = { 0 };

#define BOT_FBGL_DRAWING_AREA_GET_PRIVATE(o) (G_TYPE_INSTANCE_GET_PRIVATE ((o), BOT_TYPE_FBGL_DRAWING_AREA, BotFbglDrawingAreaPrivate))
typedef struct _BotFbglDrawingAreaPrivate BotFbglDrawingAreaPrivate;

struct _BotFbglDrawingAreaPrivate {
    GLXContext context;
    uint framebuffer;
    uint renderbuffers[2];
    uint depthbuffers[2];
    uint output_pbos[2];
    int buffer_pending;
    int currbuf;
    GLenum format;
};

G_DEFINE_TYPE (BotFbglDrawingArea, bot_fbgl_drawing_area, G_TYPE_OBJECT);

static void bot_fbgl_drawing_area_finalize (GObject * obj);

static void
bot_fbgl_drawing_area_class_init (BotFbglDrawingAreaClass * klass)
{
    GObjectClass * gobject_class = G_OBJECT_CLASS (klass);

    gobject_class->finalize = bot_fbgl_drawing_area_finalize;

    g_type_class_add_private (gobject_class, sizeof (BotFbglDrawingAreaPrivate));

    /**
     * BotFbglDrawingArea::buffer-ready
     * @glarea: the BotFbglDrawingArea emitting the signal
     * @buffer: the pointer to the available data buffer
     *
     * The buffer-ready signal is emitted when the BotFbglDrawingArea has
     * a new buffer available for reading.  The data should be
     * completely processed or copied elsewhere during the signal handler.
     * Its pointer will become invalid at the conclusion of the handler.
     */
    signals[BUFFER_READY_SIGNAL] = g_signal_new("buffer-ready",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST,
            0, NULL, NULL, g_cclosure_marshal_VOID__POINTER,
            G_TYPE_NONE, 1, G_TYPE_POINTER);
}

static void
bot_fbgl_drawing_area_init (BotFbglDrawingArea * self)
{
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    priv->context = NULL;
}

static void bot_fbgl_drawing_area_finalize (GObject * obj)
{
    BotFbglDrawingArea * self = BOT_FBGL_DRAWING_AREA (obj);
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    glDeleteBuffersARB (2, priv->output_pbos);
    glDeleteRenderbuffersEXT (2, priv->renderbuffers);
    glDeleteFramebuffersEXT (1, &priv->framebuffer);

    G_OBJECT_CLASS (bot_fbgl_drawing_area_parent_class)->finalize(obj);
}

BotFbglDrawingArea *
bot_fbgl_drawing_area_new (gboolean new_context, int width, int height,
        GLenum format)
{
    // check that we can actually generate a renderbuffer of the requested size
    int rbsize_max;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE_EXT, &rbsize_max);
    if (rbsize_max < width || rbsize_max < height) {
        fprintf (stderr, "BotFbglDrawingArea: max buffer size is %dx%d, "
                "but requested %dx%d\n", rbsize_max, rbsize_max, width, height);
        return NULL;
    }
    if (width <= 0 || height <= 0) {
        fprintf (stderr, "BotFbglDrawingArea: invalid buffer size %dx%d\n",
                width, height);
        return NULL;
    }

    // construct a new object
    GObject * object = g_object_new (BOT_TYPE_FBGL_DRAWING_AREA, NULL);
    BotFbglDrawingArea * self = BOT_FBGL_DRAWING_AREA (object);
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    if (new_context) {
        fprintf (stderr, "BotFbglDrawingArea new context not implemented\n");
    }

    // setup the framebuffer object and renderbuffer objects
    glGenFramebuffersEXT (1, &priv->framebuffer);
    glGenRenderbuffersEXT (2, priv->renderbuffers);
    glGenRenderbuffersEXT (2, priv->depthbuffers);

    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, priv->framebuffer);
    int i;
    for (i = 0; i < 2; i++) {
        // attach a color buffer
        glBindRenderbufferEXT (GL_RENDERBUFFER_EXT, priv->renderbuffers[i]);
        glRenderbufferStorageEXT (GL_RENDERBUFFER_EXT, GL_RGBA8,
                width, height);
        glFramebufferRenderbufferEXT (GL_FRAMEBUFFER_EXT,
                GL_COLOR_ATTACHMENT0_EXT + i, GL_RENDERBUFFER_EXT,
                priv->renderbuffers[i]);

        // attach a depth buffer
        glBindRenderbufferEXT (GL_RENDERBUFFER_EXT, priv->depthbuffers[i]);
        glRenderbufferStorageEXT (GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
                width, height);
        glFramebufferRenderbufferEXT (GL_FRAMEBUFFER_EXT, 
                GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 
                priv->depthbuffers[i]);
    }

    // setup the pixelbuffer object for fast data transfers
    glGenBuffersARB (2, priv->output_pbos);
    for (i = 0; i < 2; i++) {
        glBindBufferARB (GL_PIXEL_PACK_BUFFER_ARB, priv->output_pbos[i]);
        glBufferDataARB (GL_PIXEL_PACK_BUFFER_ARB, 4*width*height,
                NULL, GL_DYNAMIC_READ_ARB);
        glBindBufferARB (GL_PIXEL_PACK_BUFFER_ARB, 0);
    }

    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);

    priv->currbuf = 0;
    priv->buffer_pending = 0;
    self->width = width;
    self->height = height;
    priv->format = format;

    return self;
}

void
bot_fbgl_drawing_area_swap_buffers (BotFbglDrawingArea * self)
{
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, priv->framebuffer);
    glReadBuffer (GL_COLOR_ATTACHMENT0_EXT + priv->currbuf);
    glBindBufferARB (GL_PIXEL_PACK_BUFFER_ARB,
            priv->output_pbos[priv->currbuf]);
    glReadPixels (0, 0, self->width, self->height, priv->format,
            GL_UNSIGNED_BYTE, NULL);

    bot_fbgl_drawing_area_flush (self);

    priv->buffer_pending = 1;

    priv->currbuf = !priv->currbuf;
    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);
}

int
bot_fbgl_drawing_area_begin (BotFbglDrawingArea * self)
{
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, priv->framebuffer);
    glDrawBuffer (GL_COLOR_ATTACHMENT0_EXT + priv->currbuf);
    glViewport (0, 0, self->width, self->height);
    return 0;
}

int
bot_fbgl_drawing_area_end (BotFbglDrawingArea * self)
{
    glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);
    return 0;
}

int
bot_fbgl_drawing_area_flush (BotFbglDrawingArea * self)
{
    BotFbglDrawingAreaPrivate * priv = BOT_FBGL_DRAWING_AREA_GET_PRIVATE (self);

    if (!priv->buffer_pending)
        return 0;

    glBindBufferARB (GL_PIXEL_PACK_BUFFER_ARB,
            priv->output_pbos[!priv->currbuf]);
    uint8_t * result = (uint8_t *) glMapBufferARB (GL_PIXEL_PACK_BUFFER_ARB,
            GL_READ_ONLY_ARB);

    g_signal_emit (G_OBJECT(self), signals[BUFFER_READY_SIGNAL], 0, result);

    glUnmapBufferARB (GL_PIXEL_PACK_BUFFER_ARB);
    priv->buffer_pending = 0;
    return 0;
}
