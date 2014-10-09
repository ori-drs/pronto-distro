#ifndef __BOT_FBGL_DRAWING_AREA_H__
#define __BOT_FBGL_DRAWING_AREA_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

/**
 * @defgroup BotFBGLDrawingArea OpenGL offscreen rendering
 * @brief Offscreen rendering using Framebuffer objects
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-vis`
 * @{
 */

G_BEGIN_DECLS

#define BOT_TYPE_FBGL_DRAWING_AREA            (bot_fbgl_drawing_area_get_type ())
#define BOT_FBGL_DRAWING_AREA(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), BOT_TYPE_FBGL_DRAWING_AREA, BotFbglDrawingArea))
#define BOT_FBGL_DRAWING_AREA_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), BOT_TYPE_FBGL_DRAWING_AREA, BotFbglDrawingAreaClass))
#define FB_IS_GL_DRAWING_AREA(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), BOT_TYPE_FBGL_DRAWING_AREA))
#define FB_IS_GL_DRAWING_AREA_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), BOT_TYPE_FBGL_DRAWING_AREA))
#define BOT_FBGL_DRAWING_AREA_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), BOT_TYPE_FBGL_DRAWING_AREA, BotFbglDrawingAreaClass))

typedef struct _BotFbglDrawingArea        BotFbglDrawingArea;
typedef struct _BotFbglDrawingAreaClass   BotFbglDrawingAreaClass;

struct _BotFbglDrawingArea {
    GObject parent;

    int width, height;
};

struct _BotFbglDrawingAreaClass {
    GObjectClass parent;
};

GType       bot_fbgl_drawing_area_get_type (void);
BotFbglDrawingArea * bot_fbgl_drawing_area_new (gboolean new_context,
        int width, int height, GLenum format);
void        bot_fbgl_drawing_area_swap_buffers (BotFbglDrawingArea * glarea);
int         bot_fbgl_drawing_area_begin (BotFbglDrawingArea * glarea);
int         bot_fbgl_drawing_area_end (BotFbglDrawingArea * glarea);
int         bot_fbgl_drawing_area_flush (BotFbglDrawingArea * glarea);

G_END_DECLS

/**
 * @}
 */

#endif
