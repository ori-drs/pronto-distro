#ifndef __BOT_GTK_GL_GL_DRAWING_AREA_H__
#define __BOT_GTK_GL_GL_DRAWING_AREA_H__

#include <gdk/gdk.h>
#include <gtk/gtkwidget.h>
#include <gtk/gtkdrawingarea.h>

/**
 * @defgroup BotGtkGlDrawingArea BotGTKGlDrawingArea 
 * @brief GTK+ drawing-area widget with an OpenGL context
 * @ingroup BotVisGtk
 * @include: bot_vis/bot_vis.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot-vis`
 * @{
 */

G_BEGIN_DECLS

#define BOT_GTK_TYPE_GL_DRAWING_AREA            (bot_gtk_gl_drawing_area_get_type ())
#define BOT_GTK_GL_DRAWING_AREA(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), BOT_GTK_TYPE_GL_DRAWING_AREA, BotGtkGlDrawingArea))
#define BOT_GTK_GL_DRAWING_AREA_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), BOT_GTK_TYPE_GL_DRAWING_AREA, BotGtkGlDrawingAreaClass))
#define BOT_GTK_IS_GL_DRAWING_AREA(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), BOT_GTK_TYPE_GL_DRAWING_AREA))
#define BOT_GTK_IS_GL_DRAWING_AREA_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), BOT_GTK_TYPE_GL_DRAWING_AREA))
#define BOT_GTK_GL_DRAWING_AREA_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), BOT_GTK_TYPE_GL_DRAWING_AREA, BotGtkGlDrawingAreaClass))

typedef struct _BotGtkGlDrawingArea        BotGtkGlDrawingArea;
typedef struct _BotGtkGlDrawingAreaClass   BotGtkGlDrawingAreaClass;

struct _BotGtkGlDrawingArea {
    GtkDrawingArea  area;

    gboolean vblank_sync;
};

struct _BotGtkGlDrawingAreaClass {
    GtkDrawingAreaClass parent_class;
};

GType       bot_gtk_gl_drawing_area_get_type (void);
GtkWidget * bot_gtk_gl_drawing_area_new (gboolean vblank_sync);
void        bot_gtk_gl_drawing_area_set_vblank_sync (BotGtkGlDrawingArea * glarea,
        gboolean vblank_sync);
void        bot_gtk_gl_drawing_area_swap_buffers (BotGtkGlDrawingArea * glarea);
int         bot_gtk_gl_drawing_area_set_context (BotGtkGlDrawingArea * glarea);
void        bot_gtk_gl_drawing_area_invalidate (BotGtkGlDrawingArea * glarea);

G_END_DECLS

/**
 * @}
 */

#endif
