#ifndef __BOT_GTK_GL_IMAGE_AREA_H__
#define __BOT_GTK_GL_IMAGE_AREA_H__

/**
 * @defgroup BotGtkGlImageArea BotGTKGlImageArea 
 * @brief GTK+ widget to draw images with OpenGL
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot-vis`
 * @{
 */

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "gl_drawing_area.h"

G_BEGIN_DECLS

#define BOT_GTK_TYPE_GL_IMAGE_AREA            (bot_gtk_gl_image_area_get_type ())
#define BOT_GTK_GL_IMAGE_AREA(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), BOT_GTK_TYPE_GL_IMAGE_AREA, BotGtkGlImageArea))
#define BOT_GTK_GL_IMAGE_AREA_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), BOT_GTK_TYPE_GL_IMAGE_AREA, BotGtkGlImageAreaClass))
#define GTK_IS_GL_IMAGE_AREA(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), BOT_GTK_TYPE_GL_IMAGE_AREA))
#define GTK_IS_GL_IMAGE_AREA_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), BOT_GTK_TYPE_GL_IMAGE_AREA))
#define BOT_GTK_GL_IMAGE_AREA_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), BOT_GTK_TYPE_GL_IMAGE_AREA, BotGtkGlImageAreaClass))

typedef struct _BotGtkGlImageArea        BotGtkGlImageArea;
typedef struct _BotGtkGlImageAreaClass   BotGtkGlImageAreaClass;

struct _BotGtkGlImageArea {
    BotGtkGlDrawingArea  parent;

    /*< private >*/
    GLenum target;
    GLint int_format;
    GLint format;
    GLuint texname;
    int width;
    int height;

    GLuint texc_width;
    GLuint texc_height;

    GLuint pbo;
    int use_pbo;
    int max_data_size;
};

struct _BotGtkGlImageAreaClass {
    GtkDrawingAreaClass parent_class;
};

GType       bot_gtk_gl_image_area_get_type (void);
GtkWidget * bot_gtk_gl_image_area_new (void);
int         bot_gtk_gl_image_area_set_image_format (BotGtkGlImageArea *self,
        int width, int height, GLenum format);
int         bot_gtk_gl_image_area_upload_image (BotGtkGlImageArea * self,
        const void *data, int row_stride);

G_END_DECLS

/**
 * @}
 */

#endif

