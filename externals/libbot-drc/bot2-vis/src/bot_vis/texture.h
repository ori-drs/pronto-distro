#ifndef BOT_GL_TEXTURE_H
#define BOT_GL_TEXTURE_H

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#endif

/**
 * @defgroup BotGlTexture Textures
 * @brief Rendering images/textures
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * Convenience class for uploading and rendering textures in OpenGL.
 *
 * Linking: `pkg-config --libs bot2-vis`
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BotGlTexture BotGlTexture;

/**
 * Constructor.
 * @param width texture width, in pixels.
 * @param height texture height, in pixels.
 * @param max_data_size maximum number of bytes required to store the texture data in memory.
 *
 * Allocates a new texture object using the current OpenGL context.
 */
BotGlTexture * bot_gl_texture_new (int width, int height, int max_data_size);

/**
 * Destructor.  The same OpenGL context used to allocated the texture must be active.
 */
void bot_gl_texture_free (BotGlTexture * t);

/**
 * Uploads texture data.
 * @param format OpenGL format of the texture.  Supported formats are: 
 *    GL_LUMINANCE, GL_ALPHA, GL_LUMINANCE_ALPHA, GL_RGB, GL_BGR, GL_RGBA,
 *    GL_BGRA.
 * @param type OpenGL data type used to represent the texture pixels.  Supported types are:
 *    GL_UNSIGNED_BYTE, GL_BYTE, GL_UNSIGNED_SHORT, GL_SHORT, GL_UNSIGNED_INT,
 *    GL_INT, GL_FLOAT.
 * @param stride number of bytes separating the start of each row in the texture data.
 * @param data the texture data.
 *
 * Uploads texture data to OpenGL-managed memory.  The original data is not needed for rendering
 * after an upload.
 */
int bot_gl_texture_upload (BotGlTexture * t, GLenum format,
        GLenum type, int stride, const void * data);

/**
 * Renders the texture in a unit square from (0, 0) to (1, 1).  Texture
 * coordinate mapping:
 *
 * <pre>
 * texture          opengl
 * 0, 0          -> 0, 0
 * width, 0      -> 1, 0
 * 0, height     -> 0, 1
 * width, height -> 1, 1
 * </pre>
 *
 * all opengl Z coordinates are 0.
 */
void
bot_gl_texture_draw (BotGlTexture * t);

/**
 * Renders the texture in a quadrilateral using the specified coordinates
 */
void
bot_gl_texture_draw_coords (BotGlTexture * t, 
        double x_top_left,  double y_top_left,  double z_top_left,
        double x_bot_left,  double y_bot_left,  double z_bot_left,
        double x_bot_right, double y_bot_right, double z_bot_right,
        double x_top_right, double y_top_right, double z_top_right);

/**
 * Sets the rendering interpolation mode.
 * @param nearest_or_linear typically GL_LINEAR or GL_NEAREST.  default is GL_LINEAR
 *
 * sets the interpolation mode when the texture is not drawn at a 1:1 scale.
 */
void bot_gl_texture_set_interp (BotGlTexture * t, GLint nearest_or_linear);

/**
 * Overrides the internal OpenGL format used to represent the texture.
 */
void bot_gl_texture_set_internal_format (BotGlTexture *t, GLenum fmt);

/**
 * @return the texture width as passed in to the constructor.
 */
int bot_gl_texture_get_width (BotGlTexture *t);

/**
 * @return the texture height as passed in to the constructor.
 */
int bot_gl_texture_get_height (BotGlTexture *t);

/**
 * @return the raw OpenGL texture identifier.  Useful for manual rendering
 * (e.g., non-quadrilateral rendering)
 */
GLuint bot_gl_texture_get_texname (BotGlTexture *t);

/**
 * @return the OpenGL texture target used for the texture.  Useful for manual
 * rendering.
 */
GLenum bot_gl_texture_get_target (BotGlTexture *t);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
