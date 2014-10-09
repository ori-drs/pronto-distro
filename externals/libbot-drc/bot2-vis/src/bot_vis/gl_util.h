#ifndef __bot_gl_h__
#define __bot_gl_h__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
//#include <GL/freeglut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#endif

#include "texture.h"
#include "scrollplot2d.h"
#include "console.h"
#include "batch_gl.h"
#include <bot_core/bot_core.h>

/**
 * @defgroup BotGlUtil Miscellaneous OpenGL utility functions
 * @brief OpenGL Utility functions
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-vis`
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * bot_gl_draw_cube:
 *
 * renders a unit cube centered on the origin
 */
void bot_gl_draw_cube (void);

/**
 * bot_gl_draw_cube_frame:
 *
 * renders a unit cube wireframe centered on the origin
 */
void bot_gl_draw_cube_frame (void);

/**
 * bot_gl_draw_circle:
 *
 * renders a circle of radius r on the plane Z = 0 centered on the origin
 */
void bot_gl_draw_circle (double r);

/**
 * bot_gl_draw_ortho_circles_3d:
 *
 * renders a circle in the xy, xz, yz plane of unit radius at the origin with lines along the axes
 */
void
bot_gl_draw_ortho_circles_3d();

/**
 * bot_gl_draw_disk:
 *
 * renders a solid disk of radius r on the plane Z = 0 centered on the origin
 */
void bot_gl_draw_disk(double r);

/**
 * bot_gl_build_circle:
 *
 * builds a circle as a display list
 */
void bot_gl_build_circle (GLuint id);

/**
 * bot_gl_draw_ellipse:
 *
 * renders an ellipse with semimajor axis of length a, semiminor axis of length
 * b, and an angle of theta between the semimajor axis and the X axis
 */
void bot_gl_draw_ellipse (double a, double b, double theta, int npoints);

/**
 * bot_gl_draw_arrow_2d:
 *
 * renders an arrow centered on the origin pointing along the X axis
 */
void bot_gl_draw_arrow_2d (double length, double head_width, double head_length,
        double body_width, int fill);

/**
 * bot_gl_draw_arrow_3d
 *
 * renders an arrow centered on the origina pointin along the x axis using cylinders
 * so it is visible from all angles
 */
void
bot_gl_draw_arrow_3d (double length, double head_width, double head_length,
        double body_width);

#define BOT_GL_DRAW_TEXT_DROP_SHADOW      1
#define BOT_GL_DRAW_TEXT_JUSTIFY_LEFT     2
#define BOT_GL_DRAW_TEXT_JUSTIFY_RIGHT    4
#define BOT_GL_DRAW_TEXT_JUSTIFY_CENTER   8
#define BOT_GL_DRAW_TEXT_ANCHOR_LEFT     16
#define BOT_GL_DRAW_TEXT_ANCHOR_RIGHT    32
#define BOT_GL_DRAW_TEXT_ANCHOR_TOP      64
#define BOT_GL_DRAW_TEXT_ANCHOR_BOTTOM  128
#define BOT_GL_DRAW_TEXT_ANCHOR_HCENTER 256
#define BOT_GL_DRAW_TEXT_ANCHOR_VCENTER 512
#define BOT_GL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES 1024
#define BOT_GL_DRAW_TEXT_MONOSPACED 2048

/** Draw text centered at xyz in the current projection, but the text
 * will be drawn in pixel coordinates (so it will be "rectified"). A
 * font may be specified, as well as an optional drop shadow.
 * 
 * Text will be drawn using the current GL color
 *
 * We DO support multi-line text.
 *
 **/
void bot_gl_draw_text (const double xyz[3], void *font, const char *text, 
        int flags);

/** Iterate through OpenGL error list, printing the error string (from
 * gluErrorString()) for each. Returns the total number of errors.
 */
int _bot_gl_check_errors(const char *file, int line);
#define bot_gl_check_errors() _bot_gl_check_errors(__FILE__, __LINE__)

/** Prints the current OpenGL matrix (e.g. MODELVIEW, PROJECTION, etc)
 * using printf(). Also displays the current matrix's stack depth.
 */
void bot_gl_print_current_matrix(void);


/**
 * bot_glutBitmapString
 *
 * Renders a given string in the specified font in the current window. 
 * Reverts to the FreeGLUT implementation, if available.
 */
void
bot_glutBitmapString(void* font, const unsigned char* text);

/**
 * bot_gl_draw_axes():
 *
 * draw rgb xyz axes of unit length
 */
void
bot_gl_draw_axes();

/**
 * bot_gl_multTrans():
 *
 * apply the BotTrans transform to the matrix stack
 *
 */
void bot_gl_multTrans(BotTrans * trans);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
