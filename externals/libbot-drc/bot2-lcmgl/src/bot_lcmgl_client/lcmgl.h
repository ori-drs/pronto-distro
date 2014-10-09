#ifndef __bot_lcmgl_h__
#define __bot_lcmgl_h__

#include <lcm/lcm.h>

/**
 * @defgroup BotLCMGL LCMGL
 * @brief Transmit and render OpenGL commands via LCM
 */

/**
 * @defgroup BotLCMGLCLient LCMGL client
 * @ingroup BotLCMGL
 * @brief OpenGL rendering via LCM - client routines
 * @include: bot_lcmgl_client/lcmgl.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-lcmgl-client`
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lcmgl bot_lcmgl_t;

/**
 * bot_lcmgl_init:  
 * 
 * Constructor
 */
bot_lcmgl_t *bot_lcmgl_init(lcm_t *lc, const char *name);

/**
 * bot_lcmgl_destroy:  
 * 
 * Destructor
 */
void bot_lcmgl_destroy(bot_lcmgl_t *lcmgl);

/**
 * bot_lcmgl_switch_buffer:
 *
 * Transmits all queued operations.
 */
void bot_lcmgl_switch_buffer(bot_lcmgl_t *lcmgl);

/* ================ OpenGL functions ===========
 *
 * These functions map directly to the OpenGL API, and all arguments should
 * be identical to their OpenGL equivalents.
 */

void bot_lcmgl_begin(bot_lcmgl_t *lcmgl, unsigned int glenum_mode);
void bot_lcmgl_end(bot_lcmgl_t *lcmgl);

void bot_lcmgl_vertex2d(bot_lcmgl_t *lcmgl, double v0, double v1);
void bot_lcmgl_vertex2f(bot_lcmgl_t *lcmgl, float v0, float v1);
void bot_lcmgl_vertex3d(bot_lcmgl_t *lcmgl, double v0, double v1, double v2);
void bot_lcmgl_vertex3f(bot_lcmgl_t *lcmgl, float v0, float v1, float v2);
void bot_lcmgl_color3f(bot_lcmgl_t *lcmgl, float v0, float v1, float v);
void bot_lcmgl_color4f(bot_lcmgl_t *lcmgl,
        float v0, float v1, float v2, float v3);
void bot_lcmgl_normal3f(bot_lcmgl_t *lcmgl, float v0, float v1, float v2);
void bot_lcmgl_scalef(bot_lcmgl_t *lcmgl, float v0, float v1, float v2);

void bot_lcmgl_point_size(bot_lcmgl_t *lcmgl, float v);
void bot_lcmgl_line_width(bot_lcmgl_t *lcmgl, float line_width);

void bot_lcmgl_enable(bot_lcmgl_t *lcmgl, unsigned int v);
void bot_lcmgl_disable(bot_lcmgl_t *lcmgl, unsigned int v);

void bot_lcmgl_translated(bot_lcmgl_t *lcmgl, double v0, double v1, double v2);
void bot_lcmgl_rotated(bot_lcmgl_t *lcmgl,
        double angle, double x, double y, double z);
void bot_lcmgl_push_matrix(bot_lcmgl_t * lcmgl);
void bot_lcmgl_pop_matrix(bot_lcmgl_t * lcmgl);
void bot_lcmgl_load_identity(bot_lcmgl_t *lcmgl);
void bot_lcmgl_mult_matrixf(bot_lcmgl_t * lcmgl, const float m[16]);
void bot_lcmgl_mult_matrixd(bot_lcmgl_t * lcmgl, const double m[16]);
void bot_lcmgl_matrix_mode(bot_lcmgl_t * lcmgl, unsigned int mode);
void bot_lcmgl_ortho(bot_lcmgl_t *lcmgl, double left, double right,
    double bottom, double top, double nearVal, double farVal);

void bot_lcmgl_materialf(bot_lcmgl_t * lcmgl, int face, int name, float c0, float c1, float c2, float c3);

void bot_lcmgl_push_attrib(bot_lcmgl_t *lcmgl, unsigned int attrib);
void bot_lcmgl_pop_attrib(bot_lcmgl_t *lcmgl);

void bot_lcmgl_depth_func(bot_lcmgl_t *lcmgl, unsigned int func);



//vector versions of some of the above
static inline void bot_lcmgl_vertex2fv(bot_lcmgl_t * lcmgl, const float v[2]){bot_lcmgl_vertex2f(lcmgl,v[0],v[1]);}
static inline void bot_lcmgl_vertex2dv(bot_lcmgl_t * lcmgl, const double v[2]){bot_lcmgl_vertex2d(lcmgl,v[0],v[1]);}


static inline void bot_lcmgl_vertex3fv(bot_lcmgl_t * lcmgl, const float v[3]){bot_lcmgl_vertex3f(lcmgl,v[0],v[1],v[2]);}
static inline void bot_lcmgl_vertex3dv(bot_lcmgl_t * lcmgl, const double v[3]){bot_lcmgl_vertex3d(lcmgl,v[0],v[1],v[2]);}

static inline void bot_lcmgl_color3fv(bot_lcmgl_t * lcmgl, const float v[3]){bot_lcmgl_color3f(lcmgl,v[0],v[1],v[2]);}
static inline void bot_lcmgl_color4fv(bot_lcmgl_t * lcmgl, const double v[4]){bot_lcmgl_color4f(lcmgl,v[0],v[1],v[2],v[3]);}

// these macros provide better "work-alike" interface to GL.  They
// expect that an lcmgl* is defined in the current scope.

#define lcmglBegin(v) bot_lcmgl_begin(lcmgl, v)
#define lcmglEnd() bot_lcmgl_end(lcmgl)

#define lcmglVertex2d(v0, v1) bot_lcmgl_vertex2d(lcmgl, v0, v1)
#define lcmglVertex2dv(v) bot_lcmgl_vertex2d(lcmgl, v[0], v[1])

#define lcmglVertex2f(v0, v1) bot_lcmgl_vertex2f(lcmgl, v0, v1)
#define lcmglVertex2fv(v) bot_lcmgl_vertex2f(lcmgl, v[0], v[1])

#define lcmglVertex3d(v0, v1, v2) bot_lcmgl_vertex3d(lcmgl, v0, v1, v2)
#define lcmglVertex3dv(v) bot_lcmgl_vertex3d(lcmgl, v[0], v[1], v[2])

#define lcmglVertex3f(v0, v1, v2) bot_lcmgl_vertex3f(lcmgl, v0, v1, v2)
#define lcmglVertex3fv(v) bot_lcmgl_vertex3f(lcmgl, v[0], v[1], v[2])
#define lcmglNormal3f(v0, v1, v2) bot_lcmgl_normal3f(lcmgl, v0, v1, v2)
#define lcmglNormal3fv(v) bot_lcmgl_normal3f(lcmgl, v[0], v[1], v[2])
#define lcmglScalef(v0, v1, v2) bot_lcmgl_scalef(lcmgl, v0, v1, v2)

#define lcmglColor3f(v0, v1, v2) bot_lcmgl_color3f(lcmgl, v0, v1, v2)
#define lcmglColor3fv(v) bot_lcmgl_color3f(lcmgl, v[0], v[1], v[2])

#define lcmglColor4f(v0, v1, v2, v3) bot_lcmgl_color4f(lcmgl, v0, v1, v2, v3)
#define lcmglColor4fv(v) bot_lcmgl_color4f(lcmgl, v[0], v[1], v[2], v[3])

#define lcmglPointSize(v) bot_lcmgl_point_size(lcmgl, v)
#define lcmglEnable(v) bot_lcmgl_enable(lcmgl, v)
#define lcmglDisable(v) bot_lcmgl_disable(lcmgl, v)

#define lcmglLineWidth(size) bot_lcmgl_line_width(lcmgl, size);

#define lcmglTranslated(v0, v1, v2) bot_lcmgl_translated(lcmgl, v0, v1, v2)
#define lcmglTranslatef lcmglTranslated

#define lcmglRotated(angle, x, y, z) bot_lcmgl_rotated(lcmgl, angle, x, y, z)
#define lcmglRotatef lcmglRotated

#define lcmglLoadIdentity() bot_lcmgl_load_identity(lcmgl)

#define lcmglPushMatrix() bot_lcmgl_push_matrix(lcmgl)
#define lcmglPopMatrix() bot_lcmgl_pop_matrix(lcmgl)
#define lcmglMultMatrixf(m) bot_lcmgl_mult_matrixf(lcmgl, m)
#define lcmglMultMatrixd(m) bot_lcmgl_mult_matrixd(lcmgl, m)
#define lcmglMaterialf(face,name,c0,c1,c2) bot_lcmgl_materialf(lcmgl, face, name, c0, c1, c2, c3);
#define lcmglMaterialfv(face,name,c) bot_lcmgl_materialf(lcmgl, face, name, c[0], c[1], c[2], c[3]);
#define lcmglMateriald lcmglMaterialf
#define lcmglMaterialdv lcmglMaterialfv


//Defines of a few basic opengl constants
/* Primitives */
#define LCMGL_POINTS                               0x0000
#define LCMGL_LINES                                0x0001
#define LCMGL_LINE_LOOP                            0x0002
#define LCMGL_LINE_STRIP                           0x0003
#define LCMGL_TRIANGLES                            0x0004
#define LCMGL_TRIANGLE_STRIP                       0x0005
#define LCMGL_TRIANGLE_FAN                         0x0006
#define LCMGL_QUADS                                0x0007
#define LCMGL_QUAD_STRIP                           0x0008
#define LCMGL_POLYGON                              0x0009





/* ================ drawing routines not part of OpenGL ===============
 * 
 * These routines do not have a direct correspondence to the OpenGL API, but
 * are included for convenience.
 */

/**
 * bot_lcmgl_rect:
 *
 * Draws a rectangle on the X-Y plane, centered on @xyz.
 */
void bot_lcmgl_rect(bot_lcmgl_t *lcmgl, double xyz[3], double size[2], 
        int filled);
/**
 * bot_lcmgl_box:
 *
 * Draws a box, centered on @xyz.
 */
void bot_lcmgl_box(bot_lcmgl_t *lcmgl, double xyz[3], float size[3]);

/**
 * bot_lcmgl_circle:
 *
 * Draws a circle on the X-Y plane, centered on @xyz.
 */
void bot_lcmgl_circle(bot_lcmgl_t *lcmgl, double xyz[3], double radius);

/**
 * bot_lcmgl_sphere:
 * @see: gluSphere
 */
void bot_lcmgl_sphere(bot_lcmgl_t *lcmgl, double xyz[3], double radius,
        int slices, int stacks);


void bot_lcmgl_disk(bot_lcmgl_t *lcmgl,
        double xyz[3], double r_in, double r_out);
/**
 * bot_lcmgl_cylinder:
 * @see: gluCylinder
 */
void bot_lcmgl_cylinder(bot_lcmgl_t *lcmgl,
        double base_xyz[3], double base_radius, double top_radius, double height,
        int slices, int stacks);

void bot_lcmgl_text(bot_lcmgl_t *lcmgl, const double xyz[3], const char *text);
void bot_lcmgl_text_ex(bot_lcmgl_t *lcmgl,
        const double xyz[3], const char *text, uint32_t font, uint32_t flags);

/**
 * bot_lcmgl_draw_axes():
 *
 * draw rgb xyz axes of unit length
 */
void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl);

/**
 * bot_gl_draw_ortho_circles_3d:
 *
 * renders a circle in the xy, xz, yz plane of unit radius at the origin with lines along the axes
 */
void bot_lcmgl_draw_ortho_circles_3d(bot_lcmgl_t * lcmgl);


/**
 * bot_lcmgl_draw_arrow_3d():
 *
 * draw arrow along x axis starting at origin
 */
void bot_lcmgl_draw_arrow_3d (bot_lcmgl_t * lcmgl, double length, double head_width, double head_length,
        double body_width);


#define lcmglBox(xyz, dim) bot_lcmgl_box(lcmgl, xyz, dim)
#define lcmglCircle(xyz, radius) bot_lcmgl_circle(lcmgl, xyz, radius)
#define lcmglDisk(xyz, r_in, r_out) bot_lcmgl_disk(lcmgl, xyz, r_in, r_out)
#define lcmglCylinder(xyz, base_radius, top_radius, height, slices, stacks) bot_lcmgl_cylinder(lcmgl, xyz, base_radius, top_radius, height, slices, stacks)


/**
 * bot_lcmgl_scale_to_viewer_aspect_ratio:
 * This function scales the opengl workspace such that it is proportional to the
 * aspect ratio of the viewer window.
 *
 * It's useful for displaying things with square "pixels".
 * It maps to:
 *     GLint viewport[4];
 *     glGetIntegerv (GL_VIEWPORT, viewport);
 *     float vp_width = viewport[2] - viewport[0];
 *     float vp_height = viewport[3] - viewport[1];
 *     float ar = vp_width/vp_height;
 *     glScalef(1,ar,1);
 *
 */
void bot_lcmgl_scale_to_viewer_ar(bot_lcmgl_t *lcmgl);


// texture API
typedef enum {
    BOT_LCMGL_LUMINANCE = 0x1909, //values pulled from gl.h
    BOT_LCMGL_RGB = 0x1907,
    BOT_LCMGL_RGBA = 0x1908
} bot_lcmgl_texture_format_t;

typedef enum {
    BOT_LCMGL_UNSIGNED_BYTE = 0x1401, //values pulled from gl.h
    BOT_LCMGL_BYTE =0x1400,
    BOT_LCMGL_UNSIGNED_SHORT =0x1403,
    BOT_LCMGL_SHORT = 0x1402,
    BOT_LCMGL_UNSIGNED_INT = 0x1405,
    BOT_LCMGL_INT = 0x1404,
    BOT_LCMGL_FLOAT = 0x1406
} bot_lcmgl_texture_type_t;


typedef enum {
    BOT_LCMGL_COMPRESS_NONE = 0,
    BOT_LCMGL_COMPRESS_ZLIB = 1,
} bot_lcmgl_compress_mode_t;

/**
 * bot_lcmgl_texture:
 *
 * Creates a texture.
 *
 * @return: the texture ID.  This ID is valid until bot_lcmgl_switch_buffer is
 * called.
 */
int bot_lcmgl_texture2d(bot_lcmgl_t *lcmgl, const void *data, 
        int width, int height, int row_stride,
        bot_lcmgl_texture_format_t format,
        bot_lcmgl_texture_type_t type,
        bot_lcmgl_compress_mode_t compression);

/**
 * Renders the specified texture with the active OpenGL color.
 */
void bot_lcmgl_texture_draw_quad(bot_lcmgl_t *lcmgl, int texture_id,
        double x_top_left,  double y_top_left,  double z_top_left,
        double x_bot_left,  double y_bot_left,  double z_bot_left,
        double x_bot_right, double y_bot_right, double z_bot_right,
        double x_top_right, double y_top_right, double z_top_right);

/* ======================== */

enum _bot_lcmgl_enum_t 
{
    BOT_LCMGL_BEGIN=4,
    BOT_LCMGL_END,
    BOT_LCMGL_VERTEX3F,
    BOT_LCMGL_VERTEX3D,
    BOT_LCMGL_COLOR3F,
    BOT_LCMGL_COLOR4F,
    BOT_LCMGL_POINTSIZE,
    BOT_LCMGL_ENABLE,
    BOT_LCMGL_DISABLE,
    BOT_LCMGL_BOX,
    BOT_LCMGL_CIRCLE,
    BOT_LCMGL_LINE_WIDTH,
    BOT_LCMGL_NOP,
    BOT_LCMGL_VERTEX2D,
    BOT_LCMGL_VERTEX2F,
    BOT_LCMGL_TEXT,
    BOT_LCMGL_DISK,
    BOT_LCMGL_TRANSLATED,
    BOT_LCMGL_ROTATED,
    BOT_LCMGL_LOAD_IDENTITY,
    BOT_LCMGL_PUSH_MATRIX,
    BOT_LCMGL_POP_MATRIX,
    BOT_LCMGL_RECT,
    BOT_LCMGL_TEXT_LONG,
    BOT_LCMGL_NORMAL3F,
    BOT_LCMGL_SCALEF,
    BOT_LCMGL_MULT_MATRIXF,
    BOT_LCMGL_MULT_MATRIXD,
    BOT_LCMGL_MATERIALF,
    BOT_LCMGL_PUSH_ATTRIB,
    BOT_LCMGL_POP_ATTRIB,
    BOT_LCMGL_DEPTH_FUNC,
    BOT_LCMGL_TEX_2D,
    BOT_LCMGL_TEX_DRAW_QUAD,
    BOT_LCMGL_SPHERE,
    BOT_LCMGL_CYLINDER,
    BOT_LCMGL_MATRIX_MODE,
    BOT_LCMGL_ORTHO,
    BOT_LCMGL_SCALE_TO_VIEWER_AR
};

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
