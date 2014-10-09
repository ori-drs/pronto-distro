#ifndef __bgl_h__
#define __bgl_h__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
//#include <GL/freeglut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
//#include <GL/freeglut.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup BotVisBatchGL Batched OpenGL commands
 * @brief Batching OpenGL commands for delayed rendering
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * Batched GL drawing allows a program to issue some basic OpenGL drawing
 * commands while an OpenGL context is not active.  Commands will be queued up,
 * stored, and then executed when requested via bot_bgl_render().
 *
 * Linking: `pkg-config --libs bot2-vis`
 *
 * @{
 */

typedef struct _BotBatchGl BotBatchGl;

BotBatchGl *bot_bgl_new (void);
void bot_bgl_destroy (BotBatchGl *bgl);

void bot_bgl_begin (BotBatchGl *bgl, GLenum mode);
void bot_bgl_end (BotBatchGl *bgl);

void bot_bgl_vertex2d (BotBatchGl *bgl, double v0, double v1);
void bot_bgl_vertex2dv(BotBatchGl *bgl, const double *v);
void bot_bgl_vertex2f (BotBatchGl *bgl, float v0, float v1);
void bot_bgl_vertex2fv(BotBatchGl *bgl, const float *v);

void bot_bgl_vertex3d (BotBatchGl *bgl, double v0, double v1, double v2);
void bot_bgl_vertex3dv(BotBatchGl *bgl, const double *v);
void bot_bgl_vertex3f (BotBatchGl *bgl, float v0, float v1, float v2);
void bot_bgl_vertex3fv(BotBatchGl *bgl, const float *v);

void bot_bgl_color3f (BotBatchGl *bgl, float v0, float v1, float v);
void bot_bgl_color4f (BotBatchGl *bgl, float v0, float v1, float v2, float v3);
void bot_bgl_point_size (BotBatchGl *bgl, float v);
void bot_bgl_line_width (BotBatchGl *bgl, float line_width);

void bot_bgl_enable (BotBatchGl *bgl, GLenum v);
void bot_bgl_disable (BotBatchGl *bgl, GLenum v);

void bot_bgl_blend_func (BotBatchGl *bgl, GLenum sfactor, GLenum dfactor);

void BotBatchGlranslated (BotBatchGl *bgl, double v0, double v1, double v2);
void BotBatchGlranslatef (BotBatchGl *bgl, float v0, float v1, float v2);
void bot_bgl_rotated (BotBatchGl *bgl, double angle, double x, double y, double z);
void bot_bgl_rotatef (BotBatchGl *bgl, float angle, float x, float y, float z);
void bot_bgl_push_matrix (BotBatchGl * bgl);
void bot_bgl_pop_matrix (BotBatchGl * bgl);
void bot_bgl_load_identity (BotBatchGl *bgl);

void bot_bgl_mult_matrixd (BotBatchGl *bgl, const double *matrix);
void bot_bgl_mult_matrixf (BotBatchGl *bgl, const float *matrix);

void bot_bgl_render (BotBatchGl *bgl);
void bot_bgl_switch_buffer (BotBatchGl *bgl);

//void bot_bgl_box_3dv_3fv(BotBatchGl *bgl, double xyz[3], float dim[3]);
//void bot_bgl_circle(BotBatchGl *bgl, double xyz[3], double radius);
//void bot_bgl_disk(BotBatchGl *bgl, double xyz[3], double r_in, double r_out);
//void BotBatchGlext(BotBatchGl *bgl, const double xyz[3], const char *text);
//void BotBatchGlext_ex(BotBatchGl *bgl, const double xyz[3], const char *text, uint32_t font, uint32_t flags);
//
//void bot_bgl_rect(BotBatchGl *bgl, double xyz[3], double size[2], double theta_rad, int filled);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
