#include <stdio.h>
#include <string.h>

#include <glib.h>

#include "batch_gl.h"

typedef struct _BotBatchGl_command bot_bgl_command_t;
struct _BotBatchGl_command {
    void (*execute) (bot_bgl_command_t *);
    void (*destroy) (bot_bgl_command_t *);
};

struct _BotBatchGl 
{
    GPtrArray *front_buffer;
    GPtrArray *back_buffer;
};

BotBatchGl * bot_bgl_new ()
{
    BotBatchGl *bgl = g_slice_new (BotBatchGl);
    bgl->front_buffer = g_ptr_array_new ();
    bgl->back_buffer = g_ptr_array_new ();
    return bgl;
}

void bot_bgl_destroy (BotBatchGl *bgl)
{
    for (int i=0; i<bgl->front_buffer->len; i++) {
        bot_bgl_command_t *cmd = g_ptr_array_index (bgl->front_buffer, i);
        cmd->destroy (cmd);
    }
    g_ptr_array_free (bgl->front_buffer, TRUE);
    for (int i=0; i<bgl->back_buffer->len; i++) {
        bot_bgl_command_t *cmd = g_ptr_array_index (bgl->back_buffer, i);
        cmd->destroy (cmd);
    }
    g_ptr_array_free (bgl->back_buffer, TRUE);
    g_slice_free (BotBatchGl, bgl);
}

void bot_bgl_render (BotBatchGl *bgl)
{
    for (int i=0; i<bgl->front_buffer->len; i++) {
        bot_bgl_command_t *cmd = g_ptr_array_index (bgl->front_buffer, i);
        cmd->execute (cmd);
    }
}

void bot_bgl_switch_buffer (BotBatchGl *bgl)
{
    for (int i=0; i<bgl->front_buffer->len; i++) {
        bot_bgl_command_t *cmd = g_ptr_array_index (bgl->front_buffer, i);
        cmd->destroy (cmd);
    }
    g_ptr_array_free (bgl->front_buffer, TRUE);
    bgl->front_buffer = bgl->back_buffer;
    bgl->back_buffer = g_ptr_array_new ();
}

#define DECL_BGL_0(glfunc, name) \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_command_t, bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    glfunc (); \
} \
void bot_bgl_##name (BotBatchGl *bgl) { \
    bot_bgl_command_t *cmd = g_slice_new (bot_bgl_command_t); \
    cmd->execute = bot_bgl_##name##_execute; \
    cmd->destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

#define DECL_BGL_1(glfunc, name, type) \
typedef struct { \
    bot_bgl_command_t parent; \
    type v; \
} bot_bgl_cmd_##name##_t; \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_cmd_##name##_t, (bot_bgl_cmd_##name##_t*) bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    bot_bgl_cmd_##name##_t *cmd = (bot_bgl_cmd_##name##_t*) bcmd; \
    glfunc (cmd->v); \
} \
void bot_bgl_##name (BotBatchGl *bgl, type v) { \
    bot_bgl_cmd_##name##_t *cmd = g_slice_new (bot_bgl_cmd_##name##_t); \
    cmd->v = v; \
    cmd->parent.execute = bot_bgl_##name##_execute; \
    cmd->parent.destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

#define DECL_BGL_2(glfunc, name, type_1, type_2) \
typedef struct { \
    bot_bgl_command_t parent; \
    type_1 a; \
    type_2 b; \
} bot_bgl_cmd_##name##_t; \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_cmd_##name##_t, (bot_bgl_cmd_##name##_t*) bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    bot_bgl_cmd_##name##_t *cmd = (bot_bgl_cmd_##name##_t*) bcmd; \
    glfunc (cmd->a, cmd->b); \
} \
void bot_bgl_##name (BotBatchGl *bgl, type_1 a, type_2 b) { \
    bot_bgl_cmd_##name##_t *cmd = g_slice_new (bot_bgl_cmd_##name##_t); \
    cmd->a = a; \
    cmd->b = b; \
    cmd->parent.execute = bot_bgl_##name##_execute; \
    cmd->parent.destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

#define DECL_BGL_3(glfunc, name, type_1, type_2, type_3) \
typedef struct { \
    bot_bgl_command_t parent; \
    type_1 a; \
    type_2 b; \
    type_3 c; \
} bot_bgl_cmd_##name##_t; \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_cmd_##name##_t, (bot_bgl_cmd_##name##_t*) bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    bot_bgl_cmd_##name##_t *cmd = (bot_bgl_cmd_##name##_t*) bcmd; \
    glfunc (cmd->a, cmd->b, cmd->c); \
} \
void bot_bgl_##name (BotBatchGl *bgl, type_1 a, type_2 b, type_3 c) { \
    bot_bgl_cmd_##name##_t *cmd = g_slice_new (bot_bgl_cmd_##name##_t); \
    cmd->a = a; \
    cmd->b = b; \
    cmd->c = c; \
    cmd->parent.execute = bot_bgl_##name##_execute; \
    cmd->parent.destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

#define DECL_BGL_4(glfunc, name, type_1, type_2, type_3, type_4) \
typedef struct { \
    bot_bgl_command_t parent; \
    type_1 a; \
    type_2 b; \
    type_3 c; \
    type_4 d; \
} bot_bgl_cmd_##name##_t; \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_cmd_##name##_t, (bot_bgl_cmd_##name##_t*) bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    bot_bgl_cmd_##name##_t *cmd = (bot_bgl_cmd_##name##_t*) bcmd; \
    glfunc (cmd->a, cmd->b, cmd->c, cmd->d); \
} \
void bot_bgl_##name (BotBatchGl *bgl, type_1 a, type_2 b, type_3 c, type_4 d) { \
    bot_bgl_cmd_##name##_t *cmd = g_slice_new (bot_bgl_cmd_##name##_t); \
    cmd->a = a; \
    cmd->b = b; \
    cmd->c = c; \
    cmd->d = d; \
    cmd->parent.execute = bot_bgl_##name##_execute; \
    cmd->parent.destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

DECL_BGL_0 (glEnd, end)
DECL_BGL_0 (glPushMatrix, push_matrix)
DECL_BGL_0 (glPopMatrix, pop_matrix)
DECL_BGL_0 (glLoadIdentity, load_identity)

DECL_BGL_1 (glBegin, begin, GLenum)
DECL_BGL_1 (glEnable, enable, GLenum)
DECL_BGL_1 (glDisable, disable, GLenum)
DECL_BGL_1 (glPointSize, point_size, float)
DECL_BGL_1 (glLineWidth, line_width, float)

DECL_BGL_2 (glVertex2f, vertex2f, float, float)
DECL_BGL_2 (glVertex2d, vertex2d, double, double)
DECL_BGL_2 (glBlendFunc, blend_func, GLenum, GLenum)

DECL_BGL_3 (glVertex3d, vertex3d, double, double, double)
DECL_BGL_3 (glVertex3f, vertex3f, float, float, float)
DECL_BGL_3 (glTranslated, translated, double, double, double)
DECL_BGL_3 (glTranslatef, translatef, float, float, float)
DECL_BGL_3 (glColor3f, color3f, float, float, float)

DECL_BGL_4 (glColor4f, color4f, float, float, float, float)
DECL_BGL_4 (glRotated, rotated, double, double, double, double)
DECL_BGL_4 (glRotatef, rotatef, float, float, float, float)


#define DECL_BGL_1V(glfunc, name, type, size) \
typedef struct { \
    bot_bgl_command_t parent; \
    type v[size]; \
} bot_bgl_cmd_##name##_t; \
static void bot_bgl_cmd_##name##_destroy (bot_bgl_command_t *bcmd) { \
    g_slice_free (bot_bgl_cmd_##name##_t, (bot_bgl_cmd_##name##_t*) bcmd);  \
} \
static void bot_bgl_##name##_execute (bot_bgl_command_t *bcmd) { \
    bot_bgl_cmd_##name##_t *cmd = (bot_bgl_cmd_##name##_t*) bcmd; \
    glfunc (cmd->v); \
} \
void bot_bgl_##name (BotBatchGl *bgl, const type *v) { \
    bot_bgl_cmd_##name##_t *cmd = g_slice_new (bot_bgl_cmd_##name##_t); \
    memcpy(cmd->v, v, sizeof(cmd->v)); \
    cmd->parent.execute = bot_bgl_##name##_execute; \
    cmd->parent.destroy = bot_bgl_cmd_##name##_destroy; \
    g_ptr_array_add (bgl->back_buffer, cmd); \
}

DECL_BGL_1V (glVertex2dv, vertex2dv, double, 2)
DECL_BGL_1V (glVertex3dv, vertex3dv, double, 3)
DECL_BGL_1V (glVertex2fv, vertex2fv, float, 2)
DECL_BGL_1V (glVertex3fv, vertex3fv, float, 3)
DECL_BGL_1V (glMultMatrixd, mult_matrixd, double, 16)
DECL_BGL_1V (glMultMatrixf, mult_matrixf, float, 16)
