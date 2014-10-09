#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>

#include <bot_core/fasttrig.h>
#include "gl_util.h"

void 
bot_gl_draw_cube ()
{
    glBegin (GL_QUADS);
    glNormal3d (0, 0, -1);
    glVertex3d (-0.5, -0.5, -0.5);
    glVertex3d ( 0.5, -0.5, -0.5);
    glVertex3d ( 0.5,  0.5, -0.5);
    glVertex3d (-0.5,  0.5, -0.5);

    glNormal3d (0, -1, 0);
    glVertex3d (-0.5, -0.5, -0.5);
    glVertex3d ( 0.5, -0.5, -0.5);
    glVertex3d ( 0.5, -0.5,  0.5);
    glVertex3d (-0.5, -0.5,  0.5);

    glNormal3d (-1, 0, 0);
    glVertex3d (-0.5, -0.5, -0.5);
    glVertex3d (-0.5,  0.5, -0.5);
    glVertex3d (-0.5,  0.5,  0.5);
    glVertex3d (-0.5, -0.5,  0.5);

    glNormal3d (1, 0, 0);
    glVertex3d ( 0.5,  0.5, -0.5);
    glVertex3d ( 0.5, -0.5, -0.5);
    glVertex3d ( 0.5, -0.5,  0.5);
    glVertex3d ( 0.5,  0.5,  0.5);

    glNormal3d (0, 1, 0);
    glVertex3d ( 0.5,  0.5, -0.5);
    glVertex3d (-0.5,  0.5, -0.5);
    glVertex3d (-0.5,  0.5,  0.5);
    glVertex3d ( 0.5,  0.5,  0.5);

    glNormal3d (0, 0, 1);
    glVertex3d (-0.5, -0.5,  0.5);
    glVertex3d ( 0.5, -0.5,  0.5);
    glVertex3d ( 0.5,  0.5,  0.5);
    glVertex3d (-0.5,  0.5,  0.5);
    glEnd ();
}

void
bot_gl_draw_cube_frame ()
{
    glBegin (GL_LINE_LOOP);
    glVertex3d (-0.5, -0.5, -0.5);
    glVertex3d (0.5, -0.5, -0.5);
    glVertex3d (0.5, 0.5, -0.5);
    glVertex3d (-0.5, 0.5, -0.5);
    glEnd ();
    glBegin (GL_LINE_LOOP);
    glVertex3d (-0.5, -0.5, 0.5);
    glVertex3d (0.5, -0.5, 0.5);
    glVertex3d (0.5, 0.5, 0.5);
    glVertex3d (-0.5, 0.5, 0.5);
    glEnd ();
    glBegin (GL_LINES);
    glVertex3d (-0.5, -0.5, -0.5);
    glVertex3d (-0.5, -0.5, 0.5);
    glVertex3d (0.5, -0.5, -0.5);
    glVertex3d (0.5, -0.5, 0.5);
    glVertex3d (0.5, 0.5, -0.5);
    glVertex3d (0.5, 0.5, 0.5);
    glVertex3d (-0.5, 0.5, -0.5);
    glVertex3d (-0.5, 0.5, 0.5);
    glEnd ();
}

float _circle_points[][2] = {
    { 1.000000, 0.000000 }, { 0.999848, 0.017452 }, { 0.999391, 0.034899 },
    { 0.998630, 0.052336 }, { 0.997564, 0.069756 }, { 0.996195, 0.087156 },
    { 0.994522, 0.104528 }, { 0.992546, 0.121869 }, { 0.990268, 0.139173 },
    { 0.987688, 0.156434 }, { 0.984808, 0.173648 }, { 0.981627, 0.190809 },
    { 0.978148, 0.207912 }, { 0.974370, 0.224951 }, { 0.970296, 0.241922 },
    { 0.965926, 0.258819 }, { 0.961262, 0.275637 }, { 0.956305, 0.292372 },
    { 0.951057, 0.309017 }, { 0.945519, 0.325568 }, { 0.939693, 0.342020 },
    { 0.933580, 0.358368 }, { 0.927184, 0.374607 }, { 0.920505, 0.390731 },
    { 0.913545, 0.406737 }, { 0.906308, 0.422618 }, { 0.898794, 0.438371 },
    { 0.891007, 0.453990 }, { 0.882948, 0.469472 }, { 0.874620, 0.484810 },
    { 0.866025, 0.500000 }, { 0.857167, 0.515038 }, { 0.848048, 0.529919 },
    { 0.838671, 0.544639 }, { 0.829038, 0.559193 }, { 0.819152, 0.573576 },
    { 0.809017, 0.587785 }, { 0.798636, 0.601815 }, { 0.788011, 0.615661 },
    { 0.777146, 0.629320 }, { 0.766044, 0.642788 }, { 0.754710, 0.656059 },
    { 0.743145, 0.669131 }, { 0.731354, 0.681998 }, { 0.719340, 0.694658 },
    { 0.707107, 0.707107 }, { 0.694658, 0.719340 }, { 0.681998, 0.731354 },
    { 0.669131, 0.743145 }, { 0.656059, 0.754710 }, { 0.642788, 0.766044 },
    { 0.629320, 0.777146 }, { 0.615661, 0.788011 }, { 0.601815, 0.798636 },
    { 0.587785, 0.809017 }, { 0.573576, 0.819152 }, { 0.559193, 0.829038 },
    { 0.544639, 0.838671 }, { 0.529919, 0.848048 }, { 0.515038, 0.857167 },
    { 0.500000, 0.866025 }, { 0.484810, 0.874620 }, { 0.469472, 0.882948 },
    { 0.453990, 0.891007 }, { 0.438371, 0.898794 }, { 0.422618, 0.906308 },
    { 0.406737, 0.913545 }, { 0.390731, 0.920505 }, { 0.374607, 0.927184 },
    { 0.358368, 0.933580 }, { 0.342020, 0.939693 }, { 0.325568, 0.945519 },
    { 0.309017, 0.951057 }, { 0.292372, 0.956305 }, { 0.275637, 0.961262 },
    { 0.258819, 0.965926 }, { 0.241922, 0.970296 }, { 0.224951, 0.974370 },
    { 0.207912, 0.978148 }, { 0.190809, 0.981627 }, { 0.173648, 0.984808 },
    { 0.156434, 0.987688 }, { 0.139173, 0.990268 }, { 0.121869, 0.992546 },
    { 0.104528, 0.994522 }, { 0.087156, 0.996195 }, { 0.069756, 0.997564 },
    { 0.052336, 0.998630 }, { 0.034899, 0.999391 }, { 0.017452, 0.999848 },
    { 0.000000, 1.000000 }, { -0.017452, 0.999848 }, { -0.034899, 0.999391 },
    { -0.052336, 0.998630 }, { -0.069756, 0.997564 }, { -0.087156, 0.996195 },
    { -0.104528, 0.994522 }, { -0.121869, 0.992546 }, { -0.139173, 0.990268 },
    { -0.156434, 0.987688 }, { -0.173648, 0.984808 }, { -0.190809, 0.981627 },
    { -0.207912, 0.978148 }, { -0.224951, 0.974370 }, { -0.241922, 0.970296 },
    { -0.258819, 0.965926 }, { -0.275637, 0.961262 }, { -0.292372, 0.956305 },
    { -0.309017, 0.951057 }, { -0.325568, 0.945519 }, { -0.342020, 0.939693 },
    { -0.358368, 0.933580 }, { -0.374607, 0.927184 }, { -0.390731, 0.920505 },
    { -0.406737, 0.913545 }, { -0.422618, 0.906308 }, { -0.438371, 0.898794 },
    { -0.453990, 0.891007 }, { -0.469472, 0.882948 }, { -0.484810, 0.874620 },
    { -0.500000, 0.866025 }, { -0.515038, 0.857167 }, { -0.529919, 0.848048 },
    { -0.544639, 0.838671 }, { -0.559193, 0.829038 }, { -0.573576, 0.819152 },
    { -0.587785, 0.809017 }, { -0.601815, 0.798636 }, { -0.615661, 0.788011 },
    { -0.629320, 0.777146 }, { -0.642788, 0.766044 }, { -0.656059, 0.754710 },
    { -0.669131, 0.743145 }, { -0.681998, 0.731354 }, { -0.694658, 0.719340 },
    { -0.707107, 0.707107 }, { -0.719340, 0.694658 }, { -0.731354, 0.681998 },
    { -0.743145, 0.669131 }, { -0.754710, 0.656059 }, { -0.766044, 0.642788 },
    { -0.777146, 0.629320 }, { -0.788011, 0.615661 }, { -0.798636, 0.601815 },
    { -0.809017, 0.587785 }, { -0.819152, 0.573576 }, { -0.829038, 0.559193 },
    { -0.838671, 0.544639 }, { -0.848048, 0.529919 }, { -0.857167, 0.515038 },
    { -0.866025, 0.500000 }, { -0.874620, 0.484810 }, { -0.882948, 0.469472 },
    { -0.891007, 0.453990 }, { -0.898794, 0.438371 }, { -0.906308, 0.422618 },
    { -0.913545, 0.406737 }, { -0.920505, 0.390731 }, { -0.927184, 0.374607 },
    { -0.933580, 0.358368 }, { -0.939693, 0.342020 }, { -0.945519, 0.325568 },
    { -0.951057, 0.309017 }, { -0.956305, 0.292372 }, { -0.961262, 0.275637 },
    { -0.965926, 0.258819 }, { -0.970296, 0.241922 }, { -0.974370, 0.224951 },
    { -0.978148, 0.207912 }, { -0.981627, 0.190809 }, { -0.984808, 0.173648 },
    { -0.987688, 0.156434 }, { -0.990268, 0.139173 }, { -0.992546, 0.121869 },
    { -0.994522, 0.104528 }, { -0.996195, 0.087156 }, { -0.997564, 0.069756 },
    { -0.998630, 0.052336 }, { -0.999391, 0.034899 }, { -0.999848, 0.017452 },
    { -1.000000, 0.000000 }, { -0.999848, -0.017452 }, { -0.999391, -0.034899 },
    { -0.998630,-0.052336 }, { -0.997564, -0.069756 }, { -0.996195, -0.087156 },
    { -0.994522,-0.104528 }, { -0.992546, -0.121869 }, { -0.990268, -0.139173 },
    { -0.987688,-0.156434 }, { -0.984808, -0.173648 }, { -0.981627, -0.190809 },
    { -0.978148,-0.207912 }, { -0.974370, -0.224951 }, { -0.970296, -0.241922 },
    { -0.965926,-0.258819 }, { -0.961262, -0.275637 }, { -0.956305, -0.292372 },
    { -0.951057,-0.309017 }, { -0.945519, -0.325568 }, { -0.939693, -0.342020 },
    { -0.933580,-0.358368 }, { -0.927184, -0.374607 }, { -0.920505, -0.390731 },
    { -0.913545,-0.406737 }, { -0.906308, -0.422618 }, { -0.898794, -0.438371 },
    { -0.891007,-0.453990 }, { -0.882948, -0.469472 }, { -0.874620, -0.484810 },
    { -0.866025,-0.500000 }, { -0.857167, -0.515038 }, { -0.848048, -0.529919 },
    { -0.838671,-0.544639 }, { -0.829038, -0.559193 }, { -0.819152, -0.573576 },
    { -0.809017,-0.587785 }, { -0.798636, -0.601815 }, { -0.788011, -0.615661 },
    { -0.777146,-0.629320 }, { -0.766044, -0.642788 }, { -0.754710, -0.656059 },
    { -0.743145,-0.669131 }, { -0.731354, -0.681998 }, { -0.719340, -0.694658 },
    { -0.707107,-0.707107 }, { -0.694658, -0.719340 }, { -0.681998, -0.731354 },
    { -0.669131,-0.743145 }, { -0.656059, -0.754710 }, { -0.642788, -0.766044 },
    { -0.629320,-0.777146 }, { -0.615661, -0.788011 }, { -0.601815, -0.798636 },
    { -0.587785,-0.809017 }, { -0.573576, -0.819152 }, { -0.559193, -0.829038 },
    { -0.544639,-0.838671 }, { -0.529919, -0.848048 }, { -0.515038, -0.857167 },
    { -0.500000,-0.866025 }, { -0.484810, -0.874620 }, { -0.469472, -0.882948 },
    { -0.453990,-0.891007 }, { -0.438371, -0.898794 }, { -0.422618, -0.906308 },
    { -0.406737,-0.913545 }, { -0.390731, -0.920505 }, { -0.374607, -0.927184 },
    { -0.358368,-0.933580 }, { -0.342020, -0.939693 }, { -0.325568, -0.945519 },
    { -0.309017,-0.951057 }, { -0.292372, -0.956305 }, { -0.275637, -0.961262 },
    { -0.258819,-0.965926 }, { -0.241922, -0.970296 }, { -0.224951, -0.974370 },
    { -0.207912,-0.978148 }, { -0.190809, -0.981627 }, { -0.173648, -0.984808 },
    { -0.156434,-0.987688 }, { -0.139173, -0.990268 }, { -0.121869, -0.992546 },
    { -0.104528,-0.994522 }, { -0.087156, -0.996195 }, { -0.069756, -0.997564 },
    { -0.052336,-0.998630 }, { -0.034899, -0.999391 }, { -0.017452, -0.999848 },
    { -0.000000, -1.000000 }, { 0.017452, -0.999848 }, { 0.034899, -0.999391 },
    { 0.052336, -0.998630 }, { 0.069756, -0.997564 }, { 0.087156, -0.996195 },
    { 0.104528, -0.994522 }, { 0.121869, -0.992546 }, { 0.139173, -0.990268 },
    { 0.156434, -0.987688 }, { 0.173648, -0.984808 }, { 0.190809, -0.981627 },
    { 0.207912, -0.978148 }, { 0.224951, -0.974370 }, { 0.241922, -0.970296 },
    { 0.258819, -0.965926 }, { 0.275637, -0.961262 }, { 0.292372, -0.956305 },
    { 0.309017, -0.951057 }, { 0.325568, -0.945519 }, { 0.342020, -0.939693 },
    { 0.358368, -0.933580 }, { 0.374607, -0.927184 }, { 0.390731, -0.920505 },
    { 0.406737, -0.913545 }, { 0.422618, -0.906308 }, { 0.438371, -0.898794 },
    { 0.453990, -0.891007 }, { 0.469472, -0.882948 }, { 0.484810, -0.874620 },
    { 0.500000, -0.866025 }, { 0.515038, -0.857167 }, { 0.529919, -0.848048 },
    { 0.544639, -0.838671 }, { 0.559193, -0.829038 }, { 0.573576, -0.819152 },
    { 0.587785, -0.809017 }, { 0.601815, -0.798636 }, { 0.615661, -0.788011 },
    { 0.629320, -0.777146 }, { 0.642788, -0.766044 }, { 0.656059, -0.754710 },
    { 0.669131, -0.743145 }, { 0.681998, -0.731354 }, { 0.694658, -0.719340 },
    { 0.707107, -0.707107 }, { 0.719340, -0.694658 }, { 0.731354, -0.681998 },
    { 0.743145, -0.669131 }, { 0.754710, -0.656059 }, { 0.766044, -0.642788 },
    { 0.777146, -0.629320 }, { 0.788011, -0.615661 }, { 0.798636, -0.601815 },
    { 0.809017, -0.587785 }, { 0.819152, -0.573576 }, { 0.829038, -0.559193 },
    { 0.838671, -0.544639 }, { 0.848048, -0.529919 }, { 0.857167, -0.515038 },
    { 0.866025, -0.500000 }, { 0.874620, -0.484810 }, { 0.882948, -0.469472 },
    { 0.891007, -0.453990 }, { 0.898794, -0.438371 }, { 0.906308, -0.422618 },
    { 0.913545, -0.406737 }, { 0.920505, -0.390731 }, { 0.927184, -0.374607 },
    { 0.933580, -0.358368 }, { 0.939693, -0.342020 }, { 0.945519, -0.325568 },
    { 0.951057, -0.309017 }, { 0.956305, -0.292372 }, { 0.961262, -0.275637 },
    { 0.965926, -0.258819 }, { 0.970296, -0.241922 }, { 0.974370, -0.224951 },
    { 0.978148, -0.207912 }, { 0.981627, -0.190809 }, { 0.984808, -0.173648 },
    { 0.987688, -0.156434 }, { 0.990268, -0.139173 }, { 0.992546, -0.121869 },
    { 0.994522, -0.104528 }, { 0.996195, -0.087156 }, { 0.997564, -0.069756 },
    { 0.998630, -0.052336 }, { 0.999391, -0.034899 }, { 0.999848, -0.017452 },
};

void
bot_gl_build_circle( GLuint id)
{
    glNewList(id, GL_COMPILE);
    
    GLfloat cosine, sine;
    int i;
    glBegin(GL_LINE_LOOP);
    for(i=0;i<100;i++){
        cosine=cos(i*2*3.14159/100.0);
        sine=sin(i*2*3.14159/100.0);
        glVertex2f(cosine,sine);
    }
    glEnd();
    
    glEndList();  
}

void 
bot_gl_draw_circle (double r)
{
    glPushMatrix ();
    glScalef (r, r, 0);
    glBegin (GL_LINE_LOOP);

    // decimate = 1 means no decimation
    int decimate = 4;

    for (int i=0; i<sizeof (_circle_points) / (2 * sizeof (float)); i+=decimate) {
        glVertex2f (_circle_points[i][0], _circle_points[i][1]);
    }
    glEnd ();
    glPopMatrix ();
}

void 
bot_gl_draw_disk(double r)
{
    GLUquadricObj *q = gluNewQuadric();
    gluDisk(q, 0, r, 15, 1);
    gluDeleteQuadric(q);
}

void 
bot_gl_draw_ellipse (double a, double b, double angle, int steps)
{
    // from: http://en.wikipedia.org/wiki/Ellipse

    double beta = angle;
    double sinbeta, cosbeta;
    bot_fasttrig_sincos (beta, &sinbeta, &cosbeta);

    glBegin (GL_LINE_LOOP);
    for (int i=0; i<360; i+= 360/steps) {
        double cosalpha = _circle_points[i][0];
        double sinalpha = _circle_points[i][1]; 

        double x = b * cosalpha * cosbeta - a * sinalpha * sinbeta;
        double y = b * cosalpha * sinbeta + a * sinalpha * cosbeta;

        glVertex2f (x, y);
    }
    glEnd ();
}

void bot_gl_line(double x_start, double y_start, double x_end, double y_end)
{
  glBegin(GL_LINES);
  glVertex2d(x_start, y_start);
  glVertex2d(x_end, y_end);
  glEnd();
}

void
bot_gl_draw_ortho_circles_3d()
{
  bot_gl_draw_circle(1);
  bot_gl_line(-1, 0, 1, 0);
  bot_gl_line(0, -1, 0, 1);

  glPushMatrix();
  glRotated(90,1,0,0);
  bot_gl_draw_circle(1);
  bot_gl_line(-1, 0, 1, 0);
  bot_gl_line(0, -1, 0, 1);
  glPopMatrix();


  glPushMatrix();
  glRotated(90,0,1,0);
  bot_gl_draw_circle(1);
  bot_gl_line(-1, 0, 1, 0);
  bot_gl_line(0, -1, 0, 1);
  glPopMatrix();
}

//void
//bot_gl_draw_text (double x, double y, const char *text)
//{
//    glRasterPos2f(x,y);
//
//    for (int i=0;i<strlen(text);i++) {
//        glutBitmapCharacter (GLUT_BITMAP_8_BY_13, text[i]);
//    }
//}
//
//void
//bot_gl_draw_text_window (double x, double y, const char *text, int w, int h)
//{
//    glMatrixMode(GL_PROJECTION);
//    glPushMatrix();
//    glLoadIdentity();
//    gluOrtho2D(0,w,0,h);
//
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glLoadIdentity();
//
//    bot_gl_draw_text (x, y, text);
//
//    glMatrixMode(GL_PROJECTION);
//    glPopMatrix();
//    glMatrixMode(GL_MODELVIEW);
//    glPopMatrix();
//} 

void 
bot_gl_draw_arrow_2d (double length, double head_width, double head_length,
        double body_width, int fill)
{
    if (fill) {
        glBegin (GL_TRIANGLES);
        // draw the head
        glVertex2d (length / 2, 0);
        glVertex2d (length / 2 - head_length, head_width / 2);
        glVertex2d (length / 2 - head_length, - head_width / 2);
        glEnd ();
        glBegin (GL_QUADS);
        // draw the body
        glVertex2d (length / 2 - head_length, body_width / 2);
        glVertex2d (length / 2 - head_length, - body_width / 2);
        glVertex2d (- length / 2, - body_width / 2);
        glVertex2d (- length / 2, body_width / 2);
        glEnd ();
    } else {
        glBegin (GL_LINE_LOOP);
        glVertex2d (length / 2, 0);
        glVertex2d (length / 2 - head_length, head_width / 2);
        glVertex2d (length / 2 - head_length, body_width / 2);
        glVertex2d (- length / 2, body_width / 2);
        glVertex2d (- length / 2, - body_width / 2);
        glVertex2d (length / 2 - head_length, - body_width / 2);
        glVertex2d (length / 2 - head_length, - head_width / 2);
        glEnd ();
    }
}

void
bot_gl_draw_arrow_3d (double length, double head_width, double head_length,
        double body_width)
{
    int slices = 20;
    int stacks = 20;

    //apply translations so the drawing is centered at origin along the x axis per bot_gl_draw_arrow_2d
    glPushMatrix();
    glTranslated(-length / 2, 0, 0);
    glRotated(90, 0, 1, 0);

    glPushAttrib(GL_ENABLE_BIT);
    glEnable(GL_DEPTH_TEST);

    GLUquadricObj *q = gluNewQuadric();

    //draw body
    gluCylinder(q, body_width, body_width, length - head_length, slices, stacks);

    //draw head
    glTranslated(0, 0, length - head_length);
    gluCylinder(q, head_width, 0, head_length, slices, stacks);
    gluDeleteQuadric(q);

    glPopAttrib();
    glPopMatrix();
}

int bot_glutBitmapHeight(void* font);
void bot_glutBitmapString(void* font, const unsigned char* text);

int
bot_glutBitmapHeight(void* font)
{
#ifdef FREEGLUT
    return glutBitmapHeight(font);
#else
    if( font == GLUT_BITMAP_8_BY_13        )
        return 14;
    if( font == GLUT_BITMAP_9_BY_15        )
        return 16;
    if( font == GLUT_BITMAP_HELVETICA_10   )
        return 14;
    if( font == GLUT_BITMAP_HELVETICA_12   )
        return 16;
    if( font == GLUT_BITMAP_HELVETICA_18   )
        return 23;
    if( font == GLUT_BITMAP_TIMES_ROMAN_10 )
        return 14;
    if( font == GLUT_BITMAP_TIMES_ROMAN_24 )
        return 29;
    return 20;
#endif
}

// Adapted from FreeGLUT implementation of glutBitmapString
void
bot_glutBitmapString(void* font, const unsigned char* text)
{
#ifdef FREEGLUT
    glutBitmapString(font, text);
#else
    unsigned char c;
    float x = 0.0f ;
    int font_height = bot_glutBitmapHeight (font);
    if ( !text || ! *text )
        return;

    glPushClientAttrib( GL_CLIENT_PIXEL_STORE_BIT );
    glPixelStorei( GL_UNPACK_SWAP_BYTES,  GL_FALSE );
    glPixelStorei( GL_UNPACK_LSB_FIRST,   GL_FALSE );
    glPixelStorei( GL_UNPACK_ROW_LENGTH,  0        );
    glPixelStorei( GL_UNPACK_SKIP_ROWS,   0        );
    glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0        );
    glPixelStorei( GL_UNPACK_ALIGNMENT,   1        );

    /*
     * Step through the string, drawing each character.
     * A newline will simply translate the next character's insertion
     * point back to the start of the line and down one line.
     */
    while( ( c = *text++) )
        if( c == '\n' ) {
            // Move the raster position down a line and to the left
            glBitmap ( 0, 0, 0, 0, -x, (float) -font_height, NULL );
            x = 0.0f;
        }
        else { /* Not an EOL, draw the bitmap character */
            
            glutBitmapCharacter (font, c);
            
            // Add to the known width of the current line
            x += ( float )( glutBitmapWidth (font, c) );
        }
    
    glPopClientAttrib( );
#endif
}

void bot_gl_draw_text (const double xyz[3], void *font, const char *text, int flags)
{
    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    if (font == NULL) {
        if (flags & BOT_GL_DRAW_TEXT_MONOSPACED) 
            font = GLUT_BITMAP_8_BY_13;
        else
            font = GLUT_BITMAP_HELVETICA_12;
    }

    glGetDoublev(GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev(GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv(GL_VIEWPORT, viewport);

    double winxy[2];
    
    // compute pixel coordinates corresponding to xyz input
    if (flags & BOT_GL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES) {
        winxy[0] = xyz[0] * viewport[2];
        winxy[1] = (1.0 - xyz[1]) * viewport[3];
    } else {
        // xyz is in modelview space
        double bogus;
        if (!gluProject(xyz[0], xyz[1], xyz[2], 
                        model_matrix, proj_matrix, viewport, 
                        &winxy[0], &winxy[1], &bogus)) {
            printf("GluProject failure\n");
            return;
        }
    }

    // save original matrices
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT);

    // setup very dumb projection in pixel coordinates
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0,viewport[2],0,viewport[3]);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    GPtrArray *text_lines = g_ptr_array_new();
    int start = 0;
    int text_len = strlen(text);
    for (int i = 0; i < text_len; i++) {
        if (text[i]=='\n' || i == text_len-1) {
            int line_len = i - start;

            // delete the \n if it was a \n, but keep the last char if it wasn't a \n
            if (text[i] != '\n')
                line_len ++;

            char *line = malloc(line_len + 1); // plus \0
            memcpy(line, &text[start], line_len);
            line[line_len] = 0;
            g_ptr_array_add(text_lines, line);
            start = i+1;
        }
    }
    
    // compute the bounding dimensions of this text.
    int line_height = bot_glutBitmapHeight(font);
    int height = line_height * text_lines->len;
    int max_width = 0;

    for (int i = 0; i < text_lines->len; i++) {
        unsigned char *line = g_ptr_array_index(text_lines, i);
        int thiswidth = glutBitmapLength(font, line);
        max_width = (thiswidth > max_width) ? thiswidth : max_width;
    }

    if (flags & BOT_GL_DRAW_TEXT_ANCHOR_TOP)
        winxy[1] -= height/2;

    if (flags & BOT_GL_DRAW_TEXT_ANCHOR_BOTTOM)
        winxy[1] += height/2;

    if (flags & BOT_GL_DRAW_TEXT_ANCHOR_LEFT)
        winxy[0] += max_width / 2;

    if (flags & BOT_GL_DRAW_TEXT_ANCHOR_RIGHT)
        winxy[0] -= max_width / 2;

  // drop shadow
    if (flags & BOT_GL_DRAW_TEXT_DROP_SHADOW) {
        glPushAttrib(GL_CURRENT_BIT);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glColor4f(0,0,0,.6);
        glBegin(GL_QUADS);
        double hmargin = 3; // margin in pixels
        double vmargin_top = 0;
        double vmargin_bottom = 5;
        double x0=winxy[0] - max_width/2 - hmargin, x1 = winxy[0] + max_width/2 + hmargin;
        double y0=winxy[1] - height/2.0 - vmargin_bottom, y1 = winxy[1] + height/2.0 + vmargin_top;
        glVertex2f(x0, y0);
        glVertex2f(x0, y1);
        glVertex2f(x1, y1);
        glVertex2f(x1, y0);
        glEnd();
        glPopAttrib();
    }

    // draw text
//    glColor4f(1,1,1,1);
    // winxy is now the center of the draw.
    for (int i = 0; i < text_lines->len; i++) {
        unsigned char *line = g_ptr_array_index(text_lines, i);
        int thiswidth = glutBitmapLength(font, line);

        int y = winxy[1] - height/2 + (text_lines->len - 1 - i) * line_height;
        int x = winxy[0] - thiswidth/2; // default = justify center

        if (flags & BOT_GL_DRAW_TEXT_JUSTIFY_LEFT)
            x = winxy[0] - max_width/2;
        if (flags & BOT_GL_DRAW_TEXT_JUSTIFY_RIGHT)
            x = winxy[0] + max_width/2 - thiswidth;
        if (flags & BOT_GL_DRAW_TEXT_JUSTIFY_CENTER)
            x = winxy[0] - thiswidth/2;

//        printf("drawtext: %15f,%15f %15f,%15f %d,%d\n", xyz[0], xyz[1], winxy[0], winxy[1], x, y);
        glRasterPos2f(x,y);
        bot_glutBitmapString(font, line);
    }

    // restore original matrices
    glPopAttrib();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    for (int i = 0; i < text_lines->len; i++)
        free(g_ptr_array_index(text_lines, i));
    
    g_ptr_array_free(text_lines, TRUE);
}

int 
_bot_gl_check_errors(const char *file, int line)
{
    int errCount = 0;
    GLenum errCode = glGetError();
    
    while (errCode != GL_NO_ERROR) {
        errCount++;
        const char *errStr = (char*)gluErrorString(errCode);
        fprintf(stderr, "[%s:%d] OpenGL Error: '%s' (%d)\n", file, line, 
                errStr ? errStr : "unknown", errCode);
        errCode = glGetError();
    }

    return errCount;
}

void
bot_gl_print_current_matrix(void)
{
    int matrix_mode;
    glGetIntegerv(GL_MATRIX_MODE, &matrix_mode);
    GLenum matrix_get_name;
    int stack_depth;
    char *matrix_name;
    switch (matrix_mode) {
    case GL_MODELVIEW:
        matrix_get_name = GL_MODELVIEW_MATRIX;
        glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &stack_depth);
        matrix_name = "MODEL VIEW";
        break;
    case GL_PROJECTION:
        matrix_get_name = GL_PROJECTION_MATRIX;
        glGetIntegerv(GL_PROJECTION_STACK_DEPTH, &stack_depth);
        matrix_name = "PROJECTION";
        break;
    case GL_TEXTURE:
        matrix_get_name = GL_TEXTURE_MATRIX;
        glGetIntegerv(GL_TEXTURE_STACK_DEPTH, &stack_depth);
        matrix_name = "TEXTURE";
        break;
    case GL_COLOR:
        matrix_get_name = GL_COLOR_MATRIX;
        glGetIntegerv(GL_COLOR_MATRIX_STACK_DEPTH, &stack_depth);
        matrix_name = "COLOR";
        break;
    default:
        g_assert("Error, unkown matrix mode\n");
        return;
    }

    double mtx_val[16];
    glGetDoublev(matrix_get_name, mtx_val);

    printf("%s (%d)\n", matrix_name, stack_depth);
    printf("  %8.3f %8.3f %8.3f %8.3f\n", 
           mtx_val[0], mtx_val[4], mtx_val[8], mtx_val[12]);
    printf("  %8.3f %8.3f %8.3f %8.3f\n", 
           mtx_val[1], mtx_val[5], mtx_val[9], mtx_val[13]);
    printf("  %8.3f %8.3f %8.3f %8.3f\n", 
           mtx_val[2], mtx_val[6], mtx_val[10], mtx_val[14]);
    printf("  %8.3f %8.3f %8.3f %8.3f\n", 
           mtx_val[3], mtx_val[7], mtx_val[11], mtx_val[15]);
}

void bot_gl_draw_axes()
{
  //x-axis
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //y-axis
  glBegin(GL_LINES);
  glColor3f(0, 1, 0);
  glVertex3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //z-axis
  glBegin(GL_LINES);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glEnd();

}

void bot_gl_multTrans(BotTrans * trans){
  double trans_m[16];
  bot_trans_get_mat_4x4(trans, trans_m);
  // opengl expects column-major matrices
  double trans_m_opengl[16];
  bot_matrix_transpose_4x4d(trans_m, trans_m_opengl);
  glMultMatrixd(trans_m_opengl);
}
