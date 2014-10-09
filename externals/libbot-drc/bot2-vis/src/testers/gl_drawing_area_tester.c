#include <stdio.h>
#include <stdlib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <bot_vis/bot_vis.h>

typedef struct _point2d {
    double x;
    double y;
} point2d_t;

typedef struct _state_t {
    BotGtkGlDrawingArea *gl_area;
    GtkWidget *window;
    GLUquadricObj *quadric;
    GList *todraw;

    point2d_t last_mouse;
} state_t;

static gboolean
on_gl_expose (GtkWidget * widget, GdkEventExpose * event, gpointer user_data)
{
    state_t * self = (state_t *) user_data;
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    
    glColor3f( 0, 1, 0 );
    glBegin(GL_LINE_STRIP);
    GList *piter;
    for( piter=self->todraw; piter; piter=piter->next ) {
        point2d_t *point = (point2d_t*) piter->data;
        glVertex2f( point->x, point->y );
    }
    glEnd();

    glColor3f( 1, 1, 0 );
    glPushMatrix();
    glTranslatef(self->last_mouse.x, self->last_mouse.y, 0);
    gluDisk( self->quadric, 0, 0.02, 50, 1 );
    glPopMatrix();

    bot_gtk_gl_drawing_area_swap_buffers(self->gl_area);
    return TRUE;
}

static gboolean 
on_button_press( GtkWidget *widget, GdkEventButton *event, void *user_data )
{
    state_t *self = (state_t*) user_data;

    if( event->button == 1 ) {
        point2d_t *newpoint = (point2d_t*) malloc( sizeof(point2d_t) );
        newpoint->x = event->x / widget->allocation.width;
        newpoint->y = event->y / widget->allocation.height;
        self->todraw = g_list_append( self->todraw, newpoint );
    } else if( event->button == 3 ) {
        GList *piter;
        for( piter=self->todraw; piter; piter=piter->next ) free(piter->data);
        g_list_free( self->todraw );
        self->todraw = NULL;
    }

    bot_gtk_gl_drawing_area_invalidate (self->gl_area);
    return TRUE;
}

static gboolean
on_button_release( GtkWidget *widget, GdkEventButton *event, void *user_data )
{
//    state_t *self = (state_t*) user_data;
    g_print("button %d released\n", event->button);
    return TRUE;
}

/**
 * this function is called when the user moves the mouse pointer over the
 * opengl window.  Use the event->state field to determine whether a button is
 * pressed or not
 */
static gboolean 
on_motion_notify( GtkWidget *widget, GdkEventMotion *event, void *user_data )
{
    state_t *self = (state_t*) user_data;
    self->last_mouse.x = event->x / widget->allocation.width;
    self->last_mouse.y = event->y / widget->allocation.height;
    bot_gtk_gl_drawing_area_invalidate (self->gl_area);
    return TRUE;
}

static gboolean
on_key_press (GtkWidget *widget, GdkEventKey *event, void *user_data)
{
    printf ("key pressed\n");
    return TRUE;
}

static void
setup_gtk( state_t *self )
{
    // create the main application window
    self->window = gtk_window_new( GTK_WINDOW_TOPLEVEL );
    g_signal_connect( G_OBJECT(self->window), "delete_event", 
            gtk_main_quit, NULL );
    g_signal_connect( G_OBJECT(self->window), "destroy", 
            gtk_main_quit, NULL );
    gtk_window_set_default_size(GTK_WINDOW(self->window), 600, 400);
    gtk_container_set_border_width( GTK_CONTAINER(self->window), 10 );

    // create the aspect area to maintain a 1:1 aspect ratio

    GtkWidget *aspect = gtk_aspect_frame_new( NULL, 0.5, 0.5, 1, FALSE );
    gtk_container_add(GTK_CONTAINER(self->window), aspect);
    gtk_widget_show(aspect);

    self->gl_area = BOT_GTK_GL_DRAWING_AREA(bot_gtk_gl_drawing_area_new(TRUE));
    gtk_widget_set_events(GTK_WIDGET(self->gl_area), 
            GDK_LEAVE_NOTIFY_MASK |
            GDK_BUTTON_PRESS_MASK | 
            GDK_BUTTON_RELEASE_MASK | 
            GDK_POINTER_MOTION_MASK |
            GDK_KEY_PRESS_MASK
            );
    gtk_container_add (GTK_CONTAINER(aspect), GTK_WIDGET(self->gl_area));
    gtk_widget_show (GTK_WIDGET (self->gl_area));

    g_signal_connect(G_OBJECT(self->gl_area), "expose-event",
            G_CALLBACK(on_gl_expose), self);
    g_signal_connect(G_OBJECT(self->gl_area), "button-press-event",
            G_CALLBACK(on_button_press), self);
    g_signal_connect(G_OBJECT(self->gl_area), "button-release-event",
            G_CALLBACK(on_button_release), self);
    g_signal_connect(G_OBJECT(self->gl_area), "motion-notify-event",
            G_CALLBACK(on_motion_notify), self);

    GTK_WIDGET_SET_FLAGS (GTK_WIDGET (self->gl_area), GTK_CAN_FOCUS);
    g_signal_connect(G_OBJECT(self->gl_area), "key-press-event",
            G_CALLBACK(on_key_press), self);

    gtk_widget_show( self->window );
}

static void
setup_opengl( state_t *self )
{
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    glClearColor(0,0,0,0);
    glOrtho( 0, 1, 1, 0, -1, 1 );
    self->quadric = gluNewQuadric();
    self->last_mouse.x = -1;
    self->last_mouse.y = -1;
}

int main(int argc, char **argv)
{
    gtk_init(&argc, &argv);

    state_t *self = (state_t*)calloc(1, sizeof(state_t));

    setup_gtk( self );

    setup_opengl( self );

    gtk_main();

    GList *piter;
    for( piter=self->todraw; piter; piter=piter->next ) free( piter->data );
    g_list_free( self->todraw );
    self->todraw = NULL;

    return 0;
}
