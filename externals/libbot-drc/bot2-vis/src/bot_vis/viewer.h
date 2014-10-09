#ifndef __bot2_viewer_h__
#define __bot2_viewer_h__

#include <inttypes.h>
#include <gtk/gtk.h>
#include <glib-object.h>

#include <zlib.h>

#include "gtk_util.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup BotViewer BotViewer
 * @brief Graphical debugging utility
 * @ingroup BotViewerGroup
 * @include: bot_vis/bot_vis.h
 *
 * Data structures and functions for displaying debugging information.
 *
 * Linking: `pkg-config --libs bot2-vis`
 * @{
 */

typedef struct _BotViewer BotViewer;
typedef struct _BotViewHandler BotViewHandler;
typedef struct _BotEventHandler BotEventHandler;

/**
 */
struct BotViewerMode
{
    int           mode;
    const char   *name;
    GtkMenuItem  *menu_item;
};

/*
 * follow_mode bit flags
 */
#define BOT_FOLLOW_POS 1
#define BOT_FOLLOW_YAW 2
#define BOT_FOLLOW_ORIENTATION 4

typedef enum {
  BOT_VIEW_ORTHOGRAPHIC,
  BOT_VIEW_PERSPECTIVE
} BotProjectionMode;

/*
 * View updating methods.
 */
struct _BotViewHandler
{
    void (*update_gl_matrices)  (BotViewer *viewer, BotViewHandler *vhandler);

    void (*get_eye_look)        (BotViewHandler *vhandler, double eye[3], 
                                 double lookat[3], double up[3]);

    void (*update_follow_target)(BotViewHandler *vhandler, const double pos[3], 
                                 const double quat[4]);
  
  void (*set_look_at_smooth) (BotViewHandler *vhandler, const double eye[3], const double lookat[3], const double up[3], double duration_ms);
   
    void (*set_look_at)         (BotViewHandler *vhandler, const double eye[3], 
                                 const double lookat[3], const double up[3]);
    
    void (*set_camera_perspective) (BotViewHandler *vhandler, double fov_degrees);

    void (*set_camera_orthographic) (BotViewHandler *vhandler);

    BotProjectionMode (*get_projection_mode)    (BotViewHandler *vhandler);
  /**
  * returns the vertical FOV when the camera is in perspective mode.
  * Results undefined when the camera is in orthographic mode.
  */
    double (*get_perspective_fov)(BotViewHandler *vhandler);
    
    void (*destroy)             (BotViewHandler *vhandler);

    int  follow_mode;
    void *user;
};

/**
 * BotEventHandler:
 * @name: a name used in the menu 
 * @enabled: Completely enable/disable the event handler.
 * @priority: Higher priority handlers are updated first. Do not set this
 * directly; only via set_priority.
 * @picking: Set when a picker wins the pick_query competition and the user
 * clicks.  When set, it indicates that the handler has "first dibs" on any
 * events.  Only one handler can be picking simulatenously.
 * @hovering: Set when a picker wins the hover_query competition and the user
 * hovers. Typically, this indicates that a click would cause the picking flag
 * to be set. Only one handler can be hovering simultaneously.
 * @pick_query: When a mouse press event occurs, all BotEventHandlers will
 * compete for the sequence of events that follow. The BotEventHandler that
 * returns the smallest distance will receive a pick_notify event.  Return < 0
 * if the renderer has no object to pick.
 * @hover_query: can usually be the same function pointer as pick_query. 
 * @mouse_press: Event handling methods. Return non-zero if the event was
 * consumed by this handler (and further processing should stop).
 * @mouse_release: If you register a pick_query handler, you should almost
 * certainly register a mouse_release function to set picking = 0.
 * @mouse_motion:
 * @mouse_scroll:
 * @key_press:
 * @key_release:
 * @destroy:
 * @user:
 * @cmi: enable checkbox (?)
 *
 * We support a couple different modes of operation. First, is
 * "picking": picking allows an event handler to capture user
 * input when the user clicks on some object (managed by the event
 * handler). This allows the object to be manipulated in an
 * event-handler-specific way (e.g., moved, rotated).
 *
 * "Hovering" is similar to picking, but hovering queries occur
 * when the mouse is merely moved, and does not affect the routing
 * of events. This allows a renderer to display an object
 * differently, so that the user knows the object can be clicked
 * on (thus causing the object to be "picked"). 
 *
 * Both "picking" and "hovering" call a query function; it should
 * return the distance from the click to an object. The event
 * handler that returns the smallest distance will have its
 * "picking" or "hovering" flag set. When "picking", the event
 * handler will have first "dibs" on any event that subsequently
 * occurs, regardless of the event handler's priority. If the
 * event handler has no object near the click, it should return <
 * 0.
 *
 * Finally, when an event occurs:
 *
 * If no one is picking, and the event is mouse_down, pick_query
 * is called for all event handlers. The winner will have its
 * picking flag set, which will give it first dibs on subsequent
 * events.
 *
 * If no one is picking, and the event is mouse_motion, the
 * hover_query methods are called, and the winning event handler
 * has its "hovering" flag set.
 * 
 * Finally, we find an event handler to consume the event. The
 * picking event handler, if any, gets first dibs. All other
 * handlers are then called in order of decreasing priority.
 * 
 * Only one handler "consumes" an event: if an event handler
 * returns TRUE, it ends the event processing.
 */
struct _BotEventHandler
{
    char *name;
    int enabled;
    int priority;
    int picking;
    int hovering;

    double (*pick_query)(BotViewer *viewer, BotEventHandler *ehandler, 
            const double ray_start[3], const double ray_dir[3]);

    double (*hover_query)(BotViewer *viewer, BotEventHandler *ehandler, 
            const double ray_start[3], const double ray_dir[3]);

    int (*mouse_press)   (BotViewer *viewer, BotEventHandler *ehandler,
                          const double ray_start[3], const double ray_dir[3], 
                          const GdkEventButton *event);

    int (*mouse_release) (BotViewer *viewer, BotEventHandler *ehandler,
                          const double ray_start[3], const double ray_dir[3], 
                          const GdkEventButton *event);

    int (*mouse_motion)  (BotViewer *viewer, BotEventHandler *ehandler,
                          const double ray_start[3], const double ray_dir[3], 
                          const GdkEventMotion *event);

    int (*mouse_scroll)  (BotViewer *viewer, BotEventHandler *ehandler,
                          const double ray_start[3], const double ray_dir[3],
                          const GdkEventScroll *event);

    int  (*key_press)     (BotViewer *viewer, BotEventHandler *ehandler, 
            const GdkEventKey  *event);

    int  (*key_release)     (BotViewer *viewer, BotEventHandler *ehandler, 
            const GdkEventKey  *event);

    void (*destroy)       (BotEventHandler *ehandler);

    void *user;
    GtkWidget         *cmi;
};

/**
 * BotRenderer:
 * @enabled: Whether or not to draw this renderer.
 * @priority: The priority with which this renderer will be drawn
 * @name: The name of this renderer.
 * @widget:
 * @draw: Drawing this renderer.
 * @destroy: Destroy this renderer (when done).
 * @user: User data to be used by the subclass.
 * @expanded: TRUE when this is expanded in the side pane.
 * @cmi: Enable checkbox.
 * @expander:
 * @control_frame:
 *
 * These are the meat-and-bones of the viewer.  Generally a viewer consists of
 * several renderers.  Each renderer renders one specific thing.  Common
 * renderers are:
 *
 * <itemizedlist>
 *   <listitem>
 *     <para>
 *       The bot renderer (that renders the robot)
 *     </para>
 *   </listitem>
 *   <listitem>
 *     <para>
 *       Sensor renderers (that render SICK or Hokuyo returns)
 *     </para>
 *   </listitem>
 *   <listitem>
 *     <para>
 *       LCMGL renderer (for quick and dirty rendering)
 *     </para>
 *   </listitem>
 * </itemizedlist>
 *
 * To make your own renderer, create a structure with the first variable being
 * this renderer.  For example:
 *
 * <informalexample>
 *   <programlisting>
 *     typedef struct MyBotRenderer {
 *         BotRenderer renderer
 *
 *         ... // lots of your other, needed variables.
 *     }
 *   </programlisting>
 * </informalexample>
 */
typedef struct _BotRenderer BotRenderer;
struct _BotRenderer {

    int        enabled;

    int        priority;

    char       *name;
    GtkWidget  *widget;

    void (*draw)      (BotViewer *viewer, BotRenderer *renderer);
    void (*destroy)   (BotRenderer *renderer);
    void *user;

    int expanded;
    GtkWidget         *cmi;
    GtkWidget         *expander;
    GtkWidget         *control_frame;
};

#define BOT_VIEWER(obj)  (G_TYPE_CHECK_INSTANCE_CAST((obj), bot_viewer_get_type(), BotViewer))

typedef struct _BotViewerClass BotViewerClass;

/**
 * BotViewer:
 * @parent:
 * @backgroundColor:
 * @gl_area:
 * @tips:
 * @event_handlers: Event handlers sorted by priority (decreasing).
 * @event_handlers_sorted:
 * @picking_handler: The last-known picking handler (also a member of
 * @event_handlers).
 * @renderers: BotRenderers sorted by priority (decreasing).
 * @renderers_sorted: renderers sorted by name (alphabetical).
 * @renderers_sorted_with_controls: Just those renderers with control widgets,
 * sorted by name (alphabetical).
 * @view_handler:
 * @default_view_handler:
 * @controls_box:
 * @controls_box_left: additional control on the left hand side
 * @record_button:
 * @menu_bar:
 * @mov_bgr_buf:
 * @fps_spin:
 * @movie_path:
 * @movie_buffer:
 * @movie_width:
 * @movie_height:
 * @movie_stride:
 * @movie_draw_pending:
 * @movie_frames:
 * @movie_frame_last_utime:
 * @movie_actual_fps:
 * @movie_desired_fps:
 * @movie_gzf:
 * @render_timer_id:
 * @is_recording:
 * @file_menu:
 * @renderers_menu:
 * @event_handlers_menu:
 * @last_draw_utime: When did we last render the gl view?
 * @redraw_timer_pending: Is a call to @on_redraw_timer pending?
 * @prettier_flag:
 * @window:
 * @status_bar:
 * @toolbar:
 * @status_bar_message:
 */
struct _BotViewer {
    GObject parent;

    float backgroundColor[4];

    BotGtkGlDrawingArea *gl_area;
    GtkTooltips       *tips;

    GPtrArray         *event_handlers;
    GPtrArray         *event_handlers_sorted;

    BotEventHandler      *picking_handler;

    GPtrArray         *renderers;      

    GPtrArray         *renderers_sorted; 

    GPtrArray         *renderers_sorted_with_controls;

    BotViewHandler       *view_handler;
    BotViewHandler       *default_view_handler;

    GtkWidget         *controls_box;
    GtkWidget         *controls_box_left;

    GtkWidget         *record_button;
    GtkWidget         *menu_bar;

    uint8_t           *mov_bgr_buf;
    GtkWidget         *fps_spin;

    char              *movie_path;
    uint8_t           *movie_buffer;
    int               movie_width, movie_height, movie_stride;
    int               movie_draw_pending;
    int               movie_frames;
    int64_t           movie_frame_last_utime;
    double            movie_actual_fps;
    double            movie_desired_fps;
    gzFile            *movie_gzf;

    guint             render_timer_id;
    int               is_recording;

    GtkWidget         *file_menu;
    GtkWidget         *renderers_menu;
    GtkWidget         *event_handlers_menu;
    GtkWidget         *view_menu;

    int64_t           last_draw_utime;
    int               redraw_timer_pending;

    int               prettier_flag;

    GtkWidget         *window;
    GtkWidget         *status_bar;
    GtkToolbar         *toolbar;

    char              *status_bar_message;
};

struct _BotViewerClass
{
    GObjectClass parent_class;
};

GType bot_viewer_get_type (void);

BotViewer *bot_viewer_new (const char *window_title);
void bot_viewer_unref(BotViewer *viewer);

void bot_viewer_set_window_title (BotViewer *viewer, const char *window_name);

void bot_viewer_start_recording (BotViewer *viewer);
void bot_viewer_stop_recording (BotViewer *viewer);


/**
 * bot_viewer_request_redraw:
 * @viewer: The viewer to redraw.
 *
 * Request a redraw (which will occur asynchronously).
 */
void bot_viewer_request_redraw (BotViewer *viewer);



/**
 * bot_viewer_on_side:
 * @viewer: The viewer to add the renderer to.
 * @plugin: The renderer to add.
 * @priority: The priority to assign the new renderer.
 * @control_box: Which control box to add to 0=left 1=right.
 *
 * Adds a renderer to the viewer.  BotRenderers are called at render time, in the
 * order in which they were added to the viewer.
 */
void bot_viewer_add_renderer_on_side (BotViewer *viewer, BotRenderer *plugin, int priority, int which_side);



/**
 * bot_viewer_add_renderer:
 * @viewer: The viewer to add the renderer to.
 * @plugin: The renderer to add.
 * @priority: The priority to assign the new renderer.
 *
 * Adds a renderer to the viewer.  BotRenderers are called at render time, in the
 * order in which they were added to the viewer. 
 * Defaults to right hand side (control_box=1) - the default
 */
void bot_viewer_add_renderer (BotViewer *self, BotRenderer *renderer, int priority);




/**
 * bot_viewer_remove_renderer:
 * @viewer: The viewer to add the renderer to.
 * @plugin: The renderer to remove.
 *
 * Removes a renderer from the viewer.  Once it is removed, it will not be
 * handled anymore.
 */
void bot_viewer_remove_renderer(BotViewer *viewer, BotRenderer *plugin);

/**
 * bot_viewer_set_view_handler:
 * A viewer has a single "view handler" that manages the camera projection
 * and position. It is typically also an event handler that can respond to
 * user pan/zoom/rotate commands.
 *
 * Setting the view handler to %NULL activates the default view handler.
 */
void bot_viewer_set_view_handler(BotViewer *viewer, BotViewHandler *vhandler);

/**
 * bot_viewer_add_event_handler:
 * If you want key/mouse events, you need to add an event handler.
 */
void bot_viewer_add_event_handler (BotViewer *viewer, BotEventHandler *ehandler, 
        int priority);
void bot_viewer_remove_event_handler(BotViewer *viewer, BotEventHandler *ehandler);

/**
 * bot_viewer_event_handler_set_priority:
 * The priority of an event handler can be changed dynamically. This
 * is useful, for example, when executing stateful input
 * sequences. E.g., a user clicks on an object which is handled by a
 * relatively low-priority event handler, and wants to get the *next*
 * click as well (even if it would ordinarily be consumed by some
 * other event handler). The event handler can thus temporarily
 * increase its priority.
 */
void bot_viewer_event_handler_set_priority (BotViewer *viewer, BotEventHandler *ehandler, 
        int priority);

void bot_viewer_set_status_bar_message (BotViewer *viewer, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

/**
 * bot_viewer_picking:
 *
 * Returns: Non-zero if an event handler is currently picking.
 */
int bot_viewer_picking(BotViewer *viewer);

/**
 * bot_viewer_request_pick:
 * If a event handler wants to begin a pick operation, it can request it. 
 *
 * Returns: Non-zero if the request fails.
 */
int bot_viewer_request_pick(BotViewer *viewer, BotEventHandler *ehandler);

void bot_viewer_load_preferences (BotViewer *viewer, const char *fname);

void bot_viewer_save_preferences (BotViewer *viewer, const char *fname);

typedef enum {
    BOT_VIEWER_STOCK_RENDERER_GRID
} BotViewerStockRendererId;

void bot_viewer_add_stock_renderer(BotViewer* viewer, int stock_renderer_id, int priority);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
