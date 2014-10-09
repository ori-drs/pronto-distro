#ifndef __gtk_util_h__
#define __gtk_util_h__

#include <gtk/gtk.h>
#include "param_widget.h"
#include "gl_drawing_area.h"
#include "gl_image_area.h"
#include "viewer.h"

/**
 * @defgroup BotVisGtkUtil Miscellaneous GTK+ utility functions
 * @brief GTK utility functions
 * @ingroup BotVisGtk
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
 * Adds an event handler to the GTK mainloop that calls gtk_main_quit() when 
 * SIGINT, SIGTERM, or SIGHUP are received
 */
int bot_gtk_quit_on_interrupt(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
