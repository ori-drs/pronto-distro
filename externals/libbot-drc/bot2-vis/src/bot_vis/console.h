#ifndef __bot_gl_console_h__
#define __bot_gl_console_h__

/**
 * @defgroup console Vertically scrolling text overlay
 * @brief Vertically scrolling text
 * @ingroup BotVisGl
 * @include: bot_vis/bot_vis.h
 *
 * Utility class for rendering vertically scrolling text.  Text can be added
 * with printf-style formatting, and will scroll vertically as more text is
 * added.  A decay value can also be set to fade away text over time.
 *
 * This class can be useful for displaying textual debugging or status
 * information in an OpenGL window.
 *
 * Linking: `pkg-config --libs bot2-vis`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BotGlConsole BotGlConsole;

/**
 * Constructor.
 */
BotGlConsole *bot_gl_console_new(void);

/**
 * Destructor.
 */
void bot_gl_console_destroy(BotGlConsole *console);

/**
 * Sets the rendering font.  See GLUT documentation for possible options.
 */
void bot_gl_console_set_glut_font(BotGlConsole *console, void *font);

//void bot_gl_console_set_pos (BotGlConsole *console, double x, double y);

/**
 * Sets how quickly text will fade away.
 */
void bot_gl_console_set_decay_lambda(BotGlConsole *console, double lambda);

/**
 * Sets the text color.
 */
void bot_gl_console_color3f(BotGlConsole *console, float r, float g, float b);

/**
 * Adds text to display.  Uses printf-style formatting.
 */
void bot_gl_console_printf(BotGlConsole *console, const char *format, ...);

/**
 * Render the scrolling text.
 * @param elapsed_time how much time, in seconds, has elapsed since the last
 * rendering.  This is used in combination with
 * bot_gl_console_set_decay_lambda() to determine the text transparency.  If
 * less than zero, then the elapsed time will be automatically determined using
 * the system clock.
 */
void bot_gl_console_render(BotGlConsole *console, double elapsed_time);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
