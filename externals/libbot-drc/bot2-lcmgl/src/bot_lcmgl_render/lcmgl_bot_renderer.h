#ifndef __lcmgl_bot_renderer_h__
#define __lcmgl_bot_renderer_h__

/**
 * @defgroup BotLCMGLViewerRenderer BotViewer renderer
 * @ingroup BotLCMGL
 * @brief BotVis Viewer renderer plugin
 * @include bot_lcmgl_render/lcmgl_bot_renderer.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-lcmgl-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>

#ifdef __cplusplus
extern "C" {
#endif

void bot_lcmgl_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
