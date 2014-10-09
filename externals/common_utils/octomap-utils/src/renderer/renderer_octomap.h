#ifndef _OCTOMAP_RENDERER_H_
#define _OCTOMAP_RENDERER_H_
#include <bot_vis/bot_vis.h>

#ifdef __cplusplus
extern "C" {
#endif

  void add_octomap_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm);

#ifdef __cplusplus
}
#endif

#endif /* _OCTOMAP_RENDERER_H_ */
