#ifndef LASER_UTIL_RENDERER_H_
#define LASER_UTIL_RENDERER_H_
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

  void laser_util_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm, BotParam * param,
      BotFrames * frames);

#ifdef __cplusplus
}
#endif

#endif
