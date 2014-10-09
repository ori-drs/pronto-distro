#ifndef _BOT_LCMGL_DECODE_H
#define _BOT_LCMGL_DECODE_H

#include <inttypes.h>

/**
 * @defgroup BotLCMGLRender LCMGL decoding and rendering
 * @ingroup BotLCMGL
 * @brief Executing OpenGL commands received via LCMGL
 * @include bot_lcmgl_render/lcmgl_decode.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-lcmgl-renderer`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * bot_lcmgl_decode:
 *
 * Decodes a block of LCMGL data, and executes the OpenGL commands with 
 * the current OpenGL context.
 */
void bot_lcmgl_decode(uint8_t *data, int datalen);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
