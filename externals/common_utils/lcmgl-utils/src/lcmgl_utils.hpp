#ifndef LCMGL_UTILS_HPP_
#define LCMGL_UTILS_HPP_

#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include "LcmglStore.hpp"


namespace lcmgl_utils {
// scale and x_shift should be in [0,1], with 1 beging the full window width
void drawImageTopLeft(bot_lcmgl_t * lcmgl, const bot_core_image_t * im, double x_shift, double scale);

}
#endif /* LCMGL_UTILS_HPP_ */
