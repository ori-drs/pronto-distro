#include "lcmgl_utils.hpp"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


namespace lcmgl_utils {
// scale and x_shift should be in [0,1], with 1 beging the full window width
void drawImageTopLeft(bot_lcmgl_t * lcmgl, const bot_core_image_t * im, double x_shift, double scale)
{
  // transform into window coordinates, where <0, 0> is the top left corner
  // of the window and <1, viewer_ar> is the bottom right corner of the window
  // Since we're doing it over LCMGL, we don't know the aspect ratio, so we set it to be such that
  // x=1 is the full width, and pixels are "square"
  bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_load_identity(lcmgl);
  bot_lcmgl_ortho(lcmgl, 0, 1, 1, 0, -1, 1);
  bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_load_identity(lcmgl);
  bot_lcmgl_scale_to_viewer_ar(lcmgl);

  //shift it over
  bot_lcmgl_translated(lcmgl, x_shift, 0, 0);
  //scale the image down to be <scale> fraction of the window
  bot_lcmgl_scalef(lcmgl, scale, scale, 1);

  //scale so that we can draw in pixel coordinates
  float maxdim = fmax(im->width, im->height);
  bot_lcmgl_scalef(lcmgl, 1 / maxdim, 1 / maxdim, 1);

  //actually draw the image
  bot_lcmgl_enable(lcmgl, GL_DEPTH_TEST);
  bot_lcmgl_color4f(lcmgl, 1, 1, 1, 1);
  int texid = bot_lcmgl_texture2d(lcmgl, im->data, im->width, im->height, im->row_stride, BOT_LCMGL_RGB,
      BOT_LCMGL_UNSIGNED_BYTE,
      BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(lcmgl, texid, 0, 0, 0, 0, im->height, 0, im->width, im->height, 0, im->width, 0, 0);
  bot_lcmgl_disable(lcmgl, GL_DEPTH_TEST);

  //reset stacks
  bot_lcmgl_pop_matrix(lcmgl);
  bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
  bot_lcmgl_pop_matrix(lcmgl);
  bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);

}

}
