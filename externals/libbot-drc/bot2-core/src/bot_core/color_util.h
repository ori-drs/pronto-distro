#ifndef BOT_COLOR_UTIL
#define BOT_COLOR_UTIL
/**
 * @defgroup BotColorUtil Color Utilities
 * @brief utilities for making color schemes
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

  /*
   * Get a random color
   */
  void bot_color_util_rand_color(float f[4], double alpha, double min_intensity);

  /*
   * Get the color in the JET color space associated with value v [0,1]
   */
  float *bot_color_util_jet(double v);


  /*
   * convenience arrays for commonly used colors
   */
  static const float bot_color_util_red[3]     = {1,0,0};
  static const float bot_color_util_green[3]   = {0,1,0};
  static const float bot_color_util_blue[3]    = {0,0,1};
  static const float bot_color_util_cyan[3]    = {0,1,1};
  static const float bot_color_util_magenta[3] = {1,0,1};
  static const float bot_color_util_yellow[3]  = {1,1,0};
  static const float bot_color_util_black[3]   = {0,0,0};
  static const float bot_color_util_white[3]   = {1,1,1};


#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif
