#ifndef __bot_fasttrig_h__
#define __bot_fasttrig_h__

/**
 * @defgroup BotCoreFastTrig Fast Trigonometry 
 * @ingroup BotCoreMathGeom
 * @brief Very fast, but approximate trigonometry
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

void bot_fasttrig_init(void);
void bot_fasttrig_sincos(double theta, double *s, double *c);
double bot_fasttrig_atan2(double y, double x);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
