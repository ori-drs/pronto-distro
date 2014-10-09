#ifndef BOT_RAND_UTILS_H_
#define BOT_RAND_UTILS_H_
/**
 * @defgroup BotCoreRandUtil Rand Utilities
 * @ingroup BotCoreMathGeom
 * @brief Miscellaneous random number utility functions
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/* random number between [0, 1) */
static inline float bot_randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}

/* random number between (-1, 1) */
static inline float bot_signed_randf()
{
    return bot_randf()*2.0 - 1.0;
}
/* random number between [mi, ma ] */
static inline float bot_randf_in_range(float mi, float ma)
{
    return bot_randf()*(ma-mi) + mi;
}


/* return a random integer between [0, bound) */
static inline int bot_irand(int bound)
{
    int v = (int) (bot_randf()*bound);
    assert(v >= 0);
    assert(v < bound);
    return v;
}

/*seed bot_gauss_rand (defaults to 13 if bot_gauss_rand() called before bot_gauss_rand_init()*/
void bot_gauss_rand_init(uint32_t seed);

/*return a normally distributed random number */
double bot_gauss_rand(double mu, double sigma);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* BOT_RAND_UTILS_H_ */
