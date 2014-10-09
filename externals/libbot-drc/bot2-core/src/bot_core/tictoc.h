/*
 * tictoc.h
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#ifndef __bot_tictoch_h__
#define __bot_tictoch_h__

/**
 * @defgroup BotCoreTicToc TicToc
 * @brief quick and dirty profiling tool.
 * @ingroup BotCoreTime
 * @include: bot_core/bot_core.h
 *
 * inspired by the matlab tic/toc command
 *
 * call bot_tictoc("description") to set the timer going
 * call it again with the same description to stop the timer
 *
 * Note: To get output, set the "BOT_TICTOC" environment variable to something
 *
 * @{
 */

#define BOT_TICTOC_ENV "BOT_TICTOC"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * bot_tictoc:
 *
 * basic invocation, the second time its called, it returns the time difference in microseconds
 **/
int64_t
bot_tictoc(const char *description);

/**
 * bot_tictoc_full:
 *
 * full invocation, allows you to specify an
 * exponential moving average rate, and the current EMA value is returned in the ema argument
 */
int64_t
bot_tictoc_full(const char *description, double ema_alpha, int64_t * ema);

/**
 * bot_tictoc_sort_type_t:
 *
 * Different Options for sorting the printed results
 */
typedef enum
{
    BOT_TICTOC_AVG,
    BOT_TICTOC_TOTAL,
    BOT_TICTOC_MIN,
    BOT_TICTOC_MAX,
    BOT_TICTOC_EMA,
    BOT_TICTOC_ALPHABETICAL
} bot_tictoc_sort_type_t;

/**
 * bot_tictoc_print_stats:
 *
 * Print Out the stats from tictoc
 */
void
bot_tictoc_print_stats(bot_tictoc_sort_type_t sortType);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif /* TICTOC_H_ */
