#ifndef __bot_timespec_h__
#define __bot_timespec_h__

#include <sys/time.h>
#include <time.h>

/**
 * @defgroup BotCoreTimeSpec Timespec
 * @brief Convenience functions for <literal>struct timespec</literal>
 * @ingroup BotCoreTime
 * @include: bot_core/bot_core.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

// get the current time
void bot_timespec_now(struct timespec *ts);

// add ms milliseconds to the timespec (ms > 0)
void bot_timespec_addms(struct timespec *ts, long ms);

// add ns nanoseconds to the timespec (ns > 0)
void bot_timespec_addns(struct timespec *ts, long ns);

void bot_timespec_adjust(struct timespec *ts, double dt);

// compare a and b
int bot_timespec_compare(struct timespec *a, struct timespec *b);

// display the timespec
void bot_timespec_print(struct timespec *a);

// computes a = a-b
void bot_timespec_subtract(struct timespec *a, struct timespec *b);

// convert the timespec into milliseconds (may overflow)
int bot_timespec_milliseconds(struct timespec *a);

void bot_timeval_set(struct timeval *tv, double dt);

void bot_timespec_to_timeval(struct timespec *ts, struct timeval *tv);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
