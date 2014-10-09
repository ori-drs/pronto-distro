#ifndef __bot_timestamp_h__
#define __bot_timestamp_h__

#include <stdint.h>
#include <sys/time.h>
#include <time.h>

/**
 * @defgroup BotCoreTimestamp Timestamp
 * @brief Retrieving the current time, synchronizing clocks
 * @ingroup BotCoreTime
 * @include: bot_core/bot_core.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

typedef struct bot_timestamp_sync_state bot_timestamp_sync_state_t;

struct bot_timestamp_sync_state {
    double  dev_ticks_per_second; // how fast does device clock count? (nominal)
    int64_t dev_ticks_wraparound; // device clock counts modulo what?
    double  max_rate_error;       // how fast do we need to count to ensure we're counting faster than device?

    int64_t sync_host_time;       // when we last synced, what time was it for the host?
    int64_t dev_ticks_since_sync; // how many device ticks have elapsed since the last sync?

    int64_t last_dev_ticks;        // what device time was it when we were last called?

    uint8_t is_valid;             // have we ever synced?
};

#ifdef __cplusplus
extern "C" {
#endif

int64_t bot_timestamp_now(void);
int64_t bot_timestamp_seconds(int64_t v);
int64_t bot_timestamp_useconds(int64_t v);
void bot_timestamp_to_timeval(int64_t v, struct timeval *tv);
void bot_timestamp_to_timespec(int64_t v, struct timespec *ts);

/** Create a new time synchronizer.
    @param dev_ticks_per_second  The nominal rate at which the device time increments
    @param dev_ticks_wraparound  Assume that dev_ticks wraps around every wraparound ticks
    @param rate                  An upper bound on the rate error

    The syncronization algorithm is described in:
    @inproceedings{olson2010,
      TITLE      = {A Passive Solution to the Sensor Synchronization Problem},
      AUTHOR     = { Edwin Olson},
      BOOKTITLE  = {Proceedings of the {IEEE/RSJ} International Conference on Intelligent
                     Robots and Systems {(IROS)}},
      YEAR       = {2010},
      MONTH      = {October},
      VOLUME     = {},
      NUMBER     = {},
      PAGES      = {},
      KEYWORDS   = {sensor calibration, time synchronization},
      ISSN       = { },
    }
**/
bot_timestamp_sync_state_t *
bot_timestamp_sync_init (double dev_ticks_per_second, int64_t dev_ticks_wraparound,
        double rate);
void
bot_timestamp_sync_free (bot_timestamp_sync_state_t * s);
int64_t
bot_timestamp_sync (bot_timestamp_sync_state_t * s, int64_t dev_ticks,
        int64_t host_utime);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
