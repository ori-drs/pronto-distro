#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/select.h>

#include "timestamp.h"

int64_t bot_timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int64_t bot_timestamp_seconds(int64_t v)
{
    return v/1000000;
}

int64_t bot_timestamp_useconds(int64_t v)
{
    return v%1000000;
}

void bot_timestamp_to_timeval(int64_t v, struct timeval *tv)
{
    tv->tv_sec  = bot_timestamp_seconds(v);
    tv->tv_usec = bot_timestamp_useconds(v);
}

void bot_timestamp_to_timespec(int64_t v, struct timespec *ts)
{
    ts->tv_sec  = bot_timestamp_seconds(v);
    ts->tv_nsec = bot_timestamp_useconds(v)*1000;
}

bot_timestamp_sync_state_t *
bot_timestamp_sync_init (double dev_ticks_per_second, int64_t dev_ticks_wraparound,
        double rate)
{
    bot_timestamp_sync_state_t * s;

    s = malloc (sizeof (bot_timestamp_sync_state_t));
    if (!s)
        return NULL;
    memset (s, 0, sizeof (bot_timestamp_sync_state_t));

    s->dev_ticks_per_second = dev_ticks_per_second;
    s->dev_ticks_wraparound = dev_ticks_wraparound;
    s->max_rate_error = rate;
    s->is_valid = 0;

    return s;
}

void
bot_timestamp_sync_free (bot_timestamp_sync_state_t * s)
{
    free (s);
}

int64_t
bot_timestamp_sync (bot_timestamp_sync_state_t * s, int64_t dev_ticks,
        int64_t host_utime)
{
    if (!s->is_valid) {
        /* The first sync has no history */
        s->is_valid = 1;

        s->sync_host_time = host_utime;
	s->last_dev_ticks = dev_ticks;
	s->dev_ticks_since_sync = 0;

        return host_utime;
    }

    // how many device ticks since the last invocation?
    int64_t dticks = dev_ticks - s->last_dev_ticks;
    s->last_dev_ticks = dev_ticks;
    if (dticks < 0)
        dticks += s->dev_ticks_wraparound;

    s->dev_ticks_since_sync += dticks;

    // overestimate device time by a factor of s->rate
    double rate = 1000000.0 / s->dev_ticks_per_second * s->max_rate_error;

    // estimate of the host's time corresponding to the device's time
    int64_t dev_utime = s->sync_host_time + (s->dev_ticks_since_sync * rate);

    int64_t time_err = host_utime - dev_utime;

    /* If time_err is very large, resynchronize, emitting a warning. if
     * it is negative, we're just adjusting our timebase (it means
     * we got a nice new low-latency measurement.) */
    if (time_err > 1000000000LL) { /* 1000 seconds */
        fprintf (stderr, "Warning: Time sync has drifted by more than 1000 seconds\n");
	s->sync_host_time = host_utime;
	s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
    if (time_err < 0) {
	s->sync_host_time = host_utime;
	s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
    //printf ("%llu %llu %lld %lld\n", host_utime, dev_utime, time_err, dev_utime + s->time_offset);

    return dev_utime;
}
