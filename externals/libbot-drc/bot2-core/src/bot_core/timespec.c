#include <sys/time.h>
#include <time.h>
#include <stdio.h>

#include "timespec.h"

void bot_timespec_to_timeval(struct timespec *ts, struct timeval *tv)
{
  tv->tv_sec  = ts->tv_sec;
  tv->tv_usec = ts->tv_nsec / 1000;
}

void bot_timespec_now(struct timespec *ts)
{
	struct timeval  tv;

	// get the current time
	gettimeofday(&tv, NULL);
	ts->tv_sec  = tv.tv_sec;
	ts->tv_nsec = tv.tv_usec*1000;  
}

void bot_timespec_addms(struct timespec *ts, long ms)
{
	int sec=ms/1000;
	ms=ms-sec*1000;

	// perform the addition
	ts->tv_nsec+=ms*1000000;

	// adjust the time
	ts->tv_sec+=ts->tv_nsec/1000000000 + sec;
	ts->tv_nsec=ts->tv_nsec%1000000000;
}

void bot_timespec_addns(struct timespec *ts, long ns)
{
	int sec=ns/1000000000;
	ns=ns - sec*1000000000;

	// perform the addition
	ts->tv_nsec+=ns;

	// adjust the time
	ts->tv_sec+=ts->tv_nsec/1000000000 + sec;
	ts->tv_nsec=ts->tv_nsec%1000000000;

}

void bot_timeval_set(struct timeval *tv, double dt)
{
	long us = dt*1000000;

	tv->tv_sec = us / 1000000;
	tv->tv_usec = us - tv->tv_sec*1000000;
}

void bot_timespec_adjust(struct timespec *ts, double dt)
{
	int sec;
	long ns;

	sec = (int) dt;
	ns = (dt - sec) * 1000000000;

	while (ns < 0) {
		ns += 1000000000;
		sec--;
	}

	// perform the addition
	ts->tv_nsec+=ns;

	// adjust the time
	ts->tv_sec+=ts->tv_nsec/1000000000 + sec;
	ts->tv_nsec=ts->tv_nsec%1000000000;

}

int bot_timespec_compare(struct timespec *a, struct timespec *b)
{
	if (a->tv_sec!=b->tv_sec)
		return a->tv_sec-b->tv_sec;

	return a->tv_nsec-b->tv_nsec;
}

// computes a = a-b
void bot_timespec_subtract(struct timespec *a, struct timespec *b)
{
	a->tv_nsec = a->tv_nsec - b->tv_nsec;
	if (a->tv_nsec < 0) {
		// borrow.
		a->tv_nsec += 1000000000;
		a->tv_sec --;
	}

	a->tv_sec = a->tv_sec - b->tv_sec;
}

// convert the timespec into milliseconds (may overflow)
int bot_timespec_milliseconds(struct timespec *a) 
{
	return a->tv_sec*1000 + a->tv_nsec/1000000;
}

void bot_timespec_print(struct timespec *a)
{
	printf("%li.%09li\n",a->tv_sec, a->tv_nsec);
}
