/*
 * misc_utils.h
 *
 *  Created on: Oct 2, 2010
 *      Author: abachrac
 */
#include <lcm/lcm.h>
#include <sys/time.h>
#include <sys/select.h>

#ifndef MISC_UTILS_H_
#define MISC_UTILS_H_

/**
 * lcm_sleep:
 * @lcm: The lcm_t object.
 * @sleeptime max time to wait in seconds
 *
 *  Waits for up to @sleeptime seconds for an LCM message to arrive.
 *  It handles the first message if one arrives.
 *
 */
static inline void lcm_sleep(lcm_t * lcm, double sleeptime)
{ //
  int lcm_fileno = lcm_get_fileno(lcm);

  fd_set rfds;
  int retval;
  FD_ZERO(&rfds);
  FD_SET(lcm_fileno, &rfds);
  struct timeval tv;
  tv.tv_sec = (int) sleeptime;
  tv.tv_usec = (int) ((sleeptime - tv.tv_sec) * 1.0e6);
  retval = select(lcm_fileno + 1, &rfds, NULL, NULL, &tv);
  if (retval == -1) {
    fprintf(stderr, "bot_lcm_poll: select() failed!\n");
    return;
  }
  if (retval) {
    if (FD_ISSET(lcm_fileno, &rfds)) {
      lcm_handle(lcm);
    }
  }
}

static inline int64_t _timestamp_now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


#endif /* MISC_UTILS_H_ */
