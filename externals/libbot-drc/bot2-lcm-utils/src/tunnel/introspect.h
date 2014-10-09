#ifndef __introspect_h__
#define __introspect_h__

#include <lcm/lcm.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _introspect_t introspect_t;

/**
 * introspection only works for udpm LCM networks
 */
introspect_t * introspect_new(lcm_t * lcm);

void introspect_send_introspection_packet(introspect_t* ipi);

void introspect_destroy(introspect_t *ipi);

int introspect_is_message_from_self(introspect_t * ipi,
        const lcm_recv_buf_t * rbuf, const char *channel);

int introspect_is_ready(const introspect_t * ipi);

#ifdef __cplusplus
}
#endif

#endif
