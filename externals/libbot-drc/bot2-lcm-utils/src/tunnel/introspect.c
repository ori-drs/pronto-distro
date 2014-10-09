#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <glib.h>

#include "introspect.h"

struct _introspect_t
{
    int ready;
    int waiting_for_packet;
    lcm_t * lcm;

    lcm_subscription_t * subscription;
    uint8_t sent_buf[64];

    uint32_t host;
    uint16_t port;
    int64_t sig;
};

// XXX these structs must be identical to the ones defined in lcm/lcm_udp.c
struct _lcm2_header_short {
    uint32_t magic;
    uint32_t msg_seqno;
};

typedef struct _lcm_buf {
    char  channel_name[LCM_MAX_CHANNEL_NAME_LENGTH+1];
    int   channel_size;      // length of channel name

    int64_t recv_utime;      // timestamp of first datagram receipt
    char *buf;               // pointer to beginning of message.  This includes
                             // the header for unfragmented messages, and does
                             // not include the header for fragmented messages.

    int   data_offset;       // offset to payload
    int   data_size;         // size of payload
    int   buf_from_ringbuf;  // 1 if the data at buf is managed by the
                             // ringbuffer, 0 if it's from malloc

    int   packet_size;       // total bytes received
    int   buf_size;          // bytes allocated

    struct sockaddr from;    // sender
    socklen_t fromlen;
    struct _lcm_buf *next;
} lcm_buf_t;

static inline int64_t
packet_sig(const char *channel)
{
    lcm_buf_t *lbuf = (lcm_buf_t*) channel;
    struct sockaddr_in * sin = (struct sockaddr_in*) &lbuf->from;
    return (((int64_t)sin->sin_addr.s_addr) << 32) | sin->sin_port;
}

static void
on_lcm_tunnel_introspect(const lcm_recv_buf_t *rbuf, const char *channel,
        void *user_data)
{
    introspect_t *self = (introspect_t*)user_data;
    if(!self->waiting_for_packet)
        return;
    int n = sizeof(self->sent_buf);

    // sanity check
    const lcm_buf_t *lbuf = (const lcm_buf_t*) channel;
    int doff = lbuf->data_offset;
    int data_offset_okay = (doff == 0 && !lbuf->buf_from_ringbuf) ||
        (doff == sizeof(struct _lcm2_header_short) + strlen(channel) + 1 &&
         lbuf->buf_from_ringbuf);
    if(! data_offset_okay || 
       rbuf->data != lbuf->buf + doff ||
       lbuf->recv_utime != rbuf->recv_utime ||
       rbuf->data_size != lbuf->data_size) {
        fprintf(stderr, 
                "Received introspection packet, but failed validation check\n"
                "This does not appear to be a udpm LCM network..\n");
        self->ready = 0;
        return;
    }

    if(rbuf->data_size == n && 0 == memcmp(rbuf->data, self->sent_buf, n)) {
        self->ready = 1;

        self->sig = packet_sig(channel);
        self->host = self->sig >> 32;
        self->port = self->sig & 0xFFFF;
        self->waiting_for_packet = 0;
    }
}

introspect_t *
introspect_new(lcm_t * lcm)
{
    introspect_t * self = malloc(sizeof(introspect_t));
    memset(self, 0, sizeof(introspect_t));

    self->ready = 0;
    self->lcm = lcm;

    self->subscription = lcm_subscribe(self->lcm, "LCM_TUNNEL_INTROSPECT",
            on_lcm_tunnel_introspect, self);

    introspect_send_introspection_packet(self);
    return self;
}

void 
introspect_send_introspection_packet(introspect_t* self)
{
    // generate a random bitstring
    GRand * rng = g_rand_new();
    for(int i=0, n=sizeof(self->sent_buf); i<n; i++) {
        self->sent_buf[i] = g_rand_int_range(rng, 0, 256);
    }
    g_rand_free(rng);

    // send the bitstring over LCM
    lcm_publish(self->lcm, "LCM_TUNNEL_INTROSPECT", self->sent_buf,
            sizeof(self->sent_buf));
    self->waiting_for_packet = 1;
}

void
introspect_destroy(introspect_t *self)
{
    lcm_unsubscribe(self->lcm, self->subscription);
    self->subscription = NULL;
    free(self);
}

int
introspect_is_message_from_self(introspect_t * self,
        const lcm_recv_buf_t * rbuf, const char *channel)
{
    return packet_sig(channel) == self->sig;
}

int
introspect_is_ready(const introspect_t * self)
{
    return self->ready;
}
