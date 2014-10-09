#ifndef __lcm_tunnel_h__
#define __lcm_tunnel_h__

#include <inttypes.h>
#include <deque>
#include <glib.h>

#include "ldpc/ldpc_wrapper.h"
#include "lcm_tunnel_params_t.h"
#include "lcm_tunnel_sub_msg_t.h"
#include "lcm_tunnel_udp_msg_t.h"
#include "lcm_tunnel_disconnect_msg_t.h"

#include "ssocket.h"
#include "introspect.h"

#define DEFAULT_PORT 6141

#define MIN_NUM_FRAGMENTS_FOR_FEC  3
  //#define MAX_PAYLOAD_BYTES_PER_FRAGMENT 1400
  //size of packets EXCLUDING the header!
  //must be a multiple of 16 for LDPC, setting it a bit smaller to mitigate wireless interference a bit
#define MAX_PAYLOAD_BYTES_PER_FRAGMENT 1024

 //wakeup the send thread immediately if there are this many bytes in the queue
#define NUM_BYTES_TO_SEND_IMMEDIATELY (5*MAX_PAYLOAD_BYTES_PER_FRAGMENT)


#define MAX_SEND_BUFFER_SIZE 33554432 //2^25 ~33MB

#define MAX_NUM_FRAGMENTS 32768  //since we're using a int16_t for the fragment number
  //and wrap around explicitly at this value
#define SEQNO_WRAP_VAL 30000
  //at wrap around, the prev once should be at least this much bigger
#define SEQNO_WRAP_GAP 5000

static inline int getNumFragments(int32_t msgSize){
  return (int) ceil((float) msgSize / MAX_PAYLOAD_BYTES_PER_FRAGMENT);
}

typedef struct {
    uint16_t port;
    int verbose;
    char lcm_url[1024];
    int startedAsClient;
} tunnel_server_params_t;


class  TunnelLcmMessage {
public:
  TunnelLcmMessage(const lcm_recv_buf_t *rbuf, const char *chan){
    sub_msg = (lcm_tunnel_sub_msg_t *) malloc(sizeof(lcm_tunnel_sub_msg_t));
    sub_msg->channel= strdup(chan);
    recv_utime = rbuf->recv_utime;
    sub_msg->data_size = rbuf->data_size;
    sub_msg->data = (uint8_t *)malloc(sub_msg->data_size);
    memcpy(sub_msg->data,rbuf->data,sub_msg->data_size);
    encoded_size = lcm_tunnel_sub_msg_t_encoded_size(sub_msg);
  }
  ~TunnelLcmMessage(){
    lcm_tunnel_sub_msg_t_destroy(sub_msg);
  }

  lcm_tunnel_sub_msg_t * sub_msg;
  int64_t recv_utime;
  int encoded_size;
} ;



class LcmTunnel {
public:
  LcmTunnel(bool verbose, const char *channel); //just allocates buffers and whatnot
  int connectToClient(lcm_t * lcm_, introspect_t *introspect_, GMainLoop *
      mainloop_, ssocket_t * sock_, tunnel_server_params_t *server_params_);
  int connectToServer(lcm_t * lcm_, introspect_t *introspect_, GMainLoop *
      mainloop_, char * server_addr_str, int port, char *
      channels_to_recv, lcm_tunnel_params_t * tunnel_params_,
      tunnel_server_params_t * server_params_);

  void send_to_remote(const void *data, uint32_t len, const char *lcm_channel);
  void send_to_remote(const lcm_recv_buf_t *rbuf, const char *lcm_channel);
  bool match_regex(const char *channel);
  void init_regex(const char *channel);

  ~LcmTunnel();

  static void on_lcm_message(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data);
  static gpointer sendThreadFunc(gpointer user_data);
  bool send_lcm_messages(std::deque<TunnelLcmMessage *> &msgQueue,uint32_t bytesInQueue);
  static int on_tcp_data(GIOChannel * source, GIOCondition cond, void *user_data);
  static int on_udp_data(GIOChannel * source, GIOCondition cond, void *user_data);
  int publishLcmMessagesInBuf(int numBytes);

  bool verbose;

  char name[1024]; //address and port for client

private:
  GRegex * regex;

  tunnel_server_params_t * server_params; //params of parent server
  lcm_tunnel_params_t * tunnel_params;
  lcm_t * lcm; //pointer to the server's lcm
  introspect_t *introspect;
  GMainLoop * mainloop; //pointer to the server's mainloop



  //tcp socket stuff
  ssocket_t * tcp_sock;
  GIOChannel * tcp_ioc;
  guint tcp_sid;
  void closeTCPSocket();
  typedef enum {
    CLIENT_MSG_SZ, CLIENT_MSG_DATA, SERVER_MSG_SZ, SERVER_MSG_DATA,
    RECV_CHAN_SZ, RECV_CHAN, RECV_DATA_SZ, RECV_DATA
  } tunnel_state_t;
  tunnel_state_t tunnel_state;

  int bytes_to_read;
  int bytes_read;

  //threaded sending stuff:
  bool stopSendThread;
  uint32_t bytesInQueue;
  uint32_t minBytesToSendImmediately;
  GThread * sendThread;
  std::deque<TunnelLcmMessage *> sendQueue;
  GMutex * sendQueueLock;
  GCond* sendQueueCond; //thread waits on this
  bool flushImmediately;




  //buffers to store incoming messages
  char *channel;
  int channel_sz;
  char *buf;
  int buf_sz;

  int udp_fd;
  int server_udp_port;
  GIOChannel * udp_ioc;
  guint udp_sid;
  uint32_t udp_send_seqno;

  //stuff to keep track of received fragments
  char * recFlags;
  int recFlags_sz;
  int32_t cur_seqno;
  uint32_t numFragsRec;
  uint32_t completeTo_fragno;
  uint32_t nfrags;
  uint32_t fragment_buf_offset;
  int message_complete;

  //for monitoring the UDP link status
  void checkUDPSendStatus(int send_status);
  int64_t errorStartTime;
  int64_t lastErrorPrintTime;
  int numSuccessful;


  //forward error correction variables
  ldpc_dec_wrapper * ldpc_dec;

  lcm_subscription_t *subscription;

};

#endif
