#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <getopt.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <signal.h>

#include <lcm/lcm.h>

#include "ssocket.h"
#include "lcm_tunnel.h"
#include "lcm_tunnel_server.h"

static inline void check_ret(int ret)
{
  assert(ret==0);
}

static inline int64_t _timestamp_now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static inline void _timestamp_to_GTimeVal(int64_t v, GTimeVal *ts)
{
  ts->tv_sec = v / 1000000;
  ts->tv_usec = v % 1000000;
}

static int _fileutils_write_fully(int fd, const void *b, int len)
{
  int cnt = 0;
  int thiscnt;
  unsigned char *bb = (unsigned char*) b;

  while (cnt < len) {
    thiscnt = write(fd, &bb[cnt], len - cnt);
    if (thiscnt < 0) {
      perror("write");
      return -1;
    }
    cnt += thiscnt;
  }

  return cnt;
}

LcmTunnel::LcmTunnel(bool verbose, const char *lcm_channel) :
  verbose(verbose), regex(NULL), buf_sz(65536), buf((char*) calloc(65536, sizeof(char))), channel_sz(65536), channel(
      (char*) calloc(65536, sizeof(char))), recFlags_sz(1024), recFlags((char*) calloc(1024, sizeof(char))), ldpc_dec(
      NULL), udp_fd(-1), server_udp_port(-1), udp_send_seqno(0), stopSendThread(false), bytesInQueue(0), cur_seqno(0),
      errorStartTime(-1), numSuccessful(0), lastErrorPrintTime(-1)
{
  //allocate and initialize things

  init_regex(lcm_channel);

  //sendThread stuff
  sendQueueLock = g_mutex_new();
  sendQueueCond = g_cond_new();
  sendThread = g_thread_create(sendThreadFunc, (void *) this, 1, NULL);

}

void LcmTunnel::init_regex(const char *lcm_channel)
{
  if (lcm_channel && strlen(lcm_channel)) {
    char *rchannel = (char*) calloc(strlen(lcm_channel) + 3, sizeof(char));
    sprintf(rchannel, "%c%s%c", '^', lcm_channel, '$');

    if (regex != NULL) {
      if (verbose)
        printf("Replacing regex with \"%s\"\n", rchannel);
      g_regex_unref(regex);
    }
    GError *rerr = NULL;
    regex = g_regex_new(rchannel, (GRegexCompileFlags) 0, (GRegexMatchFlags) 0, &rerr);
    if (rerr != NULL)
      fprintf(stderr, "Invalid regex: \"%s\"\n", rchannel);
    free(rchannel);
  }
}

bool LcmTunnel::match_regex(const char *lcm_channel)
{
  return regex != NULL && (g_regex_match(regex, lcm_channel, (GRegexMatchFlags) 0, NULL));
}

LcmTunnel::~LcmTunnel()
{
  if (subscription) {
    lcm_unsubscribe(lcm, subscription);
  }

  //cleanup the sending thread state
  g_mutex_lock(sendQueueLock);
  stopSendThread = true;
  g_cond_broadcast(sendQueueCond);
  g_mutex_unlock(sendQueueLock);
  g_thread_join(sendThread); //wait for thread to exit

  g_mutex_lock(sendQueueLock);
  while (!sendQueue.empty()) {
    delete sendQueue.front();
    sendQueue.pop_front();
  }
  g_mutex_unlock(sendQueueLock);

  g_mutex_free(sendQueueLock);
  g_cond_free(sendQueueCond);


  if (udp_fd >= 0) {
    //send out a disconnect message
    lcm_tunnel_disconnect_msg_t disc_msg;
    disc_msg.utime = _timestamp_now();
    int msg_sz = lcm_tunnel_disconnect_msg_t_encoded_size(&disc_msg);
    uint8_t msg_buf[msg_sz];
    lcm_tunnel_disconnect_msg_t_encode(msg_buf, 0, msg_sz, &disc_msg);
    send(udp_fd, msg_buf, msg_sz, 0);

    //close UDP socket
    close(udp_fd);
    g_io_channel_unref(udp_ioc);
    g_source_remove(udp_sid);
  }

  //close TCP socket
  closeTCPSocket();

  if (regex != NULL)
    g_regex_unref(regex);

  free(buf);
  free(channel);
  free(recFlags);
  free(tunnel_params);

  if (ldpc_dec != NULL)
    delete ldpc_dec;

}

void LcmTunnel::closeTCPSocket()
{
  //  fprintf(stderr, "closing TCP socket\n");
  if (tcp_sock != NULL)
    ssocket_destroy(tcp_sock);
  tcp_sock = NULL;
  if (tcp_ioc != NULL)
    g_io_channel_unref(tcp_ioc);
  tcp_ioc = NULL;
  if (tcp_sid > 0)
    g_source_remove(tcp_sid);
  tcp_sid = -1;
}

int LcmTunnel::connectToClient(lcm_t * lcm_, introspect_t *introspect_, GMainLoop * mainloop_, ssocket_t * sock_,
    tunnel_server_params_t * server_params_)
{ //for a client that connected to this server
  //parameters will be passed over from the client
  server_params = server_params_;
  lcm = lcm_;
  introspect = introspect_;
  mainloop = mainloop_;
  tcp_sock = sock_;

  struct sockaddr_in client_addr;
  socklen_t addrlen = sizeof(client_addr);
  getpeername(tcp_sock->socket, (struct sockaddr*) &client_addr, &addrlen);
  uint32_t client_port = ntohs(client_addr.sin_port);
  snprintf(name, sizeof(name), "%s:%d", inet_ntoa(client_addr.sin_addr), client_port);
  printf("Accepted connection from %s\n", name);

  tcp_ioc = g_io_channel_unix_new(ssocket_get_fd(tcp_sock));
  tcp_sid = g_io_add_watch(tcp_ioc, G_IO_IN, on_tcp_data, this);

  bytes_to_read = 4;
  bytes_read = 0;
  tunnel_state = CLIENT_MSG_SZ; //we're waiting for the client connect message

  return 1;
}

int LcmTunnel::connectToServer(lcm_t * lcm_, introspect_t *introspect_, GMainLoop * mainloop_, char * server_addr_str,
    int port, char * channels_to_recv, lcm_tunnel_params_t * tunnel_params_, tunnel_server_params_t * server_params_)
{ //for a client that should initiate a connection with a server

  tunnel_params = lcm_tunnel_params_t_copy(tunnel_params_);
  server_params = server_params_;
  lcm = lcm_;
  introspect = introspect_;
  mainloop = mainloop_;

  if (tunnel_params->udp) {
    // allocate UDP socket
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
      perror("allocating UDP socket");
      return 0;
    }

    struct sockaddr_in udp_addr;
    socklen_t udp_addr_len = sizeof(udp_addr);
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = 0;

    if (bind(udp_fd, (struct sockaddr*) &udp_addr, sizeof(udp_addr)) < 0) {
      perror("binding UDP socket");
      return 0;
    }

    getsockname(udp_fd, (struct sockaddr*) &udp_addr, &udp_addr_len);
    tunnel_params->udp_port = ntohs(udp_addr.sin_port);

    udp_ioc = g_io_channel_unix_new(udp_fd);
    udp_sid = g_io_add_watch(udp_ioc, G_IO_IN, LcmTunnel::on_udp_data, this);
  }
  else {
    udp_fd = -1;
  }

  // connect
  tcp_sock = ssocket_create();
  if (0 != ssocket_connect(tcp_sock, server_addr_str, port)) {
    perror("connecting");
    return 0;
  }
  tcp_ioc = g_io_channel_unix_new(ssocket_get_fd(tcp_sock));
  tcp_sid = g_io_add_watch(tcp_ioc, G_IO_IN, on_tcp_data, this);

  //fill out the name info
  struct sockaddr_in server_addr;
  socklen_t addrlen = sizeof(server_addr);
  getpeername(tcp_sock->socket, (struct sockaddr*) &server_addr, &addrlen);
  uint32_t server_port = ntohs(server_addr.sin_port);
  snprintf(name, sizeof(name), "%s:%d", inet_ntoa(server_addr.sin_addr), server_port);
  fprintf(stderr, "Connected to %s\n", name);

  // transmit subscription information
  lcm_tunnel_params_t * tun_params_to_send = lcm_tunnel_params_t_copy(tunnel_params);
  //put the channels the server should send in the params we're sending it.
  free(tun_params_to_send->channels);
  tun_params_to_send->channels = strdup(channels_to_recv);
  int msg_sz = lcm_tunnel_params_t_encoded_size(tun_params_to_send);
  uint8_t * msg = (uint8_t *) calloc(msg_sz, sizeof(uint8_t));
  lcm_tunnel_params_t_encode(msg, 0, msg_sz, tun_params_to_send);
  uint32_t msg_sz_n = htonl(msg_sz);
  if (4 != _fileutils_write_fully(ssocket_get_fd(tcp_sock), &msg_sz_n, 4)) {
    perror("sending subscription data");
    ssocket_destroy(tcp_sock);
    free(msg);
    lcm_tunnel_params_t_destroy(tun_params_to_send);
    return 0;
  }
  if (msg_sz != _fileutils_write_fully(ssocket_get_fd(tcp_sock), msg, msg_sz)) {
    perror("sending subscription data");
    ssocket_destroy(tcp_sock);
    free(msg);
    lcm_tunnel_params_t_destroy(tun_params_to_send);
    return 0;
  }
  lcm_tunnel_params_t_destroy(tun_params_to_send);
  free(msg);

  //set state for tcp receptions
  bytes_to_read = 4;
  bytes_read = 0;
  if (tunnel_params->udp) {
    tunnel_state = SERVER_MSG_SZ; //wait for udp port from server
  }
  else {
    tunnel_state = RECV_CHAN_SZ;
    //subscribe to the channels we want to send out
    //only subscribe if we're doing TCP, since UDP socket hasn't been setup yet
    subscription = lcm_subscribe(lcm, tunnel_params->channels, on_lcm_message, this);

  }
  return 1;
}

int LcmTunnel::publishLcmMessagesInBuf(int numBytes)
{
  uint32_t msgOffset = 0;
  while (msgOffset < numBytes) {
    //decode
    lcm_tunnel_sub_msg_t p;
    msgOffset += lcm_tunnel_sub_msg_t_decode(buf, msgOffset, numBytes - msgOffset, &p);
    // and publish
    LcmTunnelServer::check_and_send_to_tunnels(p.channel, p.data, p.data_size, this);
    lcm_publish(lcm, p.channel, p.data, p.data_size);
    if (verbose)
      printf("publishing [%s] (%.3fKb)\n", p.channel, p.data_size * 1e-3);

    check_ret(lcm_tunnel_sub_msg_t_decode_cleanup(&p));
  }
  assert(msgOffset==numBytes);
}

int LcmTunnel::on_udp_data(GIOChannel * source, GIOCondition cond, void *user_data)
{
  LcmTunnel * self = (LcmTunnel*) user_data;

  uint8_t recv_buffer[65535];

  int recv_status = recv(self->udp_fd, recv_buffer, 65535, 0);

  if (recv_status < 0) {
    perror("recv error: ");
    return TRUE;
    //    LcmTunnelServer::disconnectClient(self);
  }

  lcm_tunnel_udp_msg_t * recv_udp_msg = (lcm_tunnel_udp_msg_t *) calloc(1, sizeof(lcm_tunnel_udp_msg_t));
  int decode_ret = lcm_tunnel_udp_msg_t_decode(recv_buffer, 0, recv_status, recv_udp_msg);
  if (decode_ret < 0) {
    lcm_tunnel_disconnect_msg_t disc_msg;
    decode_ret = lcm_tunnel_disconnect_msg_t_decode(recv_buffer, 0, recv_status, &disc_msg);
    if (decode_ret >= 0) {
      fprintf(stderr, "Received a disconnect message... disconnecting!\n");
      LcmTunnelServer::disconnectClient(self);
    }
    else {
      fprintf(stderr, "Received Corrupted UDP packet!\n");
    }

  }

  //  printf("received: %d, %d / %d\n", recv_udp_msg->seqno, recv_udp_msg->fragment, recv_udp_msg->nfrags);


  if (self->verbose, recv_udp_msg->seqno < self->cur_seqno) {
    printf("Got Out of order packet!\n");
  }

  // start of a new message?
  if (recv_udp_msg->seqno > self->cur_seqno || recv_udp_msg->seqno < (int32_t) self->cur_seqno - SEQNO_WRAP_GAP) { //handle wrap-around with second part
    if (!self->message_complete && self->cur_seqno > 0 || recv_udp_msg->seqno > (self->cur_seqno + 1)) {
      printf("packets %d to %d dropped! with %d of %d fragments received, ", self->cur_seqno, recv_udp_msg->seqno - 1,
          self->numFragsRec, self->nfrags);
      if (self->tunnel_params->fec > 1 && self->nfrags >= MIN_NUM_FRAGMENTS_FOR_FEC) {
        printf("was FECed\n");
      }
      else
        printf("not FECed\n");
    }
    self->cur_seqno = recv_udp_msg->seqno;
    self->nfrags = getNumFragments(recv_udp_msg->payload_size);
    self->numFragsRec = 0;
    //increase the recFlags buffers
    if (self->recFlags_sz < self->nfrags) {
      self->recFlags_sz = self->nfrags;
      self->recFlags = (char *) realloc(self->recFlags, self->recFlags_sz);
    }
    memset(self->recFlags, 0, self->recFlags_sz); //mark all frags as unreceived
    self->completeTo_fragno = 0;
    self->fragment_buf_offset = 0;

    int messageSize = recv_udp_msg->payload_size;
    // increase buffer size if needed, also make enough space for the channel in case we're using FEC
    if (self->buf_sz < messageSize) {
      self->buf = (char *) realloc(self->buf, messageSize);
      self->buf_sz = messageSize;
    }

    //create a new FEC decoder
    if (self->ldpc_dec != NULL) {
      delete self->ldpc_dec; //delete the old one if we haven't already
      self->ldpc_dec = NULL;
    }
    if (self->tunnel_params->fec > 1 && self->nfrags >= MIN_NUM_FRAGMENTS_FOR_FEC) {
      //allocate the new one
      self->ldpc_dec = new ldpc_dec_wrapper(messageSize, MAX_PAYLOAD_BYTES_PER_FRAGMENT, self->tunnel_params->fec);
    }
    self->message_complete = 0;
  }

  if (!self->message_complete && recv_udp_msg->seqno == self->cur_seqno && getNumFragments(recv_udp_msg->payload_size)
      == self->nfrags) {
    self->numFragsRec++;
    if (self->tunnel_params->fec < 1 || self->nfrags < MIN_NUM_FRAGMENTS_FOR_FEC) { //we're not using FEC for this message
      // have we already received this fragment?
      if (recv_udp_msg->fragno < self->nfrags && !self->recFlags[recv_udp_msg->fragno]) {
        self->recFlags[recv_udp_msg->fragno] = 1;

        //copy everything to the app->buf
        int64_t pos_start = recv_udp_msg->fragno * MAX_PAYLOAD_BYTES_PER_FRAGMENT;
        int64_t pos_end = MIN(recv_udp_msg->payload_size, (recv_udp_msg->fragno + 1) * MAX_PAYLOAD_BYTES_PER_FRAGMENT);
        int64_t curPayloadSize = pos_end - pos_start;
        assert(recv_udp_msg->data_size==curPayloadSize);
        memcpy(self->buf + pos_start, recv_udp_msg->data, curPayloadSize);

        self->message_complete = 1;
        for (int i = self->completeTo_fragno; i < self->nfrags; i++) {
          if (!self->recFlags[i]) {
            self->message_complete = 0;
            break;
          }
          else
            self->completeTo_fragno = i;
        }

        if (self->message_complete) {
          //publish all the lcm messages in the buffer
          self->publishLcmMessagesInBuf(recv_udp_msg->payload_size);
        }

      }
      else if (self->verbose) {
        printf("ignoring udp packet\n");
        //        printf("seqno: %d (%d)  fragment: %d (%d) nfrags: %d (%d)\n",
        //                recv_udp_msg->seqno, app->cur_seqno,
        //                recv_udp_msg->fragment, app->expected_fragno,
        //                recv_udp_msg->nfrags, app->nfrags);
      }
    }
    else { //we're using FEC
      int dec_done = self->ldpc_dec->processPacket(recv_udp_msg->data, recv_udp_msg->fragno);
      if (dec_done != 0) {
        if (dec_done == 1) {
          check_ret(self->ldpc_dec->getObject((uint8_t*) self->buf));
          //publish all the lcm messages in the buffer
          self->publishLcmMessagesInBuf(recv_udp_msg->payload_size);
        }
        else {
          fprintf(stderr, "ldpc got all the sent packets, but couldn't reconstruct... this shouldn't happen!\n");
        }
        self->message_complete = 1;
        delete self->ldpc_dec; //we're all done, so we can delete it
        self->ldpc_dec = NULL;
      }
    }
  }
  else if (self->verbose && !self->message_complete) {
    //    if (!self->message_complete && recv_udp_msg->seqno == self->cur_seqno && recv_udp_msg->nfrags == self->nfrags) {
    printf("ignoring udp packet seqno=%d, nfrag =%d, \t self-> seqno=%d, nfrags=%d\n", recv_udp_msg->seqno,
        getNumFragments(recv_udp_msg->payload_size), self->cur_seqno, self->nfrags);
  }

  lcm_tunnel_udp_msg_t_destroy(recv_udp_msg);

  return TRUE;
}

int LcmTunnel::on_tcp_data(GIOChannel * source, GIOCondition cond, void *user_data)
{
  int ret = TRUE;
  LcmTunnel * self = (LcmTunnel*) user_data;

  // increase buffer size if needed
  if (self->buf_sz < self->bytes_to_read) {
    assert(self->bytes_read == 0);
    self->buf = (char *) realloc(self->buf, self->bytes_to_read);
    self->buf_sz = self->bytes_to_read;
  }

  ssize_t nread = read(ssocket_get_fd(self->tcp_sock), self->buf + self->bytes_read, self->bytes_to_read
      - self->bytes_read);

  if (nread <= 0) {
    perror("tcp receive error: ");
    LcmTunnelServer::disconnectClient(self);
    return FALSE;
  }

  self->bytes_read += nread;
  assert(self->bytes_read <= self->bytes_to_read);

  if (self->bytes_read != self->bytes_to_read)
    return TRUE;

  switch (self->tunnel_state) {
  case CLIENT_MSG_SZ:
    self->bytes_to_read = ntohl(*(uint32_t*) self->buf);
    self->tunnel_state = CLIENT_MSG_DATA;
    break;
  case CLIENT_MSG_DATA:
    {
      lcm_tunnel_params_t tp_rec;
      int decode_status = lcm_tunnel_params_t_decode(self->buf, 0, self->bytes_read, &tp_rec);
      if (decode_status <= 0) {
        fprintf(stdout, "invalid request (%d)\n", decode_status);
        return FALSE;
      }
      self->tunnel_params = lcm_tunnel_params_t_copy(&tp_rec);

      if (self->udp_fd >= 0) {
        close(self->udp_fd);
      }
      self->udp_fd = -1;

      if (self->tunnel_params->udp) {
        //setup our UDP socket, and send info to client
        struct sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        getpeername(self->tcp_sock->socket, (struct sockaddr*) &client_addr, &addrlen);
        self->server_udp_port = ntohs(client_addr.sin_port);
        client_addr.sin_port = htons(self->tunnel_params->udp_port);

        // allocate UDP socket
        self->udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (self->udp_fd < 0) {
          perror("allocating UDP socket");
          LcmTunnelServer::disconnectClient(self);
          return FALSE;
        }

        connect(self->udp_fd, (struct sockaddr*) &client_addr, sizeof(client_addr));

        // transmit the udp port info
        struct sockaddr_in udp_addr;
        socklen_t udp_addr_len = sizeof(udp_addr);
        memset(&udp_addr, 0, sizeof(udp_addr));
        udp_addr.sin_family = AF_INET;
        udp_addr.sin_addr.s_addr = INADDR_ANY;
        udp_addr.sin_port = 0;
        getsockname(self->udp_fd, (struct sockaddr*) &udp_addr, &udp_addr_len);
        lcm_tunnel_params_t tp_port_msg;
        tp_port_msg.channels = (char *) " ";
        tp_port_msg.udp_port = ntohs(udp_addr.sin_port);
        int msg_sz = lcm_tunnel_params_t_encoded_size(&tp_port_msg);
        uint8_t msg[msg_sz];
        lcm_tunnel_params_t_encode(msg, 0, msg_sz, &tp_port_msg);
        uint32_t msg_sz_n = htonl(msg_sz);
        if (4 != _fileutils_write_fully(ssocket_get_fd(self->tcp_sock), &msg_sz_n, 4)) {
          perror("sending subscription data");
          LcmTunnelServer::disconnectClient(self);
          return FALSE;
        }
        if (msg_sz != _fileutils_write_fully(ssocket_get_fd(self->tcp_sock), msg, msg_sz)) {
          perror("sending subscription data");
          LcmTunnelServer::disconnectClient(self);
          return FALSE;
        }

        self->udp_ioc = g_io_channel_unix_new(self->udp_fd);
        self->udp_sid = g_io_add_watch(self->udp_ioc, G_IO_IN, LcmTunnel::on_udp_data, self);

        //we're done setting up the UDP connection...Disconnect tcp socket
        self->closeTCPSocket();
        ret = false;
      }

      //get ready to receive
      self->tunnel_state = RECV_CHAN_SZ;
      self->bytes_to_read = 4;

      //      if (self->server_params->verbose)
      fprintf(stderr, "%s subscribed to \"%s\" -- ", self->name, self->tunnel_params->channels);

      if (self->udp_fd >= 0) {
        if (self->tunnel_params->fec > 1)
          fprintf(stderr, "UDP with FEC rate of %.2f and max_delay of %dms\n", self->tunnel_params->fec,
              self->tunnel_params->max_delay_ms);
        else if (self->tunnel_params->fec < -1)
          fprintf(stderr, "UDP with DUP rate of %d and max_delay of %dms\n", (int) -self->tunnel_params->fec,
              self->tunnel_params->max_delay_ms);
        else
          fprintf(stderr, "UDP with a max_delay of %dms\n", self->tunnel_params->max_delay_ms);
      }
      else {
        fprintf(stderr, "TCP with max_delay of %dms and tcp_max_age_ms of %d\n", self->tunnel_params->max_delay_ms,
            self->tunnel_params->tcp_max_age_ms);
      }

      self->init_regex(self->tunnel_params->channels);

      //subscribe to the LCM channels
      if (self->subscription) {
        lcm_unsubscribe(self->lcm, self->subscription);
      }
      self->subscription = lcm_subscribe(self->lcm, self->tunnel_params->channels, on_lcm_message, self);

    }
    break;
  case SERVER_MSG_SZ:
    self->bytes_to_read = ntohl(*(uint32_t*) self->buf);
    self->tunnel_state = SERVER_MSG_DATA;
    break;
  case SERVER_MSG_DATA:
    {
      lcm_tunnel_params_t tp_rec;
      int decode_status = lcm_tunnel_params_t_decode(self->buf, 0, self->bytes_read, &tp_rec);
      if (decode_status <= 0) {
        fprintf(stderr, "invalid request (%d)\n", decode_status);
        return FALSE;
      }
      assert(self->udp_fd>0);
      struct sockaddr_in client_addr;
      socklen_t addrlen = sizeof(client_addr);
      getpeername(self->tcp_sock->socket, (struct sockaddr*) &client_addr, &addrlen);
      self->server_udp_port = tp_rec.udp_port;
      client_addr.sin_port = htons(tp_rec.udp_port);
      //connect the udp socket
      connect(self->udp_fd, (struct sockaddr*) &client_addr, sizeof(client_addr));

      //now we can subscribe to LCM
      fprintf(stderr, "%s subscribed to \"%s\" \n", self->name, self->tunnel_params->channels);
      self->subscription = lcm_subscribe(self->lcm, self->tunnel_params->channels, on_lcm_message, self);

      //we're done setting up the UDP connection...Disconnect tcp socket
      self->closeTCPSocket();
      ret = FALSE; //don't want the TCP handler to be run again
    }
    break;
  case RECV_CHAN_SZ:
    self->bytes_to_read = ntohl(*(uint32_t*) self->buf);
    self->tunnel_state = RECV_CHAN;

    if (self->channel_sz < self->bytes_to_read + 1) {
      self->channel = (char *) realloc(self->channel, self->bytes_to_read + 1);
      self->channel_sz = self->bytes_to_read + 1;
    }
    break;
  case RECV_CHAN:
    memcpy(self->channel, self->buf, self->bytes_read);
    self->channel[self->bytes_read] = 0;

    self->bytes_to_read = 4;
    self->tunnel_state = RECV_DATA_SZ;
    break;
  case RECV_DATA_SZ:
    self->bytes_to_read = ntohl(*(uint32_t*) self->buf);
    self->tunnel_state = RECV_DATA;
    break;
  case RECV_DATA:
    if (self->verbose)
      printf("Recieved TCP message on channel \"%s\"\n", self->channel);
    LcmTunnelServer::check_and_send_to_tunnels(self->channel, self->buf, self->bytes_read, self);
    lcm_publish(self->lcm, self->channel, (uint8_t*) self->buf, self->bytes_read);

    self->bytes_to_read = 4;
    self->tunnel_state = RECV_CHAN_SZ;
    break;
  }

  self->bytes_read = 0;

  return ret;
}

gpointer LcmTunnel::sendThreadFunc(gpointer user_data)
{

  LcmTunnel *self = (LcmTunnel*) user_data;

  g_mutex_lock(self->sendQueueLock);
  int64_t nextFlushTime = 0;
  while (!self->stopSendThread) {
    if (self->sendQueue.empty()) {
      g_cond_wait(self->sendQueueCond, self->sendQueueLock);
      nextFlushTime = _timestamp_now() + self->tunnel_params->max_delay_ms * 1000;
      continue;
    }
    int64_t now = _timestamp_now();
    if (self->tunnel_params->max_delay_ms > 0 && self->bytesInQueue < NUM_BYTES_TO_SEND_IMMEDIATELY && nextFlushTime
        > now && !self->flushImmediately) {
      GTimeVal next_timeout;
      _timestamp_to_GTimeVal(nextFlushTime, &next_timeout);
      g_cond_timed_wait(self->sendQueueCond, self->sendQueueLock, &next_timeout);

      continue;
    }
    //there is stuff in the queue that we need to handle
    self->flushImmediately = false;

    //take current contents out of the queue
    std::deque<TunnelLcmMessage *> tmpQueue;
    tmpQueue.swap(self->sendQueue);
    uint32_t bytesInTmpQueue = self->bytesInQueue;
    self->bytesInQueue = 0;
    g_mutex_unlock(self->sendQueueLock);
    //release lock for sending

    //process whats in the queue
    bool success = self->send_lcm_messages(tmpQueue, bytesInTmpQueue);

    //reaquire lock to go around the loop
    g_mutex_lock(self->sendQueueLock);
    if (!success)
      break;
  }
  g_mutex_unlock(self->sendQueueLock);

  g_thread_exit(NULL);
}

void LcmTunnel::send_to_remote(const void *data, uint32_t len, const char *lcm_channel)
{
  lcm_recv_buf_t rbuf;
  rbuf.data = (void *) data;
  rbuf.data_size = len;
  rbuf.recv_utime = _timestamp_now(); //use current timestamp, should be close to when received...
  rbuf.lcm = this->lcm;
  send_to_remote(&rbuf, lcm_channel);
 }

void LcmTunnel::send_to_remote(const lcm_recv_buf_t *rbuf, const char *lcm_channel)
{
  g_mutex_lock(sendQueueLock);
  TunnelLcmMessage * new_msg = new TunnelLcmMessage(rbuf, lcm_channel);
  bytesInQueue += new_msg->encoded_size;
  sendQueue.push_back(new_msg);
  while (bytesInQueue > MAX_SEND_BUFFER_SIZE) {
    fprintf(stderr, "Warning: send queue is too big (%dMB), dropping messages\n", bytesInQueue / (2 << 20));
    //need to drop some stuff
    TunnelLcmMessage * drop_msg = sendQueue.front();
    sendQueue.pop_front();
    bytesInQueue -= drop_msg->encoded_size;
    delete drop_msg;
  }
  //hack to not delay time sync messages
  flushImmediately = strcmp(lcm_channel, "TIMESYNC") == 0;
  g_mutex_unlock(sendQueueLock);
  g_cond_broadcast(sendQueueCond); //signal to say there is a message waiting
}

void LcmTunnel::on_lcm_message(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)
{
  LcmTunnel *self = (LcmTunnel*) user_data;
  if (introspect_is_message_from_self(self->introspect, rbuf, channel)) {
    if (self->verbose && !LcmTunnelServer::matches_a_client(channel))
      printf("Warning: Got message from self. There's a loop scenario.\n");
    return;
  }

  self->send_to_remote(rbuf, channel);
}

void LcmTunnel::checkUDPSendStatus(int send_status)
{
  int64_t now = _timestamp_now();
  if (send_status < 0) {
    if (errorStartTime < 0)
      errorStartTime = now;
    if (now - lastErrorPrintTime > 1e6) {
      fprintf(stderr, "errno %d, send_status %d :", errno, send_status);
      perror("error sending UDP Data: ");
      errno = 0;
      lastErrorPrintTime = now;
    }
    numSuccessful = 0;
  }
  else if (lastErrorPrintTime > 0) {
    if (numSuccessful > 3) {
      fprintf(stderr, "Connection may be back up after %fsec send_status=%d numSuccessful=%d\n", (double) (now
          - errorStartTime) * 1e-6, send_status, numSuccessful);
      perror("perror: ");
      lastErrorPrintTime = -1;
      errorStartTime = -1;
      errno = 0;
    }
    numSuccessful++;
  }
}

bool LcmTunnel::send_lcm_messages(std::deque<TunnelLcmMessage *> &msgQueue, uint32_t bytesInQueue)
{
  if (udp_fd >= 0) {
    if (server_udp_port <= 0)
      return true; //connection hasn't been setup yet.

    udp_send_seqno++;//increment the sequence counter
    udp_send_seqno = udp_send_seqno % SEQNO_WRAP_VAL;
    if (verbose)
      printf("sending %d bytes from %d lcm messages\n", bytesInQueue, static_cast<int> (msgQueue.size()));

    uint32_t msgSize = bytesInQueue;
    int nfragments = getNumFragments(msgSize);
    if ((tunnel_params->fec <= 0 && nfragments > MAX_NUM_FRAGMENTS) || (tunnel_params->fec > 0 && nfragments
        > MAX_NUM_FRAGMENTS / tunnel_params->fec)) {
      uint32_t maxMsgSize;
      if (tunnel_params->fec > 0)
        maxMsgSize = MAX_PAYLOAD_BYTES_PER_FRAGMENT * MAX_NUM_FRAGMENTS / tunnel_params->fec;
      else
        maxMsgSize = MAX_PAYLOAD_BYTES_PER_FRAGMENT * MAX_NUM_FRAGMENTS;
      fprintf(stderr,
          "WARNING! Queue contains more than the max message size of %d bytes... we're WAY behind, dropping msgs\n",
          maxMsgSize);
      while (msgSize < maxMsgSize) {
        //drop messages
        TunnelLcmMessage * drop_msg = msgQueue.front();
        msgQueue.pop_front();
        msgSize -= drop_msg->encoded_size;
        delete drop_msg;
      }
    }
    //put the entire queue into 1 big buffer
    uint8_t * msgBuf = (uint8_t *) malloc(msgSize * sizeof(uint8_t));
    uint32_t msgBufOffset = 0;
    while (!msgQueue.empty()) {
      TunnelLcmMessage * msg = msgQueue.front();
      msgQueue.pop_front();
      lcm_tunnel_sub_msg_t_encode(msgBuf, msgBufOffset, msgSize - msgBufOffset, msg->sub_msg);
      msgBufOffset += msg->encoded_size;
      delete msg;
    }
    assert(msgBufOffset==msgSize);

    if (tunnel_params->fec < 1 || nfragments < MIN_NUM_FRAGMENTS_FOR_FEC) { //don't use FEC
      int sendRepeats = 1;
      if (fabs(tunnel_params->fec) > 1) { //fec <0 means always send duplicates
        sendRepeats = (int) ceil(fabs(tunnel_params->fec)); //send ceil of the fec rate times
      }
      for (int r = 0; r < sendRepeats; r++) {
        msgBufOffset = 0;
        for (int i = 0; i < nfragments; i++) {
          lcm_tunnel_udp_msg_t msg;
          msg.seqno = udp_send_seqno;
          msg.fragno = i;
          msg.payload_size = msgSize;

          msg.data_size = MIN(MAX_PAYLOAD_BYTES_PER_FRAGMENT, msgSize - msgBufOffset);
          msg.data = msgBuf + msgBufOffset;

          msgBufOffset += msg.data_size;

          int msg_sz = lcm_tunnel_udp_msg_t_encoded_size(&msg);
          uint8_t msg_buf[msg_sz];
          lcm_tunnel_udp_msg_t_encode(msg_buf, 0, msg_sz, &msg);
          //          printf("sending: %d, %d / %d\n", msg.seqno, msg.fragment, msg.nfrags);
          int send_status = send(udp_fd, msg_buf, msg_sz, 0);
          //          fprintf(stderr,"sent packet\n");
          checkUDPSendStatus(send_status);
        }
      }
    }
    else { //use tunnel error correction to send
      ldpc_enc_wrapper * ldpc_enc = new ldpc_enc_wrapper(msgBuf, msgSize, MAX_PAYLOAD_BYTES_PER_FRAGMENT,
          tunnel_params->fec);

      lcm_tunnel_udp_msg_t msg;
      msg.seqno = udp_send_seqno;
      msg.payload_size = msgSize;

      //      printf("sending: %d, %d / %d\n", recv_udp_msg->seqno, recv_udp_msg->fragment, recv_udp_msg->nfrags);

      int enc_done = 0;
      while (!enc_done) {
        uint8_t data_buf[MAX_PAYLOAD_BYTES_PER_FRAGMENT];
        msg.data_size = MAX_PAYLOAD_BYTES_PER_FRAGMENT;
        msg.data = data_buf;
        enc_done = ldpc_enc->getNextPacket(msg.data, &msg.fragno);

        int msg_sz = lcm_tunnel_udp_msg_t_encoded_size(&msg);
        uint8_t msg_buf[msg_sz];
        lcm_tunnel_udp_msg_t_encode(msg_buf, 0, msg_sz, &msg);
        //          printf("sending: %d, %d / %d\n", msg.seqno, msg.fragment, msg.nfrags);
        int send_status = send(udp_fd, msg_buf, msg_sz, 0);
        checkUDPSendStatus(send_status);
      }
      //          printf("finished encoding and sending packet %d for channel: %s\n",recv_udp_msg->seqno,channel);
      delete ldpc_enc;
    }
    free(msgBuf);
  }
  else {
    int cfd = ssocket_get_fd(tcp_sock);
    assert(cfd>0);

    while (!msgQueue.empty()) {
      TunnelLcmMessage * msg = msgQueue.front();
      msgQueue.pop_front();

      int64_t now = _timestamp_now();
      double age_ms = (now - msg->recv_utime) * 1.0e-3;
      if (tunnel_params->tcp_max_age_ms > 0 && age_ms > tunnel_params->tcp_max_age_ms) {
        // message has been queued up for too long.  Drop it.
        if (verbose)
          fprintf(stderr, "%s message too old (age = %d, param = %d), dropping.\n", msg->sub_msg->channel,
              (int) age_ms, tunnel_params->tcp_max_age_ms);
      }
      else {
        // send channel
        int chan_len = strlen(msg->sub_msg->channel);
        uint32_t chan_len_n = htonl(chan_len);
        if (4 != _fileutils_write_fully(cfd, &chan_len_n, 4)) {
          delete msg;
          return false;
        }
        if (chan_len != _fileutils_write_fully(cfd, msg->sub_msg->channel, chan_len)) {
          delete msg;
          return false;
        }

        // send data
        int data_size_n = htonl(msg->sub_msg->data_size);
        if (4 != _fileutils_write_fully(cfd, &data_size_n, 4)) {
          delete msg;
          return false;
        }
        if (msg->sub_msg->data_size != _fileutils_write_fully(cfd, msg->sub_msg->data, msg->sub_msg->data_size)) {
          delete msg;
          return false;
        }
      }
      if (verbose)
        printf("Sent \"%s\".\n", msg->sub_msg->channel);
      delete msg;
    }
  }

  return true;
}

static gboolean on_introspect_timer(void* user_data)
{
  introspect_t* ini = (introspect_t*) user_data;
  introspect_send_introspection_packet(ini);
  return TRUE;
}

typedef struct {
  bool connectToServer;
  char server_addr_str[1024];
  int server_port;
  char channels_recv[1024];
  char channels_send[1024];
  int udp;
  int port;
  int verbose;
  char lcm_url[1024];
  int tcp_max_age_ms;
  int max_delay_ms;
  float fec;
} app_params_t;

static void usage(const char *progname)
{
  char *basename = g_path_get_basename(progname);
  printf("Usage: %s [options] <server_addr[:server_port]>\n"
    "     If server_port not specified, default (%d) will be used\n"
    "\n"
    "Options:\n"
    "\n"
    "    -h, --help                Shows this help text and exits\n"
    "    -q, --quiet               Quiet mode\n"
    "    -r, --channels_recv=CHAN  Ask the server to Subscribe to \n"
    "                              regex CHAN.  CHAN\n"
    "                              is automatically surrounded by ^ and $.\n"
    "                              (Default: .*)\n"
    "    -s, --channels_send=CHAN  Subscribe to regex CHAN.  CHAN\n"
    "                              these channels will be forwarded to the server\n"
    "                              is automatically surrounded by ^ and $.\n"
    "                              (Default: .*)\n"
    "    -R/-S                      Capitalize to invert the regex match\n"
    "\n"
    "    -l, --lcm-url=URL         Transmit to specified LCM URL\n"
    "    -p, --port=N              Start server on control port N instead of\n"
    "                              default (%d)\n"
    "\n"
    "    -u, --udp                 Request server transmit via UDP instead of TCP\n"
    "\n"
    "    -m, --tcp-max-age-ms=AGE  Instructs the server not to tunnel messages\n"
    "                              that have been queued up and waiting for\n"
    "                              delivery for more than AGE ms.  If less than\n"
    "                              or equal to zero, then messages can be queued\n"
    "                              indefinitely.  This option is not used when -u\n"
    "                              is specified.  Default: 10000\n"
    "\n"
    "    -f, --fec=FEC             Request server to use UDP packets with Forward\n"
    "                              Error Correction applied at a rate of FEC \n"
    "                              (must be >1) small messages will be duplicated \n"
    "                              ceil(FEC) times instead of coding.       \n"
    "\n"
    "    -d, --dup=NDUP            Request server to use UDP packets with each\n"
    "                              packet sent NDUP times for drop resiliency\n"
    "                              --fec and --dup cannot be used together\n"
    "\n"
    "\n"
    "    -w, --wait-time-ms=TIME   Request server to queue up lcm messages for\n"
    "                              TIME ms before sending as a group\n"
    "                              for efficiency reasons\n"
    "\n"
    "Examples:\n"
    "\n"
    " %s \n"
    "    Start a server. Clients can then connect using one of the examples below.\n"
    "    NOTE: Any node can be connected to, and will act as a server.\n"
    "\n"
    " %s 192.168.1.1\n"
    "    Tunnels traffic on all LCM channels between us and 192.168.1.1\n"
    "\n"
    " %s -s ABC 192.168.1.1\n"
    "    Server at 192.168.1.1 forwards traffic on channel ABC, we forward all\n"
    "    channels back.\n"
    "\n"
    " %s -r ABC 192.168.1.1\n"
    "    Server at 192.168.1.1 forwards all channels, we forward traffic on channel\n"
    "    ABC back.\n"
    "\n"
    " %s -u -f 1.5 -s \"ABC|DEF\" -r \"\" 192.168.1.1\n"
    "    We forward traffic on channels ABC and DEF to 192.168.1.1 via UDP with\n"
    "    FEC 1.5.  Server does not forward anything back.\n"
    "\n", basename, DEFAULT_PORT, DEFAULT_PORT, basename, basename, basename, basename, basename);
  free(basename);
  exit(1);
}

int main(int argc, char **argv)
{
  setlinebuf(stdout);

  const char *optstring = "hvqur:s:R:S:p:f:l:m:d:w:";

  app_params_t params;
  memset(&params, 0, sizeof(params));

  params.connectToServer = false;
  params.server_port = DEFAULT_PORT;
  params.port = DEFAULT_PORT;
  params.verbose = 0;
  params.udp = 0;
  params.tcp_max_age_ms = 10000;
  params.max_delay_ms = 0;
  params.fec = 0;
  strcpy(params.channels_recv, ".*");
  strcpy(params.channels_send, ".*");
  memset(params.lcm_url, 0, sizeof(params.lcm_url));

  struct option long_opts[] = { { "help", no_argument, 0, 'h' },
      { "verbose", no_argument, 0, 'v' },
      { "quiet", no_argument, 0, 'q' },
      { "udp", no_argument, 0, 'u' },
      { "channels_recv", required_argument, 0, 'r' },
      { "channels_send", required_argument, 0, 's' },
      { "port", required_argument, 0, 'p' },
      { "fec", required_argument, 0, 'f' },
      { "dup", required_argument, 0, 'd' },
      { "wait-time-us", required_argument, 0, 'w' },
      { "lcm-url", required_argument, 0, 'l' },
      { "tcp-max-age-ms", required_argument, 0, 'm' },
      { 0, 0, 0, 0 } };

  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      break;
    case 'v':
      params.verbose = 1;
      break;
    case 'q':
      params.verbose = 0;
      break;
    case 'p':
      {
        char *e;
        params.port = strtol(optarg, &e, 0);
        if (*e != '\0' || params.port > 65535 || params.port < 0)
          usage(argv[0]);
        break;
      }
    case 'r':
      if (strlen(optarg) > sizeof(params.channels_recv) - 1) {
        fprintf(stderr, "recv channels string too long\n");
        return 1;
      }
      strcpy(params.channels_recv, optarg);
      break;
    case 's':
      if (strlen(optarg) > sizeof(params.channels_send) - 1) {
        fprintf(stderr, "send channels string too long\n");
        return 1;
      }
      strcpy(params.channels_send, optarg);
      break;

    case 'R':
      if (strlen(optarg) > sizeof(params.channels_recv) - 1) {
        fprintf(stderr, "recv channels string too long\n");
        return 1;
      }
      sprintf(params.channels_recv, "^(?!^%s$).*+$", optarg);
      printf("params.channels_recv=%s\n", params.channels_recv);
      break;
    case 'S':
      if (strlen(optarg) > sizeof(params.channels_send) - 1) {
        fprintf(stderr, "send channels string too long\n");
        return 1;
      }
      sprintf(params.channels_send, "^(?!^%s$).*+$", optarg);
      break;
    case 'u':
      params.udp = 1;
      break;
    case 'l':
      if (strlen(optarg) > sizeof(params.lcm_url) - 1) {
        fprintf(stderr, "LCM URL string too long\n");
        return 1;
      }
      strcpy(params.lcm_url, optarg);
      break;
    case 'm':
      {
        char *e;
        params.tcp_max_age_ms = strtol(optarg, &e, 0);
        if (*e != '\0')
          usage(argv[0]);
        break;
      }
    case 'w':
      {
        char *e;
        params.max_delay_ms = strtol(optarg, &e, 0);
        if (*e != '\0')
          usage(argv[0]);
        break;
      }
    case 'f':
      {
        char *e;
        if (params.fec < 0)
          usage(argv[0]);
        params.fec = strtod(optarg, &e);
        if (*e != '\0' || params.fec <= 1)
          usage(argv[0]);
        params.udp = 1; //use udp to send FEC data
        break;
      }
    case 'd':
      {
        char *e;
        if (params.fec > 0)
          usage(argv[0]);
        params.fec = strtod(optarg, &e);
        if (*e != '\0' || params.fec < 2)
          usage(argv[0]);
        params.udp = 1; //use udp to send FEC data
        params.fec = -round(params.fec); //if fec is negative, its handled as duplicates
        break;
      }

    default:
      usage(argv[0]);
      break;
    }
  }

  if (optind < argc - 1) {
    usage(argv[0]);
  }
  params.connectToServer = (optind == argc - 1);
  if (params.connectToServer) {
    strcpy(params.server_addr_str, argv[optind]);
    if (strlen(argv[optind]) > sizeof(params.server_addr_str) - 1) {
      fprintf(stderr, "server address string too long\n");
      return 1;
    }
    //pull out the server's port if its specified
    char * colon = strchr(params.server_addr_str, ':');
    if (colon != NULL) {
      *colon = '\0';
      char * e;
      params.server_port = strtol(colon + 1, &e, 0);
      if (*e != '\0' || params.server_port > 65535 || params.server_port < 0)
        usage(argv[0]);
    }
  }

  if (params.connectToServer) {
    fprintf(stderr, "Sending:   \"%s\"\n", params.channels_send);
    fprintf(stderr, "Recieving: \"%s\"\n", params.channels_recv);
  }
  else {
    fprintf(stderr, "Not connecting to any Servers\n");
  }

  // block SIGPIPE (allow write() to return -1 on broken pipe)
  sigset_t toblock;
  sigemptyset(&toblock);
  sigaddset(&toblock, SIGPIPE);
  sigprocmask(SIG_BLOCK, &toblock, NULL);

  //setup the server side of things
  tunnel_server_params_t serv_params;
  serv_params.startedAsClient = params.connectToServer;
  serv_params.port = params.port;
  strcpy(serv_params.lcm_url, params.lcm_url);
  serv_params.verbose = params.verbose;
  if(!LcmTunnelServer::initializeServer(&serv_params)) {
    exit(1);
  }

  //create this client
  if (params.connectToServer) {
    lcm_tunnel_params_t tunnel_params;
    tunnel_params.fec = params.fec;
    tunnel_params.tcp_max_age_ms = params.tcp_max_age_ms;
    tunnel_params.udp = params.udp;
    tunnel_params.max_delay_ms = params.max_delay_ms;
    tunnel_params.channels = strdup(params.channels_send);
    LcmTunnel * tunnelClient = new LcmTunnel(params.verbose, NULL);
    int ret = tunnelClient->connectToServer(LcmTunnelServer::lcm, LcmTunnelServer::introspect,
        LcmTunnelServer::mainloop, params.server_addr_str, params.server_port, params.channels_recv, &tunnel_params,
        &LcmTunnelServer::params);

    if (ret) {
      LcmTunnelServer::clients_list.push_front(tunnelClient);
    }
    else {
      fprintf(stderr, "Could not connect to server, exiting\n");
      exit(1);
    }

    free(tunnel_params.channels);
  }

  // periodically send an introspection packet in case network routes change
  g_timeout_add(30000, on_introspect_timer, LcmTunnelServer::introspect);

  // run
  g_main_loop_run(LcmTunnelServer::mainloop);

  fprintf(stderr, "Destroying the LcmTunnelServer and Exiting\n");
  LcmTunnelServer::destroyServer();

  return 0;
}
