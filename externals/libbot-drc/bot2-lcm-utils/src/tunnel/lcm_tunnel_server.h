#ifndef __lcm_tunnel_server_h__
#define __lcm_tunnel_server_h__

#include <inttypes.h>
#include "lcm_tunnel.h"
#include "ldpc/ldpc_wrapper.h"
#include "introspect.h"
#include "ssocket.h"
#include <list>

class LcmTunnelServer {
public:
  static int initializeServer(tunnel_server_params_t * params);
  static void destroyServer();

  static bool initialized;

  static int acceptClient(GIOChannel *source, GIOCondition cond, void *user_data);
  static int disconnectClient(LcmTunnel * client);

  static GMainLoop * mainloop;
  static lcm_t * lcm;
  static introspect_t * introspect;

  static bool matches_a_client(const char *channel);
  static void check_and_send_to_tunnels(const char *channel,
      const void *data, unsigned int len, LcmTunnel * to_skip);

  static std::list<LcmTunnel *> clients_list;

  static ssocket_t * server_sock;
  static GIOChannel * server_sock_ioc;
  static guint server_sock_sid;

  static tunnel_server_params_t params;

};

#endif
