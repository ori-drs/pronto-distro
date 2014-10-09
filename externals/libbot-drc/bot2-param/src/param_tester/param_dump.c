#include <stdio.h>
#include <stdlib.h>

#include <bot_param/param_client.h>
#include <lcmtypes/bot2_param.h>
#include "../param_client/param_internal.h"

static void _on_param_update(const lcm_recv_buf_t *rbuf, const char * channel, const bot_param_update_t * msg,
    void * user)
{

  printf("%s", msg->params);
  exit(1);
}

int main()
{
  lcm_t * lcm = lcm_create(NULL);

  bot_param_update_t_subscribe(lcm, BOT_PARAM_UPDATE_CHANNEL, _on_param_update, NULL);

  while (1)
    lcm_handle(lcm);

}
