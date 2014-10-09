#ifndef _SSOCKET_H
#define _SSOCKET_H


#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ssocket ssocket_t;

struct ssocket
{
	int type;
	int socket;

	struct sockaddr addr;
	socklen_t addrlen;

};

ssocket_t *ssocket_create(void);
void ssocket_destroy(ssocket_t *s);

// returns < 0 on error
int ssocket_connect(ssocket_t *s, const char *hostname, int port);

int ssocket_disable_nagle(ssocket_t *s);
int ssocket_listen(ssocket_t *s, int port, int listenqueue, int localhostOnly);
ssocket_t *ssocket_accept(ssocket_t *s);

/**
 * ssocket_get_remote_ip:
 *
 * retrieves the IP address of the remote end of the connection.  each octet of
 * the address is stored in a separate array element
 */
void ssocket_get_remote_ip(ssocket_t *s, int ip[4]);

int ssocket_get_fd(ssocket_t *s);


#ifdef __cplusplus
}
#endif
#endif
