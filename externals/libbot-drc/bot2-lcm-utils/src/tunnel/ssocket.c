#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h>
#include <strings.h>
#include <string.h>
#include <signal.h>

#define SSOCKET_UNKNOWN_TYPE 0
#define SSOCKET_SERVER_TYPE 1
#define SSOCKET_CLIENT_TYPE 2

#include "ssocket.h"

ssocket_t *ssocket_create()
{
    ssocket_t *s = (ssocket_t*) calloc(1, sizeof(ssocket_t));
    s->socket = -1;

    return s;
}

void ssocket_destroy(ssocket_t *s)
{
    if (s->socket >= 0)
        close(s->socket);
    free(s);
}

int ssocket_connect(ssocket_t *s, const char *hostname, int port)
{
	struct hostent *host = NULL;
	struct sockaddr_in sa, ca;
	int thesocket;

	/* let's find out about this host */
    int host_lookup = 0;
    if(0 == inet_aton(hostname, &ca.sin_addr)) {
        host_lookup = 1;
        host=gethostbyname(hostname);
        if (host==NULL)
        {
            //      perror(hostname);
            return -1;
        }
    }

	/* create the socket */
	thesocket=socket(AF_INET,SOCK_STREAM,0);

	/* fill in the fields */
	bzero(&sa,sizeof(sa));
	sa.sin_family=AF_INET;
	sa.sin_port=htons(0);
	sa.sin_addr.s_addr=htonl(INADDR_ANY);

	/* bind it to the port */
	if (bind (thesocket, (struct sockaddr *) &sa, sizeof (sa)) <0)
	{
		close(thesocket);
		//      perror ("bind failed");
		return -1;
	}

	sa.sin_port=htons(port);
    if (host_lookup) 
        sa.sin_addr=*(struct in_addr *) host->h_addr;
    else
        sa.sin_addr=ca.sin_addr;

	if (connect(thesocket, (struct sockaddr *) &sa, sizeof (sa)))
	{
		if (errno!=EINPROGRESS)
		{
			//        perror("connect failed");
			close(thesocket);
			return -1;
		}
	}

	s->type=SSOCKET_CLIENT_TYPE;
	s->socket=thesocket;

	// prevent "broken pipe" signals.
	signal(SIGPIPE, SIG_IGN);

	return 0;
}

// returns 0 on success
int ssocket_disable_nagle(ssocket_t *s)
{
	int n=1;

	if (setsockopt (s->socket, IPPROTO_TCP, TCP_NODELAY,
			(char *) &n, sizeof(n))<0)
	{
		perror("could not setsockopt");
		close(s->socket);
		return -1;
	}
	
	return 0;
}

int ssocket_listen(ssocket_t *s, int port, int listenqueue, int localhostOnly)
{
	int thesocket,n;
	struct sockaddr_in sa;

	thesocket=socket(AF_INET,SOCK_STREAM,0);
	if (thesocket==-1)
		return -1;

	/* avoid address already in use errors */
	n=1;
	if (setsockopt (thesocket, SOL_SOCKET, SO_REUSEADDR,
			(char *) &n, sizeof(n))<0)
	{
		//      perror("could not setsockopt");
		close(thesocket);
		return -1;
	}

	/* fill in the fields */
	bzero(&sa,sizeof(sa));
	sa.sin_family=AF_INET;
	sa.sin_port=htons(port);
	sa.sin_addr.s_addr=htonl(localhostOnly ? INADDR_LOOPBACK : INADDR_ANY);

	/* try to bind to the port */

	if (bind (thesocket, (struct sockaddr *) &sa, sizeof (sa)) <0)
	{
		close(thesocket);
		//      perror ("bind failed\n");
		return -1;
	}

	/* start listening */
	if (listen(thesocket,listenqueue))
	{
		close(thesocket);
		perror("listen");
		return -1;
	}

	s->type=SSOCKET_SERVER_TYPE;
	s->socket=thesocket;

	return 0;
}

ssocket_t *ssocket_accept(ssocket_t *s)
{
	int thesocket;
	ssocket_t *cs = ssocket_create();
       
	s->addrlen=sizeof(struct sockaddr);

	if (s->type!=SSOCKET_SERVER_TYPE)
	{
		printf("not server type\n");
		return NULL;
	}

	/* actually accept the connection */
	if ((thesocket=accept(s->socket,&s->addr,&s->addrlen))<0)
	{
		perror("accept failed");
		return NULL;
	}

	cs->type = SSOCKET_CLIENT_TYPE;
	cs->socket = thesocket;

	return cs;
}

void ssocket_get_remote_ip(ssocket_t *s, int *ip)
{
	struct sockaddr addr;
	socklen_t addrlen;
	
	getpeername(s->socket,&addr,&addrlen);
	
	ip[0] = (int) (addr.sa_data[2])&0xff;
	ip[1] = (int) (addr.sa_data[3])&0xff;
	ip[2] = (int) (addr.sa_data[4])&0xff;
	ip[3] = (int) (addr.sa_data[5])&0xff;
}

int ssocket_get_fd(ssocket_t *s)
{
	return s->socket;
}
