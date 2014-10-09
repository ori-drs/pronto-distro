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
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h>
#include <strings.h>
#include <string.h>
#include <signal.h>

#define BOT_SSOCKET_UNKNOWN_TYPE 0
#define BOT_SSOCKET_SERVER_TYPE 1
#define BOT_SSOCKET_CLIENT_TYPE 2

#include "ssocket.h"

bot_ssocket_t *bot_ssocket_create()
{
	bot_ssocket_t *s = (bot_ssocket_t*) calloc(1, sizeof(bot_ssocket_t));

	return s;
}

void bot_ssocket_destroy(bot_ssocket_t *s)
{
	close(s->socket);
	free(s);
}

int bot_ssocket_connect(bot_ssocket_t *s, const char *hostname, int port)
{
	struct hostent *host;
	struct sockaddr_in sa;
	int thesocket;
	
	/* let's find out about this host */
	host=gethostbyname(hostname);
	if (host==NULL)
	{
		//      perror(hostname);
		return -1;
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
	sa.sin_addr=*(struct in_addr *) host->h_addr;

	if (connect(thesocket, (struct sockaddr *) &sa, sizeof (sa)))
	{
		if (errno!=EINPROGRESS)
		{
			//        perror("connect failed");
			close(thesocket);
			return -1;
		}
	}

	s->type=BOT_SSOCKET_CLIENT_TYPE;
	s->socket=thesocket;

	// prevent "broken pipe" signals.
	signal(SIGPIPE, SIG_IGN);

	return 0;
}

// returns 0 on success
int bot_ssocket_disable_nagle(bot_ssocket_t *s)
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

int bot_ssocket_listen(bot_ssocket_t *s, int port, int listenqueue, int localhostOnly)
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

	s->type=BOT_SSOCKET_SERVER_TYPE;
	s->socket=thesocket;

	return 0;
}

bot_ssocket_t *bot_ssocket_accept(bot_ssocket_t *s)
{
	int thesocket;
	bot_ssocket_t *cs = bot_ssocket_create();
       
	s->addrlen=sizeof(struct sockaddr);

	if (s->type!=BOT_SSOCKET_SERVER_TYPE)
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

	cs->type = BOT_SSOCKET_CLIENT_TYPE;
	cs->socket = thesocket;

	return cs;
}

void bot_ssocket_get_remote_ip(bot_ssocket_t *s, int *ip)
{
	struct sockaddr addr;
	socklen_t addrlen;
	
	getpeername(s->socket,&addr,&addrlen);
	
	ip[0] = (int) (addr.sa_data[2])&0xff;
	ip[1] = (int) (addr.sa_data[3])&0xff;
	ip[2] = (int) (addr.sa_data[4])&0xff;
	ip[3] = (int) (addr.sa_data[5])&0xff;
}

int bot_ssocket_get_fd(bot_ssocket_t *s)
{
	return s->socket;
}
