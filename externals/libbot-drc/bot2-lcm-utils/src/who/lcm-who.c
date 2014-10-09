/**
 * bot-lcm-who is a utility for inspecting LCM UDPM traffic to determine
 * traffic sources.
 *
 * If a client is spamming an LCM network when it should not be, this utility
 * can be used to determine the source IP address and port of the offender.
 *
 * 
 *
 */
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/uio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <assert.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>

#include <glib.h>
#include "signal_pipe.h"

#define LCM_DEFAULT_URL "udpm://239.255.76.67:7667?ttl=0"
#define PROGNAME "lcm-who"

#ifdef __APPLE__
#define USE_REUSEPORT
#else
#ifdef __FreeBSD__
#define USE_REUSEPORT
#endif
#endif

#define DEFAULT_REPORT_INTERVAL_SECONDS 1

static inline int64_t timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

typedef struct {
    struct in_addr addr;
    uint16_t port;
    int64_t last_recvtime;
    int key;
    char *id_str;

    double bandwidth;
} sender_t;

static sender_t *
sender_new(struct in_addr addr, uint16_t port)
{
    sender_t *sender = g_slice_new(sender_t);
    sender->addr = addr;
    sender->port = port;
    sender->key = (int) port;
    sender->last_recvtime = 0;
    sender->id_str = g_strdup_printf("%s:%d", inet_ntoa(addr), port);
    return sender;
}

static void
sender_destroy(sender_t *sender)
{
    free(sender->id_str);
    g_slice_free(sender_t, sender);
}

typedef struct {
    struct in_addr addr;
    int64_t last_recvtime;
    char *addr_str;
    GHashTable * senders;
} host_t;

static host_t *
host_new(struct in_addr addr)
{
    host_t *host = g_slice_new(host_t);
    host->addr = addr;
    host->last_recvtime = 0;
    host->addr_str = strdup(inet_ntoa(addr));
    host->senders = g_hash_table_new_full(g_int_hash, g_int_equal, NULL,
            (GDestroyNotify) sender_destroy);
    return host;
}

static void
host_destroy(host_t *host)
{
    free(host->addr_str);
    g_hash_table_destroy(host->senders);
    g_slice_free(host_t, host);
}

static sender_t * 
host_get_sender(host_t *host, uint16_t port)
{
    int kv = (int)port;
    return g_hash_table_lookup(host->senders, &kv);
}

static sender_t * 
host_add_new_sender(host_t *host, uint16_t port)
{
    sender_t *sender  = host_get_sender(host, port);
    assert(!sender );
    sender = sender_new(host->addr, port);
    g_hash_table_insert(host->senders, &sender->key, sender);
    return sender;
}

typedef struct {
    int recvfd;
    struct in_addr mc_addr;
    uint16_t mc_port;
    char *lcmurl;
    int recv_buf_size;

    GHashTable *hosts;

    GMainLoop *mainloop;
    GIOChannel *ioc;
    guint sid;

    int64_t next_report_utime;
    int64_t report_interval_usec;
} state_t;

static int
lcm_parse_url(const char * url, struct in_addr *mc_addr, uint16_t *mc_port)
{
    if (!url || !strlen (url))
        return -1;

    char ** provider_networkargs = g_strsplit (url, "://", 2);
    if (!provider_networkargs[1]) {
        g_strfreev (provider_networkargs);
        return -1;
    }
    
    if(strcmp(provider_networkargs[0], "udpm")) {
        fprintf(stderr, "%s only works with udpm provider\n", PROGNAME);
        return -1;
    }

    char **network_args = g_strsplit(provider_networkargs[1], "?", 2);
    g_strfreev (provider_networkargs);

    const char *network = network_args[0];

    if (!network || !strlen(network)) {
        network = "239.255.76.67:7667";
    }

    char **words = g_strsplit(network, ":", 2);
    if (inet_aton(words[0], (struct in_addr*)mc_addr) < 0) {
        fprintf(stderr, "Error: Bad multicast IP address \"%s\"\n", words[0]);
        perror("inet_aton");
        goto fail;
    }
    if (words[1]) {
        char *st = NULL;
        int port = strtol (words[1], &st, 0);
        if (st == words[1] || port < 0 || port > 65535) {
            fprintf (stderr, "Error: Bad multicast port \"%s\"\n", words[1]);
            goto fail;
        }
        *mc_port = port;
    }

    g_strfreev(words);
    g_strfreev(network_args);
    return 0;
fail:
    g_strfreev(words);
    g_strfreev(network_args);
    return -1;
}

static int
setup_socket(state_t *app)
{
    // allocate multicast socket
    app->recvfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (app->recvfd < 0) {
        perror ("allocating socket");
        return 1;
    }

    // allow other applications on the local machine to also bind to this
    // multicast address and port
    int opt=1;
    if (setsockopt(app->recvfd, SOL_SOCKET, SO_REUSEADDR, 
            (char*)&opt, sizeof (opt)) < 0) {
        perror ("setsockopt (SOL_SOCKET, SO_REUSEADDR)");
        return -1;
    }

#ifdef USE_REUSEPORT
    /* Mac OS and FreeBSD require the REUSEPORT option in addition
     * to REUSEADDR or it won't let multiple processes bind to the
     * same port, even if they are using multicast. */
    if (setsockopt(app->recvfd, SOL_SOCKET, SO_REUSEPORT, 
            (char*)&opt, sizeof (opt)) < 0) {
        perror ("setsockopt (SOL_SOCKET, SO_REUSEPORT)");
        return -1;
    }
#endif

    /* Enable per-packet timestamping by the kernel, if available */
    struct sockaddr_in addr;
    memset (&addr, 0, sizeof (addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(app->mc_port);

    if (bind(app->recvfd, (struct sockaddr*)&addr, sizeof (addr)) < 0) {
        perror ("bind");
        return -1;
    }

    struct ip_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr = app->mc_addr;
    mreq.imr_interface.s_addr = INADDR_ANY;
    // join the multicast group
    if (setsockopt (app->recvfd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
            (char*)&mreq, sizeof (mreq)) < 0) {
        perror ("setsockopt (IPPROTO_IP, IP_ADD_MEMBERSHIP)");
        return -1;
    }
    return 0;
}

static int
on_message_ready(GIOChannel *source, GIOCondition cond, void *user_data)
{
    state_t *app = (state_t*) user_data;

    char buf[65536];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(struct sockaddr);

    int sz = recvfrom(app->recvfd, buf, 65536, 0, 
                       (struct sockaddr*) &from, &fromlen);

    if (sz < 0) {
        perror("receiving message");
        return TRUE;
    }
    int64_t recv_utime = timestamp_now();
    time_t recv_t = recv_utime / 1000000;
    struct tm *recv_tm = localtime(&recv_t);
    char recv_tm_buf[200];
    strftime(recv_tm_buf, sizeof(recv_tm_buf), "%b %d %H:%M:%S", recv_tm);

    int *key = (int*) &from.sin_addr.s_addr;
    host_t * host = g_hash_table_lookup(app->hosts, key);
    if(!host) {
        host = host_new(from.sin_addr);
        g_hash_table_insert(app->hosts, &host->addr.s_addr, host);
        printf("%s - new host detected!    %s\n", recv_tm_buf, host->addr_str);
    }

    sender_t *sender = host_get_sender(host, ntohs(from.sin_port));
    if(!sender) {
        sender = host_add_new_sender(host, ntohs(from.sin_port));
        printf("%s - new sender detected!  %s\n", recv_tm_buf, sender->id_str);
    }

    sender->last_recvtime = recv_utime;

    host->last_recvtime = recv_utime;

    return TRUE;
}

static gboolean
on_timer(void *user_data)
{
    state_t *app = (state_t*) user_data;

    int64_t now = timestamp_now();
    if(now > app->next_report_utime) {
//        int nhosts = g_hash_table_size(app->hosts);
//        printf("%d host%s transmitting\n", nhosts, nhosts == 1 ? "" : "s");

        app->next_report_utime = now + app->report_interval_usec;
    }
    return TRUE;
}

static const char *
get_default_lcm_url(void)
{
    const char *url = getenv("LCM_DEFAULT_URL");
    if (!url || !strlen(url))
        url = LCM_DEFAULT_URL;
    return url;
}

int main(int argc, char **argv)
{
    state_t *app = calloc(1, sizeof(state_t));
    app->lcmurl = strdup(get_default_lcm_url());

    if (0 != lcm_parse_url(app->lcmurl, &app->mc_addr, &app->mc_port)) {
        fprintf (stderr, "invalid URL [%s]\n", app->lcmurl);
        return 1;
    }

    struct in_addr ia;
    ia = app->mc_addr;
    printf("MC group: %s port: %d\n", inet_ntoa(ia), app->mc_port);
    if(0 != setup_socket(app)) {
        return 1;
    }

    app->hosts = g_hash_table_new_full(g_int_hash, g_int_equal, NULL, 
            (GDestroyNotify)host_destroy);

    app->mainloop = g_main_loop_new(NULL, FALSE);

    app->ioc = g_io_channel_unix_new(app->recvfd);
    app->sid = g_io_add_watch(app->ioc, G_IO_IN, (GIOFunc)on_message_ready, 
            app);

    g_timeout_add(100, on_timer, app);
    app->report_interval_usec = DEFAULT_REPORT_INTERVAL_SECONDS * 1000000;
    app->next_report_utime = timestamp_now() + 1000000;

    bot_signal_pipe_glib_quit_on_kill(app->mainloop);
    g_main_loop_run(app->mainloop);

    g_io_channel_unref(app->ioc);
    g_source_remove(app->sid);

    shutdown(app->recvfd, SHUT_RDWR);

    g_hash_table_destroy(app->hosts);
    free(app->lcmurl);
    free(app);
    return 0;
}
