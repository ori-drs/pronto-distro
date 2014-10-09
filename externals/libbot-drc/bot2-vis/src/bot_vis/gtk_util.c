#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

#include <gtk/gtk.h>

#include "gtk_util.h"

#define dbg(args...) fprintf(stderr, args)
#undef dbg
#define dbg(args...)


typedef void (*signal_pipe_glib_handler_t)( int signal, void *user_data );

typedef struct _signal_pipe {
    int fds[2];
    GIOChannel *ioc;
    guint ios;

    signal_pipe_glib_handler_t userfunc;
    void *userdata;
} signal_pipe_t;

static signal_pipe_t g_sp;
static int g_sp_initialized = 0;

static int 
signal_pipe_init()
{
    if( g_sp_initialized ) {
        fprintf(stderr, "signal_pipe already initialized!!\n");
        return -1;
    }

    if( 0 != pipe( g_sp.fds ) ) {
        perror("signal_pipe");
        return -1;
    }

    int flags = fcntl( g_sp.fds[1], F_GETFL );
    fcntl( g_sp.fds[1], F_SETFL, flags | O_NONBLOCK );

    g_sp_initialized = 1;

    dbg("signal_pipe: initialized\n");
    return 0;
}

static int 
signal_pipe_cleanup()
{
    if( g_sp_initialized ) {
        close( g_sp.fds[0] );
        close( g_sp.fds[1] );
        g_io_channel_unref( g_sp.ioc );
        g_sp_initialized = 0;
        return 0;
    }

    dbg("signal_pipe: destroyed\n");
    return -1;
}

static void
signal_handler( int signal )
{
    dbg("signal_pipe: caught signal %d\n", signal);
    if(write(g_sp.fds[1], &signal, sizeof(int)) <= 0) {
        perror("signal-notify-write");
    }
}

static int
signal_handler_glib( GIOChannel *source, GIOCondition condition, void *ud )
{
    int signal;
    int status;
    status = read( g_sp.fds[0], &signal, sizeof(int) );

    if( status != sizeof(int) ) {
        fprintf(stderr, "wtf!? signal_handler_glib is confused (%s:%d)\n", 
                __FILE__, __LINE__ );
        return TRUE;
    }

    if( g_sp.userfunc ) {
        g_sp.userfunc( signal, g_sp.userdata );
    }

    return TRUE;
}

static void 
signal_pipe_add_signal( int sig )
{
    // TODO use sigaction instead of signal()
#if 0
    struct sigaction siga;
    siga.sa_handler = signal_handler;
    siga.sa_sigaction = NULL;
    sigemptyset( &siga.sa_mask );
    siga.sa_flags = 0;
    siga.sa_restorer = 0;

    int status = sigaction( sig, &siga, NULL );
    if( 0 != status ) {
        perror("signal_pipe: sigaction failed");
    }
#else
    signal( sig, signal_handler );
#endif

    return;
}

static int 
signal_pipe_attach_glib( signal_pipe_glib_handler_t func, gpointer user_data )
{
    if( ! g_sp_initialized ) return -1;

    if( g_sp.ioc ) return -1;

    g_sp.ioc = g_io_channel_unix_new( g_sp.fds[0] );
    g_io_channel_set_flags( g_sp.ioc, 
            g_io_channel_get_flags( g_sp.ioc ) | G_IO_FLAG_NONBLOCK, NULL );
    g_sp.ios = g_io_add_watch( g_sp.ioc, G_IO_IN | G_IO_PRI, 
            (GIOFunc) signal_handler_glib, NULL );

    g_sp.userfunc = func;
    g_sp.userdata = user_data;

    return 0;
}


static void
gqoi_handler( int signal, void *user )
{
    gtk_main_quit( );
    signal_pipe_cleanup();
}

int bot_gtk_quit_on_interrupt()
{
    if( 0 != signal_pipe_init() ) return -1;

    signal_pipe_add_signal( SIGINT );
    signal_pipe_add_signal( SIGTERM );
    signal_pipe_add_signal( SIGHUP );

    return signal_pipe_attach_glib( gqoi_handler, NULL );
}
