#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#include "signal_pipe.h"

#define dbg(args...) fprintf(stderr, args)
#undef dbg
#define dbg(args...)

typedef struct _bot_signal_pipe {
    int fds[2];
    GIOChannel *ioc;
    guint ios;

    bot_signal_pipe_glib_handler_t userfunc;
    void *userdata;
} bot_signal_pipe_t;

static bot_signal_pipe_t g_sp;
static int g_sp_initialized = 0;

int 
bot_signal_pipe_init()
{
    if (g_sp_initialized) {
        fprintf(stderr, "bot_signal_pipe already initialized!!\n");
        return -1;
    }

    if (0 != pipe (g_sp.fds)) {
        perror("bot_signal_pipe");
        return -1;
    }

    int flags = fcntl (g_sp.fds[1], F_GETFL);
    fcntl (g_sp.fds[1], F_SETFL, flags | O_NONBLOCK);

    g_sp_initialized = 1;

    dbg("bot_signal_pipe: initialized\n");
    return 0;
}

int 
bot_signal_pipe_cleanup()
{
    if (g_sp_initialized) {
        close (g_sp.fds[0]);
        close (g_sp.fds[1]);
        g_io_channel_unref (g_sp.ioc);
        g_sp_initialized = 0;
        return 0;
    }

    dbg("bot_signal_pipe: destroyed\n");
    return -1;
}

static void
bot_signal_handler (int signal)
{
    dbg("bot_signal_pipe: caught signal %d\n", signal);
    int ignored = write (g_sp.fds[1], &signal, sizeof(int));
}

static int
bot_signal_handler_glib (GIOChannel *source, GIOCondition condition, void *ud)
{
    int signal;
    int status;
    status = read (g_sp.fds[0], &signal, sizeof(int));

    if (status != sizeof(int)) {
        fprintf(stderr, "wtf!? bot_signal_handler_glib is confused (%s:%d)\n", 
                __FILE__, __LINE__);
        return TRUE;
    }

    if (g_sp.userfunc) {
        g_sp.userfunc (signal, g_sp.userdata);
    }

    return TRUE;
}

void 
bot_signal_pipe_add_signal (int sig)
{
    // TODO use sigaction instead of signal()
#if 0
    struct sigaction siga;
    siga.sa_handler = bot_signal_handler;
    siga.sa_sigaction = NULL;
    sigemptyset (&siga.sa_mask);
    siga.sa_flags = 0;
    siga.sa_restorer = 0;

    int status = sigaction (sig, &siga, NULL);
    if (0 != status) {
        perror("bot_signal_pipe: sigaction failed");
    }
#else
    signal (sig, bot_signal_handler);
#endif

    return;
}

int 
bot_signal_pipe_attach_glib (bot_signal_pipe_glib_handler_t func, gpointer user_data)
{
    if (! g_sp_initialized) return -1;

    if (g_sp.ioc) return -1;

    g_sp.ioc = g_io_channel_unix_new (g_sp.fds[0]);
    g_io_channel_set_flags (g_sp.ioc, 
            g_io_channel_get_flags (g_sp.ioc) | G_IO_FLAG_NONBLOCK, NULL);
    g_sp.ios = g_io_add_watch (g_sp.ioc, G_IO_IN | G_IO_PRI, 
            (GIOFunc) bot_signal_handler_glib, NULL);

    g_sp.userfunc = func;
    g_sp.userdata = user_data;

    return 0;
}


static void
spgqok_handler (int signal, void *user)
{
    GMainLoop *mainloop = (GMainLoop*) user;
    g_main_loop_quit (mainloop);
    bot_signal_pipe_cleanup();
}

int 
bot_signal_pipe_glib_quit_on_kill (GMainLoop *mainloop)
{
    if (0 != bot_signal_pipe_init()) return -1;

    bot_signal_pipe_add_signal (SIGINT);
    bot_signal_pipe_add_signal (SIGTERM);
    bot_signal_pipe_add_signal (SIGKILL);
    bot_signal_pipe_add_signal (SIGHUP);

    return bot_signal_pipe_attach_glib (spgqok_handler, mainloop);
}
