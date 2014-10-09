/*
 * process management core code
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>

#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

#include <glib.h>

#include <libgen.h>

#include "procman.h"
#include "procinfo.h"

static void dbgt (const char *fmt, ...)
{
    va_list ap;
    va_start (ap, fmt);

    char timebuf[80];
    struct timeval now_tv;
    gettimeofday(&now_tv, NULL);
    struct tm now_tm;
    localtime_r(&now_tv.tv_sec, &now_tm);
    int pos = strftime(timebuf, sizeof(timebuf), "%FT%T", &now_tm);
    pos += snprintf(timebuf + pos, sizeof(timebuf)-pos, ".%03d", (int)(now_tv.tv_usec / 1000));
    strftime(timebuf + pos, sizeof(timebuf)-pos, "%z", &now_tm);

    char buf[4096];
    vsnprintf (buf, sizeof(buf), fmt, ap);

    va_end (ap);

    fprintf (stderr, "%s %s", timebuf, buf);
}

static procman_cmd_t * procman_cmd_create (const char *cmd, const char* cmd_name, int32_t cmd_id);
static void procman_cmd_destroy (procman_cmd_t *cmd);
static void procman_cmd_split_str (procman_cmd_t *pcmd, GHashTable* variables);

struct _procman {
    procman_params_t params;
    GList *commands;
    GHashTable* variables;
};

void procman_params_init_defaults (procman_params_t *params, int argc,
       char **argv)
{
    memset (params, 0, sizeof (procman_params_t));

    // infer the path of procman.  This will be used with execv to start the
    // child processes, as it's assumed that child executables reside in same
    // directory as procman (or specified as a relative path or absolute path)
    if (argc <= 0) {
        fprintf (stderr, "procman: INVALID argc (%d)\n", argc);
        abort();
    }

    char * dirpath, * argv0;

    argv0 = strdup (argv[0]);
    dirpath = dirname (argv0);
    snprintf ( params->bin_path, sizeof (params->bin_path), "%s/", dirpath);
    free (argv0);
}

procman_t *procman_create (const procman_params_t *params)
{
    procman_t *pm = (procman_t*)calloc(1, sizeof (procman_t));
    if (NULL == pm) return NULL;

    memcpy (&pm->params, params, sizeof (procman_params_t));

    pm->variables = g_hash_table_new_full(g_str_hash, g_str_equal,
            (GDestroyNotify)g_free, (GDestroyNotify)g_free);

    // add the bin path to the PATH environment variable
    //
    // TODO check and see if it's already there
    char *path = getenv ("PATH");
    int newpathlen = strlen (path) + strlen(params->bin_path) + 2;
    char *newpath = calloc(1, newpathlen);
    sprintf (newpath, "%s:%s", params->bin_path, path);
    printf ("setting PATH to %s\n", newpath);
    setenv ("PATH", newpath, 1);
    free (newpath);

    return pm;
}

void
procman_destroy (procman_t *pm)
{
    GList *iter;
    for (iter = pm->commands; iter != NULL; iter = iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        procman_cmd_destroy (p);
    }
    g_list_free (pm->commands);
    pm->commands = NULL;

    g_hash_table_destroy(pm->variables);

    free (pm);
}

int procman_start_cmd (procman_t *pm, procman_cmd_t *p)
{
    int status;

    if (0 != p->pid) {
        dbgt ("[%s] has non-zero PID.  not starting again\n", p->cmd_name);
        return -1;
    } else {
        dbgt ("[%s] starting\n", p->cmd_name);

        procman_cmd_split_str(p, pm->variables);

        // close existing fd's
        if (p->stdout_fd >= 0) {
            close (p->stdout_fd);
            p->stdout_fd = -1;
        }
        p->stdin_fd = -1;
        p->exit_status = 0;

        // make a backup of stderr, in case something bad happens during exec.
        // if exec succeeds, then we have a dangling file descriptor that
        // gets closed when the child exits... that's okay
        int stderr_backup = dup(STDERR_FILENO);

        int pid = forkpty(&p->stdin_fd, NULL, NULL, NULL);
        if (0 == pid) {
//            // block SIGINT (only allow the procman to kill the process now)
//            sigset_t toblock;
//            sigemptyset (&toblock);
//            sigaddset (&toblock, SIGINT);
//            sigprocmask (SIG_BLOCK, &toblock, NULL);

            // set environment variables from the beginning of the command
            for (int i=0;i<p->envc;i++){
                setenv(p->envp[i][0],p->envp[i][1],1);
            }

            // go!
            execvp (p->argv[0], p->argv);

            char ebuf[1024];
            snprintf (ebuf, sizeof(ebuf), "%s", strerror(errno));
            dbgt("[%s] ERRROR executing [%s]\n", p->cmd_name, p->cmd->str);
            dbgt("[%s] execv: %s\n", p->cmd_name, ebuf);

            // if execv returns, the command did not execute successfully
            // (e.g. permission denied or bad path or something)

            // restore stderr so we can barf a real error message
            close(STDERR_FILENO);
            dup2(stderr_backup, STDERR_FILENO);
            dbgt("[%s] ERROR executing [%s]\n", p->cmd_name, p->cmd->str);
            dbgt("[%s] execv: %s\n", p->cmd_name, ebuf);
            close(stderr_backup);

            exit(-1);
        } else if (pid < 0) {
            perror("forkpty");
            close(stderr_backup);
            return -1;
        } else {
            p->pid = pid;
            p->stdout_fd = p->stdin_fd;
            close(stderr_backup);
        }
    }
    return 0;
}

int procman_start_all_cmds (procman_t *pm)
{
    GList *iter;
    int status;
    for (iter = pm->commands; iter != NULL; iter = iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        if (0 == p->pid) {
            status = procman_start_cmd (pm, p);
            if (0 != status) {
                // if the command couldn't be started, then abort everything
                // and return
                procman_stop_all_cmds (pm);

                return status;
            }
        }
    }
    return 0;
}

int
procman_kill_cmd (procman_t *pm, procman_cmd_t *p, int signum)
{
    if (0 == p->pid) {
        dbgt ("[%s] has no PID.  not stopping (already dead)\n", p->cmd_name);
        return -EINVAL;
    }
    // get a list of the process's descendants
    GArray* descendants = procinfo_get_descendants(p->pid);

    dbgt ("[%s] stop (signal %d)\n", p->cmd_name, signum);
    if (0 != kill (p->pid, signum)) {
        g_array_free(descendants, TRUE);
        return -errno;
    }

    // send the same signal to all of the process's descendants
    for(int i=0; i<descendants->len; i++) {
        int child_pid = g_array_index(descendants, int, i);
        dbgt("signal %d to descendant %d (%p)\n", signum, child_pid, p);
        kill(child_pid, signum);

        int new_descendant = 1;
        for(int j=0; j<p->descendants_to_kill->len; j++) {
            if(g_array_index(p->descendants_to_kill, int, j) == child_pid) {
                new_descendant = 0;
                break;
            }
        }
        if(new_descendant)
            g_array_append_val(p->descendants_to_kill, child_pid);
    }
    return 0;
}

int procman_stop_cmd (procman_t *pm, procman_cmd_t *p)
{
    return procman_kill_cmd (pm, p, SIGINT);
}

int procman_stop_all_cmds (procman_t *pm)
{
    GList *iter;
    int ret = 0;
    int status;

    // loop through each managed process and try to stop it
    for (iter = pm->commands; iter != NULL; iter = iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        status = procman_stop_cmd (pm, p);

        if (0 != status) {
            ret = status;
            // If something bad happened, try to stop the other processes, but
            // still return an error
        }
    }
    return ret;
}

int
procman_check_for_dead_children (procman_t *pm, procman_cmd_t **dead_child)
{
    int status;

    // check for dead children
    *dead_child = NULL;
    int pid = waitpid (-1, &status, WNOHANG);
    if(pid <= 0)
        return 0;

    GList *iter;
    for (iter = pm->commands; iter != NULL; iter = iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        if(p->pid == 0 || pid != p->pid)
            continue;
        *dead_child = p;
        p->pid = 0;
        p->exit_status = status;

        if (WIFSIGNALED (status)) {
            int signum = WTERMSIG (status);
            dbgt ("[%s] terminated by signal %d (%s)\n",
                    p->cmd_name, signum, strsignal (signum));
        } else if (status != 0) {
            dbgt ("[%s] exited with status %d\n",
                    p->cmd_name, WEXITSTATUS (status));
        } else {
          dbgt ("[%s] exited\n", p->cmd_name);
        }

        // check for and kill orphaned children.
        for(int orphan_index=0; orphan_index<p->descendants_to_kill->len; orphan_index++) {
            int child_pid = g_array_index(p->descendants_to_kill, int, orphan_index);
            if(procinfo_is_orphaned_child_of(child_pid, pid)) {
                dbgt("sending SIGKILL to orphan process %d\n", child_pid);
                kill(child_pid, SIGKILL);
            }
        }

        return status;
    }

    dbgt ("reaped [%d] but couldn't find process\n", pid);
    return 0;
}

int
procman_close_dead_pipes (procman_t *pm, procman_cmd_t *cmd)
{
    if (cmd->stdout_fd < 0 && cmd->stdin_fd < 0)
        return 0;

    if (cmd->pid) {
        dbgt ("refusing to close pipes for command "
                "with nonzero pid [%s] [%d]\n",
                cmd->cmd_name, cmd->pid);
        return 0;
    }
    if (cmd->stdout_fd >= 0) {
        close (cmd->stdout_fd);
    }
    cmd->stdin_fd = -1;
    cmd->stdout_fd = -1;
    return 0;
}

/**
 * same as g_strsplit_set, but removes empty tokens
 */
static char **
strsplit_set_packed(const char *tosplit, const char *delimiters, int max_tokens)
{
    char **tmp = g_strsplit_set(tosplit, delimiters, max_tokens);
    int i;
    int n=0;
    for(i=0; tmp[i]; i++) {
        if(strlen(tmp[i])) n++;
    }
    char **result = calloc(n+1, sizeof(char*));
    int c=0;
    for(i=0; tmp[i]; i++) {
        if(strlen(tmp[i])) {
            result[c] = g_strdup(tmp[i]);
            c++;
        }
    }
    g_strfreev(tmp);
    return result;
}

typedef struct {
    const char* w;
    int w_len;
    int pos;
    char cur_tok;
    GString* result;
    GHashTable* variables;
} subst_parse_context_t;

static int
is_valid_variable_char(char c, int pos)
{
    const char* valid_start = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_";
    const char* valid_follow = "1234567890";
    return (strchr(valid_start, c) != NULL ||
            ((0 == pos) && (strchr(valid_follow, c) != NULL)));
}

static char
subst_vars_has_token(subst_parse_context_t* ctx)
{
    return (ctx->pos < ctx->w_len);
}

static char
subst_vars_peek_token(subst_parse_context_t* ctx)
{
    return subst_vars_has_token(ctx) ? ctx->w[ctx->pos] : 0;
}

static int
subst_vars_eat_token(subst_parse_context_t* ctx)
{
    if(subst_vars_has_token(ctx)) {
        ctx->cur_tok = ctx->w[ctx->pos];
        ctx->pos++;
        return TRUE;
    } else {
        ctx->cur_tok = 0;
        return FALSE;
    }
}

static int
subst_vars_parse_variable(subst_parse_context_t* ctx)
{
    int start = ctx->pos;
    if(!subst_vars_has_token(ctx)) {
        g_string_append_c(ctx->result, '$');
        return 0;
    }
    int has_braces = subst_vars_peek_token(ctx) == '{';
    if(has_braces)
        subst_vars_eat_token(ctx);
    int varname_start = ctx->pos;
    int varname_len = 0;
    while(subst_vars_has_token(ctx) &&
          is_valid_variable_char(subst_vars_peek_token(ctx), varname_len)) {
        varname_len++;
        subst_vars_eat_token(ctx);
    }
    char* varname = g_strndup(&ctx->w[varname_start], varname_len);
    int braces_ok = TRUE;
    if(has_braces && ((!subst_vars_eat_token(ctx)) || ctx->cur_tok != '}'))
        braces_ok = FALSE;
    int ok = varname_len && braces_ok;
    if(ok) {
        // first lookup the variable in our stored table
        char* val = g_hash_table_lookup(ctx->variables, varname);
        // if that fails, then check for a similar environment variable
        if(!val)
            val = getenv(varname);
        if(val)
            g_string_append(ctx->result, val);
        else
            ok = FALSE;
    }
    if(!ok)
        g_string_append_len(ctx->result, &ctx->w[start - 1], ctx->pos - start + 1);
    g_free(varname);
    return ok;
}

/**
 * Do variable expansion on a command argument.  This searches the argument for
 * text of the form $VARNAME and ${VARNAME}.  For each discovered variable, it
 * then expands the variable.  Values defined in the hashtable vars are used
 * first, followed by environment variable values.  If a variable expansion
 * fails, then the corresponding text is left unchanged.
 */
static char*
subst_vars(const char* w, GHashTable* vars)
{
    subst_parse_context_t ctx;
    ctx.w = w;
    ctx.w_len = strlen(w);
    ctx.pos = 0;
    ctx.result = g_string_sized_new(ctx.w_len * 2);
    ctx.variables = vars;

    while(subst_vars_eat_token(&ctx)) {
        char c = ctx.cur_tok;
        if('\\' == c) {
            if(subst_vars_eat_token(&ctx)) {
                g_string_append_c(ctx.result, c);
            } else {
                g_string_append_c(ctx.result, '\\');
            }
            continue;
        }
        // variable?
        if('$' == c) {
            subst_vars_parse_variable(&ctx);
        } else {
            g_string_append_c(ctx.result, c);
        }
    }
    char* result = g_strdup(ctx.result->str);
    g_string_free(ctx.result, TRUE);
    return result;
}

static void
procman_cmd_split_str (procman_cmd_t *pcmd, GHashTable* variables)
{
    if (pcmd->argv) {
        g_strfreev (pcmd->argv);
        pcmd->argv = NULL;
    }
    if (pcmd->envp) {
        for(int i=0;i<pcmd->envc;i++)
            g_strfreev (pcmd->envp[i]);
        free(pcmd->envp);
        pcmd->envp = NULL;
    }

    // TODO don't use g_shell_parse_argv... it's not good with escape characters
    char ** argv=NULL;
    int argc = -1;
    GError *err = NULL;
    gboolean parsed = g_shell_parse_argv(pcmd->cmd->str, &argc,
            &argv, &err);

    if(!parsed || err) {
        // unable to parse the command string as a Bourne shell command.
        // Do the simple thing and split it on spaces.
        pcmd->envp = calloc(1, sizeof(char**));
        pcmd->envc = 0;
        pcmd->argv = strsplit_set_packed(pcmd->cmd->str, " \t\n", 0);
        for(pcmd->argc=0; pcmd->argv[pcmd->argc]; pcmd->argc++);
        g_error_free(err);
        return;
    }

    // extract environment variables
    int envCount=0;
    char * equalSigns[512];
    while((equalSigns[envCount]=strchr(argv[envCount],'=')))
        envCount++;
    pcmd->envc=envCount;
    pcmd->argc=argc-envCount;
    pcmd->envp = calloc(pcmd->envc+1,sizeof(char**));
    pcmd->argv = calloc(pcmd->argc+1,sizeof(char*));
    for (int i=0;i<argc;i++) {
        if (i<envCount)
            pcmd->envp[i]=g_strsplit(argv[i],"=",2);
        else {
            // substitute variables
            pcmd->argv[i-envCount]=subst_vars(argv[i], variables);
        }
    }
    g_strfreev(argv);
}

static procman_cmd_t *
procman_cmd_create (const char *cmd, const char* cmd_name, int32_t cmd_id)
{
    procman_cmd_t *pcmd = (procman_cmd_t*)calloc(1, sizeof (procman_cmd_t));
    pcmd->cmd = g_string_new ("");
    pcmd->cmd_id = cmd_id;
    pcmd->cmd_name = g_strdup(cmd_name);
    pcmd->stdout_fd = -1;
    pcmd->stdin_fd = -1;
    g_string_assign (pcmd->cmd, cmd);

    pcmd->argv = NULL;
    pcmd->argc = 0;
    pcmd->envp = NULL;
    pcmd->envc = 0;
    pcmd->descendants_to_kill = g_array_new(FALSE, FALSE, sizeof(int));

    return pcmd;
}

static void
procman_cmd_destroy (procman_cmd_t *cmd)
{
    g_string_free (cmd->cmd, TRUE);
    g_strfreev (cmd->argv);
    g_array_free(cmd->descendants_to_kill, TRUE);
    for(int i=0;i<cmd->envc;i++)
       g_strfreev (cmd->envp[i]);
    free(cmd->envp);
    g_free(cmd->cmd_name);
    free (cmd);
}

const GList *
procman_get_cmds (procman_t *pm) {
    return pm->commands;
}

void
procman_set_variable(procman_t* pm, const char* name, const char* val)
{
    g_hash_table_insert(pm->variables, g_strdup(name), g_strdup(val));
}

void
procman_remove_variable(procman_t* pm, const char* name)
{
    g_hash_table_remove(pm->variables, name);
}

void
procman_remove_all_variables(procman_t* pm)
{
    g_hash_table_remove_all(pm->variables);
}

procman_cmd_t*
procman_add_cmd (procman_t *pm, const char *cmd_str, const char* cmd_name)
{
    // pick a suitable ID
    int32_t cmd_id;

    // TODO make this more efficient (i.e. sort the existing cmd_ids)
    //      this implementation is O (n^2)
    for (cmd_id=1; cmd_id<INT_MAX; cmd_id++) {
        int collision = 0;
        GList *iter;
        for (iter=pm->commands; iter; iter=iter->next) {
            procman_cmd_t *cmd = (procman_cmd_t*)iter->data;
            if (cmd->cmd_id == cmd_id) {
                collision = 1;
                break;
            }
        }
        if (! collision) break;
    }
    if (cmd_id == INT_MAX) {
        dbgt ("way too many commands on the system....\n");
        return NULL;
    }

    procman_cmd_t *newcmd = procman_cmd_create (cmd_str, cmd_name, cmd_id);
    if (newcmd) {
        pm->commands = g_list_append (pm->commands, newcmd);
    }

    dbgt ("[%s] new command [%s]\n", newcmd->cmd_name, newcmd->cmd->str);
    return newcmd;
}

int
procman_remove_cmd (procman_t *pm, procman_cmd_t *cmd)
{
    // check that cmd is actually in the list
    GList *toremove = g_list_find (pm->commands, cmd);
    if (! toremove) {
        dbgt ("procman ERRROR: %s does not appear to be managed "
                "by this procman!!\n",
                cmd->cmd->str);
        return -1;
    }

    // stop the command (if it's running)
    if (cmd->pid) {
        dbgt ("procman ERROR: refusing to remove running command %s\n",
                cmd->cmd->str);
        return -1;
    }

    procman_close_dead_pipes (pm, cmd);

    // remove and free
    pm->commands = g_list_remove_link (pm->commands, toremove);
    g_list_free_1 (toremove);
    procman_cmd_destroy (cmd);
    return 0;
}

int32_t
procman_get_cmd_status (procman_t *pm, procman_cmd_t *cmd)
{
    if (cmd->pid > 0) return PROCMAN_CMD_RUNNING;
    if (cmd->pid == 0) return PROCMAN_CMD_STOPPED;

    return 0;
}

procman_cmd_t *
procman_find_cmd (procman_t *pm, const char *cmd_str)
{
    GList *iter;
    for (iter=pm->commands; iter; iter=iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        if (! strcmp (p->cmd->str, cmd_str)) return p;
    }
    return NULL;
}

procman_cmd_t *
procman_find_cmd_by_id (procman_t *pm, int32_t cmd_id)
{
    GList *iter;
    for (iter=pm->commands; iter; iter=iter->next) {
        procman_cmd_t *p = (procman_cmd_t*)iter->data;
        if (p->cmd_id == cmd_id) return p;
    }
    return NULL;
}

void
procman_cmd_change_str (procman_cmd_t *cmd, const char *cmd_str)
{
    g_string_assign (cmd->cmd, cmd_str);
}

void
procman_cmd_set_name(procman_cmd_t* cmd, const char* cmd_name)
{
    g_free(cmd->cmd_name);
    cmd->cmd_name = g_strdup(cmd_name);
}
