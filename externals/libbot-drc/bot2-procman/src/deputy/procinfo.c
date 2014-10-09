/*
 * code for reading detailed process information on a GNU/Llinux system
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <inttypes.h>
#include <assert.h>

#include <glib.h>

#include "procinfo.h"

static void strsplit (char *buf, char **words, int maxwords)
{
    int inword = 0;
    int i;
    int wordind = 0;
    for (i=0; buf[i] != 0; i++) {
        if (isspace (buf[i])) {
            inword = 0;
            buf[i] = 0;
        } else {
            if (! inword) {
                words[wordind] = buf + i;
                wordind++;
                if (wordind >= maxwords) break;
                inword = 1;
            }
        }
    }
    words[wordind] = NULL;
}

#ifdef __linux__
static int
procinfo_read_proc_cpu_mem_linux(int pid, proc_cpu_mem_t *s)
{
    memset (s, 0, sizeof (proc_cpu_mem_t));
    char fname[80];
    sprintf (fname, "/proc/%d/stat", pid);
    FILE *fp = fopen (fname, "r");
    if (! fp) { return -1; }

    char buf[4096];
    if(!fgets (buf, sizeof (buf), fp)) {
        return -1;
    }
    char *words[50];
    memset (words, 0, sizeof(words));
    strsplit (buf, words, 50);

    s->user = atoi (words[13]);
    s->system = atoi (words[14]);
    s->vsize = strtoll (words[22], NULL, 10);
    s->rss = strtoll (words[23], NULL, 10) * getpagesize();

    fclose (fp);

    sprintf (fname, "/proc/%d/statm", pid);
    fp = fopen (fname, "r");
    if (! fp) { return -1; }

    if(!fgets (buf, sizeof(buf), fp)) {
        return -1;
    }
    memset (words, 0, sizeof(words));
    strsplit (buf, words, 50);

    s->shared = atoi (words[2]) * getpagesize();
    s->text = atoi (words[3]) * getpagesize();
    s->data = atoi (words[5]) * getpagesize();

    fclose (fp);

    return 0;
}

static int
procinfo_read_sys_cpu_mem_linux(sys_cpu_mem_t *s)
{
    memset (s, 0, sizeof(sys_cpu_mem_t));
    FILE *fp = fopen ("/proc/stat", "r");
    if (! fp) { return -1; }

    char buf[4096];
    char tmp[80];

    while (! feof (fp)) {
        if(!fgets (buf, sizeof (buf), fp)) {
            if(feof(fp))
                break;
            else
                return -1;
        }

        if (! strncmp (buf, "cpu ", 4)) {
            sscanf (buf, "%s %u %u %u %u",
                    tmp,
                    &s->user,
                    &s->user_low,
                    &s->system,
                    &s->idle);
            break;
        }
    }
    fclose (fp);

    fp = fopen ("/proc/meminfo", "r");
    if (! fp) { return -1; }
    while (! feof (fp)) {
        char units[10];
        memset (units,0,sizeof(units));
        if(!fgets (buf, sizeof (buf), fp)) {
            if(feof(fp))
                break;
            else
                return -1;
        }

        if (! strncmp ("MemTotal:", buf, strlen ("MemTotal:"))) {
            sscanf (buf, "MemTotal: %"PRId64" %9s", &s->memtotal, units);
            s->memtotal *= 1024;
        } else if (! strncmp ("MemFree:", buf, strlen ("MemFree:"))) {
            sscanf (buf, "MemFree: %"PRId64" %9s", &s->memfree, units);
            s->memfree *= 1024;
        } else if (! strncmp ("SwapTotal:", buf, strlen("SwapTotal:"))) {
            sscanf (buf, "SwapTotal: %"PRId64" %9s", &s->swaptotal, units);
            s->swaptotal *= 1024;
        } else if (! strncmp ("SwapFree:", buf, strlen("SwapFree:"))) {
            sscanf (buf, "SwapFree: %"PRId64" %9s", &s->swapfree, units);
            s->swapfree *= 1024;
        } else {
            continue;
        }

        if (0 != strcmp (units, "kB")) {
            fprintf (stderr, "unknown units [%s] while reading "
                    "/proc/meminfo!!!\n", units);
        }
    }

    fclose (fp);

    return 0;
}

typedef struct _pid_info_t pid_info_t;
struct _pid_info_t
{
    int pid;
    int ppid;
    int pgrp;
    int session;
    char state;
    GPtrArray* children;
};

static void pid_info_destroy(pid_info_t* pinfo)
{
    g_ptr_array_free(pinfo->children, TRUE);
    memset(pinfo, 0, sizeof(pid_info_t));
    g_slice_free1(sizeof(pid_info_t), pinfo);
}
static pid_info_t* pid_info_new(int pid)
{
    pid_info_t* result = g_slice_new0(pid_info_t);
    result->pid = pid;
    result->children = g_ptr_array_new();
    char* fname = g_strdup_printf("/proc/%d/stat", pid);
    char* stat_contents = NULL;
    FILE* fp = fopen(fname, "r");
    if(!fp) {
        g_free(fname);
        pid_info_destroy(result);
        return NULL;
    }
    int read_pid;
    char exec_name[PATH_MAX + 1];
    int numwords = fscanf(fp, "%d %s %c %d %d %d", &read_pid, exec_name,
            &result->state, &result->ppid, &result->pgrp, &result->session);
    fclose(fp);
    g_free(fname);
    if(6 != numwords) {
        pid_info_destroy(result);
        return NULL;
    }
    return result;
}
static void pid_info_get_descendants(pid_info_t* pinfo, GArray* result)
{
    for(int i=0; i<pinfo->children->len; i++) {
        pid_info_t* child = (pid_info_t*)g_ptr_array_index(pinfo->children, i);
        assert(child->ppid == pinfo->pid);
        g_array_append_val(result, child->pid);
        pid_info_get_descendants(child, result);
    }
}

int
procinfo_is_orphaned_child_of(int orphan, int parent)
{
    pid_info_t* pinfo = pid_info_new(orphan);
    if(!pinfo)
        return 0;
    int result = (pinfo->ppid == 1 && pinfo->pgrp == parent && pinfo->session == parent);
    pid_info_destroy(pinfo);
    return result;
}

static GHashTable*
get_all_pids_and_ppids()
{
    GHashTable* result = g_hash_table_new_full(g_int_hash, g_int_equal, NULL,
            (GDestroyNotify)pid_info_destroy);
    GDir* dir = g_dir_open("/proc", 0, NULL);
    if(!dir)
        return result;
    for(const char* entry = g_dir_read_name(dir);
            entry;
            entry = g_dir_read_name(dir)) {
        int pid = atoi(entry);
        if(!pid)
            continue;
        pid_info_t* pinfo = pid_info_new(pid);
        if(!pinfo)
            continue;
        pid_info_t* parent = g_hash_table_lookup(result, &pinfo->ppid);
        if(parent) {
            g_ptr_array_add(parent->children, pinfo);
        }
        g_hash_table_replace(result, &pinfo->pid, pinfo);
    }
    g_dir_close(dir);
    return result;
}

GArray*
procinfo_get_descendants (int pid)
{
    GArray* result = g_array_new(FALSE, FALSE, sizeof(int));
    GHashTable* pid_graph = get_all_pids_and_ppids();
    pid_info_t* root = g_hash_table_lookup(pid_graph, &pid);
    if(root)
        pid_info_get_descendants(root, result);
    return result;
}
#else
GArray*
procinfo_get_descendants (int pid)
{
    return g_array_new(FALSE, FALSE, sizeof(int));
}
int
procinfo_is_orphaned_child_of(int orphan, int parent)
{
    return 0;
}
#endif

int
procinfo_read_proc_cpu_mem (int pid, proc_cpu_mem_t *s)
{
#ifdef __linux__
    return procinfo_read_proc_cpu_mem_linux(pid, s);
#else
    memset(s, 0, sizeof(proc_cpu_mem_t));
    return 0;
#endif
}

int
procinfo_read_sys_cpu_mem (sys_cpu_mem_t *s)
{
#ifdef __linux__
    return procinfo_read_sys_cpu_mem_linux(s);
#else
    memset(s, 0, sizeof(sys_cpu_mem_t));
    return 0;
#endif
}
