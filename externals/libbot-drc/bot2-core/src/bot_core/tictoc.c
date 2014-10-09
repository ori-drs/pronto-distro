/*
 * tictoc.c
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <glib.h>
#include <sys/time.h>

#include "tictoc.h"

//simple, quick and dirty profiling tool...

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


typedef struct
{
    int64_t t;
    int64_t totalT;
    int64_t ema;
    int64_t min;
    int64_t max;
    int numCalls;
    char flag;
    const char * description;
} _tictoc_t;

static void
_tictoc_t_print(gpointer data, gpointer user_data)
{
    _tictoc_t *tt = (_tictoc_t *) data;
    if (tt->numCalls < 1)
        return;
    double totalT = (double) tt->totalT / 1.0e6;
    double avgT = ((double) tt->totalT / (double) tt->numCalls) / 1.0e6;
    double minT = (double) tt->min / 1.0e6;
    double maxT = (double) tt->max / 1.0e6;
    double emaT = (double) tt->ema / 1.0e6;
    printf(
            "%30s: numCalls = %11d   totalT=%10.2f   avgT=%8.4f   minT=%8.4f   maxT=%8.4f   emaT=%8.4f\n",
            tt->description, tt->numCalls, totalT, avgT, minT, maxT, emaT);

}

static gint
_tictoc_t_avgTimeCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return (t1->totalT / t1->numCalls) < (t2->totalT / t2->numCalls);
}
static gint
_tictoc_t_totalTimeCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->totalT < t2->totalT;
}
static gint
_tictoc_t_maxTimeCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->max < t2->max;
}
static gint
_tictoc_t_minTimeCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->min > t2->min;
}
static gint
_tictoc_t_emaTimeCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->ema < t2->ema;
}

static gint
_tictoc_t_alphCompare(gconstpointer a, gconstpointer b)
{
    _tictoc_t *t1 = (_tictoc_t *) a;
    _tictoc_t *t2 = (_tictoc_t *) b;
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return strcmp(t1->description, t2->description);
}

GStaticMutex tictoc_mutex =
G_STATIC_MUTEX_INIT;

static int _tictoc_enabled = 1;
static int _tictoc_initialized = 0;
static GHashTable* _tictoc_table;

static void
_initializeTictoc()
{
    const char *tmp;
    tmp = getenv(BOT_TICTOC_ENV);
    if (tmp != NULL) {
        _tictoc_enabled = 1;
        _tictoc_table = g_hash_table_new(g_str_hash, g_str_equal);
    } else {
        _tictoc_enabled = 0;
    }
}

int64_t
bot_tictoc(const char *description)
{
    return bot_tictoc_full(description, .01, NULL);
}

int64_t
bot_tictoc_full(const char *description, double ema_alpha, int64_t * ema)
{
    if (!_tictoc_enabled)
        return 0;

    int64_t ret = 0;

    g_static_mutex_lock(&tictoc_mutex); //aquire the lock
    if (!_tictoc_initialized) {
        _tictoc_initialized = 1;
        _initializeTictoc();
        if (!_tictoc_enabled) {
            g_static_mutex_unlock(&tictoc_mutex); //release
            return 0;
        }

    }

    int64_t tictoctime = _timestamp_now();
    _tictoc_t * entry = (_tictoc_t *) g_hash_table_lookup(_tictoc_table,
            description);
    if (entry == NULL) {
        //first time around, allocate and set the timer goin...
        entry = (_tictoc_t *) malloc(sizeof(_tictoc_t));
        entry->flag = 1;
        entry->t = tictoctime;
        entry->totalT = 0;
        entry->numCalls = 0;
        entry->max = -1e15;
        entry->min = 1e15;
        entry->ema = 0;
        entry->description = strdup(description);
        g_hash_table_insert(_tictoc_table, (gpointer) description,
                (gpointer) entry);
        ret = tictoctime;
    } else if (entry->flag == 0) {
        entry->flag = 1;
        entry->t = tictoctime;
        ret = tictoctime;
    } else {
        entry->flag = 0;
        int64_t dt = tictoctime - entry->t;
        entry->numCalls++;
        entry->totalT += dt;
        if (dt < entry->min)
            entry->min = dt;
        if (dt > entry->max)
            entry->max = dt;
        entry->ema = (1.0 - ema_alpha) * entry->ema + ema_alpha * dt;
        if (ema != NULL)
            *ema = entry->ema;
        ret = dt;
    }

    g_static_mutex_unlock(&tictoc_mutex); //release
    return ret;
}

static void
_get_all_vals_helper (gpointer key, gpointer value, gpointer user_data)
{
    GList ** vals = (GList **) user_data;
    *vals = g_list_prepend (*vals, value);
}


void
bot_tictoc_print_stats(bot_tictoc_sort_type_t sortType)
{
    if (!_tictoc_enabled) {
        return;
    }
    g_static_mutex_lock(&tictoc_mutex); //acquire lock for table
    if (!_tictoc_initialized) {
        g_static_mutex_unlock(&tictoc_mutex); //release
        return;
    }
    GList * list = NULL;
    g_hash_table_foreach (_tictoc_table, _get_all_vals_helper, &list);
    printf("\n--------------------------------------------\n");
    printf("tictoc Statistics, sorted by ");
    switch (sortType)
        {
    case BOT_TICTOC_AVG:
        printf("average time\n");
        list = g_list_sort(list, _tictoc_t_avgTimeCompare);
        break;
    case BOT_TICTOC_MIN:
        printf("min time\n");
        list = g_list_sort(list, _tictoc_t_minTimeCompare);
        break;
    case BOT_TICTOC_MAX:
        printf("max time\n");
        list = g_list_sort(list, _tictoc_t_maxTimeCompare);
        break;
    case BOT_TICTOC_EMA:
        printf("EMA time\n");
        list = g_list_sort(list, _tictoc_t_emaTimeCompare);
        break;
    case BOT_TICTOC_TOTAL:
        printf("total time\n");
        list = g_list_sort(list, _tictoc_t_totalTimeCompare);
        break;
    case BOT_TICTOC_ALPHABETICAL:
        printf("alphabetically\n");
        list = g_list_sort(list, _tictoc_t_alphCompare);
        break;
    default:
        fprintf(stderr, "WARNING: invalid sort type in tictoc, using AVG\n");
        list = g_list_sort(list, _tictoc_t_avgTimeCompare);
        break;
        }
    printf("--------------------------------------------\n");
    g_list_foreach(list, _tictoc_t_print, NULL);
    printf("--------------------------------------------\n");
    g_list_free(list);

    g_static_mutex_unlock(&tictoc_mutex); //release
}
