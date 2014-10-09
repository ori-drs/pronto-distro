#ifndef __bot_glib_util_h__
#define __bot_glib_util_h__

#include <stdint.h>

/**
 * SECTION:glib_util
 * @title:GLib Utilities
 * @short_description: useful functions missing from GLib
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 */

#include <glib.h>
//#include "gu_circular.h"
//#include "gu_ptr_circular.h"
//#include "gu_minheap.h"
//#include "gu_set.h"
//#include "gu_disjoint_set_forest.h"

#ifdef __cplusplus
extern "C" {
#endif

// calls func on every element of the specified list, and then
// frees the list
void bot_g_list_free_with_func (GList *list, GDestroyNotify func);
    
// calls func on every element of the specified queue, and then
// frees the queue
void bot_g_queue_free_with_func (GQueue *queue, GDestroyNotify func);

// calls func on every element of the specified pointer array, and then
// frees the pointer array.
void bot_g_ptr_array_free_with_func (GPtrArray *a, GDestroyNotify func);

// creates a newly allocated copy of a GPtrArray
GPtrArray * bot_g_ptr_array_new_copy (const GPtrArray *a);
    
/**
 * Returns: 1 if time1 is after time2
 *          0 if time1 and time2 are equal
 *         -1 if time1 is before time2
 */
int bot_g_time_val_compare (const GTimeVal *time1, const GTimeVal *time2);

GList * bot_g_hash_table_get_keys (GHashTable *hash_table);

GList * bot_g_hash_table_get_vals (GHashTable *hash_table);

GPtrArray * bot_g_hash_table_get_vals_array (GHashTable *hash_table);

#ifndef bot_g_ptr_array_size
#define bot_g_ptr_array_size(ptrarray) ((ptrarray)->len)
#endif

//
//#ifndef g_ptr_array_set
//#define g_ptr_array_set(ptrarray, idx, val) (ptrarray)->pdata[(idx)] = (val);
//#endif
//
int bot_g_ptr_array_find_index(GPtrArray *a, gconstpointer v);
//#define gu_ptr_array_find_index g_ptr_array_find_index
//
guint bot_pint64_hash(gconstpointer _key);
gboolean bot_pint64_equal(gconstpointer _a, gconstpointer _b);

#ifdef __cplusplus
}
#endif

#endif
