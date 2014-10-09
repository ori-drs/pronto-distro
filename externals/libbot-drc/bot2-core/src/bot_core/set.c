#include <stdio.h>
#include <stdlib.h>

#include "set.h"

struct _BotSet {
    GHashTable *hash_table;
    GHashFunc hash_func;
    GEqualFunc equal_func;
    GDestroyNotify element_destroy_func;
};

BotSet * 
bot_set_new (GHashFunc hash_func, GEqualFunc equal_func)
{
    return bot_set_new_full (hash_func, equal_func, NULL);
}

BotSet * 
bot_set_new_full (GHashFunc hash_func, GEqualFunc equal_func,
        GDestroyNotify element_destroy_func)
{
    BotSet *set = g_slice_new (BotSet);
    set->hash_table = g_hash_table_new_full (hash_func, equal_func,
            element_destroy_func, NULL);
    set->hash_func = hash_func;
    set->equal_func = equal_func;
    set->element_destroy_func = element_destroy_func;
    return set;
}

static void 
_new_union_add (gpointer key, gpointer value, void *user_data)
{
    BotSet *set = user_data;
    bot_set_add (set, key);
}

BotSet *
bot_set_new_union (const BotSet *set1, const BotSet *set2)
{
    if (set1->hash_func != set2->hash_func) {
        g_error ("Can't union BotSet objects with different hash functions");
        return NULL;
    }
    if (set1->equal_func != set2->equal_func) {
        g_error ("Can't union BotSet objects with different equal functions");
        return NULL;
    }
    if (set1->element_destroy_func != set2->element_destroy_func) {
        g_error ("Can't union BotSet objects with different element destroy functions");
        return NULL;
    }
    BotSet *result = bot_set_new_full (set1->hash_func, set1->equal_func,
            set1->element_destroy_func);
    g_hash_table_foreach (set1->hash_table, _new_union_add, result);
    g_hash_table_foreach (set2->hash_table, _new_union_add, result);
    return result;
}

struct _set_intersect_data {
    const BotSet *other_set;
    BotSet *new_set;
};

static void
_new_intersect_add (gpointer key, gpointer value, void *user_data)
{
    struct _set_intersect_data *d = user_data;
    if (bot_set_contains (d->other_set, key)) bot_set_add (d->new_set, key);
}

BotSet *
bot_set_new_intersection (const BotSet *set1, const BotSet *set2)
{
    if (set1->hash_func != set2->hash_func) {
        g_error ("Can't instersect BotSet objects with different hash functions");
        return NULL;
    }
    if (set1->equal_func != set2->equal_func) {
        g_error ("Can't instersect BotSet objects with different equal functions");
        return NULL;
    }
    if (set1->element_destroy_func != set2->element_destroy_func) {
        g_error ("Can't instersect BotSet objects with different element destroy functions");
        return NULL;
    }
    BotSet *result = bot_set_new_full (set1->hash_func, set1->equal_func,
            set1->element_destroy_func);
    if (bot_set_size (set1) < bot_set_size (set2)) {
        struct _set_intersect_data d = { set2, result };
        g_hash_table_foreach (set1->hash_table, _new_intersect_add, &d);
    } else {
        struct _set_intersect_data d = { set1, result };
        g_hash_table_foreach (set2->hash_table, _new_intersect_add, &d);
    }
    return result;
}

static void
_new_copy_add (gpointer key, gpointer element, gpointer user_data)
{
    g_hash_table_insert ((GHashTable*) user_data, key, element);
}

BotSet * 
bot_set_new_copy (const BotSet *set)
{
    BotSet *result = bot_set_new_full (set->hash_func, set->equal_func,
            set->element_destroy_func);
    g_hash_table_foreach ((GHashTable*)set->hash_table, _new_copy_add, 
            result->hash_table);
    return result;
}

static void
_maybe_remove_from_set1 (gpointer element, gpointer user_data)
{
    BotSet *set1 = (BotSet*) user_data;
    if (bot_set_contains (set1, element))
        bot_set_remove (set1, element);
}

void
bot_set_subtract (BotSet *set1, const BotSet *set2)
{
    bot_set_foreach ((BotSet*) set2, _maybe_remove_from_set1, set1);
}

void 
bot_set_destroy (BotSet *set)
{
    g_hash_table_destroy (set->hash_table);
    g_slice_free (BotSet, set);
}

void 
bot_set_add (BotSet *set, gpointer element)
{
    g_hash_table_insert (set->hash_table, element, element);
}

void 
bot_set_add_list (BotSet *set, GList *list)
{
    for (GList *iter=list; iter; iter=iter->next)
        bot_set_add (set, iter->data);
}

void 
bot_set_remove (BotSet *set, gpointer element)
{
    g_hash_table_remove (set->hash_table, element);
}

void 
bot_set_remove_all (BotSet *set)
{
    g_hash_table_remove_all (set->hash_table);
}

int
bot_set_size (const BotSet *set)
{
    return g_hash_table_size (set->hash_table);
}

gboolean 
bot_set_contains (const BotSet *set, gpointer element)
{
    return g_hash_table_lookup (set->hash_table, element) == element;
}

struct _foreach_data {
    BotSetForeachFunc func;
    gpointer user_data;
};

static void
_foreach_func (gpointer key, gpointer value, gpointer user_data)
{
    struct _foreach_data *d = user_data;
    d->func (key, d->user_data);
}

void 
bot_set_foreach (BotSet *set, BotSetForeachFunc func, gpointer user_data)
{
    struct _foreach_data d = { func, user_data };
    g_hash_table_foreach (set->hash_table, _foreach_func, &d);
}

static void
_get_elements_foreach (gpointer key, gpointer value, gpointer user_data)
{
    g_ptr_array_add ((GPtrArray*)user_data, key);
}

GPtrArray *
bot_set_get_elements (BotSet *set)
{
    GPtrArray *result = 
        g_ptr_array_sized_new (g_hash_table_size (set->hash_table));
    g_hash_table_foreach (set->hash_table, _get_elements_foreach, result);
    return result;
}
