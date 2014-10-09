#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "small_linalg.h"
#include "rotations.h"
#include "glib_util.h"
#include "circular.h"
#include <glib.h>

#include "ctrans.h"

//#define dbg(...) fprintf(stderr, __VA_ARGS__)
#define dbg(...)

/**
 * BotCTransPath:
 *
 * Represents a sequence of rigid body transformations that relates two
 * cartesian coordinate frames that are not directly related.  If the
 * transformation between two coordinate frames is going to computed many
 * times, then it may be useful to create and use a BotCTransPath structure,
 * as it saves some computation.
 */
typedef struct _BotCTransPath BotCTransPath;

/**
 * bot_ctrans_path_destroy:
 * Releases memory used by a BotCTransPath
 */
void bot_ctrans_path_destroy(BotCTransPath * path);

/**
 * bot_ctrans_path_get_frame_from:
 * Returns: the source coordinate frame for the specified transform path
 */
const char* bot_ctrans_path_get_frame_from(BotCTransPath *path);

/**
 * bot_ctrans_path_get_frame_to:
 * Returns: the target coordinate frame for the specified transform path
 */
const char * bot_ctrans_path_get_frame_to(BotCTransPath *path);
        
/**
 * bot_ctrans_get_new_path:
 *
 * Computes a sequence of transformations that relates one coordinate frame to
 * another.
 *
 * Returns: A newly allocated BotCTransPath, which must be freed with
 * bot_ctrans_path_destroy (it is not automatically freed by 
 * bot_ctrans_destroy).  Returns NULL if no such path exists.
 */
BotCTransPath * bot_ctrans_get_new_path(BotCTrans * ctrans,
        const char *from_frame_id,
        const char *to_frame_id);

/**
 * bot_ctrans_path_to_trans:
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_ctrans_path_to_trans(const BotCTransPath * path,
        int64_t utime, BotTrans *result);

/**
 * bot_ctrans_path_to_trans_latest:
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_ctrans_path_to_trans_latest(const BotCTransPath * path,
        BotTrans *result);

/**
 * bot_ctrans_path_latest_timestamp:
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_ctrans_path_latest_timestamp(const BotCTransPath * path,
        int64_t *timestamp);

/**
 * bot_ctrans_path_have_trans:
 *
 * Returns: 1 if a transformation is available for this path, 0 if not.
 */
int bot_ctrans_path_have_trans(const BotCTransPath *path);

typedef struct _BotCTransFrame BotCTransFrame;
struct _BotCTransFrame
{
    char *id;
    GPtrArray * links;
};

typedef struct {
    int64_t utime;
    BotTrans trans;
} TimestampedTrans;

struct _BotCTransLink
{
    BotCTransFrame *frame_from;
    BotCTransFrame *frame_to;
    char * id;
    int history_maxlen;

    BotTrans static_trans;
    BotCircular * trans_history;
};

// ============ frame ==========

const char *bot_ctrans_frame_get_id(const BotCTransFrame *frame);

static BotCTransFrame *
_frame_new(const char *id)
{
    BotCTransFrame *frame = g_slice_new(BotCTransFrame);
    frame->id = strdup(id);
    frame->links = g_ptr_array_new();
    return frame;
}

static void
_frame_add_link(BotCTransFrame *frame, BotCTransLink *link)
{
    assert(frame == link->frame_from || frame == link->frame_to);
    for(int i=0, n=bot_g_ptr_array_size(frame->links); i<n; i++) {
        assert(link != g_ptr_array_index(frame->links, i));
    }
    g_ptr_array_add(frame->links, link);
}

static inline void
_frame_remove_link(BotCTransFrame *frame, BotCTransLink *link)
{
    assert(frame == link->frame_from || frame == link->frame_to);
    for(int i=0, n=bot_g_ptr_array_size(frame->links); i<n; i++) {
        if(link == g_ptr_array_index(frame->links, i)) {
            g_ptr_array_remove_index_fast(frame->links, i);
            return;
        }
    }
    assert(FALSE);
}

static void
_frame_destroy(BotCTransFrame * frame)
{
    free(frame->id);
    g_ptr_array_free(frame->links, TRUE);
    g_slice_free(BotCTransFrame, frame);
}

const char *
bot_ctrans_frame_get_id(const BotCTransFrame *frame)
{
    return frame->id;
}

// =========== link ==========

static inline char * 
_make_link_id(const char * frame_a_id, const char * frame_b_id)
{
    int v = strcmp(frame_a_id, frame_b_id);
    if(v > 0) {
        return g_strdup_printf("%s-%s", frame_a_id, frame_b_id);
    } else {
        return g_strdup_printf("%s-%s", frame_b_id, frame_a_id);
    }
}

static inline void
_make_link_id2(const char * frame_a_id, const char * frame_b_id, 
        char *result, int buf_sz)
{
    int v = strcmp(frame_a_id, frame_b_id);
    if(v > 0) {
        snprintf(result, buf_sz, "%s-%s", frame_a_id, frame_b_id);
    } else {
        snprintf(result, buf_sz, "%s-%s", frame_b_id, frame_a_id);
    }
}

static BotCTransLink *
_link_new(BotCTransFrame *frame_from, BotCTransFrame *frame_to,
        int history_maxlen)
{
    BotCTransLink *link = g_slice_new(BotCTransLink);
    link->frame_from = frame_from;
    link->frame_to = frame_to;
    link->id = _make_link_id(frame_from->id, frame_to->id);
    link->history_maxlen = history_maxlen;
    
    link->trans_history = bot_circular_new(history_maxlen, 
            sizeof(TimestampedTrans));
    return link;
};

static void
_link_destroy(BotCTransLink *link)
{
    bot_circular_free(link->trans_history);
    free(link->id);
    g_slice_free(BotCTransLink, link);
};

const char * 
bot_ctrans_link_get_from_frame(BotCTransLink *link)
{
    return link->frame_from->id;
}

const char * 
bot_ctrans_link_get_to_frame(BotCTransLink *link)
{
    return link->frame_to->id;
}

void 
bot_ctrans_link_update(BotCTransLink * link, const BotTrans *transformation,
        int64_t utime)
{
    TimestampedTrans ttrans;
    ttrans.utime = utime;
    memcpy(&ttrans.trans, transformation, sizeof(BotTrans));

    // if we've gone back in time, then clear the transformation history
    TimestampedTrans *last_ttrans = 
        bot_circular_peek_nth(link->trans_history, 0);
    if(utime < last_ttrans->utime) {
        bot_circular_clear(link->trans_history);
    } else if(utime == last_ttrans->utime) {
        bot_circular_pop_head(link->trans_history, NULL);
    }

    bot_circular_push_head(link->trans_history, &ttrans);
}

static gboolean
_link_have_trans(const BotCTransLink *link)
{
    return ! bot_circular_is_empty(link->trans_history);
}

static gboolean
_link_get_trans_latest(const BotCTransLink *link, BotTrans *trans)
{
    if(bot_circular_is_empty(link->trans_history))
        return FALSE;
    TimestampedTrans * latest = bot_circular_peek_nth(link->trans_history, 0);
    memcpy(trans, &latest->trans, sizeof(BotTrans));
    return TRUE;
}

static gboolean
_link_get_trans_interp(const BotCTransLink *link, int64_t utime, 
        BotTrans *result)
{
    if(bot_circular_is_empty(link->trans_history))
        return FALSE;
    TimestampedTrans * t1 = NULL;
    TimestampedTrans * t2 = NULL;
    int i = 0;
    while (i < link->trans_history->len) {
        t2 = t1;
        t1 = bot_circular_peek_nth (link->trans_history, i);
        if (t1->utime <= utime)
            break;
        i++;
    }
    if (i == link->trans_history->len){
        printf("Beyond start of trans history requested %lld\n", utime);
        t2 = NULL;
    }

    if(!t2) {
        memcpy(result, &t1->trans, sizeof(BotTrans));
    } else {
        assert(t1->utime < t2->utime);
        double weight_2 = 
            (double)((utime - t1->utime)) / (t2->utime - t1->utime);
        bot_trans_interpolate(result, &t1->trans, &t2->trans, weight_2);
    }

    // Added by Matt Antone and Maurice to extrapolate utimes in the future
    // Can result in erroneous estimates when weight_2 is too large, which may happen if
    // delta for the future utime is much larger than the delta for the last two utimes
    /*if (1 == link->trans_history->len) {
        memcpy(result, &t1->trans, sizeof(BotTrans));
    } else {
        if (i == 0) {
            t2 = t1;
            t1 = bot_circular_peek_nth (link->trans_history, 1);
        }
        assert(t1->utime < t2->utime);
        double weight_2 = 
            (double)((utime - t1->utime)) / (t2->utime - t1->utime);
        bot_trans_interpolate(result, &t1->trans, &t2->trans, weight_2);
        }*/
    return TRUE;
}

int 
bot_ctrans_link_get_n_trans(const BotCTransLink * link)
{
    return bot_circular_size(link->trans_history);
}

int 
bot_ctrans_link_get_nth_trans(BotCTransLink * link,
        int index, BotTrans *transformation, int64_t *utime)
{
    if(index >= bot_circular_size(link->trans_history) || index < 0)
        return 0;
    const TimestampedTrans *ttrans = 
        bot_circular_peek_nth(link->trans_history, index);
    if(transformation)
        memcpy(transformation, &ttrans->trans, sizeof(BotTrans));
    if(utime)
        *utime = ttrans->utime;
    return 1;
}

// ========== ctrans ============

struct _BotCTrans
{
    GHashTable * frames;

    GHashTable * links;

    GHashTable * path_cache;
};

BotCTrans * 
bot_ctrans_new(void)
{
    BotCTrans * ctrans = g_slice_new(BotCTrans);
    ctrans->frames = g_hash_table_new_full(g_str_hash, g_str_equal,
            NULL, (GDestroyNotify)_frame_destroy);
    ctrans->links = g_hash_table_new_full(g_str_hash, g_str_equal,
            NULL, (GDestroyNotify)_link_destroy);
    ctrans->path_cache = g_hash_table_new_full(g_str_hash, g_str_equal,
            free, (GDestroyNotify)bot_ctrans_path_destroy);
    return ctrans;
}

void 
bot_ctrans_destroy(BotCTrans *ctrans)
{
    g_hash_table_destroy(ctrans->frames);
    g_hash_table_destroy(ctrans->links);
    g_hash_table_destroy(ctrans->path_cache);
    g_slice_free(BotCTrans, ctrans);
}

static BotCTransFrame * 
bot_ctrans_get_frame(BotCTrans * ctrans, const char *frame_id)
{
    return (BotCTransFrame*) g_hash_table_lookup(ctrans->frames, frame_id);
}

static inline BotCTransFrame *
_get_frame_or_warn(BotCTrans * ctrans, const char *frame_id)
{
    BotCTransFrame *frame = bot_ctrans_get_frame(ctrans, frame_id);
    if(!frame) {
        g_warning("%s is not a valid coordinate frame\n", frame_id);
    }
    return frame;
}

int
bot_ctrans_add_frame(BotCTrans * ctrans, const char *id)
{
    assert(id);
    BotCTransFrame * frame = bot_ctrans_get_frame(ctrans, id);
    if(frame) {
        g_warning("%s: coordinate frame %s already exists\n", __FUNCTION__, id);
        return 0;
    } else {
        frame = _frame_new(id);
        g_hash_table_insert(ctrans->frames, frame->id, frame);
        g_hash_table_remove_all(ctrans->path_cache);
        return 1;
    }
}

BotCTransLink * 
bot_ctrans_get_link(BotCTrans * ctrans,
        const char * from_frame, const char * to_frame)
{
    char slenf = strlen(from_frame);
    char slent = strlen(to_frame);
    int blen = slenf + slent + 2;
    char buf[blen];
    _make_link_id2(from_frame, to_frame, buf, blen);
    BotCTransLink *result = g_hash_table_lookup(ctrans->links, buf);
    return result;
}

BotCTransLink * 
bot_ctrans_link_frames(BotCTrans * ctrans, 
        const char *from_frame_id, const char * to_frame_id, int history_maxlen)
{
    BotCTransFrame *from_frame = _get_frame_or_warn(ctrans, from_frame_id);
    BotCTransFrame *to_frame = _get_frame_or_warn(ctrans, to_frame_id);
    if(!from_frame || !to_frame) 
        return NULL;
    // check if the link will result in a graph cycle.  A cycle means
    // an overconstrained graph
    BotCTransPath * existing_path = bot_ctrans_get_new_path(ctrans,
            from_frame_id, to_frame_id);
    if(existing_path) {
        g_warning("%s: %s and %s already related. \n"
                "         Coordinate frame graph will be overconstrained\n",
                __FUNCTION__, from_frame->id, to_frame->id);
        bot_ctrans_path_destroy(existing_path);
    }
    if(history_maxlen < 1) {
        g_warning("%s: invalid history_maxlen (%d), coercing to 1\n", 
                __FUNCTION__, history_maxlen);
        history_maxlen = 1;
    }

    BotCTransLink *link = _link_new(from_frame, to_frame, history_maxlen);
    g_hash_table_insert(ctrans->links, link->id, link);
    _frame_add_link(from_frame, link);
    _frame_add_link(to_frame, link);
    g_hash_table_remove_all(ctrans->path_cache);
    return link;
}

static BotCTransPath * 
_get_path(BotCTrans * ctrans, const char *from_frame, const char *to_frame)
{
    char slenf = strlen(from_frame);
    char slent = strlen(to_frame);
    int blen = slenf + slent + 2;
    char buf[blen];
    snprintf(buf, blen, "%s-%s", from_frame, to_frame);
    BotCTransPath *path = g_hash_table_lookup(ctrans->path_cache, buf);
    if(!path) {
        path = bot_ctrans_get_new_path(ctrans, from_frame, to_frame);
        if(path)
            g_hash_table_insert(ctrans->path_cache, strdup(buf), path);
    }
    return path;
}

int 
bot_ctrans_get_trans(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame, int64_t utime, BotTrans *result)
{
    BotCTransPath * path = _get_path(ctrans, from_frame, to_frame);
    if(!path)
        return 0;
    return bot_ctrans_path_to_trans(path, utime, result);
}

int 
bot_ctrans_get_trans_latest(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame, BotTrans *result)
{
    BotCTransPath * path = _get_path(ctrans, from_frame, to_frame);
    if(!path)
        return 0;
    return bot_ctrans_path_to_trans_latest(path, result);
}

int 
bot_ctrans_have_trans(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame)
{
    BotCTransPath * path = _get_path(ctrans, from_frame, to_frame);
    if(!path) {
        g_warning("%s: invalid transformation requested (%s -> %s)\n", 
                __FUNCTION__, from_frame, to_frame);
        return 0;
    }
    return bot_ctrans_path_have_trans(path);
}

int 
bot_ctrans_get_trans_latest_timestamp(BotCTrans *ctrans, 
        const char *from_frame, const char *to_frame, int64_t *timestamp)
{
    BotCTransPath * path = _get_path(ctrans, from_frame, to_frame);
    if(!path)
        return 0;
    return bot_ctrans_path_latest_timestamp(path, timestamp);
}

// ========= path ==========

struct _BotCTransPath {
    int nlinks;
    BotCTransLink ** links;
    int *invert;
};

static BotCTransPath * 
_path_new(int nlinks)
{
    BotCTransPath * path = g_slice_new(BotCTransPath);
    path->nlinks = nlinks;
    path->links = g_slice_alloc0(nlinks*sizeof(BotCTransLink*));
    path->invert = g_slice_alloc0(nlinks*sizeof(int));
    return path;
}

void 
bot_ctrans_path_destroy(BotCTransPath * path)
{
    g_slice_free1(path->nlinks*sizeof(BotCTransLink*), path->links);
    g_slice_free1(path->nlinks*sizeof(int), path->invert);
    g_slice_free(BotCTransPath, path);
}

const char * 
bot_ctrans_path_get_frame_from(BotCTransPath *path)
{
    if(0 == path->nlinks)
        return NULL;
    if(path->invert[0]) {
        return path->links[0]->frame_to->id;
    } else {
        return path->links[0]->frame_from->id;
    }
}

const char * 
bot_ctrans_path_get_frame_to(BotCTransPath *path)
{
    if(0 == path->nlinks)
        return NULL;
    if(path->invert[path->nlinks-1]) {
        return path->links[path->nlinks-1]->frame_from->id;
    } else {
        return path->links[path->nlinks-1]->frame_to->id;
    }
}

typedef struct _SPNodeData SPNodeData;
struct _SPNodeData {
    int distance;
    SPNodeData * previous;
    BotCTransLink * previous_link;
    BotCTransFrame * frame;
    int invert_transformation;
};

static inline SPNodeData *
_spnode_data_new(BotCTransFrame *frame)
{
    SPNodeData *ndata = g_slice_new0(SPNodeData);
    ndata->frame = frame;
    return ndata;
}

static void
_spnode_data_destroy(SPNodeData *ndata)
{
    g_slice_free(SPNodeData, ndata);
}

BotCTransPath * 
bot_ctrans_get_new_path(BotCTrans * ctrans,
        const char * from_frame_id,
        const char * to_frame_id)
{
    dbg("%s (%s, %s)\n", __FUNCTION__,
            from_frame_id, to_frame_id);

    BotCTransFrame *from_frame = _get_frame_or_warn(ctrans, from_frame_id);
    BotCTransFrame *to_frame = _get_frame_or_warn(ctrans, to_frame_id);
    if(!from_frame || !to_frame) 
        return NULL;
    // do a djikstra shortest path search
 
    GHashTable *Q = g_hash_table_new(g_direct_hash, g_direct_equal);
    GPtrArray *all_ndata = g_ptr_array_new();

    GList *frames_list = bot_g_hash_table_get_vals(ctrans->frames);
    for(GList *fiter=frames_list; fiter; fiter=fiter->next) {
        BotCTransFrame *frame = fiter->data;
        SPNodeData *ndata = _spnode_data_new(frame);
        if(frame == from_frame) 
            ndata->distance = 0;
        else
            ndata->distance = -1;
        g_hash_table_insert(Q, frame, ndata);
        g_ptr_array_add(all_ndata, ndata);
    }

    int first_iter = 1;
    while(g_hash_table_size(Q) > 0) {
        dbg("===\n");

        // find the node with the shortest distance
        SPNodeData *u = NULL;
        if(first_iter) {
            u = g_hash_table_lookup(Q, from_frame);
            first_iter = 0;
        } else {
            for(GList *fiter=frames_list; fiter; fiter=fiter->next) {
                BotCTransFrame *frame = fiter->data;

                SPNodeData *ndata = g_hash_table_lookup(Q, frame);
                if(!ndata)
                    continue;

                dbg("   %s: %d\n", frame->id, ndata->distance);

                if(!u || u->distance < 0 ||
                   (ndata->distance >= 0 && ndata->distance < u->distance)) {
                    u = ndata;
                }
            }
        }

        dbg("u: %s (%d)\n", u->frame->id, u->distance);

        if(u->distance < 0 || u->frame == to_frame) {
            break;
        }

        g_hash_table_remove(Q, u->frame);
        
        // check each neighbor of u
        for(int lind=0; lind<u->frame->links->len; lind++) {
            BotCTransLink *link = g_ptr_array_index(u->frame->links, lind);

            const BotCTransFrame *nbr_frame = NULL;
            int invert_transformation = 0;
            if(link->frame_from == u->frame) {
                nbr_frame = link->frame_to;
                invert_transformation = 0;
            } else {
                nbr_frame = link->frame_from;
                invert_transformation = 1;
            }

            dbg("  nbr: %s\n", nbr_frame->id);

            SPNodeData *nbr_ndata = g_hash_table_lookup(Q, nbr_frame);
            if(!nbr_ndata)
                continue;

            int alt_dist = u->distance + 1;
            if(nbr_ndata->distance < 0 ||
               alt_dist < nbr_ndata->distance) {
                nbr_ndata->distance = alt_dist;
                nbr_ndata->previous = u;
                nbr_ndata->previous_link = link;
                nbr_ndata->invert_transformation = invert_transformation;

                dbg("  set: %s (%d)\n", nbr_ndata->frame->id, 
                        nbr_ndata->distance);
            }
        }
    }

    SPNodeData *to_node = g_hash_table_lookup(Q, to_frame);
    if(!to_node || to_node->distance < 0) {
        g_hash_table_destroy(Q);
        bot_g_ptr_array_free_with_func(all_ndata, 
                (GDestroyNotify)_spnode_data_destroy);
        g_list_free(frames_list);

        return NULL;
    }

    // how long is the path?
    int nlinks = 0;
    SPNodeData *node = to_node;
    while(node && node->frame != from_frame) {
        node = node->previous;
        nlinks++;
    }

    BotCTransPath *path = _path_new(nlinks);
    node = to_node;
    int nind = nlinks - 1;
    while(node && node->frame != from_frame) {
        path->links[nind] = node->previous_link;
        assert(node->previous_link);
        path->invert[nind] = node->invert_transformation;
        node = node->previous;
        nind--;
    }
    assert(nind == -1);

    g_hash_table_destroy(Q);
    bot_g_ptr_array_free_with_func(all_ndata, 
            (GDestroyNotify)_spnode_data_destroy);
    g_list_free(frames_list);

    return path;
}
        
int
bot_ctrans_path_to_trans(const BotCTransPath * path,
        int64_t utime, BotTrans *result)
{
    bot_trans_set_identity(result);
    BotTrans temp_trans;
    for(int lind=0; lind<path->nlinks; lind++) {
        BotCTransLink *link = path->links[lind];
        int have_trans = _link_get_trans_interp(link, utime, &temp_trans);
        if(!have_trans) {
            return 0;
        }
        if(path->invert[lind]) {
            bot_trans_invert(&temp_trans);
        }
        bot_trans_apply_trans(result, &temp_trans);
    }
    return 1;
}

int
bot_ctrans_path_to_trans_latest(const BotCTransPath * path, BotTrans *result)
{
    bot_trans_set_identity(result);
    BotTrans temp_trans;
    for(int lind=0; lind<path->nlinks; lind++) {
        BotCTransLink *link = path->links[lind];
        int have_trans = _link_get_trans_latest(link, &temp_trans);
        if(!have_trans) {
            return 0;
        }
        if(path->invert[lind]) {
            bot_trans_invert(&temp_trans);
        }
        bot_trans_apply_trans(result, &temp_trans);
    }
    return 1;
}

int 
bot_ctrans_path_latest_timestamp(const BotCTransPath * path,
        int64_t *timestamp)
{
    int64_t result = 0;
    for(int lind=0; lind<path->nlinks; lind++) {
        BotCTransLink *link = path->links[lind];
        int64_t link_timestamp;
        int have_trans = bot_ctrans_link_get_nth_trans(link, 0, NULL, 
                &link_timestamp);
        if(!have_trans) {
            return 0;
        }
        if(0 == lind || link_timestamp > result)
            result = link_timestamp;
    }
    assert(timestamp);
    *timestamp = result;
    return 1;
}

int 
bot_ctrans_path_have_trans(const BotCTransPath *path)
{
    for(int lind=0; lind<path->nlinks; lind++) {
        if(!_link_have_trans(path->links[lind])) {
            return 0;
        }
    }
    return 1;
}

void bot_ctrans_path_dump(const BotCTransPath *path);
void
bot_ctrans_path_dump(const BotCTransPath *path) 
{
    printf("%s\n", __FUNCTION__);
    for(int i=0; i<path->nlinks; i++) {
        BotCTransLink *link = path->links[i];
        printf("%2d: %s -> %s (%d)\n", i, link->frame_from->id, 
                link->frame_to->id, path->invert[i]);
    }
}
