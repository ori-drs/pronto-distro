#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <glib.h>

#include "minheap.h"

struct _BotMinheapNode 
{
    int index;
    void *ptr;
    double score;
};

struct _BotMinheap 
{
    GPtrArray *nodes;
};

#define _get_node(mh, ind) ((BotMinheapNode*) g_ptr_array_index(mh->nodes,ind))

static inline void 
_swap_nodes (BotMinheap *mh, int a, int b)
{
    BotMinheapNode *nodea = _get_node (mh, a);
    BotMinheapNode *nodeb = _get_node (mh, b);
    
    g_ptr_array_index (mh->nodes, a) = nodeb;
    g_ptr_array_index (mh->nodes, b) = nodea;

    nodeb->index = a;
    nodea->index = b;
}

static void 
fixup (BotMinheap *mh, int parent_ind)
{
    if (parent_ind >= mh->nodes->len) return;

    int left_ind = parent_ind*2 + 1;
    int right_ind = left_ind + 1;

    // leaf node, exit.
    if (left_ind >= mh->nodes->len) return;

    BotMinheapNode *node_parent = _get_node (mh, parent_ind);
    assert (node_parent->index == parent_ind);

    // only left node is valid
    if (right_ind >= mh->nodes->len) {
        BotMinheapNode *node_left = _get_node (mh, left_ind);
        assert (node_left->index == left_ind);

        // do we need to swap the parent and the leaf?
        // we don't need to call fixupMaxheap recursively since the
        // left node must be a leaf.
        if (node_left->score < node_parent->score) {
            _swap_nodes(mh, left_ind, parent_ind);
            return;
        }
        return;
    }

    // general node case: both right and left are valid nodes.
    BotMinheapNode *node_left = _get_node (mh, left_ind);
    BotMinheapNode *node_right = _get_node (mh, right_ind);
    assert (node_left->index == left_ind);
    assert (node_right->index == right_ind);
    
    // if parent is the minimum, we're done.
    if (node_parent->score < node_left->score &&
        node_parent->score < node_right->score)
        return;
    
    // parent is greater than either the left, right, or both
    // children
    if (node_left->score < node_right->score)  {
        _swap_nodes(mh, left_ind, parent_ind);
        fixup(mh, left_ind);
    } else {
        _swap_nodes(mh, right_ind, parent_ind);
        fixup(mh, right_ind);
    }
}

BotMinheap *bot_minheap_new()
{
    return bot_minheap_sized_new (8);
}

BotMinheap *bot_minheap_sized_new(int capacity)
{
    if (capacity < 8) capacity = 8;

    BotMinheap *mh = g_slice_new (BotMinheap);
    mh->nodes = g_ptr_array_sized_new (capacity);
    return mh;
}

void bot_minheap_free(BotMinheap *mh)
{
    for (int i=0; i<mh->nodes->len; i++) {
        BotMinheapNode *node = _get_node (mh, i);
        g_slice_free (BotMinheapNode, node);
    }
    g_ptr_array_free (mh->nodes, TRUE);
    g_slice_free (BotMinheap, mh);
}

BotMinheapNode *
bot_minheap_add(BotMinheap *mh, void *ptr, double score)
{
    BotMinheapNode *node = g_slice_new (BotMinheapNode);
    int node_ind = mh->nodes->len;
    node->index = node_ind;
    node->ptr = ptr;
    node->score = score;
    g_ptr_array_add (mh->nodes, node);

    do {
        node_ind = (node_ind - 1)/2;
        fixup (mh, node_ind);
    } while (node_ind);
    return node;
}

int bot_minheap_size(BotMinheap *mh)
{
    return mh->nodes->len;
}

void *
bot_minheap_remove_min (BotMinheap *mh, double *score)
{
    if (!mh->nodes->len) return NULL;
    BotMinheapNode *root = 
        (BotMinheapNode*) g_ptr_array_remove_index_fast (mh->nodes, 0);
    void *result = root->ptr;
    if (score) *score = root->score;
    g_slice_free (BotMinheapNode, root);

    if (mh->nodes->len) {
        BotMinheapNode *replacement = _get_node (mh, 0);
        replacement->index = 0;
        fixup(mh, 0);
    }

    return result;
}

void 
bot_minheap_decrease_score (BotMinheap *mh, BotMinheapNode *node, double score)
{
    if (score > node->score) {
        g_warning ("GUMinHeap: refusing to increase the score of a node\n");
        return;
    }
    node->score = score;
    assert (_get_node (mh, node->index) == node);
    int node_ind = node->index;
    do {
        node_ind = (node_ind - 1)/2;
        fixup (mh, node_ind);
    } while (node_ind);
}

gboolean 
bot_minheap_is_empty (BotMinheap *mh)
{
    return mh->nodes->len == 0;
}
