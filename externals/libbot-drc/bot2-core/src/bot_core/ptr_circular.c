#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "ptr_circular.h"

// NB: a capacity of 0 is legal!

BotPtrCircular *bot_ptr_circular_new(unsigned int capacity, BotPtrCircularDestroy handler, void *user)
{
    BotPtrCircular *circ = (BotPtrCircular*) calloc(1, sizeof(BotPtrCircular));
    if (capacity > 0)
        circ->p = (void**) calloc(capacity, sizeof(void*));
    else
        circ->p = NULL;
    
    circ->size = 0;
    circ->next = 0;
    circ->capacity = capacity;
    circ->handler = handler;
    circ->user = user;
    
    return circ;
}

// call destroy() on any elements, then deallocate the circular buffer
void bot_ptr_circular_destroy(BotPtrCircular *circ)
{
    for (unsigned int i = 0; i < circ->capacity; i++) {
        if (circ->p[i] && circ->handler)
            circ->handler(circ->user, circ->p[i]);
    }

    if (circ->p)
        free(circ->p);

    circ->p = NULL;
    free(circ);
}

// adds a new element to the buffer, possibly evicting the oldest
void bot_ptr_circular_add(BotPtrCircular *circ, void *p)
{
    if (circ->capacity == 0)
        return;

    // free the old element
    if (circ->size == circ->capacity && circ->handler)
        circ->handler(circ->user, circ->p[circ->next]);

    // add the new one
    circ->p[circ->next] = p;
    circ->next++;
    if (circ->next == circ->capacity)
        circ->next = 0;
    if (circ->size < circ->capacity)
        circ->size++;
}

// return the number of valid elements in the buffer
unsigned int bot_ptr_circular_size(BotPtrCircular *circ)
{
    return circ->size;
}

void *bot_ptr_circular_index(BotPtrCircular *circ, unsigned int idx)
{
    assert (idx < circ->size);

    unsigned int offset = circ->capacity + circ->next - 1 - idx;
    if (offset >= circ->capacity)
        offset -= circ->capacity;

    assert(offset < circ->capacity);

    return circ->p[offset];
}

void bot_ptr_circular_resize(BotPtrCircular *circ, unsigned int new_capacity)
{
    // nothing to do?
    if (new_capacity == circ->capacity)
        return;
    
    // strategy: create a new circular buffer that contains the required data,
    // then swap out our internal representation. We have to be careful
    // about freeing any unneeded data.

    BotPtrCircular *newcirc = bot_ptr_circular_new(new_capacity, circ->handler, circ->user);

    // how many elements in the new one?
    // (so that we are guaranteed to never evict an element while
    // we populate the new data structure)
    unsigned int new_size = circ->size < new_capacity ? circ->size : new_capacity;

    // copy the data into the new structure, oldest first.
    for (unsigned int i = 0; i < new_size; i++)
        bot_ptr_circular_add(newcirc, bot_ptr_circular_index(circ, new_size - 1 - i));

    // free any elements we didn't copy
    for (unsigned int i = new_size; i < circ->size; i++) {
        void *p = bot_ptr_circular_index(circ, i);
        circ->handler(circ->user, p);
    }

    // okay, switch over the data structure
    if (circ->p)
        free(circ->p);
    memcpy(circ, newcirc, sizeof(BotPtrCircular));
    
    // free the new circ container, but not its storage
    free(newcirc);
}

void bot_ptr_circular_clear(BotPtrCircular *circ)
{
    for (unsigned int i = 0; i < circ->size; i++) {
        void *p = bot_ptr_circular_index(circ, i);
        circ->handler(circ->user, p);
    }

    circ->size = 0;
}
