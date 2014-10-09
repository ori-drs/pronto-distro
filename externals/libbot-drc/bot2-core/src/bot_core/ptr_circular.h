#ifndef __bot_ptr_circular_h__
#define __bot_ptr_circular_h__

/**
 * @defgroup BotCorePtrCircular Circular Pointer Array
 * @brief Circular array of pointers
 * @ingroup BotCoreDataStructures
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*BotPtrCircularDestroy)(void *user, void *p);

/** A circular Buffer, implemented with a PtrArray **/
typedef struct gcircular BotPtrCircular;
struct gcircular
{
    void **p;     // storage
    unsigned int size;     // number of valid elements in the buffer
    unsigned int next;     // where the next add will go
    unsigned int capacity; // maximum allowed capacity

    void *user;
    BotPtrCircularDestroy handler;
};

// create a new circular buffer; the destroy handler will be called
// whenever an element is evicted. (NULL means no handler).
BotPtrCircular *bot_ptr_circular_new(unsigned int capacity, BotPtrCircularDestroy handler, void *user);

// call destroy() on any elements, then deallocate the circular buffer
void bot_ptr_circular_destroy(BotPtrCircular *circ);

// adds a new element to the buffer, possibly evicting the oldest
void bot_ptr_circular_add(BotPtrCircular *circ, void *p);

// return the number of valid elements in the buffer
unsigned int bot_ptr_circular_size(BotPtrCircular *circ);

// An index of zero corresponds to the most recently added item.
void *bot_ptr_circular_index(BotPtrCircular *circ, unsigned int idx);

// resize the circular buffer, freeing elements as required
void bot_ptr_circular_resize(BotPtrCircular *circ, unsigned int capacity);

// remove all elements from the buffer.
void bot_ptr_circular_clear(BotPtrCircular *circ);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
