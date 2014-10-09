#ifndef __bot_circular_h__
#define __bot_circular_h__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup BotCoreCircular Circular Array
 * @brief Fixed-capacity circular array 
 * @ingroup BotCoreDataStructures
 * @include: bot_core/bot_core.h
 *
 * BotCircular is a hybrid of the glib types GArray and GQueue.  BotCircular
 * acts like a GQueue in the sense that you can push on one end and pop
 * from the other.  It acts like a GArray in the sense that its contents
 * are statically allocated rather than pointers to user-allocated buffers.
 * For this reason, its size is fixed and allocated when the BotCircular
 * is created (TODO: set_size function).  If a new element is pushed
 * when the BotCircular is already full, the last element on the tail is
 * automatically overwritten.
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */
typedef struct _BotCircular BotCircular;

struct _BotCircular {
    int len;
    int capacity;
    int element_size;
    int head;
    void * array;
};

BotCircular *
bot_circular_new (int capacity, int element_size);
void
bot_circular_free (BotCircular * circular);
void
bot_circular_clear (BotCircular * circular);
int
bot_circular_push_head (BotCircular * circular, const void * data);
int
bot_circular_pop_tail (BotCircular * circular, void * data);
int
bot_circular_pop_head (BotCircular * circular, void * data);

/**
 * bot_circular_size:
 * Returns: the number of valid elements.
 */
int bot_circular_size(BotCircular *circular);

#define bot_circular_is_empty(a) ((a)->len == 0)

#define bot_circular_is_full(a) ((a)->len >= (a)->capacity)

#define bot_circular_peek_nth(a,i) \
    ((void*)((char*)(a)->array + (((a)->head + (i)) % (a)->capacity) * (a)->element_size))

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
