#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "circular.h"

BotCircular *
bot_circular_new (int capacity, int element_size)
{
    BotCircular * circular = malloc (sizeof (BotCircular));
    if (!circular)
        return NULL;
    memset (circular, 0, sizeof (BotCircular));

    circular->array = calloc (capacity, element_size);
    if (!circular->array) {
        free (circular);
        return NULL;
    }
    circular->capacity = capacity;
    circular->element_size = element_size;
    return circular;
}

void
bot_circular_free (BotCircular * circular)
{
    free (circular->array);
    memset (circular, 0, sizeof (BotCircular));
    free (circular);
}

void
bot_circular_clear (BotCircular * circular)
{
    circular->head = 0;
    circular->len = 0;
}

int
bot_circular_push_head (BotCircular * circular, const void * data)
{
    circular->head--;
    if (circular->head < 0)
        circular->head = circular->capacity - 1;

    if (circular->len < circular->capacity)
        circular->len++;

    memcpy (bot_circular_peek_nth (circular, 0), data, circular->element_size);

    return 0;
}

int
bot_circular_pop_tail (BotCircular * circular, void * data)
{
    if (bot_circular_is_empty (circular))
        return -1;

    if (data)
        memcpy (data, bot_circular_peek_nth (circular, circular->len - 1),
                circular->element_size);

    circular->len--;

    return 0;
}

int
bot_circular_pop_head (BotCircular * circular, void * data)
{
    if (bot_circular_is_empty (circular))
        return -1;

    if (data)
        memcpy (data, bot_circular_peek_nth (circular, 0),
                circular->element_size);

    circular->len--;
    circular->head = (circular->head + 1) % circular->capacity;

    return 0;
}

int 
bot_circular_size(BotCircular *circular)
{
    return circular->len;
}
