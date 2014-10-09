#ifndef __bot_ringbuf_h__
#define __bot_ringbuf_h__
#include <stdint.h>

/**
 * @defgroup BotCoreRingbuf A simple ring buffer
 * @brief A fixed capacity ring buffer
 * @ingroup BotCoreDataStructures
 * @include: bot_core/bot_core.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 *
 */
typedef struct _BotRingBuf BotRingBuf;

/*
 * Create buffer, and allocate space for size bytes
 */
BotRingBuf * bot_ringbuf_create(int size);

/*
 * Destroy it
 */
void bot_ringbuf_destroy(BotRingBuf * cbuf);

/*
 * Copy numBytes from the head of the buffer, and move read pointers
 */
int bot_ringbuf_read(BotRingBuf * cbuf, int numBytes, uint8_t * buf);

/*
 * Copy numBytes from buf to end of buffer
 */
int bot_ringbuf_write(BotRingBuf * cbuf, int numBytes, uint8_t * buf);


/*
 * Fill the ringbuff with data from the file descriptor.
 * Either read numBytes from the fd,
 * or if numBytes<0, get all available bytes
 */
int bot_ringbuf_fill_from_fd(BotRingBuf * cbuf, int fd, int numBytes);


/*
 * Copy numBytes from the head of the buffer, but DON'T move read pointers
 */
int bot_ringbuf_peek(BotRingBuf * cbuf, int numBytes, uint8_t * buf); //read numBytes from start of buffer, but don't move readPtr


/**
 * Return a pointer to a contiguous buffer with the next numBytes of data to be read
 */
const uint8_t * bot_ringbuf_peek_buf(BotRingBuf * cbuf, int numBytes);

/*
 * flush numBytes from the buffer.
 * pass in -1 to empy buffer completely!
 */
int bot_ringbuf_flush(BotRingBuf * cbuf, int numBytes);
/*
 * Get the amount of data currently stored in the buffer (not the allocated size)
 */
int bot_ringbuf_available(BotRingBuf * cbuf);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
