#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ringbuf.h"
#include "serial.h"

#ifndef MIN
#define MIN(a,b)((a < b) ? a : b)
#endif

struct _BotRingBuf{
        uint8_t * buf;
        int readOffset;
        int writeOffset;
        int numBytes;
        int maxSize;
        uint8_t * read_buf;
        int read_buf_sz;
};

BotRingBuf * bot_ringbuf_create(int size)
{
  //create buffer, and allocate space for size bytes
  BotRingBuf * cbuf = (BotRingBuf *) calloc(1,sizeof(BotRingBuf));
  cbuf->buf = (uint8_t *) malloc(size * sizeof(uint8_t));
  cbuf->readOffset = cbuf->writeOffset = cbuf->numBytes = 0;
  cbuf->maxSize = size;

  cbuf->read_buf_sz = 256;
  cbuf->read_buf = (uint8_t *) malloc(cbuf->read_buf_sz * sizeof(uint8_t));

  return cbuf;
}

void bot_ringbuf_destroy(BotRingBuf * cbuf)
{
  //destroy
  if (cbuf->buf!=NULL)
    free(cbuf->buf);
  free(cbuf);
}

int bot_ringbuf_read(BotRingBuf * cbuf, int numBytes, uint8_t * buf)
{
  //read numBytes
  int bytes_read = bot_ringbuf_peek(cbuf, numBytes, buf);

  bot_ringbuf_flush(cbuf,bytes_read);

  return bytes_read;

}

int bot_ringbuf_write(BotRingBuf * cbuf, int numBytes, uint8_t * buf)
{

  //check if there is enough space... maybe this should just wrap around??
  if (numBytes + cbuf->numBytes > cbuf->maxSize) {
    fprintf(stderr, "CIRC_BUF ERROR: not enough space in circular buffer,Discarding data!\n");
    numBytes = cbuf->maxSize - cbuf->numBytes;

  }
  //write to wrap around point.
  int bytes_written = MIN(cbuf->maxSize - cbuf->writeOffset, numBytes);
  memcpy(cbuf->buf + cbuf->writeOffset, buf, bytes_written * sizeof(char));
  numBytes -= bytes_written;

  //write the rest from start of buffer
  if (numBytes > 0) {
    memcpy(cbuf->buf, buf + bytes_written, numBytes * sizeof(char));
    bytes_written += numBytes;
  }

  //move writePtr
  cbuf->numBytes += bytes_written;
  cbuf->writeOffset = (cbuf->writeOffset + bytes_written) % cbuf->maxSize;

  return bytes_written;

}

int bot_ringbuf_peek(BotRingBuf * cbuf, int numBytes, uint8_t * buf)
{
  //read numBytes from start of buffer, but don't move readPtr
  if (numBytes > cbuf->numBytes || numBytes > cbuf->maxSize) {
    fprintf(stderr, "CIRC_BUF ERROR: Can't read %d bytes from the circular buffer, only %d available! \n",
        numBytes,cbuf->numBytes);
    return -1;
  }
  //read up to wrap around point
  int bytes_read = MIN(cbuf->maxSize - cbuf->readOffset, numBytes);
  memcpy(buf, cbuf->buf + cbuf->readOffset, bytes_read * sizeof(char));
  numBytes -= bytes_read;

  //read again from beginning if there are bytes left
  if (numBytes > 0) {
    memcpy(buf + bytes_read, cbuf->buf, numBytes * sizeof(char));
    bytes_read += numBytes;
  }
  return bytes_read;
}

int bot_ringbuf_flush(BotRingBuf * cbuf, int numBytes)
{
  //move pointers to "empty" the read buffer
  if (numBytes<0)
    cbuf->readOffset = cbuf->writeOffset = cbuf->numBytes = 0;
  else{
    //move readPtr
    cbuf->numBytes -= numBytes;
    cbuf->readOffset = (cbuf->readOffset + numBytes) % cbuf->maxSize;
  }
  return 0;
}

int bot_ringbuf_available(BotRingBuf * cbuf) {
        return cbuf->numBytes;
}

int bot_ringbuf_fill_from_fd(BotRingBuf * cbuf, int fd, int numBytes)
{
  if (numBytes < 0) {
    numBytes = bot_serial_bytes_available(fd);
    if (numBytes <= 0)
      return numBytes;
  }

  //check if there is enough space... maybe this should just wrap around??
  if (numBytes + cbuf->numBytes > cbuf->maxSize) {
    numBytes = cbuf->maxSize - cbuf->numBytes;
  }
  //write to wrap around point.
  int bytes_written = MIN(cbuf->maxSize - cbuf->writeOffset, numBytes);
  int num_read = read(fd, cbuf->buf + cbuf->writeOffset, bytes_written);
  if (num_read != bytes_written) {
    fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, bytes_written);
  }
  numBytes -= bytes_written;

  //write the rest from start of buffer
  if (numBytes > 0) {
    int num_read = read(fd, cbuf->buf, numBytes);
    if (num_read != numBytes) {
      fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, numBytes);
    }
    bytes_written += numBytes;
  }

  //move writePtr
  cbuf->numBytes += bytes_written;
  cbuf->writeOffset = (cbuf->writeOffset + bytes_written) % cbuf->maxSize;

  return bytes_written;
}

const uint8_t * bot_ringbuf_peek_buf(BotRingBuf * cbuf, int numBytes)
{
  if (numBytes > cbuf->maxSize) {
    fprintf(stderr, "ERROR: can't read %d bytes from ringbuf, maxsize is %d\n", numBytes, cbuf->maxSize);
    return NULL;
  }
  else if (numBytes > cbuf->numBytes) {
    fprintf(stderr, "ERROR: can't read %d bytes from ringbuf, currently containts is %d\n", numBytes, cbuf->numBytes);
    return NULL;
  }

  int contiguous_bytes = cbuf->maxSize - cbuf->readOffset;
  if (numBytes < contiguous_bytes)
    return cbuf->buf + cbuf->readOffset;

  if (numBytes > cbuf->read_buf_sz) {
    cbuf->read_buf_sz = numBytes;
    cbuf->read_buf = (uint8_t *) realloc(cbuf->read_buf, cbuf->read_buf_sz * sizeof(uint8_t));
  }

  bot_ringbuf_peek(cbuf, numBytes, cbuf->read_buf);
  return cbuf->read_buf;
}

#if 0
void bot_ringbuf_unit_test()
{
  uint8_t * testString = "iuerrlfkladbytes_writtenbytes_writte";
  char comp[1000];
  uint8_t * comp_p = comp;
  BotRingBuf cbuf;
  bot_ringbuf_create(&cbuf, 13);

  int numWritten = 0;
  int writeAmount = 0;
  int numRead = 0;
  int readAmount = 0;

  writeAmount = 4;
  bot_ringbuf_write(&cbuf, writeAmount, testString + numWritten);
  numWritten += writeAmount;

  writeAmount = 6;
  bot_ringbuf_write(&cbuf, writeAmount, testString + numWritten);
  numWritten += writeAmount;

  readAmount = 9;
  bot_ringbuf_read(&cbuf, readAmount, comp_p + numRead);
  numRead += readAmount;

  writeAmount = 11;
  bot_ringbuf_write(&cbuf, writeAmount, testString + numWritten);
  numWritten += writeAmount;

  readAmount = 12;
  bot_ringbuf_read(&cbuf, readAmount, comp_p + numRead);
  numRead += readAmount;

  printf("at end, there are %d bytes left, should be %d \n", cbuf.numBytes, numWritten - numRead);

  if (strncmp(testString, comp, numRead) == 0)
    printf("WOOOHOO! the strings match :-)\n");
  else
    printf("BOOOO Somethings wrong");

  bot_ringbuf_destroy(&cbuf);
}
#endif
