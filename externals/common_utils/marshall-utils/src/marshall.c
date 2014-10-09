#include "marshall.h"
#include <stdlib.h>
#include <string.h>

/*
 * returns 1 if system is little endian, 0 for big endian
 */
int system_little_endian_check()
{
  int16_t word = 0x0001;
  char *bytes = (char *) &word;
  if (bytes[0] == 0)
    return 0;
  else
    return 1;
}

void reverse_memcpy(void * dest, const void * source, int num_bytes)
{
  int ii, jj;
  jj = num_bytes - 1;
  for (ii = 0; ii < num_bytes; ii++) {
    ((unsigned char *) dest)[ii] = ((unsigned char *) source)[jj];
    jj--;
  }
}

/**
 * functions for converting bytes into integers
 */
int16_t bytes_to_int16(const unsigned char bytes[2], int switch_endian)
{
  int16_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 2);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 2);
  }
  return dest_num;
}

int32_t bytes_to_int32(const unsigned char bytes[4], int switch_endian)
{
  int32_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 4);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 4);
  }
  return dest_num;
}

int64_t bytes_to_int64(const unsigned char bytes[8], int switch_endian)
{
  int64_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 8);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 8);
  }
  return dest_num;
}

uint16_t bytes_to_uint16(const unsigned char bytes[2], int switch_endian)
{
  uint16_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 2);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 2);
  }
  return dest_num;
}

uint32_t bytes_to_uint32(const unsigned char bytes[4], int switch_endian)
{
  uint32_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 4);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 4);
  }
  return dest_num;
}

uint64_t bytes_to_uint64(const unsigned char bytes[8], int switch_endian)
{
  int64_t dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 8);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 8);
  }
  return dest_num;
}

/**
 * functions for converting bytes into floats
 */
float bytes_to_single_float(const unsigned char bytes[4], int switch_endian)
{
  float dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 4);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 4);
  }
  return dest_num;
}

double bytes_to_double_float(const unsigned char bytes[8], int switch_endian)
{
  double dest_num;
  void * dest_ptr = (void *) &dest_num;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, (void *) bytes, 8);
  }
  else {
    memcpy(dest_ptr, (void *) bytes, 8);
  }
  return dest_num;
}

/**
 * functions for converting ints into bytes
 */
void int16_to_bytes(int16_t source, unsigned char bytes[2], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 2);
  }
  else {
    memcpy(dest_ptr, source_ptr, 2);
  }
}

void int32_to_bytes(int32_t source, unsigned char bytes[4], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 4);
  }
  else {
    memcpy(dest_ptr, source_ptr, 4);
  }
}

void int64_to_bytes(int32_t source, unsigned char bytes[8], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 8);
  }
  else {
    memcpy(dest_ptr, source_ptr, 8);
  }
}

void uint16_to_bytes(uint16_t source, unsigned char bytes[2], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 2);
  }
  else {
    memcpy(dest_ptr, source_ptr, 2);
  }
}

void uint32_to_bytes(uint32_t source, unsigned char bytes[4], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 4);
  }
  else {
    memcpy(dest_ptr, source_ptr, 4);
  }
}


void uint64_to_bytes(uint64_t source, unsigned char bytes[8], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 8);
  }
  else {
    memcpy(dest_ptr, source_ptr, 8);
  }
}

/**
 * functions for converting floats into bytes
 */
void single_float_to_bytes(float source, unsigned char bytes[4], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 4);
  }
  else {
    memcpy(dest_ptr, source_ptr, 4);
  }
}

void double_float_to_bytes(double source, unsigned char bytes[8], int switch_endian)
{
  void * dest_ptr = (void *) bytes;
  void * source_ptr = (void *) &source;

  if (switch_endian) {
    reverse_memcpy(dest_ptr, source_ptr, 8);
  }
  else {
    memcpy(dest_ptr, source_ptr, 8);
  }
}
