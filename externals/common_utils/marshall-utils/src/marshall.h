#ifndef __hello_h__
#define __hello_h__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * very simple marshalling just based on binary copying and casting. doesn't do any error checking, use carefully
 */

#include <stdint.h>

/*
 * returns 1 if system is little endian, 0 for big endian
 */
int system_little_endian_check();

/**
 * functions for converting bytes into integers
 */
int16_t bytes_to_int16(const unsigned char bytes[2], int switch_endian);

int32_t bytes_to_int32(const unsigned char bytes[4], int switch_endian);

int64_t bytes_to_int64(const unsigned char bytes[8], int switch_endian);

uint16_t bytes_to_uint16(const unsigned char bytes[2], int switch_endian);

uint32_t bytes_to_uint32(const unsigned char bytes[4], int switch_endian);

uint64_t bytes_to_uint64(const unsigned char bytes[8], int switch_endian);

/**
 * functions for converting bytes into floats
 */
float bytes_to_single_float(const unsigned char bytes[4], int switch_endian);

double bytes_to_double_float(const unsigned char bytes[8], int switch_endian);

/**
 * functions for converting ints into bytes
 */
void int16_to_bytes(int16_t source, unsigned char bytes[2], int switch_endian);

void int32_to_bytes(int32_t source, unsigned char bytes[4], int switch_endian);

void int64_to_bytes(int32_t source, unsigned char bytes[8], int switch_endian);

void uint16_to_bytes(uint16_t source, unsigned char bytes[2], int switch_endian);

void uint32_to_bytes(uint32_t source, unsigned char bytes[4], int switch_endian);

void uint64_to_bytes(uint64_t source, unsigned char bytes[8], int switch_endian);

/**
 * functions for converting floast into bytes
 */
void single_float_to_bytes(float source, unsigned char bytes[4], int switch_endian);

void double_float_to_bytes(double source, unsigned char bytes[8], int switch_endian);


#ifdef __cplusplus
}
#endif

#endif
