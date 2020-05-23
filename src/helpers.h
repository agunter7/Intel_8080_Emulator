/***********************************************************************************
 *
 * Header for useful functions for general programming purposes.
 * These functions should never alter the 8080 state.
 * @Author: Andrew Gunter
 *
***********************************************************************************/

#ifndef HELPERS_H_
#define HELPERS_H_

#include "../src/cpuStructures.h"

/**
 Immediately prints content to console.
 Same call signature as printf().

 Achieves this by calling vprintf & fflush(stdout)

 @param format - print format specifier
 @param ... - variable quantity of arguments to be printed
 */
void logger(const char *format, ...);

/**
 Returns the 2's complement of an 8-bit number

 @param num - number to complement
 @return - 2's complement of num
 */
uint8_t twosComplement(uint8_t num);

/**
 * Returns a pointer to memory of a requested size.
 * Memory all zeroed.
 *
 * @param size - desired size of memory, in bytes
 * @return - pointer to requested memory
 */
void *mallocSet(size_t size);

#endif  // HELPERS_H_