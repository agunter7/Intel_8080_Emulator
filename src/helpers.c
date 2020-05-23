/***********************************************************************************
 *
 * Source for useful functions for general programming purposes.
 * These functions should never alter the 8080 state.
 * @Author: Andrew Gunter
 *
***********************************************************************************/


#include "../src/helpers.h"

void logger(const char *format, ...)
{
    va_list argList;

    // Initialize variable argument list
    va_start(argList, format);

    vprintf(format, argList);
    fflush(stdout);  // Immediately print to terminal

    // Free variable argument list
    va_end(argList);
}

uint8_t twosComplement(uint8_t num)
{
    uint8_t inverted = ~num;
    return (uint8_t)(inverted + (uint8_t)1);
}

void *mallocSet(size_t size)
{
    void *pointer = malloc(size);
    memset(pointer, 0 , size);

    return pointer;
}