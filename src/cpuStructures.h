#ifndef CPUSTRUCTURES_H_
#define CPUSTRUCTURES_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#define MEMORY_SIZE_8080 65536  // 16-bit byte-addressable memory, 2^16 == 65536 bytes
#define VRAM_SIZE_8080 7168  // Bytes
#define VRAM_START_ADDR_8080 0x2400
#define ROM_LIMIT_8080 0x2000
#define CYCLES_PER_SECOND_8080 2000000
#define NUM_INPUT_DEVICES 256
#define NUM_OUTPUT_DEVICES 256

/**
Intel 8080 condition codes can be thought of as existing in an 8-bit register.
This doesn't seem to actually be the case, but simplifies organization in emulation.
The various bits of this register correspond with different flags/conditions.
*/
typedef struct ConditionCodes {
    // Define register bit-field
    uint8_t    zero:1;  
    uint8_t    sign:1;  
    uint8_t    parity:1;    
    uint8_t    carry:1;    
    uint8_t    auxiliaryCarry:1;
    uint8_t    unusedBits:3;
} ConditionCodes;

typedef struct State8080 {
    uint8_t *memory;
    ConditionCodes flags;  /**< Each bit represents some condition state of the 8080 */
    // I/O Buffers - For communicating with emulated I/O devices
    uint8_t *inputBuffers;  /**< For receiving input data from external devices */
    uint8_t *outputBuffers;  /**< For transmitting output data to external devices */
    // Registers
    uint8_t    a;    
    uint8_t    b;    
    uint8_t    c;    
    uint8_t    d;    
    uint8_t    e;    
    uint8_t    h;    
    uint8_t    l;    
    uint16_t    sp;  /**< Stack Pointer */
    uint16_t    pc;  /**< Program Counter */
    // CPU variables
    unsigned int cyclesCompleted;  /**< Number of clock cycles executed since instantiation */
    bool interruptsEnabled;
} State8080;

#endif  // CPUSTRUCTURES_H_