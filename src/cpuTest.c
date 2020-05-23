/***********************************************************************************
 *
 * Runs a Diagnostic Test of the Intel 8080 CPU Emulator
 * @Author: Andrew Gunter
 *
***********************************************************************************/

#include "shell8080.h"
#include "helpers.h"

#define CPUDIAG_SIZE 1453

int main(int argc, char **argv)
{
    State8080 *state = initializeCPU();

    // Clear 8080 Memory
    memset(state->memory, 0, MEMORY_SIZE_8080);

    // Place diagnostic test in ROM
    FILE *cpudiag = fopen("resources/cpudiag.bin", "rb");
    if(cpudiag == NULL){
        logger("Failed to open cpudiag.bin\n");
        return 0;
    }
    uint8_t *romBuffer = getRomBuffer(cpudiag);
    memcpy(&(state->memory[0x100]), romBuffer, CPUDIAG_SIZE);  // cpudiag starts at mem location 0x100
    state->pc = 0x100;  // cpudiag starts at 0x100

    // Fix stack pointer for testing.
    // cpudiag sets stack pointer to 0x06ad,
    // but our program offsets everything by 0x0100,
    // so we need to set the stack pointer to 0x07ad.
    // Stack pointer upper byte data is at address 0x0070 in cpudiag,
    // which becomes address 0x0170 with our program's offset.
    // Hence, set address 0x0170 to have data value 0x07
    state->memory[0x0170] = 0x07;

    // Run test
    while(state->pc < ROM_LIMIT_8080){
        executeNextInstruction(state);
    }
    return 0;
}