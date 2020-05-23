/***********************************************************************************
 *
 * Provides an API for interacting with the Intel 8080 emulator
 * @Author: Andrew Gunter
 *
***********************************************************************************/

#ifndef INTEL_8080_EMULATOR_SHELL8080_H
#define INTEL_8080_EMULATOR_SHELL8080_H

#include "cpuStructures.h"

/**
 * Returns a pointer to an emulated Intel 8080 cpu state
 * All of the cpu's registers and memory will be zeroed, except for ROM, which contains Space Invaders
 * @return Initialized 8080 state pointer
 */
State8080 *initializeCPU();

/**
 * Frees memory allocated for a State8080 struct
 * @param state - The 8080 state
 */
void destroyCPU(State8080 *state);

/**
 * Returns a pointer to a copy of the 8080's current VRAM
 * @param state - The 8080 state
 * @return VRAM copy pointer
 */
uint8_t *getVideoRAM(State8080 *state);

/**
 * Triggers an 8080 interrupt
 * @param interruptNum - The number corresponding with the desired interrupt, valid from 0x00 to 0x07
 * @param state - The 8080 state
 */
void generateInterrupt(uint8_t interruptNum, State8080 *state);

/**
 * Returns a pointer to the stored binary derived from an input FILE
 * @param romFile - pointer to FILE whose data should be stored
 * @return - pointer to stored contents
 */
uint8_t *getRomBuffer(FILE *romFile);

/**
 * Execute the next instruction,
 * i.e. the instruction pointed at by the program counter
 * @param state - The 8080 state
 */
void executeNextInstruction(State8080 *state);

#endif //INTEL_8080_EMULATOR_SHELL8080_H
