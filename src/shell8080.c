/***********************************************************************************
* Emulates an Intel 8080 CPU's internal state, instruction execution, and I/O
* @Author: Andrew Gunter
***********************************************************************************/

#include "../src/instructions.h"
#include "../src/cpuStructures.h"
#include "../src/helpers.h"

#define DEBUG 1
#define LOAD_ROM 0
#define CPU_DIAG 1

// Global variable definitions and function prototypes
char instructions[256][20];
char instructionSizes[256];
char instructionFlags[256][20];
char instructionFunctions[256][100];
void initializeGlobals();
uint8_t *getRomBuffer(FILE *romFile);
void executeInstructionByOpcode(uint8_t opcode, uint8_t *operands, State8080 *state);
void executeNextInstruction(State8080 *state);
int numExec = 0;  // Counts number of executed instructions


State8080 *initializeCPU()
{
    uint8_t *romBuffer;  // Buffer for storing bytes read from Space Invaders ROM
    initializeGlobals();

    if(LOAD_ROM){
        // Open Space Invaders ROM file as binary-read-only and store contents in a buffer
        FILE *romFile = fopen("", "rb");  // Replace null string with ROM path
        if(romFile == NULL){
            logger("Failed to open ROM.\n");
            return NULL;
        }
        romBuffer = getRomBuffer(romFile);
    }

    // Initialize an 8080 state variable
    State8080 *state = mallocSet(sizeof(State8080));

    state->memory = mallocSet(MEMORY_SIZE_8080);
    ConditionCodes cc = {0};
    state->flags = cc;
    state->inputBuffers = mallocSet(NUM_INPUT_DEVICES);
    state->outputBuffers = mallocSet(NUM_OUTPUT_DEVICES);
    state->a = 0;
    state->b = 0;
    state->c = 0;
    state->d = 0;
    state->e = 0;
    state->h = 0;
    state->l = 0;
    state->sp = 0;
    state->pc = 0;
    state->cyclesCompleted = 0;
    state->interruptsEnabled = 0;

    if(LOAD_ROM){
        // Place ROM buffer data into CPU memory
        memcpy(state->memory, romBuffer, ROM_LIMIT_8080);
        free(romBuffer);
    }

    return state;
}

void destroyCPU(State8080 *state)
{
    free(state->memory);
    free(state->inputBuffers);
    free(state->outputBuffers);
    free(state);
}

void executeNextInstruction(State8080 *state)
{
    if(state->pc < ROM_LIMIT_8080){
        uint8_t operation = 0;  // next instruction opcode
        uint8_t operands[2] = {0xff, 0xff};  // next instruction operands, default 0xff as it would standout more than 0x00
        unsigned int instructionSize = 0;

        // Get next operation
        operation = state->memory[state->pc];

        // Get operands depending on instruction size
        instructionSize = instructionSizes[operation];  // Array is ordered based on opcode
        uint16_t lastOperandAddress = (state->pc)+instructionSize-1;
        int operandNum;
        for(uint16_t operandAddress = state->pc+1; operandAddress <= lastOperandAddress; operandAddress++){
            operandNum = operandAddress-(state->pc+1);
            operands[operandNum] = state->memory[operandAddress];
        }

        executeInstructionByOpcode(operation, operands, state);
    }else{
        logger("Error! Attempted to execute instruction outside of ROM!\n");
        exit(0);
    }
}

uint8_t *getVideoRAM(State8080 *state)
{
    uint8_t *vramContents = mallocSet(VRAM_SIZE_8080);

    memcpy(vramContents, &(state->memory[VRAM_START_ADDR_8080]), VRAM_SIZE_8080);

    return vramContents;
}

void generateInterrupt(uint8_t interruptNum, State8080 *state)
{
    if(interruptNum < 0x08){
        // Will need to trigger instruction RST n
        // n = interrupt num

        // Opcode for RST instructions is of form (11NNN111)b
        // NNN is the binary representation of the interrupt number
        uint8_t interruptOpcode = 0xc7 | (interruptNum << 3);  // 0xc7 == (11000111)b
        uint8_t fakeOperands[2] = {0xff, 0xff};

        if(state->interruptsEnabled){
            // RST instr will push PC+1 to the stack
            // Without this line, the instr pointed to by the current PC value will be skipped after the ISR is done.
            state->pc -= 1;

            executeInstructionByOpcode(interruptOpcode, fakeOperands, state);
        }
    }else{
        logger("Warning: Invalid interrupt attempted!\n");
    }
}

/**
 * Returns a pointer to the stored binary derived from an input FILE
 * @param romFile - pointer to FILE whose data should be stored
 * @return - uint8_t pointer to stored contents
 */
uint8_t *getRomBuffer(FILE *romFile)
{
    int romSizeInBytes = 0;
    uint8_t *romBuffer;

    // Get ROM size
    fseek(romFile, 0, SEEK_END);
    romSizeInBytes = ftell(romFile);
    fseek(romFile, 0, SEEK_SET);

    // Allocate memory for ROM
    romBuffer = mallocSet(romSizeInBytes);

    // Read ROM into buffer
    fread(romBuffer, 1, romSizeInBytes, romFile);

    return romBuffer;
}

/**
 * Given an opcode and operands, perform the resulting state changes of the 8080 CPU
 */
void executeInstructionByOpcode(uint8_t opcode, uint8_t *operands, State8080 *state)
{
    uint16_t orderedOperands = ((uint16_t)operands[1] << 8) | (uint16_t)operands[0];  // Convert from little-endian

    // variable declaration for usage in some cases of switch
    uint8_t tempL;  // A temporary place to hold the value of the L register
    uint8_t tempH;  // A temporary place to hold the value of the H register
    uint8_t tempA; // A temporary place to hold the Accumulator's value
    uint8_t subtrahend;
    uint8_t tempCarry;
    uint8_t memoryByte;
    uint8_t portNumber;
    uint8_t lowerNibble;
    uint8_t upperNibble;
    uint8_t flagsAsInt;
    uint16_t sourceAddress;
    uint16_t targetAddress;
    uint16_t oldMemValue;
    uint16_t newMemValue;

    if(DEBUG){
        logger("===\n");
        logger("%d:\n", numExec);
        logger("Operation: 0x%02x  %02x %02x\n", opcode, operands[0], operands[1]);
        logger("A: 0x%02x, B: 0x%02x, C: 0x%02x, D: 0x%02x, E: 0x%02x, H: 0x%02x, L: 0x%02x\n",
               state->a, state->b, state->c, state->d, state->e, state->h, state->l);
        logger("PC: 0x%04x, SP: 0x%04x, FLAGS (z,s,p,ac,cy): ", state->pc, state->sp);
        logger("%1x%1x%1x%1x%1x\n",
               state->flags.zero, state->flags.sign, state->flags.parity,
               state->flags.auxiliaryCarry, state->flags.carry);
        logger("Opcode: 0x%02x\n", opcode);
        logger("%s\n", instructions[opcode]);
        logger("%d\n", instructionSizes[opcode]);
        logger("%s\n", instructionFunctions[opcode]);
        logger("%s\n\n", instructionFlags[opcode]);
        logger("===\n");
    }

    switch(opcode){
        case 0x00:
            // NOP
            NOP(state);
            break;
        case 0x01: 
            // LXI B, D16
            // Load immediate into register pair BC
            LXI_RP(&(state->b), &(state->c), orderedOperands, state);
            break;
        case 0x02:
            // STAX B
            // Store accumulator in memory location (B)(C)
            // memory[(B)(C)] = A
            moveDataToBCMemory(state->a, state);
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x03: 
            // INX B
            // Increment register pair B C
            // (B)(C) = (B)(C) + 1
            INX_RP(&(state->b), &(state->c), state);
            break;
        case 0x04: 
            // INR B
            // Increment register B
            INR_R(&(state->b), state);
            break;
        case 0x05: 
            // DCR B
            // Decrement register B
            DCR_R(&(state->b), state);
            break;
        case 0x06: 
            // MVI B; D8
            // Move immediate to register B
            MVI_R(&(state->b), operands[0], state);
            break;
        case 0x07:
            // RLC
            // Rotate accumulator Right (and bypass Carry)
            // (As opposed to through Carry)
            // CY = A:7
            // A:n = A:(n-1); A:0 = A:7
            // Flags: CY
            state->flags.carry = ((state->a)&(0x80))>>7;  // CY = A:7
            state->a = (state->a)<<1;
            if(state->flags.carry == 1){
                state->a = state->a | 0x01;  // A:0 = 1
            }else{
                // A:0 = 0
            }
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x08: 
            // Unimplemented
            NOP(state);
            break;
        case 0x09: 
            // DAD B
            // Double-precision Add register pair BC to HL
            DAD_RP(state->b, state->c, state);
            break;
        case 0x0A:
            // LDAX B
            // Load Accumulator indirect from register pair B-C
            // A = memory[(B)(C)]
            sourceAddress = getValueBC(state);
            state->a = readMem(sourceAddress, state);
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x0B: 
            //DCX B
            // Decrement register pair B-C
            DCX_RP(&(state->b), &(state->c), state);
            break;
        case 0x0C: 
            // INR C
            // Increment register C
            // Flags: z,s,p,ac
            INR_R(&(state->c), state);
            break;
        case 0x0D: 
            // DCR C
            // Decrement register C
            // Flags: z,s,p,ac
            DCR_R(&(state->c), state);
            break;
        case 0x0E: 
            // MVI C, D8
            // Move Immediate to register C
            MVI_R(&(state->c), operands[0], state);
            break;
        case 0x0F: 
            // RRC
            // Rotate accumulator Right (and bypass Carry)
            // (As opposed to through Carry)
            // CY = A:0 
            // A:n = A:(n+1); A:7 = A:0
            // Flags: CY
            state->flags.carry = (state->a)&0x01;
            state->a = (state->a)>>1;
            if(state->flags.carry == 1){
                state->a = state->a | 0x80;  // A:7 = 1
            }else{
                // A:7 = 0
            }
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x10: 
            // Unimplemented
            NOP(state);
            break;
        case 0x11: 
            // LXI D, D16
            // Load Immediate into Register Pair D-E
            LXI_RP(&(state->d), &(state->e), orderedOperands, state);
            break;
        case 0x12:
            // STAX D
            // Store Accumulator in memory location (D)(E)
            // memory[(D)(E)] = A
            moveDataToDEMemory(state->a, state);
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x13: 
            // INX D
            // (D)(E) = (D)(E)+1
            INX_RP(&(state->d), &(state->e), state);
            break;
        case 0x14: 
            // INR D
            // Increment Register D
            // D = D + 1
            INR_R(&(state->d), state);
            break;
        case 0x15: 
            // DCR D
            // Decrement register D
            // D = D - 1
            DCR_R(&(state->d), state);
            break;
        case 0x16:
            // MVI D, D8
            // Move immediate into register D
            MVI_R(&(state->d), operands[0], state);
            break;
        case 0x17:
            // RAL
            // Rotate Accumulator Left through carry
            // A:n = A(n-1); CY = A:7; A:0 = CY
            tempCarry = state->flags.carry;
            state->flags.carry = ((state->a)&0x80)>>7;  // CY = A:7
            state->a = (state->a)<<1;  // rotate accumulator
            state->a = (state->a) | tempCarry;  // A:0 = old CY
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x18: 
            // Unimplemented
            NOP(state);
            break;
        case 0x19: 
            // DAD D
            // Double precision Add register pair DE to register pair HL
            DAD_RP(state->d, state->e, state);
            break;
        case 0x1A: 
            // LDAX D
            // Load Accumulator indirect from register pair D-E
            // A = memory[(D)(E)]
            sourceAddress = getValueDE(state);
            state->a = readMem(sourceAddress, state);
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x1B:
            // DCX D
            // Decrement register pair D-E
            DCX_RP(&(state->d), &(state->e), state);
            break;
        case 0x1C:
            // INR E
            // Increment register E
            INR_R(&(state->e), state);
            break;
        case 0x1D: 
            // DCR E
            // Decrement register E
            // E = E - 1
            // Flags: z,s,p,cy,ac
            DCR_R(&(state->e), state);
            break;
        case 0x1E:
            // MVI E, d8
            // Move immediate into register E
            MVI_R(&(state->e), operands[0], state);
            break;
        case 0x1F: 
            // RAR
            // Rotate Accumulator Right through carry
            // A:n = A:(n+1); CY = A:0; A:7 = CY
            tempCarry = state->flags.carry;
            state->flags.carry = (state->a) & 0x01;  // carry = bit 0
            state->a = (state->a)>>1;  // rotate Accumulator
            state->a = (state->a) | (tempCarry<<7);  // bit 7 = old carry
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x20: 
            // RIM
            // Read Interrupt Mask
            // This instruction is actually unimplemented on the 8080
            // The instruction is functional on the 8085
            // Equivalent to NOP
            NOP(state);
            break;
        case 0x21: 
            // LXI H, D16
            // Load Immediate into register pair H-L
            // H = byte 3; L = byte 2
            LXI_RP(&(state->h), &(state->l), orderedOperands, state);
            break;
        case 0x22: 
            // SHLD addr
            // Store H and L into memory directly
            // memory[addr] = L
            // memory[addr+1] = H
            writeMem(orderedOperands, state->l, state);
            writeMem(orderedOperands+1, state->h, state);
            state->pc += 3;
            state->cyclesCompleted += 16;
            break;
        case 0x23: 
            // INX H
            // (H)(L) = (H)(L)+1
            INX_RP(&state->h, &state->l, state);
            break;
        case 0x24:
            // INR H
            // Increment register H
            INR_R(&(state->h), state);
            break;
        case 0x25:
            // DCR H
            // Decrement register H
            DCR_R(&(state->h), state);
            break;
        case 0x26: 
            // MVI H, D8
            // MoVe Immediate into register H
            MVI_R(&(state->h), operands[0], state);
            break;
        case 0x27:
            // DAA
            // Decimal Adjust Accumulator
            // 1) IF Accumulator's lower nibble > 9
            //    OR auxiliary carry == 1
            //    THEN Accumulator is incremented by 6
            // 2) IF Accumulator's upper nibble > 9
            //    OR normal carry == 1
            //    THEN Accumulator's upper nibble is incremented by 6
            // Flags: z,s,p,cy,ac
            // IF a carry out of lower nibble occurs in step 1
            // THEN set auxiliary carry
            // ELSE reset auxiliary carry
            // IF a carry out of upper nibble occurs in step 2
            // THEN set carry
            // ELSE carry unaffected
            lowerNibble = state->a & 0x0f;
            // Step 1
            if(lowerNibble > 9 || state->flags.auxiliaryCarry == 1){
                state->a = (uint8_t)addWithCheckAC(state->a, 0x06, state);
            }else{
                state->flags.auxiliaryCarry = 0;
            }
            // Step 2
            upperNibble = (state->a)>>4;
            if(upperNibble > 9 || state->flags.carry == 1){
                upperNibble += 6;
                // Perform carry check
                if(upperNibble > 0x0f){
                    state->flags.carry = 1;
                }else{
                    // Carry unaffected
                }
                // Place upper nibble back into Accumulator
                upperNibble = upperNibble<<4;
                state->a = state->a & 0x0f;
                state->a = state->a | upperNibble;
            }
            // Do standard arithmetic instruction stuff
            checkStandardArithmeticFlags(state->a, state);
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x28: 
            // Unimplemented
            NOP(state);
            break;
        case 0x29: 
            // DAD H
            // Double-precision Add HL to HL
            DAD_RP(state->h, state->l, state);
            break;
        case 0x2A:
            // LHLD addr
            // Load memory at address into H and L directly
            // L = memory[addr]
            // H = memory[addr+1]
            state->l = readMem(orderedOperands, state);
            state->h = readMem(orderedOperands+1, state);
            state->pc += 3;
            state->cyclesCompleted += 16;
            break;
        case 0x2B:
            // DCX H
            // Decrement HL
            DCX_RP(&(state->h), &(state->l), state);
            break;
        case 0x2C:
            // INR L
            // Increment register L
            INR_R(&(state->l), state);
            break;
        case 0x2D:
            // DCR L
            // Decrement register L
            DCR_R(&(state->l), state);
            break;
        case 0x2E: 
            // MVI L, d8
            // Move immediate into register L
            MVI_R(&(state->l), operands[0], state);
            break;
        case 0x2F:
            // CMA
            // Complement Accumulator
            // A = !A
            state->a = ~(state->a);
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x30: 
            // SIM
            // Set Interrupt Mask
            // Unimplemented on 8080, implemented on 8085
            // Effectively NOP
            NOP(state);
            break;
        case 0x31: 
            // LXI SP, D16
            // Load Immediate into Stack Pointer
            state->sp = orderedOperands;
            state->pc += 3;
            state->cyclesCompleted += 10;
            break;
        case 0x32: 
            // STA addr
            // STore Accumulator directly in memory address
            // memory[address] = A
            writeMem(orderedOperands, state->a, state);
            state->pc += 3;
            state->cyclesCompleted += 13;
            break;
        case 0x33:
            // INX SP
            // Increment Stack Pointer
            state->sp += 1;
            state->pc += 1;
            state->cyclesCompleted += 5;
            break;
        case 0x34: 
            // INR M
            // Increment Memory
            // memory[(H)(L)] = memory[(H)(L)] + 1
            // Flags: z,s,p,cy,ac
            memoryByte = 0;
            moveDataFromHLMemory(&memoryByte, state);
            addWithCheckAC(memoryByte, 1, state);  // Do not store result, just check AC
            memoryByte = addWithCheckCY(memoryByte, 1, state);
            checkStandardArithmeticFlags(memoryByte, state);
            moveDataToHLMemory(memoryByte, state);
            state->pc += 1;
            state->cyclesCompleted += 10;
            break;
        case 0x35:
            // DCR M
            // Decrement memory
            // memory[(H)(L)] = memory[(H)(L)] - 1
            // Flags: z,s,p,ac
            targetAddress = getValueHL(state);
            oldMemValue = readMem(targetAddress, state);
            newMemValue = addWithCheckAC(oldMemValue, (uint8_t)(-1), state);
            writeMem(targetAddress, newMemValue, state);
            checkStandardArithmeticFlags(newMemValue, state);
            state->pc += 1;
            state->cyclesCompleted += 10;
            break;
        case 0x36: 
            // MVI M; D8
            // Move 8-bit immediate to memory
            // memory[(H)(L)] = D8
            moveDataToHLMemory(operands[0], state);
            state->pc += 2;
            state->cyclesCompleted += 10;
            break;
        case 0x37: 
            // STC
            // Set Carry flag
            state->flags.carry = 1;
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x38: 
            // Unimplemented
            NOP(state);
            break;
        case 0x39: 
            // DAD SP
            // Double Precision Add Stack Pointer to HL
            // (H)(L) = (H)(L) + SP
            DAD_RP((state->sp)>>8, (state->sp)&0x00ff, state);  // Separate SP into high and low bits
            break;
        case 0x3A: 
            // LDA addr
            // Load memory directly into Accumulator
            // A = address
            state->a = readMem(orderedOperands, state);
            state->pc += 3;
            state->cyclesCompleted += 13;
            break;
        case 0x3B: 
            // DCX SP
            // Decrement stack pointer
            state->sp -= 1;
            state->pc += 1;
            state->cyclesCompleted += 5;
            break;
        case 0x3C: 
            // INR A
            // Increment Accumulator
            INR_R(&(state->a), state);
            break;
        case 0x3D: 
            // DCR A
            // Decrement Accumulator
            // A = A - 1
            DCR_R(&(state->a), state);
            break;
        case 0x3E: 
            // MVI A, D8
            // Move Immediate into register A
            // A = D8
            MVI_R(&(state->a), operands[0], state);
            break;
        case 0x3F:
            // CMC
            // Complement Carry
            // CY = !CY
            state->flags.carry = ~(state->flags.carry);
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0x40:
            // MOV B, B
            // Move register B into register B
            MOV_R1_R2(&(state->b), &(state->b), state);
            break;
        case 0x41:
            // MOV B, C
            // Move the content of register C into register B
            MOV_R1_R2(&(state->b), &(state->c), state);
            break;
        case 0x42:
            // MOV B, D
            // Move the content of register D into register B
            MOV_R1_R2(&(state->b), &(state->d), state);
            break;
        case 0x43:
            // MOV B, E
            // Move the content of register E into register B
            MOV_R1_R2(&(state->b), &(state->e), state);
            break;
        case 0x44:
            // MOV B, H
            // Move the content of register H into register B
            MOV_R1_R2(&(state->b), &(state->h), state);
            break;
        case 0x45: 
            // MOV B, L
            // Move the content of register L into register B
            MOV_R1_R2(&(state->b), &(state->l), state);
            break;
        case 0x46:
            // MOV B, M
            // Move from memory into register B
            // B = memory[(H)(L)]
            MOV_R_M(&(state->b), state);
            break;
        case 0x47: 
            // MOV B, A
            // Move content of Accumulator into register B
            MOV_R1_R2(&(state->b), &(state->a), state);
            break;
        case 0x48:
            // MOV C, B
            MOV_R1_R2(&(state->c), &(state->b), state);
            break;
        case 0x49: 
            // MOV C, C
            MOV_R1_R2(&(state->c), &(state->c), state);
            break;
        case 0x4A:
            // MOV C, D
            MOV_R1_R2(&(state->c), &(state->d), state);
            break;
        case 0x4B: 
            // MOV C, E
            MOV_R1_R2(&(state->c), &(state->e), state);
            break;
        case 0x4C:
            // MOV C, H
            MOV_R1_R2(&(state->c), &(state->h), state);
            break;
        case 0x4D:
            // MOV C, L
            MOV_R1_R2(&(state->c), &(state->l), state);
            break;
        case 0x4E: 
            // MOV C, M
            // Move from memory into register C
            MOV_R_M(&(state->c), state);
            break;
        case 0x4F:
            // MOV C, A
            // Move the contents of register A into register C
            MOV_R1_R2(&(state->c), &(state->a), state);
            break;
        case 0x50:
            // MOV D, B
            MOV_R1_R2(&(state->d), &(state->b), state);
            break;
        case 0x51:
            // MOV D, C
            MOV_R1_R2(&(state->d), &(state->c), state);
            break;
        case 0x52: 
            // MOV D, D
            MOV_R1_R2(&(state->d), &(state->d), state);
            break;
        case 0x53: 
            // MOV D, E
            MOV_R1_R2(&(state->d), &(state->e), state);
            break;
        case 0x54: 
            // MOV D, H
            // Move the contents of register H into register D
            // D = H
            MOV_R1_R2(&(state->d), &(state->h), state);
            break;
        case 0x55: 
            // MOV D, L
            MOV_R1_R2(&(state->d), &(state->l), state);
            break;
        case 0x56: 
            // MOV D, M
            // Move from memory into register D
            // D = memory[(H)(L)]
            MOV_R_M(&(state->d), state);
            break;
        case 0x57: 
            // MOV D, A
            MOV_R1_R2(&(state->d), &(state->a), state);
            break;
        case 0x58: 
            // MOV E, B
            MOV_R1_R2(&(state->e), &(state->b), state);
            break;
        case 0x59: 
            // MOV E, C
            MOV_R1_R2(&(state->e), &(state->c), state);
            break;
        case 0x5A: 
            // MOV E, D
            MOV_R1_R2(&(state->e), &(state->d), state);
            break;
        case 0x5B: 
            // MOV E, E
            MOV_R1_R2(&(state->e), &(state->e), state);
            break;
        case 0x5C: 
            // MOV E, H
            MOV_R1_R2(&(state->e), &(state->h), state);
            break;
        case 0x5D: 
            // MOV E, L
            MOV_R1_R2(&(state->e), &(state->l), state);
            break;
        case 0x5E: 
            // MOV E, M
            // Move from memory into register E
            // E = memory[(H)(L)]
            MOV_R_M(&(state->e), state);
            break;
        case 0x5F:
            // MOV E, A
            // Move contents of accumulator into register E
            MOV_R1_R2(&(state->e), &(state->a), state);
            break;
        case 0x60:
            // MOV H, B
            MOV_R1_R2(&(state->h), &(state->b), state);
            break;
        case 0x61: 
            // MOV H, C
            // Move the content of register C into register H
            MOV_R1_R2(&(state->h), &(state->c), state);
            break;
        case 0x62: 
            // MOV H, D
            MOV_R1_R2(&(state->h), &(state->d), state);
            break;
        case 0x63: 
            // MOV H, E
            MOV_R1_R2(&(state->h), &(state->e), state);
            break;
        case 0x64: 
            // MOV H, H
            MOV_R1_R2(&(state->h), &(state->h), state);
            break;
        case 0x65: 
            // MOV H, L
            MOV_R1_R2(&(state->h), &(state->l), state);
            break;
        case 0x66: 
            // MOV H, M
            // Move from memory into register H
            // H = memory[(H)(L)]
            MOV_R_M(&(state->h), state);
            break;
        case 0x67: 
            // MOV H, A
            // Move Accumulator content into register H
            MOV_R1_R2(&(state->h), &(state->a), state);
            break;
        case 0x68: 
            // MOV L, B
            // Move content of register B into register L
            MOV_R1_R2(&(state->l), &(state->b), state);
            break;
        case 0x69: 
            // MOV L, C
            MOV_R1_R2(&(state->l), &(state->c), state);
            break;
        case 0x6A: 
            // MOV L, D
            MOV_R1_R2(&(state->l), &(state->d), state);
            break;
        case 0x6B:
            // MOV L, E
            MOV_R1_R2(&(state->l), &(state->e), state);
            break;
        case 0x6C: 
            // MOV L, H
            // Move the content of register H to register L
            MOV_R1_R2(&(state->l), &(state->h), state);
            break;
        case 0x6D:
            // MOV L, L
            MOV_R1_R2(&(state->l), &(state->l), state);
            break;
        case 0x6E: 
            // MOV L, M
            // Move the content of register M into register L
            // L = M
            MOV_R_M(&(state->l), state);
            break;
        case 0x6F: 
            // MOV L, A
            // Move the contents of register A into register L
            MOV_R1_R2(&(state->l), &(state->a), state);
            break;
        case 0x70: 
            // MOV M, B
            // Move the contents of register B into memory
            MOV_M_R(state->b, state);
            break;
        case 0x71: 
            // MOV M, C
            // Move content of register C into Memory
            // memory[(H)(L)] = C
            MOV_M_R(state->c, state);
            break;
        case 0x72:
            // MOV M, D
            // Move content of register D into memory
            MOV_M_R(state->d, state);
            break;
        case 0x73:
            // MOV M, E
            // Move content of register E into memory
            MOV_M_R(state->e, state);
            break;
        case 0x74:
            // MOV M, H
            // Move content of register H into memory
            MOV_M_R(state->h, state);
            break;
        case 0x75:
            // MOV M, L
            // Move content of register L into memory
            MOV_M_R(state->l, state);
            break;
        case 0x76: 
            // HLT
            // Halt
            // The program counter is incremented to
            // the address of the next sequential instruction. The CPU then
            // enters the STOPPED state and no further activity takes
            // place until an interrupt occurs.
            // TODO: Implement this?
            NOP(state);
            break;
        case 0x77: 
            // MOV M, A
            // memory[(H)(L)] = A
            MOV_M_R(state->a, state);
            break;
        case 0x78:
            // MOV A, B
            // Move contents of register B into register A
            MOV_R1_R2(&(state->a), &(state->b), state);
            break;
        case 0x79:
            // MOV A, C
            // Move contents of register C into register A
            MOV_R1_R2(&(state->a), &(state->c), state);
            break;
        case 0x7A: 
            // MOV A, D
            // Move contents of register D into register A
            MOV_R1_R2(&(state->a), &(state->d), state);
            break;
        case 0x7B: 
            // MOV A, E
            // Move the contents of register E into register A
            MOV_R1_R2(&(state->a), &(state->e), state);
            break;
        case 0x7C: 
            // MOV A, H
            // A = H
            MOV_R1_R2(&(state->a), &(state->h), state);
            break;
        case 0x7D: 
            // MOV A, L
            // Move the content of register L into register A
            // A = L
            MOV_R1_R2(&(state->a), &(state->l), state);
            break;
        case 0x7E: 
            // MOV A, M
            // Move from memory into register A
            // A = memory[(H)(L)]
            MOV_R_M(&(state->a), state);
            break;
        case 0x7F: 
            // MOV A, A
            // Effectively a NOP
            MOV_R1_R2(&(state->a), &(state->a), state);
            break;
        case 0x80: 
            // ADD B
            // ADD B to Accumulator
            ADD_R(state->b, state);
            break;
        case 0x81:
            // ADD C
            // Add C to Accumulator
            ADD_R(state->c, state);
            break;
        case 0x82:
            // ADD D
            // ADD D to Accumulator
            ADD_R(state->d, state);
            break;
        case 0x83:
            // ADD E
            // ADD E to Accumulator
            ADD_R(state->e, state);
            break;
        case 0x84:
            // ADD H
            // ADD H to Accumulator
            ADD_R(state->h, state);
            break;
        case 0x85:
            // ADD L
            // ADD L to Accumulator
            ADD_R(state->l, state);
            break;
        case 0x86:
            // ADD M
            // Add memory to Accumulator
            // A = A + memory[(H)(L)]
            // Flags: z,s,p,cy,ac
            tempA = state->a;
            moveDataFromHLMemory(&(state->a), state);  // Accumulator has memory value now
            addWithCheckAC(state->a, tempA, state);  // Do not store value, just for flag check
            state->a = addWithCheckCY(state->a, tempA, state);
            checkStandardArithmeticFlags(state->a, state);
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x87:
            // ADD A
            // ADD Accumulator to Accumulator
            ADD_R(state->a, state);
            break;
        case 0x88: 
            // ADC B
            // Add B to Accumulator with Carry
            ADC_R(state->b, state);
            break;
        case 0x89:
            // ADC C
            // Add C to Accumulator with Carry
            ADC_R(state->c, state);
            break;
        case 0x8A:
            // ADC D
            // Add D to Accumulator with Carry
            ADC_R(state->d, state);
            break;
        case 0x8B:
            // ADC E
            // Add E to Accumulator with Carry
            ADC_R(state->e, state);
            break;
        case 0x8C:
            // ADC H
            // Add H to Accumulator with Carry
            ADC_R(state->h, state);
            break;
        case 0x8D:
            // ADC L
            // Add L to Accumulator with Carry
            ADC_R(state->l, state);
            break;
        case 0x8E:
            // ADC M
            // Add memory to Accumulator with Carry
            // A = A + memory[(H)(L)] + CY
            // Flags: z,s,p,cy,ac
            moveDataFromHLMemory(&memoryByte, state);
            if(memoryByte == 0xff && state->flags.carry == 1){
                // Need to explicitly set carry, as this case will cause overflow in the addend
                // Do not need to store value as we are adding 0
                addWithCheckAC(state->a, 0x00, state);
                checkStandardArithmeticFlags(state->a, state);
                state->flags.carry = 1;
            }else{
                addWithCheckAC(state->a, memoryByte+1, state);  // Do not store value, just check flag
                state->a = addWithCheckCY(state->a, memoryByte+1, state);
                checkStandardArithmeticFlags(state->a, state);
            }
            state->pc += 1;
            state->cyclesCompleted += 7;
            break;
        case 0x8F:
            // ADC A
            // Add Accumulator to Accumulator with Carry
            ADC_R(state->a, state);
            break;
        case 0x90: 
            // SUB B
            // Subtract B from Accumulator
            SUB_R(state->b, state);
            break;
        case 0x91: 
            // SUB C
            // Subtract C from Accumulator
            SUB_R(state->c, state);
            break;
        case 0x92:
            // SUB D
            // Subtract D from Accumulator
            SUB_R(state->d, state);
            break;
        case 0x93:
            // SUB E
            // Subtract E from Accumulator
            SUB_R(state->e, state);
            break;
        case 0x94:
            // SUB H
            // Subtract H from Accumulator
            SUB_R(state->h, state);
            break;
        case 0x95:
            // SUB L
            // Subtract L from Accumulator
            SUB_R(state->l, state);
            break;
        case 0x96:
            // SUB M
            // Subtract Memory from Accumulator
            // A = A - memory[(H)(L)]
            moveDataFromHLMemory(&memoryByte, state);
            SUB_R(memoryByte, state);
            state->cyclesCompleted += 3;
            break;
        case 0x97:
            // SUB A
            // Subtract Accumulator from Accumulator
            SUB_R(state->a, state);
            break;
        case 0x98: 
            // SBB B
            // Subtract B from accumulator with borrow
            SBB_R(state->b, state);
            break;
        case 0x99:
            // SBB C
            // Subtract C from accumulator with borrow
            SBB_R(state->c, state);
            break;
        case 0x9A:
            // SBB D
            // Subtract D from accumulator with borrow
            SBB_R(state->d, state);
            break;
        case 0x9B:
            // SBB E
            // Subtract E from accumulator with borrow
            SBB_R(state->e, state);
            break;
        case 0x9C:
            // SBB H
            // Subtract H from accumulator with borrow
            SBB_R(state->h, state);
            break;
        case 0x9D:
            // SBB L
            // Subtract L from accumulator with borrow
            SBB_R(state->l, state);
            break;
        case 0x9E:
            // SBB M
            // Subtract Memory from accumulator with borrow
            // A = A - (memory[(H)(L)] + CY)
            moveDataFromHLMemory(&memoryByte, state);
            SBB_R(memoryByte, state);
            state->cyclesCompleted += 3;
            break;
        case 0x9F:
            // SBB A
            // Subtract A from accumulator with borrow
            SBB_R(state->a, state);
            break;
        case 0xA0: 
            // ANA B
            // AND Accumulator with Register B
            // A = A & B
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->b, state);
            break;
        case 0xA1: 
            // ANA C
            // AND Accumulator with C
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->c, state);
            break;
        case 0xA2:
            // ANA D
            // AND Accumulator with D
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->d, state);
            break;
        case 0xA3:
            // ANA E
            // AND Accumulator with E
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->e, state);
            break;
        case 0xA4:
            // ANA H
            // AND Accumulator with H
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->h, state);
            break;
        case 0xA5:
            // ANA L
            // AND Accumulator with
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->l, state);
            break;
        case 0xA6:
            // ANA M
            // AND Accumulator with memory
            // A = A AND memory[(H)(L)]
            // Flags: z,s,p,cy(reset),ac
            moveDataFromHLMemory(&memoryByte, state);
            ANA_R(memoryByte, state);
            state->cyclesCompleted += 3;
            break;
        case 0xA7:
            // ANA A
            // AND Accumulator with Accumulator
            // A = A & A
            // Flags: z,s,p,cy(reset),ac
            ANA_R(state->a, state);
            break;
        case 0xA8: 
            // XRA B
            // Exclusive OR Accumulator with Register B
            // A = A XOR B
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->b, state);
            break;
        case 0xA9:
            // XRA C
            // Exclusive OR Accumulator with Register C
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->c, state);
            break;
        case 0xAA:
            // XRA D
            // Exclusive OR Accumulator with Register D
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->d, state);
            break;
        case 0xAB:
            // XRA E
            // Exclusive OR Accumulator with Register E
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->e, state);
            break;
        case 0xAC:
            // XRA H
            // Exclusive OR Accumulator with Register H
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->h, state);
            break;
        case 0xAD:
            // XRA L
            // Exclusive OR Accumulator with Register L
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->l, state);
            break;
        case 0xAE: 
            // XRA M
            // XOR Memory with Accumulator
            // A = A XOR memory[(H)(L)]
            // Flags: z,s,p,cy(reset),ac(reset)
            moveDataFromHLMemory(&memoryByte, state);
            XRA_R(memoryByte, state);
            state->cyclesCompleted += 3;
            break;
        case 0xAF: 
            // XRA A
            // Exclusive Or register A with register A
            // A = A XOR A
            // Flags: z,s,p,cy(reset),ac(reset);
            XRA_R(state->a, state);
            break;
        case 0xB0:
            // ORA B
            // OR Accumulator with register B
            ORA_R(state->b, state);
            break;
        case 0xB1:
            // ORA C
            // OR Accumulator with register C
            ORA_R(state->c, state);
            break;
        case 0xB2:
            // ORA D
            // OR Accumulator with register D
            ORA_R(state->d, state);
            break;
        case 0xB3:
            // ORA E
            // OR Accumulator with register E
            ORA_R(state->e, state);
            break;
        case 0xB4: 
            // ORA H
            // OR Accumulator with register H
            ORA_R(state->h, state);
            break;
        case 0xB5:
            // ORA L
            // OR Accumulator with register L
            ORA_R(state->l, state);
            break;
        case 0xB6:
            // ORA M
            // OR Accumulator with Memory
            // A = A OR memory[(H)(L)]
            // Flags: z,s,p,cy(reset),ac(reset)
            moveDataFromHLMemory(&memoryByte, state);
            ORA_R(memoryByte, state);
            state->cyclesCompleted += 3;
            break;
        case 0xB7:
            // ORA A
            // OR Accumulator with register A
            ORA_R(state->a, state);
            break;
        case 0xB8: 
            // CMP B
            // Compare register B with accumulator
            CMP_R(state->b, state);
            break;
        case 0xB9:
            // CMP C
            // Compare register C with accumulator
            CMP_R(state->c, state);
            break;
        case 0xBA:
            // CMP D
            // Compare register d with accumulator
            CMP_R(state->d, state);
            break;
        case 0xBB:
            // CMP E
            // Compare register E with accumulator
            CMP_R(state->e, state);
            break;
        case 0xBC:
            // CMP H
            // Compare register H with accumulator
            CMP_R(state->h, state);
            break;
        case 0xBD:
            // CMP L
            // Compare register L with accumulator
            CMP_R(state->l, state);
            break;
        case 0xBE: 
            // CMP M
            // Compare Memory with Accumulator
            // A - memory[(H)(L)]
            // Flags: z,s,p,cy,ac
            moveDataFromHLMemory(&subtrahend, state);
            CMP_R(subtrahend, state);
            state->cyclesCompleted += 3;
            break;
        case 0xBF:
            // CMP A
            // Compare accumulator with accumulator
            CMP_R(state->a, state);
            break;
        case 0xC0:
            // RNZ
            // Return if Not Zero
            if(!(state->flags.zero)){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xC1: 
            // POP B
            // Pop from stack into register pair BC
            POP_RP(&state->b, &state->c, state);
            break;
        case 0xC2: 
            // JNZ addr
            // if NZ, PC = addr
            if(!(state->flags.zero)){
                JMP(orderedOperands, state);
			}else{
                state->pc += 3;
                state->cyclesCompleted += 10;
			}
            break;
        case 0xC3: 
            // JMP adr - JUMP
            JMP(orderedOperands, state);
            break;
        case 0xC4: 
            // CNZ addr
            // Call address if not zero
            // if NZ; Call addr
            if(!(state->flags.zero)){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xC5: 
            // PUSH B
            // Push register pair BC onto the stack
            PUSH_RP(state->b, state->c, state);
            break;
        case 0xC6: 
            // ADI D8
            // Add Immediate to Accumulator
            // A = A + D8
            // Flags: z,s,p,cy,ac
            addWithCheckAC(state->a, operands[0], state);
            state->a = addWithCheckCY(state->a, operands[0], state);
            checkStandardArithmeticFlags(state->a, state);
            state->pc += 2;
            state->cyclesCompleted += 7;
            break;
        case 0xC7:
            // RST 0
            RST(0, state);
            break;
        case 0xC8:
            // RZ
            // Return if Zero
            if(state->flags.zero){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xC9: 
            // RET
            RET(state);
            break;
        case 0xCA: 
            // JZ addr
            // Jump to address if zero (flag set)
            // if Z, PC=addr
            if(state->flags.zero){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xCB: 
            // Unimplemented
            NOP(state);
            break;
        case 0xCC: 
            // CZ addr
            // Call address if zero
            if(state->flags.zero){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xCD:
            // CALL addr
            #ifdef CPU_DIAG
            // Code snippet taken from http://www.emulator101.com/full-8080-emulation.html
            if(orderedOperands == 0x5){
                if (state->c == 9){
                    uint16_t offset = ((uint16_t)(state->d)<<8) | (state->e);
                    char *str = (char*)(&(state->memory[offset+3]));  //skip the prefix bytes
                    while (*str != '$'){
                        logger("%c", *str++);
                    }
                    logger("\n");
                }else if(state->c == 2){
                    // saw this in the inspected code, never saw it called
                    logger("print char routine called\n");
                }
            }else if (orderedOperands == 0){
                exit(0);
            }else{
                CALL(orderedOperands, state);
            }
            #endif
            #ifndef CPU_DIAG
            CALL(orderedOperands, state);
            #endif
            break;
        case 0xCE: 
            // ACI d8
            // Add immediate to Accumulator with carry
            // A = d8 + CY
            // Flags: z,s,p,cy,ac
            ADC_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xCF:
            // RST 1
            RST(1, state);
            break;
        case 0xD0:
            // RNC
            // Return if No Carry
            if(!(state->flags.carry)){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xD1: 
            // POP D
            // Pop register pair D-E from stack
            POP_RP(&state->d, &state->e, state);
            break;
        case 0xD2:
            // JNC Addr
            // Jump to address if no Carry
            // if NC, JMP addr
            if(!(state->flags.carry)){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xD3: 
            // OUT D8
            // Content of register A placed on 8-bit bi-directional data bus
            // for transmission to the port specified by D8
            // (data) = A
            portNumber = operands[0];
            state->outputBuffers[portNumber] = state->a;
            state->pc += 2;
            state->cyclesCompleted += 10;
            break;
        case 0xD4: 
            // CNC adr
            // Call address if No Carry
            if(!(state->flags.carry)){
                CALL(orderedOperands, state);
		    }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
		    }
            break;
        case 0xD5: 
            // PUSH D
            // PUSH register pair D-E
            PUSH_RP(state->d, state->e, state);
            break;
        case 0xD6:
            // SUI d8
            // Subtract immediate from Accumulator
            // A = A - d8
            // Flags: z,s,p,cy,ac
            SUB_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xD7:
            // RST 2
            RST(2, state);
            break;
        case 0xD8: 
            // RC
            // Return if Carry
            if(state->flags.carry){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xD9: 
            // Unimplemented
            NOP(state);
            break;
        case 0xDA:
            // JC addr
            // Jump to address if carry
            // if cy; pc = addr
            if(state->flags.carry){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xDB: 
            // IN d8
            // Read from input port
            // A = data
            portNumber = operands[0];
            state->a = state->inputBuffers[portNumber];
            state->pc += 2;
            state->cyclesCompleted += 10;
            break;
        case 0xDC: 
            // CC addr
            // Call if Carry
            // if CY, CALL addr
            if(state->flags.carry){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xDD: 
            // Unimplemented
            NOP(state);
            break;
        case 0xDE: 
            // SBI D8
            // Subtract immediate from Accumulator with borrow
            // A = A - (D8 + CY)
            // Flags: z,s,p,cy,ac
            SBB_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xDF:
            // RST 3
            RST(3, state);
            break;
        case 0xE0: 
            // RPO
            // Return if Parity ODD
            // If PO, RET
            if(!(state->flags.parity)){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xE1: 
            // POP H
            // POP from stack into register pair HL
            POP_RP(&(state->h), &(state->l), state);
            break;
        case 0xE2: 
            // JPO addr
            // Jump if Parity Odd
            // if PO, JMP addr
            if(!(state->flags.parity)){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xE3:
            // XTHL
            // Exchange stack top with H and L
            // L <-> memory[SP]
            // H <-> memory[SP+1]
            tempL = state->l;
            tempH = state->h;
            state->l = readMem(state->sp, state);
            state->h = readMem((state->sp)+1, state);
            writeMem(state->sp, tempL, state);
            writeMem((state->sp)+1, tempH, state);
            state->pc += 1;
            state-> cyclesCompleted += 18;
            break;
        case 0xE4: 
            // CPO addr
            // Call if Parity Odd
            // if PO, CALL addr
            if(!(state->flags.parity)){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xE5: 
            // PUSH H
            // Push register pair H-L onto the stack
            PUSH_RP(state->h, state->l, state);
            break;
        case 0xE6: 
            // ANI D8
            // AND Accumulator with Immediate
            // A = A AND D8
            //Flags: z,s,p,cy(reset),ac(reset)
            ANA_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xE7:
            // RST 4
            RST(4, state);
            break;
        case 0xE8: 
            // RPE
            // Return if Parity Even
            // if PE, RET
            if(state->flags.parity){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xE9:
            // PCHL
            // Jump to address (H)(L) by moving (H)(L) to PC
            // PCH = H
            // PCL = L
            state->pc = getValueHL(state);
            state->cyclesCompleted += 5;
            break;
        case 0xEA: 
            // JPE addr
            // Jump if parity is even
            // if p, then JMP addr
            if(state->flags.parity){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xEB: 
            // XCHG
            // eXCHanGe HL with DE
            // H = D; D = H 
            // L = E; E = L
            tempH = state->h;
            tempL = state->l;
            
            state->h = state->d;
            state->l = state->e;
            state->d = tempH;
            state->e = tempL;
            
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0xEC:
            // CPE addr
            // Call if Parity Even
            if(state->flags.parity){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xED: 
            // Unimplemented
            NOP(state);
            break;
        case 0xEE: 
            // XRI d8
            // Exclusive OR immediate with Accumulator
            // A = A XOR d8
            // Flags: z,s,p,cy(reset),ac(reset)
            XRA_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xEF:
            // RST 5
            RST(5, state);
            break;
        case 0xF0: 
            // RP
            // Return if Positive
            // if Pos, RET
            if(!(state->flags.sign)){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xF1: 
            // POP PSW
            // POP the Processor Status Word (and accumulator) off the stack
            // flags = memory[sp]; A = memory[sp+1]
            // sp = sp + 2
            POP_RP(&(state->a), (uint8_t*)&(state->flags), state);  // Treat flags as 8-bit uint to match function signature
            break;
        case 0xF2: 
            // JP addr
            // Jump if Positive
            // If pos, JMP addr
            if(!(state->flags.sign)){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xF3: 
            // DI
            // Disable interrupts
            state->interruptsEnabled = 0;
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0xF4: 
            // CP addr
            // Call address if Positive
            // If pos, CALL addr
            if(!(state->flags.sign)){
                CALL(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xF5: 
            // PUSH PSW
            // Push Processor Status Word (and accumulator) onto stack
            flagsAsInt = *(uint8_t*)&(state->flags);  // Can't use ConditionCodes struct directly
            PUSH_RP(state->a, flagsAsInt, state);
            break;
        case 0xF6: 
            // ORI d8
            // OR immediate with Accumulator
            // A = A OR d8
            // Flags: z,s,p,cy(reset),ac(reset)
            ORA_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xF7:
            // RST 6
            RST(6, state);
            break;
        case 0xF8: 
            // RM
            // Return if minus
            // If S, RET
            if(state->flags.sign){
                RET(state);
                state->cyclesCompleted += 1;
            }else{
                state->pc += 1;
                state->cyclesCompleted += 5;
            }
            break;
        case 0xF9: 
            // SPHL
            // Set Stack Pointer to Register Pair H-L
            // SP.hi = H
            // SP.lo = L
            state->sp = getValueHL(state);
            state->pc += 1;
            state->cyclesCompleted += 5;
            break;
        case 0xFA: 
            // JM Addr
            // Jump to address if minus
            // if M, pc = addr
            if(state->flags.sign){
                JMP(orderedOperands, state);
            }else{
                state->pc += 3;
                state->cyclesCompleted += 10;
            }
            break;
        case 0xFB: 
            // EI
            // Enable Interrupt
            state->interruptsEnabled = 1;
            state->pc += 1;
            state->cyclesCompleted += 4;
            break;
        case 0xFC: 
            // CM addr
            // Call address if Minus
            // If S, CALL addr
            if(state->flags.sign){
                CALL(orderedOperands, state);
            }else {
                state->pc += 3;
                state->cyclesCompleted += 11;
            }
            break;
        case 0xFD: 
            // Unimplemented
            NOP(state);
            break;
        case 0xFE: 
            // CPI D8
            // Compare Immediate
            // A - D8
            // Flags: z,s,p,cy,ac
            // Note: result should not be stored anywhere, this just affects flags
            // System manual does not explicitly state this, but programmer's manual does
            CMP_R(operands[0], state);
            state->pc += 1;
            state->cyclesCompleted += 3;
            break;
        case 0xFF:
            // RST 7
            RST(7, state);
            break;
	}

	numExec++;
}

/**
* Initialize global variables.
* 
* Do this here because the initializations are about 1000 lines.
* This could be done elsewhere, but it would clutter the top of the file.
* Delegate initialization to a function so it can happily remain at the bottom of the file.
*
* @return void
*/
void initializeGlobals()
{
    char instructionsLocal[256][20] = {
        "NOP",
        "LXI B;D16",
        "STAX B",
        "INX B",
        "INR B",
        "DCR B",
        "MVI B; D8",
        "RLC",
        "-",
        "DAD B",
        "LDAX B",
        "DCX B",
        "INR C",
        "DCR C",
        "MVI C;D8",
        "RRC",
        "-",
        "LXI D;D16",
        "STAX D",
        "INX D",
        "INR D",
        "DCR D",
        "MVI D; D8",
        "RAL",
        "-",
        "DAD D",
        "LDAX D",
        "DCX D",
        "INR E",
        "DCR E",
        "MVI E;D8",
        "RAR",
        "RIM",
        "LXI H;D16",
        "SHLD adr",
        "INX H",
        "INR H",
        "DCR H",
        "MVI H;D8",
        "DAA",
        "-",
        "DAD H",
        "LHLD adr",
        "DCX H",
        "INR L",
        "DCR L",
        "MVI L; D8",
        "CMA",
        "SIM",
        "LXI SP; D16",
        "STA adr",
        "INX SP",
        "INR M",
        "DCR M",
        "MVI M;D8",
        "STC",
        "-",
        "DAD SP",
        "LDA adr",
        "DCX SP",
        "INR A",
        "DCR A",
        "MVI A;D8",
        "CMC",
        "MOV B;B",
        "MOV B;C",
        "MOV B;D",
        "MOV B;E",
        "MOV B;H",
        "MOV B;L",
        "MOV B;M",
        "MOV B;A",
        "MOV C;B",
        "MOV C;C",
        "MOV C;D",
        "MOV C;E",
        "MOV C;H",
        "MOV C;L",
        "MOV C;M",
        "MOV C;A",
        "MOV D;B",
        "MOV D;C",
        "MOV D;D",
        "MOV D;E",
        "MOV D;H",
        "MOV D;L",
        "MOV D;M",
        "MOV D;A",
        "MOV E;B",
        "MOV E;C",
        "MOV E;D",
        "MOV E;E",
        "MOV E;H",
        "MOV E;L",
        "MOV E;M",
        "MOV E;A",
        "MOV H;B",
        "MOV H;C",
        "MOV H;D",
        "MOV H;E",
        "MOV H;H",
        "MOV H;L",
        "MOV H;M",
        "MOV H;A",
        "MOV L;B",
        "MOV L;C",
        "MOV L;D",
        "MOV L;E",
        "MOV L;H",
        "MOV L;L",
        "MOV L;M",
        "MOV L;A",
        "MOV M;B",
        "MOV M;C",
        "MOV M;D",
        "MOV M;E",
        "MOV M;H",
        "MOV M;L",
        "HLT",
        "MOV M;A",
        "MOV A;B",
        "MOV A;C",
        "MOV A;D",
        "MOV A;E",
        "MOV A;H",
        "MOV A;L",
        "MOV A;M",
        "MOV A;A",
        "ADD B",
        "ADD C",
        "ADD D",
        "ADD E",
        "ADD H",
        "ADD L",
        "ADD M",
        "ADD A",
        "ADC B",
        "ADC C",
        "ADC D",
        "ADC E",
        "ADC H",
        "ADC L",
        "ADC M",
        "ADC A",
        "SUB B",
        "SUB C",
        "SUB D",
        "SUB E",
        "SUB H",
        "SUB L",
        "SUB M",
        "SUB A",
        "SBB B",
        "SBB C",
        "SBB D",
        "SBB E",
        "SBB H",
        "SBB L",
        "SBB M",
        "SBB A",
        "ANA B",
        "ANA C",
        "ANA D",
        "ANA E",
        "ANA H",
        "ANA L",
        "ANA M",
        "ANA A",
        "XRA B",
        "XRA C",
        "XRA D",
        "XRA E",
        "XRA H",
        "XRA L",
        "XRA M",
        "XRA A",
        "ORA B",
        "ORA C",
        "ORA D",
        "ORA E",
        "ORA H",
        "ORA L",
        "ORA M",
        "ORA A",
        "CMP B",
        "CMP C",
        "CMP D",
        "CMP E",
        "CMP H",
        "CMP L",
        "CMP M",
        "CMP A",
        "RNZ",
        "POP B",
        "JNZ adr",
        "JMP adr",
        "CNZ adr",
        "PUSH B",
        "ADI D8",
        "RST 0",
        "RZ",
        "RET",
        "JZ adr",
        "-",
        "CZ adr",
        "CALL adr",
        "ACI D8",
        "RST 1",
        "RNC",
        "POP D",
        "JNC adr",
        "OUT D8",
        "CNC adr",
        "PUSH D",
        "SUI D8",
        "RST 2",
        "RC",
        "-",
        "JC adr",
        "IN D8",
        "CC adr",
        "-",
        "SBI D8",
        "RST 3",
        "RPO",
        "POP H",
        "JPO adr",
        "XTHL",
        "CPO adr",
        "PUSH H",
        "ANI D8",
        "RST 4",
        "RPE",
        "PCHL",
        "JPE adr",
        "XCHG",
        "CPE adr",
        "-",
        "XRI D8",
        "RST 5",
        "RP",
        "POP PSW",
        "JP adr",
        "DI",
        "CP adr",
        "PUSH PSW",
        "ORI D8",
        "RST 6",
        "RM",
        "SPHL",
        "JM adr",
        "EI",
        "CM adr",
        "-",
        "CPI D8",
        "RST 7"
    };

    char instructionSizesLocal[256] = {
        1,
        3,
        1,
        1,
        1,
        1,
        2,
        1,
        0,
        1,
        1,
        1,
        1,
        1,
        2,
        1,
        0,
        3,
        1,
        1,
        1,
        1,
        2,
        1,
        0,
        1,
        1,
        1,
        1,
        1,
        2,
        1,
        1,
        3,
        3,
        1,
        1,
        1,
        2,
        1,
        0,
        1,
        3,
        1,
        1,
        1,
        2,
        1,
        1,
        3,
        3,
        1,
        1,
        1,
        2,
        1,
        0,
        1,
        3,
        1,
        1,
        1,
        2,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        3,
        3,
        3,
        1,
        2,
        1,
        1,
        1,
        3,
        0,
        3,
        3,
        2,
        1,
        1,
        1,
        3,
        2,
        3,
        1,
        2,
        1,
        1,
        0,
        3,
        2,
        3,
        0,
        2,
        1,
        1,
        1,
        3,
        1,
        3,
        1,
        2,
        1,
        1,
        1,
        3,
        1,
        3,
        0,
        2,
        1,
        1,
        1,
        3,
        1,
        3,
        1,
        2,
        1,
        1,
        1,
        3,
        1,
        3,
        0,
        2,
        1
    };

    char instructionFlagsLocal[256][20] = {
        "",
        "",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "CY",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "CY",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "",
        "",
        "CY",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "CY",
        "",
        "",
        "Z; S; P; AC",
        "Z; S; P; AC",
        "",
        "CY",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "Z; S; P; CY; AC",
        ""
    };

    char instructionFunctionsLocal[256][100] = {
        "",
        "B <- byte 3; C <- byte 2",
        "(BC) <- A",
        "BC <- BC+1",
        "B <- B+1",
        "B <- B-1",
        "B <- byte 2",
        "A = A << 1; bit 0 = prev bit 7; CY = prev bit 7",
        "",
        "HL = HL + BC",
        "A <- (BC)",
        "BC = BC-1",
        "C <- C+1",
        "C <-C-1",
        "C <- byte 2",
        "A = A >> 1; bit 7 = prev bit 0; CY = prev bit 0",
        "",
        "D <- byte 3; E <- byte 2",
        "(DE) <- A",
        "DE <- DE + 1",
        "D <- D+1",
        "D <- D-1",
        "D <- byte 2",
        "A = A << 1; bit 0 = prev CY; CY = prev bit 7",
        "",
        "HL = HL + DE",
        "A <- (DE)",
        "DE = DE-1",
        "E <-E+1",
        "E <- E-1",
        "E <- byte 2",
        "A = A >> 1; bit 7 = prev bit 7; CY = prev bit 0",
        "special",
        "H <- byte 3; L <- byte 2",
        "(adr) <-L; (adr+1)<-H",
        "HL <- HL + 1",
        "H <- H+1",
        "H <- H-1",
        "H <- byte 2",
        "special",
        "",
        "HL = HL + HL",
        "L <- (adr); H<-(adr+1)",
        "HL = HL-1",
        "L <- L+1",
        "L <- L-1",
        "L <- byte 2",
        "A <- !A",
        "special",
        "SP.hi <- byte 3; SP.lo <- byte 2",
        "(adr) <- A",
        "SP = SP + 1",
        "(H)(L) <- (H)(L)+1",
        "(H)(L) <- (H)(L)-1",
        "(H)(L) <- byte 2",
        "CY = 1",
        "",
        "HL = HL + SP",
        "A <- (adr)",
        "SP = SP-1",
        "A <- A+1",
        "A <- A-1",
        "A <- byte 2",
        "CY=!CY",
        "B <- B",
        "B <- C",
        "B <- D",
        "B <- E",
        "B <- H",
        "B <- L",
        "B <- (H)(L)",
        "B <- A",
        "C <- B",
        "C <- C",
        "C <- D",
        "C <- E",
        "C <- H",
        "C <- L",
        "C <- (H)(L)",
        "C <- A",
        "D <- B",
        "D <- C",
        "D <- D",
        "D <- E",
        "D <- H",
        "D <- L",
        "D <- (H)(L)",
        "D <- A",
        "E <- B",
        "E <- C",
        "E <- D",
        "E <- E",
        "E <- H",
        "E <- L",
        "E <- (H)(L)",
        "E <- A",
        "H <- B",
        "H <- C",
        "H <- D",
        "H <- E",
        "H <- H",
        "H <- L",
        "H <- (H)(L)",
        "H <- A",
        "L <- B",
        "L <- C",
        "L <- D",
        "L <- E",
        "L <- H",
        "L <- L",
        "L <- (H)(L)",
        "L <- A",
        "(H)(L) <- B",
        "(H)(L) <- C",
        "(H)(L) <- D",
        "(H)(L) <- E",
        "(H)(L) <- H",
        "(H)(L) <- L",
        "special",
        "(H)(L) <- A",
        "A <- B",
        "A <- C",
        "A <- D",
        "A <- E",
        "A <- H",
        "A <- L",
        "A <- (H)(L)",
        "A <- A",
        "A <- A + B",
        "A <- A + C",
        "A <- A + D",
        "A <- A + E",
        "A <- A + H",
        "A <- A + L",
        "A <- A + (H)(L)",
        "A <- A + A",
        "A <- A + B + CY",
        "A <- A + C + CY",
        "A <- A + D + CY",
        "A <- A + E + CY",
        "A <- A + H + CY",
        "A <- A + L + CY",
        "A <- A + (H)(L) + CY",
        "A <- A + A + CY",
        "A <- A - B",
        "A <- A - C",
        "A <- A + D",
        "A <- A - E",
        "A <- A + H",
        "A <- A - L",
        "A <- A + (H)(L)",
        "A <- A - A",
        "A <- A - B - CY",
        "A <- A - C - CY",
        "A <- A - D - CY",
        "A <- A - E - CY",
        "A <- A - H - CY",
        "A <- A - L - CY",
        "A <- A - (H)(L) - CY",
        "A <- A - A - CY",
        "A <- A & B",
        "A <- A & C",
        "A <- A & D",
        "A <- A & E",
        "A <- A & H",
        "A <- A & L",
        "A <- A & (H)(L)",
        "A <- A & A",
        "A <- A ^ B",
        "A <- A ^ C",
        "A <- A ^ D",
        "A <- A ^ E",
        "A <- A ^ H",
        "A <- A ^ L",
        "A <- A ^ (H)(L)",
        "A <- A ^ A",
        "A <- A | B",
        "A <- A | C",
        "A <- A | D",
        "A <- A | E",
        "A <- A | H",
        "A <- A | L",
        "A <- A | (H)(L)",
        "A <- A | A",
        "A - B",
        "A - C",
        "A - D",
        "A - E",
        "A - H",
        "A - L",
        "A - (H)(L)",
        "A - A",
        "if NZ; RET",
        "C <- (sp); B <- (sp+1); sp <- sp+2",
        "if NZ; PC <- adr",
        "PC <= adr",
        "if NZ; CALL adr",
        "(sp-2)<-C; (sp-1)<-B; sp <- sp - 2",
        "A <- A + byte",
        "CALL $0",
        "if Z; RET",
        "PC.lo <- (sp); PC.hi<-(sp+1); SP <- SP+2",
        "if Z; PC <- adr",
        "",
        "if Z; CALL adr",
        "(SP-1)<-PC.hi;(SP-2)<-PC.lo;SP<-SP-2;PC=adr",
        "A <- A + data + CY",
        "CALL $8",
        "if NCY; RET",
        "E <- (sp); D <- (sp+1); sp <- sp+2",
        "if NCY; PC<-adr",
        "special",
        "if NCY; CALL adr",
        "(sp-2)<-E; (sp-1)<-D; sp <- sp - 2",
        "A <- A - data",
        "CALL $10",
        "if CY; RET",
        "",
        "if CY; PC<-adr",
        "special",
        "if CY; CALL adr",
        "",
        "A <- A - data - CY",
        "CALL $18",
        "if PO; RET",
        "L <- (sp); H <- (sp+1); sp <- sp+2",
        "if PO; PC <- adr",
        "L <-> (SP); H <-> (SP+1) ",
        "if PO; CALL adr",
        "(sp-2)<-L; (sp-1)<-H; sp <- sp - 2",
        "A <- A & data",
        "CALL $20",
        "if PE; RET",
        "PC.hi <- H; PC.lo <- L",
        "if PE; PC <- adr",
        "H <-> D; L <-> E",
        "if PE; CALL adr",
        "",
        "A <- A ^ data",
        "CALL $28",
        "if P; RET",
        "flags <- (sp); A <- (sp+1); sp <- sp+2",
        "if S=0; PC <- adr",
        "special",
        "if P; PC <- adr",
        "(sp-2)<-flags; (sp-1)<-A; sp <- sp - 2",
        "A <- A | data",
        "CALL $30",
        "if M; RET",
        "SP=HL",
        "if M; PC <- adr",
        "special",
        "if M; CALL adr",
        "",
        "A - data",
        "CALL $38"
    };

    memcpy(instructions, instructionsLocal, 256*20);
    memcpy(instructionSizes, instructionSizesLocal, 256);
    memcpy(instructionFlags, instructionFlagsLocal, 256*20);
    memcpy(instructionFunctions, instructionFunctionsLocal, 256*100);
}