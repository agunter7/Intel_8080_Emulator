/**
 * Implements functions for Intel 8080 emulated instructions
 * Capitalized names indicate a function that implements a full 8080 instruction's functionality
 * camelCase names indicate a function that implements a portion of an instruction's effect(s)
 *
 * By convention, only fully-implemented 8080 functions (Capitalized names) will update the program counter and
 * increase the number of clock cycles completed
 *
 * @author Andrew Gunter
 */

#include "../src/cpuStructures.h"
#include "../src/instructions.h"
#include "../src/helpers.h"

/**
 CALL addr
 PC = PC + 3
 Memory[SP-1] = PCH
 Memory[SP-2] = PCL
 SP = SP-2
 PC = (byte 3)(byte 2)
*/
void CALL(uint16_t address, State8080 *state)
{

    uint8_t pcHigh;  // program counter high 8 bits
    uint8_t pcLow;
    uint16_t pcToStore = (state->pc) + 3;
    uint16_t sp = state->sp;

    pcHigh = (uint8_t)(pcToStore >> 8);
    pcLow = (uint8_t)pcToStore;

    writeMem(sp-1, pcHigh, state);
    writeMem(sp-2, pcLow, state);
    state->sp -= 2;
    state->pc = address;
    state->cyclesCompleted += 17;
}

/**
 INX rp
 (rh)(rl) = (rh)(rl)+1
*/
void INX_RP(uint8_t *highReg, uint8_t *lowReg, State8080 *state)
{
    uint16_t concatRegValue = (((uint16_t)(*highReg)) << 8) | ((uint16_t)(*lowReg));
    concatRegValue++;
    *highReg = (uint8_t)(concatRegValue >> 8);
    *lowReg = (uint8_t)concatRegValue;
    state->pc += 1;
    state->cyclesCompleted += 5;
}

/**
 PUSH rp
 Push register pair onto the stack
 memory[sp-1] = rh
 memory[sp-2] = rl
 sp = sp-2;
 */
void PUSH_RP(uint8_t highReg, uint8_t lowReg, State8080 *state)
{
    uint16_t sp = state->sp;

    writeMem(sp-1, highReg, state);
    writeMem(sp-2, lowReg, state);
    state->sp = sp-2;

    state->pc += 1;
    state->cyclesCompleted += 11;
}

/**
 POP rp
 Pop 2 bytes from the stack and store in a register pair
 rl = memory[sp]
 rh = memory[sp+1]
 sp = sp+2
 */
void POP_RP(uint8_t *highReg, uint8_t *lowReg, State8080 *state)
{
    uint16_t sp = state->sp;
    *lowReg = readMem(sp, state);
    *highReg = readMem(sp+1, state);
    state->sp = sp+2;

    state->pc += 1;
    state->cyclesCompleted += 10;
}

/**
 DAD rp
 Concatenate the values from a register pair
 Concatenate the values from register H and register L
 Add the two concatenated values
 Store the result in register pair HL
 (H)(L) = (H)(L) + (rh)(rl)
 */
void DAD_RP(uint8_t highReg, uint8_t lowReg, State8080 *state)
{
    uint32_t result;
    uint16_t augend = getValueHL(state);
    uint16_t addend = (((uint16_t)highReg)<<8) | (uint16_t)lowReg;
    uint8_t newValueH;
    uint8_t newValueL;
    
    // Lossless addition
    result = (uint32_t)augend + (uint32_t)addend;

    // Carry check for overflow of 16-bit Addition
    // Do not set high for 8-bit overflow (which is the typical case in the 8080)
    if ((result & 0x00010000) == 0x00010000){
        state->flags.carry = 1;
    }else{
        state->flags.carry = 0;
    }

    newValueH = (uint8_t)(result>>8);
    newValueL = (uint8_t)result;
    state->h = newValueH;
    state->l = newValueL;

    state->pc += 1;
    state->cyclesCompleted += 10;
}

/**
 JMP addr
 Jump to address
 pc = address
 */
void JMP(uint16_t address, State8080 *state)
{
    state->pc = address;
    state->cyclesCompleted += 10;
}

/**
 XRA r
 Exclusive Or Accumulator with a register
 A = A XOR r
 Flags: z,s,p,cy(reset),ac(reset)
 */
void XRA_R(uint8_t data, State8080 *state)
{
    xorWithAccumulator(data, state);

    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 ANA r
 AND Accumulator with register
 A = A AND r
 Flags: z,s,p,cy(reset),ac
 */
void ANA_R(uint8_t data, State8080 *state)
{
    andWithAccumulator(data, state);

    state->pc += 1;
    state->cyclesCompleted += 1;
}

/**
 * RST n
 * Restart with subroutine 'n'
 * PC = PC + 1
 * memory[sp-1] = PCH
 * memory[sp-2] = PCL
 * SP = SP - 2
 * PC = 8 * n
 */
void RST(uint8_t restartNumber, State8080 *state)
{
    if(restartNumber >= 0x08){
        logger("Warning: Invalid interrupt attempted:\n");
    }

    if(state->interruptsEnabled){
        uint8_t pcHigh;  // program counter higher 8 bits
        uint8_t pcLow;  // program counter lower 8 bits
        uint16_t pcToStore = (state->pc) + 1;
        uint16_t sp = state->sp;

        pcHigh = (uint8_t)(pcToStore >> 8);
        pcLow = (uint8_t)pcToStore;

        writeMem(sp-1, pcHigh, state);
        writeMem(sp-2, pcLow, state);
        state->sp -= 2;
        state->pc = 8 * restartNumber;
        state->cyclesCompleted += 11;

        // It is expected for interrupts to be disabled until end of ISR. Interrupts will be enabled at end of ISR
        state->interruptsEnabled = 0;

    }
}

/**
 * RET
 * Return
 * PCL = memory[sp]
 * PCH = memory[sp+1]
 * SP = SP + 2
 */
void RET(State8080 *state)
{
    uint8_t lowByte = readMem(state->sp, state);
    uint8_t highByte = readMem((state->sp)+1, state);
    uint16_t newValuePC = (((uint16_t)highByte) << 8) | (uint16_t)lowByte;
    state->pc = newValuePC;

    state->sp += 2;
    state->cyclesCompleted += 10;
}

/**
 * ORA r
 * OR Accumulator with register
 * A = A OR r
 * Flags: z,s,p,cy(reset),ac(reset)
 */
void ORA_R(uint8_t data, State8080 *state)
{
    orWithAccumulator(data, state);

    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 * DCX rp
 * Decrement register pair
 * (rh)(rl) = (rh)(rl) - 1
 */
void DCX_RP(uint8_t *highReg, uint8_t *lowReg, State8080 *state)
{
    // Concatenate pair and decrement
    uint16_t pairValue = ((uint16_t)(*highReg))<<8 | (uint16_t)(*lowReg);
    pairValue -= 1;

    // Store result in original, separated registers
    *highReg = (uint8_t)((pairValue & 0xff00)>>8);
    *lowReg = (uint8_t)(pairValue & 0x00ff);

    state->pc += 1;
    state->cyclesCompleted += 5;
}

/**
 * MOV r1, r2
 * Move the content of r2 into r1
 * r1 = r2
 */
void MOV_R1_R2(uint8_t *destReg, uint8_t *sourceReg, State8080 *state)
{
    *destReg = *sourceReg;

    state->pc += 1;
    state->cyclesCompleted += 5;
}

/**
 * MVI r, d8
 * Move 8-bit immediate into register r
 * r = d8
 */
void MVI_R(uint8_t *destReg, uint8_t value, State8080 *state)
{
    *destReg = value;

    state->pc += 2;
    state->cyclesCompleted += 7;
}

void NOP(State8080 *state)
{
    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 * INR R
 * Increment Register
 * R = R + 1
 * Flags: z,s,p,ac
 */
void INR_R(uint8_t *reg, State8080 *state)
{
    *reg = addWithCheckAC(*reg, (uint8_t)1, state);
    checkStandardArithmeticFlags(*reg, state);

    state->pc += 1;
    state->cyclesCompleted += 5;
}

/**
 * DCR R
 * Decrement Register
 * R = R - 1
 * Flags: z,s,p,ac
 */
void DCR_R(uint8_t *reg, State8080 *state)
{
    *reg = addWithCheckAC(*reg, (uint8_t)(-1), state);
    checkStandardArithmeticFlags(*reg, state);

    state->pc += 1;
    state->cyclesCompleted += 5;
}

/**
 * MOV R, M
 * Move from memory into register R
 * R = memory[(H)(L)]
 */
void MOV_R_M(uint8_t *destReg, State8080 *state)
{
    moveDataFromHLMemory(destReg, state);

    state->pc += 1;
    state->cyclesCompleted += 7;
}

/**
 * MOV M, R
 * Move from register R into memory
 * memory[(H)(L)] = R
 */
void MOV_M_R(uint8_t data, State8080 *state)
{
    moveDataToHLMemory(data, state);

    state->pc += 1;
    state->cyclesCompleted += 7;
}

/**
 * ADD R
 * Add Register to Accumulator
 * A = A + R
 * Flags: z,s,p,cy,ac
 */
void ADD_R(uint8_t data, State8080 *state)
{
    addWithCheckAC(state->a, data, state);
    state->a = addWithCheckCY(state->a, data, state);
    checkStandardArithmeticFlags(state->a, state);

    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 * ADC R
 * Add Register to Accumulator with Carry
 */
void ADC_R(uint8_t data, State8080 *state)
{
    if(data == 0xff && state->flags.carry == 1){
        // Explicitly set carry, as this case will cause overflow in the addend
        ADD_R(0x00, state);
        state->flags.carry = 1;
    }else{
        ADD_R(data+(state->flags.carry), state);
    }
}

/**
 * CMP R
 * Compare Register with Accumulator
 * A - R
 * Flags: z,s,p,cy,ac
 */
void CMP_R(uint8_t data, State8080 *state)
{
    compareWithAccumulator(data, state);
    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 * LXI RP, D16
 * Load immediate into register pair
 * rh = byte 3 (high byte), rl = byte 2 (low byte)
 */
void LXI_RP(uint8_t *highReg, uint8_t *lowReg, uint16_t orderedOperands, State8080 *state)
{
    *highReg = (uint8_t)(orderedOperands>>8);
    *lowReg = (uint8_t)orderedOperands;
    state->pc += 3;
    state->cyclesCompleted += 10;
}

/**
 * SUB R
 * Subtract Register from Accumulator
 * A = A - R
 * Flags: z,s,p,cy,ac
 */
void SUB_R(uint8_t data, State8080 *state)
{
    subFromAccumulator(data, state);
    state->pc += 1;
    state->cyclesCompleted += 4;
}

/**
 * SBB R
 * Subtract Register from Accumulator with borrow
 * A = A - (R + CY)
 * Flags: z,s,p,cy,ac
 */
void SBB_R(uint8_t data, State8080 *state)
{
    if(data == 0xff && state->flags.carry == 1){
        // Overflow occurs here, carry out occurs, reset carry flag (due to subtraction logic)
        subFromAccumulator(0x00, state);
        state->flags.carry = 0;
    }else{
        subFromAccumulator(data+(state->flags.carry), state);
    }

    state->pc += 1;
    state->cyclesCompleted += 4;
}

uint16_t compareWithAccumulator(uint8_t subtrahend, State8080 *state)
{
    addWithCheckAC(state->a, twosComplement(subtrahend), state);  // Do not store result, just check AC
    uint16_t result = subWithCheckCY(state->a, subtrahend, state);
    uint8_t resultByte = (uint8_t)(result & 0x00ff);  // clear overflow bit in result
    checkStandardArithmeticFlags(resultByte, state);

    return result;
}

void subFromAccumulator(uint8_t subtrahend, State8080 *state)
{
    state->a = compareWithAccumulator(subtrahend, state);
}

void orWithAccumulator(uint8_t data, State8080 *state)
{
    state->a = state->a | data;

    checkStandardArithmeticFlags(state->a, state);

    state->flags.carry = 0;
    state->flags.auxiliaryCarry = 0;
}

void xorWithAccumulator(uint8_t data, State8080 *state)
{
    state->a = state->a ^ data;

    checkStandardArithmeticFlags(state->a, state);
    state->flags.carry = 0;
    state->flags.auxiliaryCarry = 0;
}

void andWithAccumulator(uint8_t data, State8080 *state)
{
    state->a = state->a & data;

    checkStandardArithmeticFlags(state->a, state);
    state->flags.carry = 0;

    state->flags.auxiliaryCarry = 0;
}

void moveDataToHLMemory(uint8_t data, State8080 *state)
{
    uint16_t destinationAddress = getValueHL(state);
    writeMem(destinationAddress, data, state);
}

void moveDataFromHLMemory(uint8_t *destination, State8080 *state)
{
    uint16_t sourceAddress = getValueHL(state);
    *destination = readMem(sourceAddress, state);
}

void moveDataToBCMemory(uint8_t data, State8080 *state)
{
    uint16_t destinationAddress = getValueBC(state);
    writeMem(destinationAddress, data, state);
}

void moveDataToDEMemory(uint8_t data, State8080 *state)
{
    uint16_t destinationAddress = getValueDE(state);
    writeMem(destinationAddress, data, state);
}

uint16_t getValueHL(State8080 *state)
{
    uint16_t highBits = (((uint16_t)(state->h))<<8);
    uint16_t lowBits = (uint16_t)(state->l);
    return (highBits | lowBits);
}

uint16_t getValueDE(State8080 *state)
{
    uint16_t highBits = (((uint16_t)(state->d))<<8);
    uint16_t lowBits = (uint16_t)(state->e);
    return (highBits | lowBits);
}

uint16_t getValueBC(State8080 *state)
{
    uint16_t highBits = (((uint16_t)(state->b))<<8);
    uint16_t lowBits = (uint16_t)(state->c);
    return (highBits | lowBits);
}

void writeMem(uint16_t address, uint8_t value, State8080 *state)
{
    if(address >= ROM_LIMIT_8080){
        state->memory[address] = value;
    }else{
        state->memory[address] = value;
        logger("Warning: ROM Overwrite!\n");
        logger("Address 0x%04x; Value 0x%02x\n", address, value);
    }
}

uint8_t readMem(uint16_t address, State8080 *state)
{
    return state->memory[address];
}

uint16_t addWithCheckAC(uint8_t op1, uint8_t op2, State8080 *state)
{
    // Perform lower-order 4-bit addition
    uint8_t nibbleResult = (op1 & 0x0f) + (op2 & 0x0f);

    if((nibbleResult & 0x10) == 0x10){
        // Carry occurred from bit 3 to bit 4
        state->flags.auxiliaryCarry = 1;
    }else{
        state->flags.auxiliaryCarry = 0;
    }

    // Return lossless result from 8-bit addition
    return ((uint16_t)op1 + (uint16_t)op2);
}

uint16_t addWithCheckCY(uint8_t op1, uint8_t op2, State8080 *state)
{
    uint16_t result = (uint16_t)op1 + (uint16_t)op2;

    if( result > 0xff){
        state->flags.carry = 1;
    }else{
        state->flags.carry = 0;
    }

    return result;
}

/**
 * Perform subtraction by taking the 2's complement of the subtrahend and add it to the minuend.
 * In 8080 subtraction operations, the minuend is interpreted as an unsigned number.
 * The logic for the carry flag is inverted relative to the logic for addition:
 *   If a carry out occurs, the carry flag is reset
 *   If no carry out occurs, the carry flag is set
 *
 * Source: Intel 8080 Programmer's Manual pg.13 and pg.18
 */
uint16_t subWithCheckCY(int8_t minuend, int8_t subtrahend, State8080 *state)
{
    uint16_t result;
    // use addition nomenclature
    uint8_t augend = minuend;
    uint8_t addend = twosComplement(subtrahend);

    // Perform subtraction as twos complement addition
    result = augend + addend;

    // Perform carry check
    uint16_t overflowBit = result>>8;  // Get bit 8
    if(overflowBit == 1){
        // Carry out occurred
        state->flags.carry = 0;
    }else{
        // No carry out occurred, a "borrow" occurred
        state->flags.carry = 1;
    }

    // If subtrahend is 0x00, twos complement process is
    // 0000 0000 -> flip bits -> 1111 1111 -> +1 -> [1] 0000 0000
    // Carry out means carry flag should be reset by subtraction logic
    // This must be performed manually as the (minuend-subtrahend) operation alone will no do this
    if(subtrahend == 0x00){
        state->flags.carry = 0;
    }

    return result;
}

void checkStandardArithmeticFlags(uint8_t result, State8080 *state)
{
    // Check zero flag
    if (result == 0){  
        state->flags.zero = 1;
    }else{
        state->flags.zero = 0;
    }

    // Check sign flag
    // Sign flag set when MSB (bit 7) is set, else reset
    // 0x80 == 1000 0000
    if((result & 0x80) == 0x80){
        state->flags.sign = 1;
    }else{
        state->flags.sign = 0;
    }

    //Check parity flag
    uint8_t mask = 0x01;
    unsigned int sum = 0;
    uint8_t maskedResult = 0;
    // Get sum of 1s in the 8-bit result
    for(int shift = 0; shift < 8; shift++){  // Each iteration targets a different bit from result
        maskedResult = result & (mask << shift);
        sum += (maskedResult >> shift);
    }
    // Set for even parity, reset for odd parity
    if (sum % 2 == 0){
        state->flags.parity = 1;
    }else{
        state->flags.parity = 0;
    }
}