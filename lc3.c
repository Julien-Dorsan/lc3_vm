// standard libraries
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
// unix onley
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

// PROCEDURE FOR THE VM TO WORK
// 1. Load one instruction from memory at the address of the PC register.
// 2. Increment the PC register.
// 3. Look at the opcode to determine which type of instruction it should perform.
// 4. Perform the instruction using the parameters in the instruction.
// 5. Go back to step 1.

//! MEMORY MAPPED REGISTER

// registers unaccessible from normal reg table
// a special address in mem is reserved for them
// to read / write to these reg, we just rw to their mem loc
// commonly used to interact with special hardware devices (such as keyboard)
// lc3 has 2 MMPR that are keyboard status register
// allows polling for keyboard state
enum
{
    MR_KBSR = 0xFE00, // keyboard status, whether a key has been pressed
    MR_KBDR = 0xFE02  // keyboard data, identifies which key was pressed
};

//! TRAP CODES

// kind of an API for the lc3
// predifines routines for performing common tasks and interacting with I/O devices
// trap routines are assigned trap codes which identifies it
enum
{
    TRAP_GETC = 0x20,  // get char from keyboard (not echoded)
    TRAP_OUT = 0x21,   // output a char
    TRAP_PUTS = 0x22,  // output a word string
    TRAP_IN = 0x23,    // get char from keyboard (echoed)
    TRAP_PUTSP = 0x24, // output a byte string
    TRAP_HALT = 0x25,  // halt the program
};

//! MEMORY STORAGE
// 2^16 memory locations to store up to 128KB of data
#define MEMORY_MAX (1 << 16)
uint16_t memory[MEMORY_MAX];

//! REGISTER STORAGE
// slot for a single value on the CPU
// R0-R7: general purpose, to perform any program calculation
// PC: program counter, address of the next instruction in memory to execute
// COND: conditional flags, information about the previous calculation
enum
{
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,
    R_COND,
    R_COUNT
};
uint16_t reg[R_COUNT];

//! INSTRUCTION SET
// command telling the CPU which operation to do
// has an opcode and a set of parameters providing inputs on the task performed
// an opcode is an operation the CPU knows how to do
// each instruction is 16bit long with 4bits to store the opcode, rest is for the parameters
enum
{
    OP_BR = 0, // branch
    OP_ADD,    // add
    OP_LD,     // load
    OP_ST,     // store
    OP_JSR,    // jump register
    OP_AND,    // btiwise and
    OP_LDR,    // load register
    OP_STR,    // store register
    OP_RTI,    // unused
    OP_NOT,    // bitwise not
    OP_LDI,    // load indirect -> load a value from a location in memory to a register
    OP_STI,    // store indirect
    OP_JMP,    // jump
    OP_RES,    // reserved (unsused)
    OP_LEA,    // load effective address
    OP_TRAP    // execute trap
};

//! CONDITION FLAGS
// R_COND stores flags providing information on most recent executed calculation
// allows to check logical conditions, each indicating the sign of the previous calculation
enum
{
    FL_POS = 1 << 0, // Positive
    FL_ZRO = 1 << 1, // Zero
    FL_NEG = 1 << 2, // Negative
};

//! INPUT BUFFERING
// platform specific code to handle inputs

struct termios original_tio;

void disable_input_buffering()
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

//! HANDLE INTERRUPT

void handle_interrupt(int signal)
{
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

//! SIGN EXTEND

/// @brief SIGN EXTEND - pads any number with 0 if x > 0 or 1 if x < 0 to 16bits.
/// @brief this preserves the original value and the sign of the number
/// @param x a number
/// @param bit_count number of bits to pad to
/// @return padded to bit_count number
uint16_t sign_extend(uint16_t x, int bit_count)
{
    if ((x >> (bit_count - 1)) & 1)
    {
        // bitwise OR
        x |= (0xFFFF << bit_count);
    }
    return x;
}

//! SWAP

/// @brief swap each loaded uint16_t values to big-edian according to lc3 standards
/// @param x a little endian uint16_t value
/// @return a big endian uint16_t value
uint16_t swap16(uint16_t x)
{
    return (x << 8) | (x >> 8);
}

//! UPDATE FLAGS

/// @brief updates R_COND in the registery depending on the sign of the result,
/// @brief called any time a value is written to a register
/// @param r the index of a register
void update_flags(uint16_t r)
{
    if (reg[r] == 0)
    {
        reg[R_COND] = FL_ZRO;
    }
    else if (reg[r] >> 15) // 1 in the leftmost bit indicates negative
    {
        reg[R_COND] = FL_NEG;
    }
    else
    {
        reg[R_COND] = FL_POS;
    }
}

//! READ IMAGE FILE

/// @brief loads content of an assembly file in an address in memory
/// @brief first 16bits of the program specify where the program should start (the origin)
/// @param file an lc3 assembled program
void read_image_file(FILE *file)
{
    // oringin tells us where the image should be placed in mem
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin);

    uint16_t max_read = MEMORY_MAX - origin;
    uint16_t *p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    // swap to little endian
    while (read-- > 0)
    {
        *p = swap16(*p);
        ++p;
    }
}

//! READ IMAGE

/// @brief wrapper for read_image_file to allow file reading from an image path
/// @param image_path path to an lc3 assembled image
/// @return 1 if the image is successfully read, otherwise 0
int read_image(const char *image_path)
{
    FILE *file = fopen(image_path, "rb");
    if (!file)
    {
        return 0;
    };
    read_image_file(file);
    fclose(file);
    return 1;
}

//! MEMORY ACCESS

// write a value to an address in memory
void mem_write(uint16_t address, uint16_t val)
{
    memory[address] = val;
}

// read content from an address in memory
uint16_t mem_read(uint16_t address)
{
    if (address == MR_KBSR)
    {
        if (check_key())
        {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        }
        else
        {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}

//! MAIN LOOP

int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        // show usage string
        printf("lc3 [image-file1]...\n");
        exit(2);
    }

    for (int j = 1; j < argc; ++j)
    {
        if (!read_image(argv[j]))
        {
            printf("failed to load image %s \n", argv[j]);
            exit(1);
        }
    }

    // on startup
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    // Zero because one conditional flag should be set at any given time
    reg[R_COND] = FL_ZRO;

    // sets PC to starting position, 0x3000 is the default
    enum
    {
        PC_START = 0x3000
    };
    reg[R_PC] = PC_START;

    int running = 1;
    while (running)
    {
        // fetches the instruction from program counter
        uint16_t instr = mem_read(reg[R_PC]++);
        // gets only the opcode from the instruction (1st 4bits)
        uint16_t op = instr >> 12;

        switch (op)
        {
        case OP_ADD:
        {
            // cf.bitwise masking for 0x7
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t r1 = (instr >> 6) & 0x7;
            uint16_t imm_flag = (instr >> 5) & 0x1;
            if (imm_flag)
            {
                // immediate mode embeds the value in the instruction
                // and thus removes the need to load it from memory
                uint16_t imm5 = sign_extend(instr & 0x1F, 5);
                reg[r0] = reg[r1] + imm5;
            }
            else
            {
                uint16_t r2 = instr & 0x7;
                reg[r0] = reg[r1] + reg[r2];
            }
            update_flags(r0);
        }
        break;
        case OP_AND:
        {
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t r1 = (instr >> 6) & 0x7;
            uint16_t imm_flag = (instr >> 5) & 0x1;
            if (imm_flag)
            {
                uint16_t imm5 = sign_extend(instr & 0x1F, 5);
                reg[r0] = reg[r1] & imm5;
            }
            else
            {
                uint16_t r2 = instr & 0x7;
                reg[r0] = reg[r1] & reg[r2];
            }
            update_flags(r0);
        }
        break;
        case OP_NOT:
        {
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t r1 = (instr >> 6) & 0x7;
            reg[r0] = ~reg[r1];
            update_flags(r0);
        }
        break;
        case OP_BR:
        {
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            uint16_t cond_flag = (instr >> 9) & 0x7;
            // n[11]z[10]p[9]
            // condition codes specified by the states of [11:9] are tested
            //[11] set ? tested : not tested (so clear)
            // then z then p
            // if any is tested, branches to the location specified by adding SEXT pc_offset
            if (cond_flag & reg[R_COND])
            {
                reg[R_PC] += pc_offset;
            }
        }
        break;
        case OP_JMP:
        {
            // also handles RET
            uint16_t r1 = (instr >> 6) & 0x7;
            // bits[8:6] indentify base register
            reg[R_PC] = reg[r1];
        }
        break;
        case OP_JSR:
        {
            uint16_t long_flag = (instr >> 11) & 1;
            reg[R_R7] = reg[R_PC];
            if (long_flag)
            {
                // JSR
                uint16_t long_pc_offset = sign_extend(instr & 0x7FF, 11);
                reg[R_PC] += long_pc_offset;
            }
            else
            {
                // JSSR
                uint16_t r1 = (instr >> 6) & 0x7;
                reg[R_PC] = reg[r1];
            }
        }
        break;
        case OP_LD:
        {
            // add content of SEXT pc_offset to incremented PC
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            reg[r0] = mem_read(reg[R_PC] + pc_offset);
            update_flags(r0);
        }
        break;
        case OP_LDI:
        {
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            // add SEXT pc_offset to current PC, look at that memory location to get the final address
            reg[r0] = mem_read(mem_read(reg[R_PC] + pc_offset));
            update_flags(r0);
        }
        break;
        case OP_LDR:
        {
            // SEXT offset is added to baseR
            // the content in memory is then loaded in DR
            // condition codes are set depending on loaded value (-, 0, +)
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t r1 = (instr >> 6) & 0x7;
            uint16_t offset = sign_extend(instr & 0x3F, 6);
            reg[r0] = mem_read(reg[r1] + offset);
            update_flags(r0);
        }
        break;
        case OP_LEA:
        {
            // SEXT pc_offset is added to incremented PC
            // address is then loaded in DR
            // condition codes are set depending on loaded value (-, 0, +)
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            reg[r0] = reg[R_PC] + pc_offset;
            update_flags(r0);
        }
        break;
        case OP_ST:
        {
            // content of SR is stored in mem loc of SEXT pc_offset and incremented PC
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            mem_write(reg[R_PC] + pc_offset, reg[r0]);
        }
        break;
        case OP_STI:
        {
            // SR's content is added to SEXT pc_offset + R_PC
            // what is in mem at this addr is the addr of the loc to which SR's data is stored
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
            mem_write(mem_read(reg[R_PC] + pc_offset), reg[r0]);
        }
        break;
        case OP_STR:
        {
            uint16_t r0 = (instr >> 9) & 0x7;
            uint16_t r1 = (instr >> 6) & 0x7;
            uint16_t offset = sign_extend(instr & 0x3F, 6);
            mem_write(reg[r1] + offset, reg[r0]);
        }
        break;
        case OP_TRAP:
            reg[R_R7] = reg[R_PC];
            switch (instr & 0xFF)
            {
            case TRAP_GETC:
            {
                // reads a single ASCII char
                reg[R_R0] = (uint16_t)getchar();
                update_flags(R_R0);
            }
            break;
            case TRAP_OUT:
            {
                putc((char)reg[R_R0], stdout);
                fflush(stdout);
            }
            break;
            case TRAP_PUTS:
            {
                // output a null-terminated ASCII string
                // characters are in consecutives mem loc starting with r0
                // WARNING 1 char / mem loc
                uint16_t *c = memory + reg[R_R0];
                // iterates over each char
                while (*c)
                {
                    putc((char)*c, stdout);
                    ++c;
                }
                fflush(stdout);
            }
            break;
            case TRAP_IN:
            {
                printf("Enter a character: ");
                char c = getchar();
                putc(c, stdout);
                fflush(stdout);
                reg[R_R0] = (uint16_t)c;
                update_flags(R_R0);
            }
            break;
            case TRAP_PUTSP:
            {
                // one char / byte, 2bytes / word
                // need to swap to big endian format
                uint16_t *c = memory + reg[R_R0];
                while (*c)
                {
                    char char1 = (*c) & 0xFF;
                    putc(char1, stdout);
                    char char2 = (*c) >> 8;
                    if (char2)
                        putc(char2, stdout);
                    ++c;
                }
                fflush(stdout);
            }
            break;
            case TRAP_HALT:
            {
                puts("HALT");
                fflush(stdout);
                running = 0;
            }
            break;
            }
            break;
        case OP_RES:
        case OP_RTI:
        default:
            abort();
            break;
        }
    }
    // on shutdown
    restore_input_buffering();
}