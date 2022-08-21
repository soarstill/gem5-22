/*
 * Copyright (c) 2022 H.S.Song
 * All rights reserved.
 *
 * Recmdstribution and use in source and binary forms, with or without
 * mocmdfication, are permitted provided that the following concmdtions are
 * met: recmdstributions of source code must retain the above copyright
 * notice, this list of concmdtions and the following cmdsclaimer;
 * recmdstributions in binary form must reproduce the above copyright
 * notice, this list of concmdtions and the following cmdsclaimer in the
 * documentation and/or other materials provided with the cmdstribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUcmdNG, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE cmdSCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY cmdRECT, INcmdRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL cmdaMAGES (INCLUcmdNG, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * cmdaTA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUcmdNG NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH cmdaMAGE.
 */

#ifndef __DEV_PUDMA_PUDMALIB2_H__
#define __DEV_PUDMA_PUDMALIB2_H__

#define USER_LEVEL // comment out if it is used for GEM5 header

// lib headers
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>

typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned int    uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned short  PioAddr_t;

/** Caution!! : SHOULD be consistant with gem5 configs parameters
 * puengine4.pio_addr == PU_PIO_ADDR,
 * puengine4.pio_size == PU_PIO_SIZE,
 * puengine4.x86pio_base_addr == X86PIO_BASE_ADDR,
 * puengine4.dram2_base_addr = DRAM2_BASE_ADDR
 *
 */
#define PU_PIO_ADDR (0x300)
// Device dependent, range (0x300~0x6FF), can change (16bits)
#define PU_PIO_SIZE (0x400)
// Device dependent, range (0x300~0x6FF), can change (16bits)
#define X86PIO_BASE_ADDR (0x8000000000000000)
// fixed by gem5 (don't touch, 64 bits)
#define DRAM2_BASE_ADDR  (0x4000000000000000)
// start of data memory (can change, 32bit or 64bit)
#define PUREG_OFFSET(REG) ((REG) * sizeof(uint32_t))

//////////////////////////////////////////////////////////////////////
//  register index of PuCmd area (all uint32_t type for simplicity) //
//////////////////////////////////////////////////////////////////////
enum PU_REGS
{
   REG_VALID = 0,
   //  (invalid while user updates the Regs, then valid if done)
    // 0 : invalid, !0 : valid
   REG_STATUS, // status of PuEngine;  (USER READ ONLY, GEM5 R/W)
    // 0: invalid, 1: ready, 2: DMA read source data A, B,
    // 3: in-computing 4: DNA write results C 5: completed
   REG_COMMAND,
    // 0: invalid command, 1: ready 2. start computing,
    // 3: abort PuCore  4: update PuCore status
   REG_FLAGS,  // Not yet defined
   REG_OPCODE, // 0: NOP, 1: ADD, 2~ : Reserved
   REG_DATAFLAGS, // Not yed defined
   REG_AADDR,  // offset of matrix A in dram2
   REG_ACOLS,  // number of rows of matrix A
   REG_AROWS,  // number of columns of matrix A
   REG_BADDR,  // offset of matrix B in dram2
   REG_BROWS,
   REG_BCOLS,
   REG_CADDR, // offset of matrix C in dram2
   REG_CROWS,
   REG_CCOLS,
   REG_LIMIT  // Not a valid register
};

typedef uint32_t PuCmd[sizeof(enum PU_REGS)];

enum PU_CMD_TYPE
{
    CMD_INVALID = 0,
    CMD_INIT,   // prepare a new PU computing
    CMD_START,   // start computing
    CMD_ABORT,   //
    CMD_STATUS,  // update status field
    CMD_LIMIT
};

enum PU_STATUS_TYPE
{
    STS_INVALID = 0,    // Not a state
    STS_IDLE,
    STS_READY,          // Ready to get a new command
    STS_DMARD_A,     // PuCore is reading source matrices
    STS_DMARD_B,     // PuCore is reading source matrices
    STS_COMPUTING,      // PuCore is computing
    STS_DMAWR_C,     // PuCore is writing the results
    STS_COMPLETED,      // PuCore successfully finished
    STS_ERROR,          // Somthing bad happend
    STS_LIMIT
};

enum PU_OP_TYPE
{
    OP_INVALID = 0,    // Not a state
    OP_ADD,             // add operation
    OP_LIMIT
};

// User Level ONLY - IF gem5 Level, comment out USER_LEVEL headers

#ifdef USER_LEVEL

#define IS_VALID_ADDR(pio) \
    (((pio) >=  PU_PIO_ADDR && (pio) < PU_PIO_ADDR + PU_PIO_SIZE) ? 1 : 0 )
#define PUREG_ADDR(REG) (PU_PIO_ADDR + PUREG_OFFSET(REG))

extern uint8_t writePuCmdByte(PioAddr_t pio, uint8_t value);
extern uint8_t readPuCmdByte(PioAddr_t pio);
extern PioAddr_t readPuCmdBytes(PioAddr_t pio, int size, uint8_t * buf);
extern PioAddr_t writePuCmdBytes(PioAddr_t pio, int size, uint8_t * buf);
extern uint32_t writePuCmdUint32(PioAddr_t pio, uint32_t value);
extern uint32_t readPuCmdUint32(PioAddr_t pio);
extern int isPuCmdValid();
extern void validatePuCmd();
extern void invalidatePuCmd();
void initPuCmd(PuCmd cmd);
extern int writePuCmd(PuCmd cmd);
extern int readPuCmd(PuCmd cmd);
extern uint32_t readPuCmdStatus();
extern void printPuCmd(PuCmd cmd);
extern int isPuCmdEqual(PuCmd cmda,  PuCmd cmdb);

#endif //USER_LEVEL

#endif // __DEV_PUDMA_PUDMALIB2_H__
