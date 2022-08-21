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

#ifndef __DEV_PU_CMD4_H__
#define __DEV_PU_CMD4_H__

// lib headers
#include <sys/io.h>

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/*
typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned int    uint32_t;
typedef unsigned long long uint64_t;
*/
typedef uint16_t  PioAddr_t;

/** Caution!! : SHOULD be consistant with gem5 configs parameters
 * puengine4.pio_addr == PU_PIO_ADDR,
 * puengine4.pio_size == PU_PIO_SIZE,
 * puengine4.x86pio_base_addr == X86PIO_BASE_ADDR,
 * puengine4.dram2_base_addr = DRAM2_BASE_ADDR
 *
 */
// fixed by gem5 (don't touch, 64 bits)
#define X86PIO_BASE_ADDR (0x8000000000000000)
// Device dependent, range (0x300~0x6FF), can change (16bits)
#define PU_PIO_ADDR (0x300)
// Device dependent, range (0x300~0x6FF), can change (16bits)
#define PU_PIO_SIZE (0x400)
// start of data memory (can change, 32bit or 64bit)
#define DRAM2_BASE_ADDR  (0x4000000000000000)
#define PUREG_OFFSET(REG) ((REG) * sizeof(uint32_t))
#define IS_VALID_ADDR(pio) \
        (((pio) >=  PU_PIO_ADDR && (pio) < PU_PIO_ADDR + PU_PIO_SIZE) ? 1 : 0 )
#define PUREG_ADDR(REG) (PU_PIO_ADDR + PUREG_OFFSET(REG))

namespace gem5
{

//////////////////////////////////////////////////////////////////////
//  register index of PuCmd area (all uint32_t type for simplicity) //
//////////////////////////////////////////////////////////////////////
enum PU_REGS
{
    REG_VALID = 0, // validate/inval- PuCmd info.
    REG_STATUS, // (USER READ ONLY, GEM5 R/W) See PU_STATUS_TYPE
    REG_COMMAND, // See PU_CMD_TYPE
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

class PuCmd
{
private:
    /**
     * @brief  PuEngine4's Physical Registers
     *
     */
    //uint32_t m_Regs[sizeof(enum PU_REGS)];
    uint32_t m_Regs[REG_LIMIT];

public:
    static PioAddr_t PioAddr(uint64_t gem5_addr)
    {
        return gem5_addr - X86PIO_BASE_ADDR;
    }

    static int PioOffset(uint64_t gem5_addr)
    {
        return gem5_addr - X86PIO_BASE_ADDR - PU_PIO_ADDR;
    }

    PuCmd() {
        //m_pioAddr = 0;
        //m_pioSize =  sizeof(enum PU_REGS) * sizeof(uint32_t) ;
        memset(&m_Regs[0], 0, sizeof(PuCmd));
    }

    PuCmd * clone()
    {
        PuCmd * pcmd = new PuCmd();
        for (int i = 0; i < REG_LIMIT; i++) {
            pcmd->m_Regs[i] = m_Regs[i];
        }

        return pcmd;
    }

    void init()
    {
        memset(&m_Regs[0], 0, sizeof(PuCmd));
    }

    int updateBytes(int offset, uint8_t *data, int size) {
        if (offset < 0 || (offset + size > sizeof(uint32_t) * REG_LIMIT)) {
            size = size - (offset + size - sizeof(uint32_t) * REG_LIMIT);
        }
        if (size > 0) memcpy(&m_Regs[0] + offset, data, size);

        return size;
    }

    uint8_t * getMemPtr()
    {
        return (uint8_t *)&m_Regs[0];
    }

    uint32_t & operator[] (int idx) {
        assert (idx >= 0 && idx < REG_LIMIT);

        return m_Regs[idx];
    }

    uint32_t get(enum PU_REGS reg)
    {
        return m_Regs[reg];
    }

    uint32_t set(enum PU_REGS reg, uint32_t value)
    {
        return m_Regs[reg] = value;
    }

    void setStatus(enum PU_STATUS_TYPE status)
    {
        m_Regs[REG_STATUS] = status;
    }

    enum PU_STATUS_TYPE getStatus()
    {
        return (enum PU_STATUS_TYPE) m_Regs[REG_STATUS];
    }

    enum PU_CMD_TYPE getCommand()
    {
        return (enum PU_CMD_TYPE) m_Regs[REG_COMMAND];
    }

    enum PU_OP_TYPE getOpcode()
    {
        return (enum PU_OP_TYPE) m_Regs[REG_OPCODE];
    }

    int valid()
    {
        uint32_t ret = m_Regs[REG_VALID];

        return (ret) ?  1 : 0;
    }

    /**
     * @brief Compare two contents
     *
     * @param cmda
     * @param cmdb
     * @return int
     *
     */
    int equals(PuCmd & cmdb)
    {
        for (int i = 0; i < REG_LIMIT; i++) {
            if (m_Regs[i] != cmdb.m_Regs[i]) return 0;
        }
        return 1;
    }

    void print()
    {
        printf("\nBEGIN Gem5 PuCmd-----------------\n");
        printf("    .REG_VALID = %#x\n", m_Regs[REG_VALID]);
        printf("    .REG_STATUS = %#x\n",m_Regs[REG_STATUS]);
        printf("    .REG_COMMAND = %#x\n", m_Regs[REG_COMMAND]);
        printf("    .REG_FLAGS = %#x\n", m_Regs[REG_FLAGS]);
        printf("    .REG_OPCODE = %#x\n",m_Regs[REG_OPCODE]);
        printf("    .REG_AADDR = %#x\n", m_Regs[REG_AADDR]);
        printf("    .REG_AROWS = %#x\n", m_Regs[REG_AROWS]);
        printf("    .REG_ACOLS = %#x\n", m_Regs[REG_ACOLS]);
        printf("    .REG_BADDR = %#x\n", m_Regs[REG_BADDR]);
        printf("    .REG_BROWS = %#x\n", m_Regs[REG_BROWS]);
        printf("    .REG_BCOLS = %#x\n", m_Regs[REG_BCOLS]);
        printf("    .REG_CADDR = %#x\n", m_Regs[REG_CADDR]);
        printf("    .REG_CROWS = %#x\n", m_Regs[REG_CROWS]);
        printf("    .REG_CCOLS = %#x\n", m_Regs[REG_CCOLS]);
        printf("END Gem5 PuCmd----------------\n\n");
    }

};

} // namespace gem5

#endif // __DEV_PU_CMD4_H__
