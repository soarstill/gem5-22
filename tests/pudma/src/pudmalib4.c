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

#include "pudmalib2.h"

void asset_valid_range(PioAddr_t pio)
{
    if (!IS_VALID_ADDR(pio)) {
        printf("Exeption: Out of PIO range of PuEngine %#x\n", pio);
        exit(1);
    }
}

/**
 * @brief Write a byte to an address in PuEngine's I/O memory range
 *
 * @param pio
 * @param value
 * @return uint8_t
 *  - value : written byte re-read
 */
uint8_t writePuCmdByte(PioAddr_t pio, uint8_t value)
{
    asset_valid_range(pio);

    outb(value, pio); // write to gem5::PuEngine4

    unsigned char retval = inb(pio);
    assert(retval == value);

    return retval;
}

uint8_t readPuCmdByte(PioAddr_t pio)
{
    asset_valid_range(pio);

    return inb(pio);
}

PioAddr_t readPuCmdBytes(PioAddr_t pio, int size, uint8_t * buf)
{
    asset_valid_range(pio);

    PioAddr_t addr = pio;
    for (int i = 0; i < size; i++, addr++) {
        buf[i] = readPuCmdByte(addr);
    }
    return pio;
}

PioAddr_t writePuCmdBytes(PioAddr_t pio, int size, uint8_t * buf)
{
    asset_valid_range(pio);

    PioAddr_t addr = pio;
    for (int i = 0; i < size; i++, addr++) {
        writePuCmdByte(addr, buf[i]);
    }
}

uint32_t writePuCmdUint32(PioAddr_t pio, uint32_t value)
{
    asset_valid_range(pio);

    writePuCmdBytes(pio, sizeof(uint32_t), (uint8_t *)&value);

    // verification
    uint32_t retVal = 0xFFFFFFFF;
    readPuCmdBytes(pio, sizeof(uint32_t), (uint8_t *)&retVal);
    assert(retVal == value);

    return retVal;
}


uint32_t readPuCmdUint32(PioAddr_t pio)
{
    uint32_t retVal = 0;

    asset_valid_range(pio);

    readPuCmdBytes(pio, sizeof(uint32_t), (uint8_t *)&retVal);

    return retVal;
}

uint32_t readPuCmdReg(enum PU_REGS reg)
{
    if (reg < 0 || reg >= REG_LIMIT)
    {
        printf("Exeption: Out of register index of PuEngine %#x\n", reg);
        exit(1);
    }

    PioAddr_t pio = PUREG_ADDR(reg);
    asset_valid_range(pio);

    uint32_t retVal = readPuCmdUint32(pio);

    return retVal;
}

uint32_t writePuCmdReg(enum PU_REGS reg, uint32_t value)
{
    if (reg < 0 || reg >= REG_LIMIT)
    {
        printf("Exeption: Out of register index of PuEngine %#x\n", reg);
        exit(1);
    }

    PioAddr_t pio = PUREG_ADDR(reg);
    asset_valid_range(pio);

    writePuCmdUint32(pio, value);

    // verification
    uint32_t retVal = readPuCmdUint32(pio);

    assert(retVal == value);

    return retVal;
}


/**
 *
 * @brief int isPuCmdValid() : Check if PuCmd registes are valid
 *
 * @return int
 *  - 1 : true
 *  - 0 : false
 */
int isPuCmdValid()
{

    const PioAddr_t pioValid = PUREG_ADDR(REG_VALID);
    uint32_t ret = readPuCmdByte(pioValid);

    return (ret) ?  1 : 0;
}

void invalidatePuCmd() // lock while writing PuCmd registers (incomplete)
{
    const PioAddr_t pioValid = PUREG_ADDR(REG_VALID);

    writePuCmdByte(pioValid, 0);
}

// for gem5 hooking, if user call validatePuCmd(), then gem5 check that
// that address and the value is 1, then hook
void validatePuCmd() // unlock while writing PuCmd registers
{
    const PioAddr_t pioValid = PUREG_ADDR(REG_VALID);

    writePuCmdByte(pioValid, 1);
}

/*
// for gem5 hooking, if user call validatePuCmd(), then gem5 check that
int checkPuCmdHook(PioAddr_t pio) // for hooking
{
    const PioAddr_t pioValid = PUREG_ADDR(REG_VALID);
    return  (pioValid == pio) ?  1 : 0;
}
*/

/**
 * @brief return status if PuCmd is valid
 *
 * @return * uint32_t
 *  - 0 : not valid
 *  - else : status
 */
uint32_t readPuCmdStatus()
{
    return readPuCmdReg(REG_STATUS);
}

void initPuCmd(PuCmd cmd)
{
    memset(&cmd[0], 0, sizeof(PuCmd));
}

/**
 * @brief Compare two contents
 *
 * @param cmda
 * @param cmdb
 * @return int
 *
 */
int isPuCmdEqual(PuCmd cmda,  PuCmd cmdb)
{
    for (int i = 0; i < REG_LIMIT; i++) {
        if (i = REG_STATUS) continue; // READ ONLY
        if (cmda[i] != cmdb[i]) return 0;
    }
    return 1;
}

// forced write regardless of PuEngine
int writePuCmd(PuCmd cmd)
{
    invalidatePuCmd(); // start to transmission (valid = 0)

    PioAddr_t addr = PUREG_ADDR(REG_STATUS) ; // the 2nd register
    PioAddr_t end = PUREG_ADDR(REG_LIMIT);
    uint8_t * pValue = (uint8_t *)&cmd[REG_STATUS];
    for (; addr < end; addr++, pValue++) {
        if (addr == PUREG_ADDR(REG_STATUS)) continue; // READ ONLY
        writePuCmdByte(addr, *pValue );
    }

    validatePuCmd(); //finish transmission (valid = 1) -> gem5 hook

    return 1;
}


/**
 * @brief return PuEngin's PuCmd Registers into cmd
 *
 * @param cmd Read all the PuEngine registers including valid reg.
 * @return int
 */
int readPuCmd(PuCmd cmd)
{
    /* for gem5
    if (!isPuCmdValid()) {
            return 0;
    }
   */
    PioAddr_t addr = PUREG_ADDR(REG_VALID); // the first reg.
    PioAddr_t end = PUREG_ADDR(REG_LIMIT);
    uint8_t * pValue = (uint8_t *)&cmd[REG_VALID];
    for (int i = 0; addr < end; addr++, pValue++) {
        pValue[i] = readPuCmdByte(addr);
    }

    return 1;
}

void printPuCmd(PuCmd cmd)
{
    printf("\nBEGIN PuCmd-----------------\n");
    printf("    .REG_VALID = %#x\n", cmd[REG_VALID]);
    printf("    .REG_STATUS = %#x\n",cmd[REG_STATUS]);
    printf("    .REG_COMMAND = %#x\n", cmd[REG_COMMAND]);
    printf("    .REG_FLAGS = %#x\n", cmd[REG_FLAGS]);
    printf("    .REG_OPCODE = %#x\n",cmd[REG_OPCODE]);
    printf("    .REG_AADDR = %#x\n", cmd[REG_AADDR]);
    printf("    .REG_AROWS = %#x\n", cmd[REG_AROWS]);
    printf("    .REG_ACOLS = %#x\n", cmd[REG_ACOLS]);
    printf("    .REG_BADDR = %#x\n", cmd[REG_BADDR]);
    printf("    .REG_BROWS = %#x\n", cmd[REG_BROWS]);
    printf("    .REG_BCOLS = %#x\n", cmd[REG_BCOLS]);
    printf("    .REG_CADDR = %#x\n", cmd[REG_CADDR]);
    printf("    .REG_CROWS = %#x\n", cmd[REG_CROWS]);
    printf("    .REG_CCOLS = %#x\n", cmd[REG_CCOLS]);
    printf("END PuCmd----------------\n\n");
}
