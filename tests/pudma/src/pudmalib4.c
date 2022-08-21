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

#ifndef __DEV_PUDMA_PUDMALIB_H__
#define __DEV_PUDMA_PUDMALIB_H__

// lib headers
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;

#define DMA_PIO_ADDR (0x300)    // Device dependent, range (0x300~0x700)
#define DMA_PIO_SIZE (0x400)    // Device dependent, range (0x300~0x700)

#define X86PIO_BASE_ADDR (0x8000000000000000)

typedef struct _PuCmd // DO NOT MODIFY
{
    uint32_t valid; // 0 : invalid all, 1 : valid
    uint32_t status;
    // status 0: invalid, 1: ready, 2: moving data,
    // 3: in-computing 4: moving to mem 5: idle
    uint32_t cmd;
    // cmd = 0: invalid command, 1: start compute,
    // 2: abort  3: report status
    uint32_t flags;
    uint32_t opcode; // 0: no op, 1: add 2: sub, 3: mul
    uint32_t addrA; // A buffer physical memory address
    uint32_t addrB; // B buffer physical memory address
    uint32_t addrC; // C buffer physical memory address
    uint32_t sizeA; // size of A
    uint32_t sizeB; // size of B
    uint32_t sizeC; // size of C
} PuCmd;

volatile const unsigned long long pioAddr = 0x300;
volatile const unsigned long long pioSize = 0x400;

// Not sure to copy this virtual address memory to the gem5 DMA buffers
float dataA[1024];
float dataB[1024];
float dataC[1024];

PuCmd UserCmdReg;

void printPuCmd(PuCmd * cmd)
{
    printf("\nPuCmd----\n");

    printf("\t.valid = %#x\n", cmd->valid);
    printf("\t.status = %#x\n", cmd->status);
    printf("\t.cmd = %#x\n", cmd->cmd);
    printf("\t.flags = %#x\n", cmd->flags);
    printf("\t.opcode = %#x\n", cmd->opcode);
    printf("\t.addrA = %#x\n", cmd->addrA);
    printf("\t.addrB = %#x\n", cmd->addrB);
    printf("\t.addrC = %#x\n", cmd->addrC);
    printf("\t.sizeA = %#x\n", cmd->sizeA);
    printf("\t.sizeB = %#x\n", cmd->sizeB);
    printf("\t.sizeC = %#x\n", cmd->sizeC);
    printf("PuCmd----\n\n");
}

int isEqual(PuCmd *cmda, PuCmd * cmdb)
{

    if ((cmda->valid == cmdb->valid)    &&
        (cmda->status == cmdb->status)  &&
        (cmda->cmd == cmdb->cmd)        &&
        (cmda->opcode == cmdb->opcode)  &&
        (cmda->flags == cmdb->flags)    &&
        (cmda->addrA == cmdb->addrA)    &&
        (cmda->addrB == cmdb->addrB)    &&
        (cmda->sizeA == cmdb->sizeA)    &&
        (cmda->sizeB == cmdb->sizeC)    &&
        (cmda->sizeC == cmdb->sizeC))
    {
        return 1;
    }
    return 0;
}

unsigned char writePuByte(unsigned short pio, unsigned char value)
{
    if (pio <  DMA_PIO_ADDR || pio >= DMA_PIO_ADDR + DMA_PIO_SIZE) {
        printf("User writePuByte: Out of PIO range %#x\n", pio);
        exit(1);
    }
    outb(value, pio); // write to gem5::PuEngine4

    unsigned char retval = inb(pio);
    assert(retval == value);

    return retval;
}


unsigned char readPuByte(unsigned short pio)
{
    if (pio <  DMA_PIO_ADDR || pio >= DMA_PIO_ADDR + DMA_PIO_SIZE) {
        printf("User readPuByte: Out of PIO range %#x\n", pio);
        exit(1);
    }
    unsigned char retval = inb(pio);

    return retval;
}

unsigned int readPuInt(unsigned short pio)
{
    if (pio <  DMA_PIO_ADDR || pio >= DMA_PIO_ADDR + DMA_PIO_SIZE) {
        printf("User readPuInt: Out of PIO range %#x\n", pio);
        exit(1);
    }
    unsigned int retval = inl(pio);

    return retval;
}

unsigned int writePuInt(unsigned short pio, unsigned int value)
{
    if (pio <  DMA_PIO_ADDR || pio >= DMA_PIO_ADDR + DMA_PIO_SIZE) {
        printf("User writePuInt: Out of PIO range %#x\n", pio);
        exit(1);
    }
    outl(value, pio); // write to gem5::PuEngine4

    unsigned int retval = inl(pio);
    assert(retval == value);

    return retval;
}

int writePuCmd(PuCmd * cmd)
{
    if (!cmd) {
        return 0; // invalid and error
    }
    const unsigned short addrValid = pioAddr;
    unsigned short addr = pioAddr;

    // NEVER change the order : just follow PuCmd structure
    writePuInt(addrValid, 0 );       addr += sizeof(uint32_t);
    writePuInt(addr, cmd->status);   addr += sizeof(uint32_t);
    writePuInt(addr, cmd->cmd);      addr += sizeof(uint32_t);
    writePuInt(addr, cmd->opcode);   addr += sizeof(uint32_t);
    writePuInt(addr, cmd->opcode );  addr += sizeof(uint32_t);
    writePuInt(addr, cmd->flags);    addr += sizeof(uint32_t);
    writePuInt(addr, cmd->addrA );   addr += sizeof(uint32_t);
    writePuInt(addr, cmd->addrB );   addr += sizeof(uint32_t);
    writePuInt(addr, cmd->sizeA );   addr += sizeof(uint32_t);
    writePuInt(addr, cmd->sizeB );   addr += sizeof(uint32_t);
    writePuInt(addrValid, cmd->valid);       // validate after write
    // NEVER change the order : just follow PuCmd structure

    // validate
    int retValid = readPuInt(addrValid);
    if (retValid) {
        UserCmdReg = *cmd; // Update User Level PuCmdRegister
    }
    return retValid; // 0: invalid (fail), 1: valid (success)
}

uint32_t readPuStatus()
{
    const unsigned short validAddr = pioAddr;
    const unsigned short statusAddr = pioAddr + 1 * sizeof(uint32_t);

    uint32_t status = 0xFFFFFFFF; // NOT a state

    uint32_t valid = readPuInt(validAddr);
    uint32_t retStatus = readPuInt(statusAddr);
    if (valid) status = retStatus;

    // fail: 0xFFFFFFFF (invalid status), success otherwise (valid status)
    return  status;
}

uint32_t writePuStatus(uint32_t status)
{
    const unsigned short validAddr = pioAddr;
    const unsigned short statusAddr = pioAddr + 1 * sizeof(uint32_t);

    uint32_t retStatus = writePuInt(statusAddr, status);

    return  retStatus;
}

#endif // __DEV_PUDMA_PUDMALIB_H__