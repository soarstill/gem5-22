/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Definitions
#define REG32(x) (*((volatile unsigned int *)(x)))
#define DMA_ADDRS   REG32(0x10000000)
#define DMA_ADDRD   REG32(0x10000004)
#define DMA_SIZE    REG32(0x10000008)
#define DMA_CMD     REG32(0x1000000C)
#define DMA_STATUS  REG32(0xC0000010)

// lib headers
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/io.h>

typedef struct _DmaInfo
{
    float * pa;
    float * pb;
    float * pc;
    unsigned int size;
    unsigned int cmd;
    // cmd = 0: just send info, 1: start compute,
    // 2: abort  3: report status
    unsigned int status;
    // status 0: idle, 1: ready, 2: start-moving data,
    // 3: in-computing 4: moving to mem 5: DONE
} DmaInfo;

DmaInfo dmaInfo ;
DmaInfo dmaInfoRet ;

volatile const unsigned long long pioAddr = 0x300;
volatile const unsigned long long pioSize = 0x400;

float dataA[1024];
float dataB[1024];
float dataC[1024];


void printDmaInfo(DmaInfo * di)
{
    printf("-- DMA Info ---\n");
    printf("  .pa = %#x \n", di->pa);
    printf("  .pb = %#x \n", di->pb);
    printf("  .pc = %#x \n", di->pc);
    printf("  .size = %d \n", di->size);
    printf("  .cmd = %d \n", di->cmd);
    printf("  .status = %d \n\n", di->status);
}

int isEqual(DmaInfo *da, DmaInfo * db)
{
    if (da->pa != da->pa) return 0;
    if (da->pb != da->pb) return 0;
    if (da->pc != da->pc) return 0;
    if (da->size != da->size) return 0;
    if (da->cmd != da->cmd) return 0;
    if (da->status != da->status) return 0;

    printf("status: 0: idle, 1: ready, 2: start-moving data,\
        3: in-computing 4: moving to mem 5: DONE\n");
    printf("command:  0: just send info, 1: start compute, \
        2: abort 3: report status\n");

    return 1;
}


void testPio1()
{
    memset(&dmaInfo, 0, sizeof(dmaInfo));

    dmaInfo.pa = &dataA[0];
    dmaInfo.pb = &dataB[0];
    dmaInfo.pc = &dataC[0];
    dmaInfo.size = 1024;
    dmaInfo.cmd = 0;
    // 0: just send info, 1: start compute, 2: abort
    // 3: report status
    dmaInfo.status = 0;
    // 0: idle, 1: ready, 2: start-moving data,
    // 3: in-computing 4: moving to mem 5: DONE

    printDmaInfo(&dmaInfo);
    outsb(pioAddr, (void *)&dmaInfo, sizeof(dmaInfo));

    memset(&dmaInfoRet, 0, sizeof(dmaInfo));
    insb(pioAddr, (void *)&dmaInfoRet, sizeof(DmaInfo));
    printDmaInfo(&dmaInfoRet);
}



int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");
    printf("VA = %#x\n", &dmaInfo);

    testPio1();

    // printed 0x100 (NOT 0x03) (hooked by gem5), READ REQ hook by PuEngine2
    printf("Hello world! - PU Done!\n\n");
    return 0;
}

