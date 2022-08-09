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
#include <sys/io.h>

typedef struct _DmaInfo
{
    unsigned int src;
    unsigned int dst;
    unsigned int size;
    unsigned int cmd;
    unsigned int status;
} DmaInfo;

volatile DmaInfo dmaInfo =
    {0x01,0x02,0x03,0x04,0x05};

unsigned long long pioAddr = 0x300;
unsigned long long pioSize = 0x400;

void testPio()
{

    outsb(pioAddr, (void *)&dmaInfo, 16);
    insb(pioAddr, ( void *)&dmaInfo, 16);
}

int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");
    printf("VA = %#x\n", &dmaInfo);
    dmaInfo.src = 0x03; // WRITE REQ hook the v address in PuEngine2, 0x100

    testPio();

    // printed 0x100 (NOT 0x03) (hooked by gem5), READ REQ hook by PuEngine2
    printf("Hello world! - Done! Dma.src = %#x\n", dmaInfo.src);
    return 0;
}

