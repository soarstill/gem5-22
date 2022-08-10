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

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;

#define X86PIO_BASE_ADDR (0x8000000000000000)
#define SRC_REG REG32(X86PIO_BASE_ADDR+0x300)

typedef struct _DmaInfo
{
    uint32_t pa;
    uint32_t pb;
    uint32_t pc;
    uint32_t size;
    uint32_t cmd;
    // cmd = 0: just send info, 1: start compute,
    // 2: abort  3: report status
    uint32_t status;
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


int testWRint(unsigned short pio, unsigned int wvalue)
{

    outl(wvalue, pio);
    printf("Hello: testWRint written = %#x\n", wvalue);

    // read from gem5::PuEngine3
    unsigned int retval = inl(pio);
    printf("Hello: testWRint read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}


int testWRbyte(unsigned short pio, unsigned char wvalue)
{

    outb(wvalue, pio);
    printf("Hello: testWRbyte written = %#x\n", wvalue);

    // read from gem5::PuEngine3
    unsigned char retval = inb(pio);
    printf("Hello: testWRbyte read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}


// PIO 포트 테스트 : 성공, pio 주소 + X86 PIO BASE 로 gem5에 전달됨
void test1()
{
    memset(&dmaInfo, 0, sizeof(dmaInfo));
    memset(&dmaInfoRet, 0, sizeof(dmaInfoRet));
    printDmaInfo(&dmaInfo);

    dmaInfo.pa = (uint32_t) &dataA[0];
    dmaInfo.pb = (uint32_t) &dataB[0];
    dmaInfo.pc = (uint32_t) &dataC[0];
    dmaInfo.size = (uint32_t) 1024;
    dmaInfo.cmd = (uint32_t) 0;
    // 0: just send info, 1: start compute, 2: abort
    // 3: report status
    dmaInfo.status = (uint32_t) 0;
    // 0: idle, 1: ready, 2: start-moving data,
    // 3: in-computing 4: moving to mem 5: DONE

    uint32_t status = 0x100;

    //printDmaInfo(&dmaInfo);
    printf("Hello: status written = %d\n", status);
    outsb(pioAddr, (void *)&status, sizeof(status));

    // change value
    status = 0x200;
    printf("Hello: status changed = %d\n", status);

    // read from gem5::PuEngine3
    insb(pioAddr, (void *)&status, sizeof(status));
    printf("Hello: status re-read = %d\n", status); // should be #0x100
}


// 직접 주소 접근 테스트 : 실패 (page fault)
void test2()
{
    SRC_REG = 0x500;
}
// inl(), outl() 테스트 : 4바이트 - Success
void test3()
{
    int status = 0x100;
    int retval = 0x000;

    outl(status, pioAddr);
    printf("Hello: status written = %#x\n", status);// 0x100
                                                   //
    // read from gem5::PuEngine3
    retval = inl(pioAddr);
     printf("Hello: status re-read = %#x %s\n",
        retval, retval==status ? "SUCCESS" : "FAIL"); // should be staus
}

// insb(), outsb() - FAIL
void test4()
{
    int status = 0x300;
    int retval = 0x600;

    outsb(pioAddr, &status, sizeof(status));
    printf("Hello: status written = %#x\n", status);// 0x300
                                                   //
    // read from gem5::PuEngine3
    insb(pioAddr, &retval, sizeof(retval));
    printf("Hello: status re-read = %#x %s\n",
        retval, retval==status ? "SUCCESS" : "FAIL"); // should be staus
}


// insb(), outsb() - FAIL
void test5()
{
    char bstatus[8] = {'S', 'O', 'N', 'G', '!', '!', '!', '\0'};
    char bretval[8] =  {'N', 'E', 'V', 'E', 'R', '!', '!','\0'};

    outsb(pioAddr, bstatus, sizeof(bstatus));
    printf("Hello: status written = %s\n", bstatus);

    // read from gem5::PuEngine3
    insb(pioAddr, bretval, sizeof(bretval));
    printf("Hello: status re-read = '%s' ", bretval);
    printf("( %s )\n", !strcmp(&bstatus[0], &bretval[0]) ? "SUCCESS" : "FAIL");
    }

// inb(), outb() - Fail
void test6()
{
    char bstatus[8] = {'S', 'O', 'N', 'G', '!', '!', '!', '\0'};
    char bretval[8] =  {'N', 'E', 'V', 'E', 'R', '!', '!','\0'};

    for (int i = 0; i < sizeof(bstatus); i++ ) {
        outb(bstatus[i], pioAddr);
    }
    printf("Hello: status written = %s\n", bstatus);

    // read from gem5::PuEngine3
    for (int i = 0; i < sizeof(bstatus); i++ ) {
        bstatus[i] = inb(pioAddr);
    }
    printf("Hello: status re-read = '%s' ", bretval);
    printf("( %s )\n", !strcmp(&bstatus[0], &bretval[0]) ? "SUCCESS" : "FAIL");
    // should be staus
}

// inb, outb 성공
void test7()
{
  char status = 0x10;
  char retval = 0x20;

    outb(status, pioAddr);
    printf("Hello: status written = %#x\n", status);// 0x100
                                                   //
    // read from gem5::PuEngine3
    retval = inb(pioAddr);
    printf("Hello: status re-read = %#x %s\n",
        retval, retval==status ? "SUCCESS" : "FAIL"); // should be staus
}

// inb(), outb() buffer
void test8()
{
    int bstatus[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
    int bretval[8] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x00};;
    //char bstatus[8] = {'S', 'O', 'N', 'G', '!', '!', '!', '\0'};
    //char bretval[8] =  {'N', 'E', 'V', 'E', 'R', '!', '!','\0'};

    for (int i = 0; i < 8; i++ ) {
        outl(bstatus[i], pioAddr+i*sizeof(int));
    }
    printf("Hello: status written = %s\n", bstatus);

    // read from gem5::PuEngine3
    for (int i = 0; i < 8; i++ ) {
        bstatus[i] = inl(pioAddr+i*sizeof(int));
    }
    printf("Hello: status re-read = '%#x' ", bretval[0]);
    printf("( %s )\n", ((bstatus[0] == bretval[0]) ? "SUCCESS" : "FAIL"));
    // should be staus
}

// inl(), outl() 테스트 : 4바이트 - Success
void test9()
{
    int bstatus[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    int bretval[8] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};;

   unsigned short addr = 0x304;
   int i = 7;
   //for (int i = 0; i < 8; i++) {
        outl(bstatus[i], pioAddr);
        printf("Hello: status written = %#x\n", bstatus[i]);// 0x100
                            //
        // read from gem5::PuEngine3
        addr = addr + (i * sizeof(int));
        bretval[i] = inl(addr);
        printf("Hello: status re-read = %#x %s\n",
            bretval[i], bretval[i]==bstatus[i] ? "SUCCESS" : "FAIL");

   //}
}


void test10() // success
{
    int value = 0x7;
    int ret = 0x0;
    unsigned short pio = 0x400;

    ret = testWRint(pio, value);

    ret = testWRint(pio + 0x10, value+0x10);
}

void test11() // success
{
    int value = 0x9;
    int ret = 0x0;
    unsigned short pio = 0x350;

    ret = testWRbyte(pio, value);

    ret = testWRbyte(pio + 0x100, value+0x100);
}

int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");
    DmaInfo * p ;

    // test3(); // SUCCESS - inl, outl
    //test4(); // FAIL - insb, outsb
    // test5(); // FAIL - insb, outsb
    // test6(); for loop
     // test9(); // inb, outb
    test11();

    printf("Hello world! - PU Done!\n\n");

    return 0;
}

