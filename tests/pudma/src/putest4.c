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

#include "pudmalib4.h"

// Unit tests
int testwritePuCmdUint32(PioAddr_t pio, uint32_t wvalue)
{

    uint32_t retval = writePuCmdUint32(pio, wvalue);

    printf("Hello: testWRint read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}


int testWRbyte(PioAddr_t pio, uint8_t wvalue)
{

    uint32_t retval = writePuCmdByte(pio, wvalue);

    printf("Hello: testWRbyte read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}

int testWritePuCmd(PuCmd cmd)
{

    printf("PuCmd to be written\n");
    printPuCmd(cmd);

    writePuCmd(cmd);

    printf("PuCmd reading\n");

    PuCmd ret;
    initPuCmd(ret);
    //do {
        readPuCmd(ret);
    //} while (! (ret[REG_STATUS] == STS_READY)) ;

    printf("PuCmd re-read!!\n");
    printPuCmd(ret);

    // success or failure of the test
    if ( isPuCmdEqual(cmd,ret)) {
        printf("testWritePuCmd : SUCCESS\n");
        return 1;
    } else {
        printf("testWritePuCmd : FAIL\n");
        return 0;
    }
}
#define ROWS 100
#define COLS 100

double A[ROWS][COLS];
double B[ROWS][COLS];
double C[ROWS][COLS];

//#define ROM1_BASE (0xfff0000000)
//#define ROM2_BASE (0xfff8000000)
//#define DRAM2_BASE (0x800000000)

int initMatrixA()
{
    int count = 0;

    for (int i = 0; i < ROWS; i++) {
        for (int j =0 ; j < COLS; j++) {
            A[i][j] = (double)i;
            count++;
        }
    }
    return count;
}
int initMatrixB()
{
    int count = 0;

    for (int i = 0; i < ROWS; i++) {
        for (int j =0 ; j < COLS; j++) {
            B[i][j] = (double)j;
            count++;
        }
    }
    return count;
}

int addMatrix()
{
    printf("Start addMatrix() - USER\n");

    initMatrixA(); //"matA.bin");
    initMatrixB(); //"matB.bin");

    int count = 0;
    for (int i = 0; i < ROWS; i++) {
        for (int j =0 ; j < COLS; j++) {
            C[i][j] = A[i][j] + B[i][j];
            assert(C[i][j] == i+j);
            count++;
        }
    }
    printf("End addMatrix() - USER\n");

    return count;
}

void gem5AddMatrix(int rows, int cols)
{
    printf("Start gem5AddMatrix() - USER\n");

    PuCmd cmd;

    cmd[REG_COMMAND] = CMD_START;
    cmd[REG_OPCODE] = OP_ADD;
    cmd[REG_AADDR] = 0;
    cmd[REG_AROWS] = rows;
    cmd[REG_ACOLS] = cols;
    cmd[REG_BADDR] = 0;
    cmd[REG_BROWS] = rows;
    cmd[REG_BCOLS] = cols;
    cmd[REG_CADDR] = 0;
    cmd[REG_CROWS] = rows;
    cmd[REG_CCOLS] = cols;

    writePuCmd(cmd); // xfer command

    printf("Waiting gem5AddMatrix().. - USER\n");
    while (readPuCmdStatus() != STS_COMPLETED);
    printf("End gem5AddMatrix() - USER\n");
}

int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");
    addMatrix();

/*
    PuCmd cmd ;

    initPuCmd(cmd);


    for (int i = 0; i < REG_LIMIT; i++) {
        cmd[i] = 0x100 + (uint32_t)i;
    }
    cmd[REG_COMMAND] = CMD_START;
    cmd[REG_OPCODE] = OP_ADD;
    testWritePuCmd(cmd);
*/

    printf("Start test Add! - PU\n");

    gem5AddMatrix(100,100);
    gem5AddMatrix(50,50);


    printf("Hello world! - PU Done!\n\n");

    return 0;
}

