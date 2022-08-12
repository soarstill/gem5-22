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

#include "pudmalib.h"

// Unit tests
int testWRint(unsigned short pio, unsigned int wvalue)
{

    unsigned int retval = writePuInt(pio, wvalue);

    printf("Hello: testWRint read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}


int testWRbyte(unsigned short pio, unsigned char wvalue)
{

    unsigned int retval = writePuByte(pio, wvalue);

    printf("Hello: testWRbyte read = %#x %s\n",
        retval, retval==wvalue ? "SUCCESS" : "FAIL"); // should be the same

    return retval;
}

void test11() // success
{
    int value = 0x9;
    int ret = 0x0;
    unsigned short pio = 0x350;

    ret = testWRbyte(pio, value);

    ret = testWRint(pio + 0x100, value+0x100);
}


void test12() // success
{
    int value = 0x9;
    int ret = 0x0;
    unsigned short pio = 0x350;

    ret = testWRint(pio + 0x500, value+0x100);
}

void testHookByte(unsigned char cmd)
{
    writePuByte(0x300, cmd); // Valid dma command
    printf("Hello User Hook! - %#8x\n", cmd);
}

void testHookInt(unsigned int cmd)
{
    writePuInt(0x300, cmd); // Valid dma command
    printf("Hello User Hook! - %#8x\n", cmd);
}

void testHookPuCmd(PuCmd * cmd)
{
    writePuInt(0x300, cmd->status); // Valid dma command
    printf("Hello User Hook! - %#8x\n", cmd->status);
}

int testWritePuStatus(uint32_t status)
{
    int retStatus = writePuStatus(status); // Valid dma command
    printf("testWritePuStatus : %s\n",
        (status == retStatus) ? "SUCCESS" : "FAIL");

    return 0;
}

int testWritePuCmd(PuCmd * cmd)
{
    int ret =  writePuCmd(cmd);
    printf("testWritePuCmd : %s\n", (ret) ? "SUCCESS" : "FAIL");

    return 0;
}

int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");

    //testHookByte(0x23);
    //testHookInt(0x345);

    testWritePuStatus(0x678);

    PuCmd cmd = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};

    testWritePuCmd(&cmd);

    printf("Hello world! - PU Done!\n\n");

    //sleep(5); // gem5 ㄴdoesn't support
    printf("Hello world! - After sleep() - terminated. !\n\n");

    return 0;
}

