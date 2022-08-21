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


    PuCmd ret;
    initPuCmd(ret);
    readPuCmd(ret);

    printf("PuCmd re-read\n");
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

int main(int argc, char* argv[])
{
    printf("Hello world! - PU\n");

    PuCmd cmd ;

    initPuCmd(cmd);

    // dummy data
    for (int i = 0; i < sizeof(enum PU_REGS); i++) {
        cmd[i] = 0x100 + i;
    }

    testWritePuCmd(cmd);

    printf("Hello world! - PU Done!\n\n");

    //sleep(5); // gem5 doesn't support
    //usleep(5000); // gem5 doesn't support
    //volatile long long count = 1000000;
    // 10000000000 = about 3 seconds in bear metal
    //while (count--);

   // printf("Hello world! - after while () - terminated. !\n\n");

    return 0;
}

