/*
 * Copyright (c) 2022 H.S.Song
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

#include "dev/pudma/pu_core4.hh"
#include "base/trace.hh"
#include "debug/PuEngine4.hh"
#include "params/PuCore4.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

PuCore4::PuCore4(const PuCore4Params &p) :
    SimObject(p), event([this]{ processEvent(); }, name()+".event"),
    bandwidth(p.write_bandwidth), bufferSize(p.buffer_size),
    buffer(nullptr), bufferUsed(0),
    bufferSizeA(p.buffer_size_a),
    bufferSizeB(p.buffer_size_b),
    bufferSizeC(p.buffer_size_c)
{
    coreLatency = p.core_latency;
    opcode = p.opcode;

    buffer = new char[bufferSize]();
    bufferA = new uint8_t[bufferSizeA]();
    bufferB = new uint8_t[bufferSizeB]();
    bufferC = new uint8_t[bufferSizeC]();

    panic_if((!buffer || !bufferA || !bufferB || !bufferC),
        "Could not allocate one of the buffers");

    DPRINTF(PuEngine4, "Created a PuCore4 object with buffer of %d\n",
                        bufferSize);
}

PuCore4::~PuCore4()
{
    delete[] buffer;
}

void
PuCore4::startup()
{
    // schedule(event, p.core_latency);
    DPRINTF(PuEngine4, "PuCore4: startup() called!\n");
}

void
PuCore4::compute(PuCmd puCmd, PuEngine4 * pue4)
{
    puengine4 = pue4;

    puengine4->setStatus(STS_READY);

    DPRINTF(PuEngine4, "PuCore4: Start compute : cmd = %#x\n",\
                puCmd.get(REG_COMMAND));

    message = "Request Compute from" + pue4->name() +  "!!";

    // Kick off the the first buffer fill. If it can't fill the whole buffer
    // because of a limited bandwidth, then this function will schedule another
    // event to finish the fill
    fillBuffer();
}

void PuCore4::sayHello(std::string mesg, PuEngine4 *pue4)
{
    std::string message = "Hello " + mesg + "from" + pue4->name() + "!! ";
    DPRINTF(PuEngine4, "PuCore4: %s\n", message);

    // Prepare DMA dummy request test
    // X Addr addr = Addr('2GiB'); // DRAM 2's start address (2G+256MB)
    Addr addr = 0x800000000; // DRAM 2's start address (2G+256MB)
    int size = 1024;
    bufferA[0] = 0x12;
    bufferB[0] = 0x34;
    bufferA[1023] = 0x23;
    bufferB[1023] = 0x46;

    Addr rom1_addr = 0xfff0000000;
    Addr rom2_addr = 0xfff8000000;

    auto event =  new EventFunctionWrapper(
            [this, pue4]{ OnComplete(pue4); }, "DMA Complete event");


    DPRINTF(PuEngine4, "PuCore4::Start DMA... at %d \n", curTick());

    pue4->dmaWrite(addr, size, event, (uint8_t *)&bufferA[0]);
    pue4->dmaRead(rom1_addr, size, event, (uint8_t *)&bufferB[0]);

    DPRINTF(PuEngine4, "PuCore4::Done DMA.. at %d \n", curTick());

}

void
PuCore4::OnComplete(PuEngine4 * pue4)
{
    DPRINTF(PuEngine4,
    "PuCore4::Complete Callback DMA  %s for %s A(%d) B(%d) !! at  %d\n",
            (bufferA[0]==bufferB[0] && bufferA[1023]==bufferB[1023]) ?
                    "SUCCESS" : "FAIL", pue4->name(),
                    bufferA[1023], bufferB[1023], curTick());
}

void
PuCore4::processEvent()
{
    DPRINTF(PuEngine4, "PuCore4::processing the event!\n");

    // Actually do the "work" of the event
    fillBuffer();
}

void
PuCore4::fillBuffer()
{
    // There better be a message
    assert(message.length() > 0);

    // Copy from the message to the buffer per byte.
    int bytes_copied = 0;
    for (auto it = message.begin();
         it < message.end() && bufferUsed < bufferSize - 1;
         it++, bufferUsed++, bytes_copied++) {
        // Copy the character into the buffer
        buffer[bufferUsed] = *it;
    }

    if (bufferUsed < bufferSize - 1) {
        // Wait for the next copy for as long as it would have taken
        DPRINTF(PuEngine4,"Scheduling another fillBuffer in %d ticks\n",
                bandwidth * bytes_copied);
        schedule(event, curTick() + bandwidth * bytes_copied);
    } else {
        DPRINTF(PuEngine4, "PuCore4: done copying!\n\n");
        // Be sure to take into account the time for the last bytes
        exitSimLoop(buffer, 0, curTick() + bandwidth * bytes_copied);
    }
}

} // namespace gem5
