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

#include "dev/pudma/pu_core3.hh"
#include "base/trace.hh"
#include "debug/PuEngine3.hh"
#include "params/PuCore3.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

PuCore3::PuCore3(const PuCore3Params &p) :
    SimObject(p), event([this]{ processEvent(); }, name()+".event"),
    bandwidth(p.write_bandwidth), bufferSize(p.buffer_size),
    buffer(nullptr), bufferUsed(0)
{
    buffer = new char[bufferSize]();
    DPRINTF(PuEngine3, "Created a PuCore3 object\n");
}

PuCore3::~PuCore3()
{
    delete[] buffer;
}

void
PuCore3::processEvent()
{
    DPRINTF(PuEngine3, "PuCore3::processing the event!\n");

    // Actually do the "work" of the event
    fillBuffer();
}

void
PuCore3::compute(std::string other_name)
{
    DPRINTF(PuEngine3, "PuCore3: Start compute to %s\n", other_name);

    message = "Computing " + other_name + "!! ";

    // Kick off the the first buffer fill. If it can't fill the whole buffer
    // because of a limited bandwidth, then this function will schedule another
    // event to finish the fill
    fillBuffer();
}

void
PuCore3::fillBuffer()
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
        DPRINTF(PuEngine3,"Scheduling another fillBuffer in %d ticks\n",
                bandwidth * bytes_copied);
        schedule(event, curTick() + bandwidth * bytes_copied);
    } else {
        DPRINTF(PuEngine3, "PuCore3: done copying!\n\n");
        // Be sure to take into account the time for the last bytes
        exitSimLoop(buffer, 0, curTick() + bandwidth * bytes_copied);
    }
}

} // namespace gem5
