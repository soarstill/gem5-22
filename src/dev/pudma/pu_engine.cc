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

#include "dev/pudma/pu_engine.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PUDMA.hh"
#include "dev/pudma/pu_core.hh"

namespace gem5
{

PuEngine::PuEngine(const PuEngineParams &params) :
    SimObject(params),
    // This is a C++ lambda. When the event is triggered, it will call the
    // processEvent() function. (this must be captured)
    event([this]{ processEvent(); }, name() + ".event"),
    pucore(params.pucore_object),
    // Note: This is not needed as you can *always* reference this->name()
    myName(params.name),
    latency(params.time_to_wait),
    timesLeft(params.number_of_fires)
{
    DPRINTF(PUDMA, "Created the PuEngine object\n");
    panic_if(!pucore, "PuEngine must have a non-null PuCore object");
}

void
PuEngine::startup()
{
    // Before simulation starts, we need to schedule the event
    schedule(event, latency);
}

void
PuEngine::processEvent()
{
    timesLeft--;
    DPRINTF(PUDMA, "Hello world! Processing the event! %d left\n",
                          timesLeft);

    if (timesLeft <= 0) {
        DPRINTF(PUDMA, "Done firing!\n");
        pucore->compute(myName);
    } else {
        schedule(event, curTick() + latency);
    }
}

} // namespace gem5
