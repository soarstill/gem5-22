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


#ifndef __PUDMA_PUCORE4_HH__
#define __PUDMA_PUCORE4_HH__

#include <string>

#include "base/trace.hh"
#include "dev/dma_device.hh"
#include "dev/pudma/pu_engine4.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/PuCore4.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

struct PuCmd;
class PuEngine4;

class PuCore4 : public SimObject
{
  PARAMS(PuCore4)

  private:
    /**
     * Fill the buffer with the next chunk of data
     */
    void processEvent();

    /// An event that wraps the above function
    EventFunctionWrapper event;

    /**
     * Fills the buffer for one iteration. If the buffer isn't full, this
     * function will enqueue another event to continue filling.
     */
    void fillBuffer();

    /// The bytes processed per tick
    float bandwidth;

    /// The size of the buffer we are going to fill
    int bufferSize;

    /// The buffer we are putting our message in
    char *buffer;

    /// The message to put into the buffer.
    std::string message;

    /// The amount of the buffer we've used so far.
    int bufferUsed;

    uint8_t *bufferA;
    uint8_t *bufferB;
    uint8_t *bufferC;

    int bufferSizeA;
    int bufferSizeB;
    int bufferSizeC;

    std::string opcode ;
    Tick coreLatency;
    PuEngine4 * puengine4;

  public:

   PuCore4(const PuCore4Params &p);
    ~PuCore4();

    /**
     * Called by an outside object. Starts off the events to fill the buffer
     * with a goodbye message.
     *
     * @param name the name of the object we are saying goodbye to.
     */
    void compute(PuCmd cmd, PuEngine4 * pue4);

    void sayHello(std::string mesg, PuEngine4 * pue4);

    void startup() override;

  protected:
    void OnComplete(PuEngine4 * pue4);
};

} // namespace gem5

#endif // __PUDMA_PUCORE4_HH__
