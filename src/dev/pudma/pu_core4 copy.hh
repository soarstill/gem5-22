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
#include "debug/PuEngine4.hh"
#include "dev/dma_device.hh"
#include "dev/pudma/pu_engine4.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/PuCore4.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

class PuCmd;
class PuEngine4;

class PuCore4 : public SimObject
{
  PARAMS(PuCore4)

  public:

    PuCore4(const PuCore4Params &p);
    ~PuCore4();

    /**
     * @brief Compute user request
     *
     * @param cmd
     * @param pue4
     */
    void compute(PuCmd * cmd, PuEngine4 * pue4);
    void sayHello(std::string mesg, PuEngine4 *pue4);

    void startup() override;

    void OnComplete(PuEngine4 * pue4);

  EventFunctionWrapper event;
  void processEvent() {} ;
private:
    uint8_t *bufferA;
    uint8_t *bufferB;
    uint8_t *bufferC;

    int bufferSizeA;
    int bufferSizeB;
    int bufferSizeC;

    std::string opcode ;
    Tick coreLatency;

    Addr m_dram2_base;
    // DRAM 2's start address (2G+256MB)
    Addr m_rom1_base; // Data A
    Addr m_rom2_base ; // Data B

private:

    // functions for Matrix add operation
    void startAdd(PuCmd *cmd, PuEngine4 *pue4); // init
    void processAdd(PuCmd *cmd, PuEngine4 *pue4); // Callback
    Tick doAdd(PuCmd *cmd); // calulate C = A + B
    EventFunctionWrapper eventAdd;


}; // end class PuCore4

} // end namespace gem5

#endif // end __PUDMA_PUCORE4_HH__
