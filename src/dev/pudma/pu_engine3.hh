/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * This devices just panics when touched. For example if you have a
 * kernel that touches the frame buffer which isn't allowed.
 */

#ifndef __DEV_PUDMA_PUENGINE3_HH__
#define __DEV_PUDMA_PUENGINE3_HH__

// main headers
#include "dev/io_device.hh"
#include "dev/pudma/pu_core3.hh"

// generated headers
#include "debug/PuEngine3.hh"
#include "params/PuEngine3.hh"

// gem5 headers
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

/**
   *  DmaInfo
   *
   *  Information for an PU+DMA operation
   *  Play a role of the device register
   */
  class DmaInfo
  {
    public:
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

    void print()
    {
        printf("-- DMA Info ---\n");
        printf("  .pa = %#x \n", (uint32_t)pa);
        printf("  .pb = %#x \n", (uint32_t)pb);
        printf("  .pc = %#x \n", (uint32_t)pc);
        printf("  .size = %d \n", size);
        printf("  .cmd = %d \n", cmd);
        printf("  .status = %d \n\n", status);
    }
  };

/**
 * PuEngine3
 * This device just panics when accessed. It is supposed to warn
 * the user that the kernel they are running has unsupported
 * options (i.e. frame buffer)
 */
class PuEngine3 : public BasicPioDevice
{
  #define X86PIO_BASE_ADDR (0x8000000000000000)

  private:
    std::string _devname;

    PuCore3 * _pucore3;


  protected:
    // device register for PuEngine3: DMA command & status
    gem5::DmaInfo reg;

    uint8_t retData8;
    uint16_t retData16;
    uint32_t retData32;
    uint64_t retData64;

    uint8_t * pioMem; // PIO Register Memory Area

  public:
    PARAMS(PuEngine3);

    /**
     * Constructor for the PUDMA_PUENGINE3 Class.
     * @param p object parameters
     * @param a base address of the write
     */
    PuEngine3(const Params &p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const override;

  /////////////////// END /////////////////////////////////////


};

} // namespace gem5

#endif // __DEV_PUDMA_PUENGINE3_HH__
