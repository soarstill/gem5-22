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

/** @file
 * This devices just panics when touched. For example if you have a
 * kernel that touches the frame buffer which isn't allowed.
 */

#ifndef __DEV_PUDMA_PUENGINE4_HH__
#define __DEV_PUDMA_PUENGINE4_HH__

// main headers
#include "dev/dma_device.hh"
#include "dev/io_device.hh"
#include "dev/pudma/pu_core4.hh"

// generated headers
#include "debug/PuEngine4.hh"
#include "params/PuEngine4.hh"

// gem5 headers
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{
  #define X86PIO_BASE_ADDR (0x8000000000000000)

  /**
   *  PuCmd : MUST BE the SAME structure with USER programs' PuCmd (x86 only)
   *  Information for an PU+DMA operation
   *  Play a role of the device register structure
   */
  struct PuCmd
  {
      uint32_t valid; // 0 : invalid, 1: valid
      uint32_t status;
      // status 0: idle, 1: ready, 2: start-moving cmdata,
      // 3: in-computing 4: moving to mem 5: DONE
      uint32_t cmd;
      // cmd = 0: just send info, 1: start compute,
      // 2: abort  3: report status
      uint32_t flags; // configuration flags
      uint32_t opcode; // 0: no op, 1: add 2: sub, 3: mul
      uint32_t addrA; // A buffer physical memory address
      uint32_t addrB; // B buffer physical memory address
      uint32_t addrC; // C buffer physical memory address
      uint32_t sizeA; // size of A
      uint32_t sizeB; // size of B
      uint32_t sizeC; // size of C
  };

/**
 * PuEngine4
 * This device just panics when accessed. It is supposed to warn
 * the user that the kernel they are running has unsupported
 * options (i.e. frame buffer)
 */
class PuEngine4 : public DmaDevice
{

  private:
    /** PuEngin's computing core */
    PuCore4 * m_pucore4;

    /** PuEnine4's Register Memory Area */
    uint8_t * m_regMemory;

  protected:
    /** User defined name (python) */
    std::string m_devicename;

     /** Address that the device listens to. */
    Addr m_pioAddr;

    /** Size that the device's address range. */
    Addr m_pioSize;

    /** Delay that the device experinces on an access. */
    Tick m_pioDelay;

    bool hookPuCmd();

  public:
    PARAMS(PuEngine4);

    /**
     * Constructor for the PUDMA_PUENGINE4 Class.
     * @param p object parameters
     * @param a base address of the write
     */
    PuEngine4(const Params &p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const override;

};

} // namespace gem5

#endif // __DEV_PUDMA_PUENGINE4_HH__
