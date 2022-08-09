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

#include "dev/io_device.hh"
#include "dev/isa_fake.hh"
#include "dev/pudma/pu_core3.hh"
#include "params/PuEngine3.hh"

namespace gem5
{

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
    Addr _pioAddr;
    Addr _pioSize;
    Tick _pioLatency;
    PuCore3 * _pucore3;
    AddrRangeList _ranges;

  public:
    using Params = PuEngine3Params;

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

};

} // namespace gem5

#endif // __DEV_PUDMA_PUENGINE3_HH__
