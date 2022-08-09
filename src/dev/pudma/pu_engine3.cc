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
 * PuEngine3 implemenation
 */

#include "dev/pudma/pu_engine3.hh"

#include <string>

#include "base/trace.hh"
#include "debug/PuEngine2.hh"
#include "debug/PuEngine3.hh"
#include "params/PuEngine3.hh"
#include "sim/system.hh"

namespace gem5
{

PuEngine3::PuEngine3(const Params &p) :
    BasicPioDevice(p, p.pio_size),
    devname(p.devicename),
    pucore3(p.pucore3)
{
    DPRINTF(PuEngine3, "Device PuEngine3 %s created\n", devname);
    DPRINTF(PuEngine3, "Pio Addr=%#x, Size=%#x\n",
            p.pio_addr, p.pio_size);
}

Tick
PuEngine3::read(PacketPtr pkt)
{
//   _super::read(pkt);

    panic("Device %s.read() not imlpmented\n", devname);
}

Tick
PuEngine3::write(PacketPtr pkt)
{
//   _super::read(pkt);

    panic("Device %s. write() not imlpmented\n", devname);
}

#define PIO_BASE_ADDR 0x8000000000000000

AddrRangeList
PuEngine3::getAddrRanges() const
{
    assert(pioSize != 0);
    AddrRangeList ranges;
    DPRINTF(PuEngine3, "registering range: %#x(%#x)\n",
            pioAddr , pioSize);
    ranges.push_back(RangeSize(pioAddr + PIO_BASE_ADDR, pioSize));
//    ranges.push_back(AddrRange(PIO_BASE_ADDR + pioAddr,
//                               PIO_BASE_ADDR + pioAddr + pioSize - 1));

    DPRINTF(PuEngine3, "registered range(size):  %#x (%#x)\n",
            ranges.front().start(), ranges.front().size());
    DPRINTF(PuEngine3, "registered range:  %s\n",
            ranges.front().to_string());
    return ranges;
}
} // namespace gem5
