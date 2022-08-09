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
    _devname(p.devicename),
    _pioSize(p.pio_size),
    _pioLatency(p.pio_latency),
    _pioAddr(p.pio_addr),
    _pucore3(p.pucore3)

{
    DPRINTF(PuEngine3, "Device PuEngine3 %s created\n", _devname);
}

Tick
PuEngine3::read(PacketPtr pkt)
{
// TODO : implement
//   _super::read(pkt);
    DPRINTF(PuEngine3, "PuEngine3: read(%#x) sz=%d requested\n",
                    pkt->getAddr(), pkt->getSize());
    panic("Device %s.read() not imlpmented\n", name());

    return _pioLatency;
}

Tick
PuEngine3::write(PacketPtr pkt)
{
// TODO : implement
//   _super::read(pkt);
    DPRINTF(PuEngine3, "PuEngine3: write(%#x) sz=%d requested\n",
                    pkt->getAddr(), pkt->getSize());

    panic("Device %s. write() not imlpmented\n", name());

    return _pioLatency;
}


AddrRangeList
PuEngine3::getAddrRanges() const
{
    AddrRangeList ranges;

    assert(_pioSize != 0);

    uint64_t start = X86PIO_BASE_ADDR + _pioAddr;
    ranges.push_back(RangeSize(start, _pioSize));

    DPRINTF(PuEngine3, "Device %s range registered:  %s\n",
            _devname, ranges.front().to_string());

    return ranges;
}

} // namespace gem5
