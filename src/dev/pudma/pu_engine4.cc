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
 * PuEngine4 implemenation
 */

#include "dev/pudma/pu_engine4.hh"

#include <string>

#include "base/trace.hh"
#include "debug/PuEngine4.hh"
#include "dev/dma_device.hh"
#include "dev/pudma/pu_core4.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/PuEngine4.hh"
#include "sim/system.hh"

namespace gem5
{

PuEngine4::PuEngine4(const Params &p) :
    DmaDevice(p),
    m_pucore4(p.pucore4),
    m_pioSize(p.pio_size),
    m_pioAddr(p.pio_addr),
    m_pioDelay(p.pio_latency),
    m_devicename(p.devicename)
{
    m_regMemory = new uint8_t[p.pio_size]();
    if (m_regMemory == nullptr) {
      panic("Cannot allocate pio register memory of PuEngine4");
    }
    memset(m_regMemory, 0, p.pio_size);

    if (m_pucore4) m_pucore4->sayHello(m_devicename);

    DPRINTF(PuEngine4, "Device PuEngine4 %s created\n", m_devicename);
}


AddrRangeList
PuEngine4::getAddrRanges() const
{
    AddrRangeList ranges;

    assert(m_pioSize != 0);

    uint64_t start = X86PIO_BASE_ADDR + params().pio_addr;
    ranges.push_back(RangeSize(start, params().pio_addr));

    DPRINTF(PuEngine4, "Device %s range registered:  %s\n",
            name(), ranges.front().to_string());

    return ranges;
}

Tick
PuEngine4::read(PacketPtr pkt)
{
    pkt->makeAtomicResponse();

    int offset = pkt->getAddr() - this->getAddrRanges().front().start();
    int size = pkt->getSize();

    // uint32_t data = pkt->getLE<uint32_t>();
    DPRINTF(PuEngine4, "PuEngine4: PKT read op  pa=%#x size=%d, data = %#x\n",
           pkt->getAddr(), pkt->getSize(), pkt->getLE<uint32_t>());
   DPRINTF(PuEngine4, "PuEngine4: REQ read  pa = %#x size=%d\n",
           pkt->req->getPaddr(), pkt->req->getSize());

    if (params().warn_access != "") {
        warn("Device %s accessed by read PKT to address %#x size=%d\n",
                name(), pkt->getAddr(), pkt->getSize());
        warn("Device %s accessed by read REQ to address %#x size=%d\n",
                name(), pkt->req->getPaddr(), pkt->req->getSize());
    }

    if (params().ret_bad_addr) {
        DPRINTF(PuEngine4, "read to bad address pa=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

        // in case of out of range
        if (offset + size > params().pio_size) {
          size -= (offset + size - params().pio_size);
        }

        DPRINTF(PuEngine4, "PuEngine4: DONE read pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());

        std::memcpy(pkt->getPtr<uint8_t>(), &m_regMemory[offset], size);

        DPRINTF(PuEngine4, "PuEngine4: DONE read pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());

    }

    return m_pioDelay;
}

Tick
PuEngine4::write(PacketPtr pkt)
{
    int offset = pkt->getAddr() - this->getAddrRanges().front().start();
    int size = pkt->getSize();

    //uint32_t data = pkt->getLE<uint32_t>();
    DPRINTF(PuEngine4, "PuEngine4: PKT write op pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), pkt->getSize(), pkt->getLE<uint32_t>());
    DPRINTF(PuEngine4, "PuEngine4: REQ write   pa = %#x size=%d\n",
           pkt->req->getPaddr(), pkt->req->getSize());

    if (params().warn_access != "") {
        warn("Device %s accessed by write  %#x size=%d data=%#x\n",
                name(), pkt->getAddr(), pkt->getSize(),
                pkt->getLE<uint32_t>());
    }

    if (params().ret_bad_addr) {
        DPRINTF(PuEngine4, "write to bad address pa=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else { // NORMAL WRITE

      assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

      if (offset + size > params().pio_size) { // OB -> reduce size
          size -= (offset + size - params().pio_size);
        }

        assert(pkt->getSize() > 0 && pkt->getSize() <= params().pio_size);

        std::memcpy(&m_regMemory[offset], pkt->getPtr<uint8_t>(), size);

        DPRINTF(PuEngine4, "PuEngine4: DONE write pa=%#x size=%d,data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());

        // Hook for User's DMA+PU command
        hookPuCmd();

    }

    //pkt->makeAtomicResponse();

    return m_pioDelay;
}

bool PuEngine4::hookPuCmd()
{
    PuCmd puCmdClone = *(PuCmd *)&m_regMemory[0];

    if (!puCmdClone.valid ) return false;

    DPRINTF(PuEngine4,"Hello PuCore4, from PuEngine4. PuCmd.status=%#8x\n",
                              puCmdClone.status);

    if (puCmdClone.cmd != 0) { // Hook!! TODO: call with the puClone
      m_pucore4->sayHello("Hello PuCore4, from gem5 PuEngine4 !!!");
      //m_pucore4->compute(puCmdClone, "PuEngine4");

      return true;
    }

    return false;
}

} // namespace gem5
