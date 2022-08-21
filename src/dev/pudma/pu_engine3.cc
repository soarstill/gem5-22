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
//
#include <string>
//
#include "base/trace.hh"
#include "debug/PuEngine2.hh"
#include "debug/PuEngine3.hh"
#include "mem/packet.hh"
#include "params/PuEngine3.hh"
#include "sim/system.hh"

namespace gem5
{

PuEngine3::PuEngine3(const Params &p) :
    BasicPioDevice(p, p.pio_size)
{
    regMemory = new uint8_t[p.pio_size]();
    if (regMemory != nullptr) {
      memset(regMemory, 0, p.pio_size);
    } else {
      panic("Cannot allocate pio Memory of PuCore3");
    }

    retData8 = p.ret_data8;
    retData16 = p.ret_data16;
    retData32 = p.ret_data32;
    retData64 = p.ret_data64;

    DPRINTF(PuEngine3, "Device PuEngine3 %s created\n", _devname);
}


AddrRangeList
PuEngine3::getAddrRanges() const
{
    AddrRangeList ranges;

    assert(pioSize != 0);

    uint64_t start = X86PIO_BASE_ADDR + params().pio_addr;
    ranges.push_back(RangeSize(start, params().pio_addr));

    DPRINTF(PuEngine3, "Device %s range registered:  %s\n",
            name(), ranges.front().to_string());

    return ranges;
}

Tick
PuEngine3::read(PacketPtr pkt)
{
    pkt->makeAtomicResponse();

    int offset = pkt->getAddr() - this->getAddrRanges().front().start();
    int size = pkt->getSize();

    // uint32_t data = pkt->getLE<uint32_t>();
    DPRINTF(PuEngine3, "PuEngine3: PKT read op  pa=%#x size=%d, data = %#x\n",
           pkt->getAddr(), pkt->getSize(), pkt->getLE<uint32_t>());
   DPRINTF(PuEngine3, "PuEngine3: REQ read  pa = %#x size=%d\n",
           pkt->req->getPaddr(), pkt->req->getSize());

    if (params().warn_access != "") {
        warn("Device %s accessed by read to address %#x size=%d\n",
                name(), pkt->getAddr(), pkt->getSize());
    }

    if (params().ret_bad_addr) {
        DPRINTF(PuEngine3, "read to bad address pa=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

        // in case of out of range
        if (offset + size > params().pio_size) {
          size -= (offset + size - params().pio_size);
        }

        DPRINTF(PuEngine3, "PuEngine3: DONE read pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());

        std::memcpy(pkt->getPtr<uint8_t>(), &regMemory[offset], size);

        DPRINTF(PuEngine3, "PuEngine3: DONE read pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());
    }

    return pioDelay;
}

Tick
PuEngine3::write(PacketPtr pkt)
{
    int offset = pkt->getAddr() - this->getAddrRanges().front().start();
    int size = pkt->getSize();

    //uint32_t data = pkt->getLE<uint32_t>();
    DPRINTF(PuEngine3, "PuEngine3: PKT write op pa=%#x size=%d, data = %#x\n",
            pkt->getAddr(), pkt->getSize(), pkt->getLE<uint32_t>());
    DPRINTF(PuEngine3, "PuEngine3: REQ write   pa = %#x size=%d\n",
           pkt->req->getPaddr(), pkt->req->getSize());

    if (params().warn_access != "") {
        uint64_t data;
        data = pkt->getLE<uint8_t>();
        warn("Device %s accessed by write  %#x size=%d data=%#x\n",
                name(), pkt->getAddr(), pkt->getSize(), data);
    }

    if (params().ret_bad_addr) {
        DPRINTF(PuEngine3, "write to bad address pa=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else { // NORMAL

      assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

      if (offset + size > params().pio_size) { // OB -> reduce size
          size -= (offset + size - params().pio_size);
        }
        //pkt->setSize(size);

        assert(pkt->getSize() > 0 && pkt->getSize() <= params().pio_size);

        DPRINTF(PuEngine3, "write - pa=%#x size=%d \n", pkt->getAddr(),
                                                        pkt->getSize());

        std::memcpy(&regMemory[offset], pkt->getPtr<uint8_t>(), size);

        DPRINTF(PuEngine3, "PuEngine3: DONE write pa=%#x size=%d,data = %#x\n",
            pkt->getAddr(), size, pkt->getLE<uint32_t>());
    }

    return pioDelay;
}


/***  VERSION 2
Tick
PuEngine3::read(PacketPtr pkt)
{
    pkt->makeAtomicResponse();

    if (params().warn_access != "")
        warn("Device %s accessed by read to address %#x size=%d\n",
                name(), pkt->getAddr(), pkt->getSize());
    if (params().ret_bad_addr) {
        DPRINTF(PuEngine3, "read to bad address va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;
// assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
        DPRINTF(PuEngine3, "read  va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
             pkt->setLE(retData64);
             break;
          case sizeof(uint32_t):
             pkt->setLE(retData32);
             break;
          case sizeof(uint16_t):
             pkt->setLE(retData16);
             break;
          case sizeof(uint8_t):
             pkt->setLE(retData8);
             break;
          default:
 //            if (params().fake_mem)
                 std::memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
 //            else
 //   panic("invalid access size! Device being accessed by cache?\n");
        }
    }
    return pioDelay;
}

Tick
PuEngine3::write(PacketPtr pkt)
{
    if (params().warn_access != "") {
        uint64_t data;
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
            data = pkt->getLE<uint64_t>();
            break;
          case sizeof(uint32_t):
            data = pkt->getLE<uint32_t>();
            break;
          case sizeof(uint16_t):
            data = pkt->getLE<uint16_t>();
            break;
          case sizeof(uint8_t):
            data = pkt->getLE<uint8_t>();
            break;
          default:
            panic("invalid access size: %u\n", pkt->getSize());
        }
        warn("Device %s accessed by write  %#x size=%d data=%#x\n",
                name(), pkt->getAddr(), pkt->getSize(), data);
    }
    if (params().ret_bad_addr) {
        DPRINTF(PuEngine3, "write to bad address va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        DPRINTF(PuEngine3, "write - va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());

        if (params().update_data) {
            switch (pkt->getSize()) {
              case sizeof(uint64_t):
                retData64 = pkt->getLE<uint64_t>();
                break;
              case sizeof(uint32_t):
                retData32 = pkt->getLE<uint32_t>();
                break;
              case sizeof(uint16_t):
                retData16 = pkt->getLE<uint16_t>();
                break;
              case sizeof(uint8_t):
                retData8 = pkt->getLE<uint8_t>();
                break;
              default:
                panic("invalid access size!\n");
            }
        }
    }
    return pioDelay;
}
*/

/*** VERSION 1
Tick
PuEngine3::read(PacketPtr pkt)
{
// TODO : implement
//   _super::read(pkt);
    DPRINTF(PuEngine3, "PuEngine3: read(%#x) sz=%d requested\n",
                    pkt->getAddr(), pkt->getSize());
    //panic("Device %s.read() not imlpmented\n", name());

    pkt->makeAtomicResponse();

    //assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

    DPRINTF(PuEngine3, "read packet va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
    DPRINTF(PuEngine3, "CPU read request paddr=%#x\n",pkt->req->getPaddr());

    std::memcpy((void *)pkt->getPtr<uint8_t>(),
                (void *)&this->reg, pkt->getSize());

    DPRINTF(PuEngine3, "Done write : value =%#x \n",
                        *pkt->getPtr<uint8_t>());

    return _pioLatency;
}

Tick
PuEngine3::read(PacketPtr pkt)
{
// TODO : implement
//   _super::read(pkt);
    DPRINTF(PuEngine3, "PuEngine3: read(%#x) sz=%d requested\n",
                    pkt->getAddr(), pkt->getSize());
    //panic("Device %s.read() not imlpmented\n", name());

    pkt->makeAtomicResponse();

    //assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

    DPRINTF(PuEngine3, "read packet va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
    DPRINTF(PuEngine3, "CPU read request paddr=%#x\n",pkt->req->getPaddr());

    std::memcpy((void *)pkt->getPtr<uint8_t>(),
                (void *)&this->reg, pkt->getSize());

    DPRINTF(PuEngine3, "Done write : value =%#x \n",
                        *pkt->getPtr<uint8_t>());

    return pioDelay;
}

Tick
PuEngine3::write(PacketPtr pkt)
{
// TODO : implement
//   _super::read(pkt);
    DPRINTF(PuEngine3, "PuEngine3: write(%#x) sz=%d requested\n",
                    pkt->getAddr(), pkt->getSize());

    //panic("Device %s. write() not imlpmented\n", name());

    //pkt->makeAtomicResponse();

    //assert(this->getAddrRanges().front().contains(pkt->getAddr())) ;

    DPRINTF(PuEngine3, "write packet pa=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
    DPRINTF(PuEngine3, "CPU read request paddr=%#x\n",pkt->req->getPaddr());


    std::memcpy((void *)&this->reg,
        (void *)pkt->getPtr<uint8_t>(), pkt->getSize());

    DPRINTF(PuEngine3, "Done write : status value =%#x \n",
                        *pkt->getPtr<uint8_t>());

    return pioDelay;
}
*/

} // namespace gem5
