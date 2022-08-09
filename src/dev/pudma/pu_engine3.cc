/*
 * Copyright (c) 2017 Jason Lowe-Power
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

//// My headers
#include "dev/pudma/pu_engine3.hh"

#include "dev/pudma/pu_core3.hh"

//// Gem5 headers
#include "base/trace.hh"
#include "dev/io_device.hh"

//// Automatic headers
#include "debug/PuEngine3.hh"

namespace gem5
{

PuEngine3::PuEngine3(const PuEngine3Params &p) :
    BasicPioDevice(p, p.pio_size),
    instPort(p.name + ".inst_port", this),
    dataPort(p.name + ".data_port", this),
    memPort(p.name + ".mem_side", this),
    blocked(false),
    pucore3(p.pucore3)
{
    DPRINTF(PuEngine3,"Created a PuEngine3!! (addr=%#x, size=%#x)\n",
        pioAddr, pioSize);
}

Port &
PuEngine3::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (PuEngine3.py)
    if (if_name == "mem_side") {
        return memPort;
    } else if (if_name == "inst_port") {
        return instPort;
    } else if (if_name == "data_port") {
        return dataPort;
    } else {
        // pass it along to our super class
        return SimObject::getPort(if_name, idx);
    }
}

void
PuEngine3::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
PuEngine3::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
PuEngine3::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        // Only send a retry if the port is now completely free
        needRetry = false;
        DPRINTF(PuEngine3, "Sending retry req for %d\n", id);
        sendRetryReq();
    }
}

void
PuEngine3::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleFunctional(pkt);
}

bool
PuEngine3::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    // Just forward to the memobj.
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    } else {
        return true;
    }
}

void
PuEngine3::CPUSidePort::recvRespRetry()
{
    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);
}

void
PuEngine3::MemSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
PuEngine3::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt);
}

void
PuEngine3::MemSidePort::recvReqRetry()
{
    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);
}

void
PuEngine3::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}
typedef struct _DmaInfo
{
    unsigned int src;
    unsigned int dst;
    unsigned int size;
    unsigned int cmd;
    unsigned int status;
} DmaInfo;

//static DmaInfo dmaInfo;

bool
PuEngine3::handleRequest(PacketPtr pkt)
{
    if (blocked) {
        // There is currently an outstanding request. Stall.
        return false;
    }

//panic_if(!(pkt->getAddrRange().isSubset(getAddrRanges().front()))),
    if (!(getAddrRanges().front().contains(pkt->getAddr()))) {
        DPRINTF(PuEngine3,"Can't handle address range for packet %s\n", \
                pkt->print());
    }

       // Simply forward to the memory port
    if (pkt->req->isInstFetch()) {
        DPRINTF(PuEngine3,"Got inst request (%s) for v addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getVaddr(), pkt->req->getSize());
        DPRINTF(PuEngine3,"Got inst request(%s) for p addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getPaddr(), pkt->req->getSize());
        DPRINTF(PuEngine3,"Got inst Packet(%s) for addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());


    } else {
        DPRINTF(PuEngine3,"Got data request(%s) for v addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getVaddr(), pkt->req->getSize());
        DPRINTF(PuEngine3,"Got data request(%s) for p addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getPaddr(), pkt->req->getSize());
        DPRINTF(PuEngine3,"Got data Packet(%s) for addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    }

    if (pkt->req->getVaddr() >= 0x8000000000000000 ) { // user PIO,inl()
        DPRINTF(PuEngine3,"PIO addr VA=%#x PA=%#x SZ=%d\n", \
        pkt->req->getVaddr(),pkt->req->getPaddr(), pkt->req->getSize());
    }

    if (pkt->req->getVaddr() == 0x80e3068) { // Tric - Hook
        DPRINTF(PuEngine3,"TRAP get IO addr VA=%#x PA=%#x--Not Imple.\n", \
            pkt->req->getVaddr(),pkt->req->getPaddr());
        if (pkt->hasData()) {
            unsigned int *dp = pkt->getPtr<unsigned int>();
            unsigned int di = *dp;
            DPRINTF(PuEngine3, "DmaInfo.src = %#x\n", di);

           // if (di.cmd == 0x08) {
           //     pucore->compute("op: 0x08");
            //}
            di = 0x100;
            pkt->setData((uint8_t *)&di);
            DPRINTF(PuEngine3, "DmaInfo.src = %#x, corrupted by PuEngine\n", \
                 *dp);
        }
    }

    // This memobj is now blocked waiting for the response to this packet.
    blocked = true;

    // Simply forward to the memory port
    memPort.sendPacket(pkt);

    return true;
}

bool
PuEngine3::handleResponse(PacketPtr pkt)
{
    assert(blocked);
    DPRINTF(PuEngine3, "Got response for addr %#x\n", pkt->getAddr());

    // The packet is now done. We're about to put it in the port, no need for
    // this object to continue to stall.
    // We need to free the resource before sending the packet in case the CPU
    // tries to send another request immediately (e.g., in the same callchain).
    blocked = false;

    // Simply forward to the memory port
    if (pkt->req->isInstFetch()) {
        instPort.sendPacket(pkt);
    } else {
        dataPort.sendPacket(pkt);
    }

    // For each of the cpu ports, if it needs to send a retry, it should do it
    // now since this memory object may be unblocked now.
    instPort.trySendRetry();
    dataPort.trySendRetry();

    return true;
}

void
PuEngine3::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    memPort.sendFunctional(pkt);
}

AddrRangeList
PuEngine3::getAddrRanges() const
{
    DPRINTF(PuEngine3, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    return memPort.getAddrRanges();
}

void
PuEngine3::sendRangeChange()
{
    instPort.sendRangeChange();
    dataPort.sendRangeChange();
}

} // namespace gem5
