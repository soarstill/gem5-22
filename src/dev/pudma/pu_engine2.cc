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

#include "dev/pudma/pu_engine2.hh"

#include "base/trace.hh"
#include "debug/PuEngine2.hh"
#include "dev/pudma/pu_core.hh"

namespace gem5
{

PuEngine2::PuEngine2(const PuEngine2Params &params) :
    SimObject(params),
    instPort(params.name + ".inst_port", this),
    dataPort(params.name + ".data_port", this),
    memPort(params.name + ".mem_side", this),
    blocked(false),
    pucore(params.pucore)
{
}

Port &
PuEngine2::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (PuEngine2.py)
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
PuEngine2::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
PuEngine2::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
PuEngine2::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        // Only send a retry if the port is now completely free
        needRetry = false;
        DPRINTF(PuEngine2, "Sending retry req for %d\n", id);
        sendRetryReq();
    }
}

void
PuEngine2::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleFunctional(pkt);
}

bool
PuEngine2::CPUSidePort::recvTimingReq(PacketPtr pkt)
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
PuEngine2::CPUSidePort::recvRespRetry()
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
PuEngine2::MemSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
PuEngine2::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt);
}

void
PuEngine2::MemSidePort::recvReqRetry()
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
PuEngine2::MemSidePort::recvRangeChange()
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
PuEngine2::handleRequest(PacketPtr pkt)
{
    if (blocked) {
        // There is currently an outstanding request. Stall.
        return false;
    }

    //panic_if(!(pkt->getAddrRange().isSubset(getAddrRanges().front()))),
    panic_if(!(getAddrRanges().front().contains(pkt->getAddr())),
            "Can't handle address range for packet %s\n", pkt->print());

       // Simply forward to the memory port
    if (pkt->req->isInstFetch()) {
        DPRINTF(PuEngine2,"Got inst request (%s) for v addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getVaddr(), pkt->req->getSize());
        DPRINTF(PuEngine2,"Got inst request(%s) for p addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getPaddr(), pkt->req->getSize());
        DPRINTF(PuEngine2,"Got inst Packet(%s) for addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());


    } else {
        DPRINTF(PuEngine2,"Got data request(%s) for v addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getVaddr(), pkt->req->getSize());
        DPRINTF(PuEngine2,"Got data request(%s) for p addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->req->getPaddr(), pkt->req->getSize());
        DPRINTF(PuEngine2,"Got data Packet(%s) for addr %#x (size=%d)\n", \
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    }



    if (pkt->req->getVaddr() == 0x80e42e0) {
        DPRINTF(PuEngine2,"TRAP get IO addr VA=%#x PA=%#x--Not Imple.\n", \
            pkt->req->getVaddr(),pkt->req->getPaddr());
        if (pkt->hasData()) {
            unsigned int *dp = pkt->getPtr<unsigned int>();
            unsigned int di = *dp;
            DPRINTF(PuEngine2, "DmaInfo.src = %#x\n", di);

           // if (di.cmd == 0x08) {
           //     pucore->compute("op: 0x08");
            //}
            di = 0x100;
            pkt->setData((uint8_t *)&di);
            DPRINTF(PuEngine2, "DmaInfo.src = %#x, corrupted by PuEngine\n", \
                 *dp);

//            pkt->writeData((uint8_t *)&di);
        }
    }

    // This memobj is now blocked waiting for the response to this packet.
    blocked = true;

    // Simply forward to the memory port
    memPort.sendPacket(pkt);

    return true;
}

bool
PuEngine2::handleResponse(PacketPtr pkt)
{
    assert(blocked);
    DPRINTF(PuEngine2, "Got response for addr %#x\n", pkt->getAddr());

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
PuEngine2::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    memPort.sendFunctional(pkt);
}

AddrRangeList
PuEngine2::getAddrRanges() const
{
    DPRINTF(PuEngine2, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    return memPort.getAddrRanges();
}

void
PuEngine2::sendRangeChange()
{
    instPort.sendRangeChange();
    dataPort.sendRangeChange();
}

} // namespace gem5
