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

#include "dev/pudma/pu_engine.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PUDMA.hh"
#include "dev/pudma/pu_core.hh"

namespace gem5
{


PuEngine::PuEngine(const PuEngineParams &params) :
    SimObject(params),
    instPort(params.name + ".inst_port", this),
    dataPort(params.name + ".data_port", this),
    memPort(params.name + ".mem_side", this),
    blocked(false),
    pucore(params.pucore_object),
    event([this] {processEvent();}, name() + ".event"),
    latency(params.time_to_wait),
    timesLeft(params.number_of_fires)

{
    DPRINTF(PUDMA,"Created the PuEngine object\n");
    panic_if(!pucore,"PuEngine must have a non-null PuCore object");
}

Port &
PuEngine::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (SimpleMemobj.py)
    if (if_name == "mem_side") return memPort;
    else if (if_name == "inst_port") return instPort;
    else if (if_name == "data_port") return dataPort;
    else {
        return SimObject::getPort(if_name, idx);
    }
}
/**
 * Return the address ranges this memobj is responsible for. Just use the
 * same as the next upper level of the hierarchy.
 *
 * @return the address ranges this memobj is responsible for
 */
AddrRangeList
PuEngine::getAddrRanges() const
{
    return memPort.getAddrRanges();
}

void
PuEngine::sendRangeChange()
{
    instPort.sendRangeChange();
    dataPort.sendRangeChange();
}


void
PuEngine::startup()
{
    // Before simulation starts, we need to schedule the event
    schedule(event, latency);
}

void
PuEngine::processEvent()
{
    DPRINTF(PUDMA,"PuEngine: Processing the event! %d left\n",
                          timesLeft);
    if (--timesLeft <= 0) {
        "Done %d firing! \n", timesLeft);
        pucore->compute("SONG ");
    } else {
        schedule(event, curTick() + latency);
    }
}

bool PuEngine::handleRequest(PacketPtr pkt)
{
    // just forward to the Mem side with blocked flag set
    if (blocked) {
        return false;
    }
    if (pkt->req->isInstFetch()) {
        DPRINTF(PUDMA,"PuEngine:: Got ifetch request from addr %#x\n",
                pkt->getAddr());
    } else {
        DPRINTF(PUDMA,"PuEngine:: Got data request from addr %#x\n",
                pkt->getAddr());
    }

    blocked = true;
    memPort.sendReqPacket(pkt);

    return true;
}


bool PuEngine::handleResponse(PacketPtr pkt)
{
    assert(blocked);
    DPRINTF(PUDMA,"PuEngine:: Get response from addr %#x\n",
            pkt->getAddr());

    blocked = false;

    if (pkt->req->isInstFetch()) {
        instPort.sendRespPacket(pkt);
    } else {
        dataPort.sendRespPacket(pkt);
    }

    return true;
}

void PuEngine::handleFunctional(PacketPtr pkt)
{
    memPort.sendFunctional(pkt);
}

/////////////////////////////////////////////////////////////////////
//// PuEngine::CPUSidePort implementation ///////

/**
 * @brief get address range list of this device owned this port
 * @param void None
 *
 */
AddrRangeList
PuEngine::CPUSidePort::getAddrRanges() const
{
    DPRINTF(PUDMA,"Sending new ranges\n");

    return owner->getAddrRanges();
}


void
PuEngine::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    // just forward to the owner
    return owner->handleFunctional(pkt);
}


bool
PuEngine::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    // just forward to the owner
    if ( !owner->handleRequest(pkt)){
        needRetry = true;
        return false;
    } else {
        return true;
    }

}

void
PuEngine::CPUSidePort::recvRespRetry()
{
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendRespPacket(pkt);

}

/**
 *
 * @brief send a response packet to the cpu_side port
 * @param - packet to send
 *
 *
*/
void
PuEngine::CPUSidePort::sendRespPacket(PacketPtr pkt)
{
   // Note: This flow control is very simple since the memobj is blocking.
    panic_if(blockedPacket != nullptr,
        "Owner never try to respond if blocked");

    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

/**
 * Send a retry to the peer port only if it is needed. This is called
 * from the PuEngine whenever it is unblocked.
 */
void
PuEngine::CPUSidePort::trySendRetry()
{
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendRespPacket(pkt);
}

/////////////////////////////////////////////////////////////////////
//// PuEngine::MemSidePort implementation ///////
void
PuEngine::MemSidePort::recvRangeChange()
{
    // just forward the responsibility to the owner: notify the CPU sides
    owner->sendRangeChange();
}

bool
PuEngine::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
PuEngine::MemSidePort::recvReqRetry()
{
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendReqPacket(pkt);
}


/**
 * @brief send a request packet pkt to devices connected to the memside_port
 *
 * @param pkt
 */
void
PuEngine::MemSidePort::sendReqPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "never try to send if blocked");

    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}


} // namespace gem5
