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

#ifndef __PUDMA_PUENGINE_HH__
#define __PUDMA_PUENGINE_HH__

#include <string>

#include "dev/pudma/pu_core.hh"
#include "dev/pudma/pu_engine.hh"
#include "mem/port.hh"
#include "params/PuEngine.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PuEngine : public SimObject
{
    /**
     * @brief Ports class definition for PuEngine py class
     * input: inst_port, data_port
     * output: mem_side
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        PuEngine *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string &name, PuEngine *owner) :
          ResponsePort(name,owner), needRetry(false), blockedPacket(nullptr)
        {}

        /**
         * @brief send a response packet to the cpu_side port
         * @param - packet to send
         */
        void sendRespPacket(PacketPtr pkt);

        /**
         * Send a retry to the peer port only if it is needed. This is called
         * from the PuEngine whenever it is unblocked.
         */
        void trySendRetry();

        /**
         * @brief get address range list of this device owned this port
         * @param void None
         */
        AddrRangeList getAddrRanges() const override;

      protected:
        /**
         * @brief we do not implement AtomicRequest packet
         *
         * @param pkt
         * @return Tick
         */
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic Unimplemented"); }

        /**
         * @brief handle receved functional request packet from input ports
         *
         * @param pkt received
         */
        void recvFunctional(PacketPtr pkt) override;

        /**
         * @brief handle recved timing request packet from input ports
         *
         * @param pkt
         */
        bool recvTimingReq(PacketPtr pkt) override;

        /**
         * @brief notified from input ports that they are read to
         *        accept resp packets
         *
         * @return true
         * @return false
         */
        void recvRespRetry() override;

    };

    class MemSidePort : public RequestPort
    {
      private:
        PuEngine *owner;

        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string &name, PuEngine *owner) :
          RequestPort(name, owner), blockedPacket(nullptr)
        {}

        /**
         * @brief send a request packet pkt to devices
         * connected to the memside_port
         *
         * @param pkt
         */
        void sendReqPacket(PacketPtr pkt);
      protected:

      /**
       *
       * @brief Receive a timing response from the response port
       *         bool recvTimingResp(PacketPtr pkt) override;
       * @param pkt
       * @return true if successful
       * @return false  if not successful (the peer should retry)
       */
        bool recvTimingResp(PacketPtr pkt) override;
        /**
         * @brief Notified by the peer that it is ready for
         *        the req packet
         *        Thus, this device can send the blocked request
         */
        void recvReqRetry() override;

        /**
         * @brief Notified that the peer changed its address range
         *        Usually send this notification to the CPU side device
         */
        void recvRangeChange() override;

    };

private:

    /// Instantiation of the CPU-side ports
    CPUSidePort instPort;
    CPUSidePort dataPort;

    /// Instantiation of the memory-side port
    MemSidePort memPort;

    /// True if this is currently blocked waiting for a response.
    bool blocked;

    // PuCore - actually compute
    PuCore * pucore;
    EventFunctionWrapper event;
    /// Latency between calling the event (in ticks)
    const Tick latency;
    /// Number of times left to fire the event before pucore
    int timesLeft;

  /**
   * @brief actually handle a request packet from the CPU side
   *
   * @param pkt - Request Packet from the CPU side
   * @return true - if successful
   * @return false - negative
   */
    bool handleRequest(PacketPtr pkt);

    /**
     * @brief actually handle the response from the memory side objects
     *
     * @param pkt responding packet
     * @return true - if successful
     * @return false - if fail
     */
    bool handleResponse(PacketPtr pkt);

    /**
     * Handle a packet functionally. Update the data on a write and get the
     * data on a read.
     *
     * @param packet to functionally handle
     */
    void handleFunctional(PacketPtr pkt);

    /**
     * Return the address ranges this memobj is responsible for. Just use the
     * same as the next upper level of the hierarchy.
     *
     * @return the address ranges this memobj is responsible for
     */
    AddrRangeList getAddrRanges() const;

    /**
     * Tell the CPU side to ask for our memory ranges.
     */
    void sendRangeChange();


  public:
    PuEngine(const PuEngineParams &p);

    /**
     * Part of a SimObject's initilaization. Startup is called after all
     * SimObjects have been constructed. It is called after the user calls
     * simulate() for the first time.
     */
    void startup();


    void processEvent();

    /**
     * @brief Get the Port object
     *
     * @param if_name Port name
     * @param idx   Index in case of VectorPort
     * @return Port&
     */
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

};

} // namespace gem5

#endif // __PUDMA_PUENGINE_HH__
