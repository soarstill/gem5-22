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

#include "dev/pudma/pu_core4.hh"

#include "base/trace.hh"
#include "debug/PuEngine4.hh"
#include "params/PuCore4.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

PuCore4::PuCore4(const PuCore4Params &p) :
    SimObject(p), event([this]{ processEvent(); }, name()+".event"),
    bandwidth(p.write_bandwidth), bufferSize(p.buffer_size),
    buffer(nullptr), bufferUsed(0),
    bufferSizeA(p.buffer_size_a),
    bufferSizeB(p.buffer_size_b),
    bufferSizeC(p.buffer_size_c)
{
    buffer = new char[bufferSize]();
    bufferA = new uint8_t[bufferSizeA]();
    bufferB = new uint8_t[bufferSizeB]();
    bufferC = new uint8_t[bufferSizeC]();

    panic_if((!buffer || !bufferA || !bufferB || !bufferC),
        "Could not allocate one of the buffers");

    coreLatency = p.core_latency;
    opcode = p.opcode;

    m_dram2_base = p.dram2_base;
    // DRAM 2's start address (2G+256MB)
    m_rom1_base = p.rom1_base;
    m_rom2_base = p.rom2_base;

    DPRINTF(PuEngine4, "Created a PuCore4 object with buffers of %d, %d, %d\n",
                        bufferSizeA, bufferSizeB, bufferSizeC);
}

PuCore4::~PuCore4()
{
    delete[] buffer;
    delete[] bufferA;
    delete[] bufferB;
    delete[] bufferC;
}

void
PuCore4::startup()
{
    // schedule(event, p.core_latency);
    DPRINTF(PuEngine4, "PuCore4: startup() called!\n");
}


void PuCore4::sayHello(std::string mesg, PuEngine4 *pue4)
{
    std::string message = "Hello " + mesg + "from" + pue4->name() + "!! ";
    DPRINTF(PuEngine4, "PuCore4: %s\n", message);

    // Prepare DMA dummy request test
    // X Addr addr = Addr('2GiB'); // DRAM 2's start address (2G+256MB)
    Addr addr = 0x800000000; // DRAM 2's start address (2G+256MB)
    int size = 1024;
    bufferA[0] = 0x12;
    bufferB[0] = 0x34;

    auto event =  new EventFunctionWrapper(
            [this, pue4]{ OnComplete(pue4); }, "DMA Complete event");

    DPRINTF(PuEngine4, "PuCore4::Start DMA... at %d \n", curTick());

    pue4->dmaWrite(addr, size, event, (uint8_t *)&bufferA[0]);
    pue4->dmaRead(addr, size, event, (uint8_t *)&bufferB[0]);

}

void
PuCore4::OnComplete(PuEngine4 * pue4)
{

    DPRINTF(PuEngine4,
    "PuCore4::Complete Callback DMA  %s for %s A(%d) B(%d) !! at  %d\n",
            bufferA[0]==bufferB[0] ? "SUCCESS" : "FAIL",pue4->name(),
            bufferA[0], bufferB[0], curTick());
}

void
PuCore4::compute(PuCmd * puCmd, PuEngine4 * pue4)
{
    //puengine4 = pue4;

    //clean();
    puCmd->print();

    pue4->setStatus(STS_READY), puCmd->setStatus(STS_READY);

    enum PU_CMD_TYPE cmd = puCmd->getCommand();
    DPRINTF(PuEngine4, "PuCore4: Start compute : cmd = %#x\n", cmd);

    if (cmd == CMD_START) {
        switch (puCmd->getOpcode()) {
            case OP_ADD:
                startAdd(puCmd, pue4);
                break;
            default:
                panic("PuCore4: Unknown opcode! op = %d\n",
                    puCmd->getOpcode());
        };

    } else {
            DPRINTF(PuEngine4, "PuCore4: Unimplemented cmd = %#x\n", cmd);
    }
}

void
PuCore4::startAdd(PuCmd *cmd, PuEngine4 *pue4)
{
    DPRINTF(PuEngine4, "PuCore4::start Add!\n");

    if (cmd->get(REG_AROWS) != cmd->get(REG_BROWS) ||
                cmd->get(REG_ACOLS) != cmd->get(REG_BCOLS))
    {
        panic("puCore4: startAdd() - invalid size (R,C should be equal");
    }

    // TODO: BUSY set
    cmd->setStatus(STS_READY), pue4->setStatus(STS_READY);

    processAdd(cmd,pue4);
}

void
PuCore4::processAdd(PuCmd *cmd, PuEngine4 *pue4)
{
    DPRINTF(PuEngine4, "PuCore4::process Add!\n");

    Addr memA = m_rom1_base + cmd->get(REG_AADDR);
    Addr memB = m_rom2_base + cmd->get(REG_BADDR);
    Addr memC = m_dram2_base + cmd->get(REG_CADDR);

    int sizeA = cmd->get(REG_AROWS) * cmd->get(REG_ACOLS) * sizeof(double);
    int sizeB = cmd->get(REG_BROWS) * cmd->get(REG_BCOLS) * sizeof(double);
    int sizeC = cmd->get(REG_CROWS) * cmd->get(REG_CCOLS) * sizeof(double);


    auto event =  new EventFunctionWrapper(
            [this, pue4, cmd]{ processAdd(cmd, pue4); }, name()+".add.event");

    if (cmd->getStatus() == STS_READY) {
        cmd->setStatus(STS_DMARD_A), pue4->setStatus(STS_DMARD_A);

        DPRINTF(PuEngine4, "PuCore4::Start DMA A... at %d \n", curTick());
        pue4->dmaRead(memA, sizeA, event, (uint8_t *)&bufferB[0]);
        schedule (event, curTick());

    } else if (cmd->getStatus() == STS_DMARD_A) {
        cmd->setStatus(STS_DMARD_B), pue4->setStatus(STS_DMARD_B);

        DPRINTF(PuEngine4, "PuCore4::Start DMA B... at %d \n", curTick());
        pue4->dmaRead(memB, sizeB, event, (uint8_t *)&bufferB[0]);
        schedule (event, curTick());

    } else if (cmd->getStatus() == STS_DMARD_B) {
        cmd->setStatus(STS_COMPUTING), pue4->setStatus(STS_COMPUTING);
        DPRINTF(PuEngine4, "PuCore4::Start Computing at %d \n", curTick());
        Tick latency =  doAdd(cmd); // actually computing, return latency
        schedule(event, curTick() + latency);

    } else if (cmd->getStatus() == STS_COMPUTING) {
        cmd->setStatus(STS_DMAWR_C), pue4->setStatus(STS_DMAWR_C);
        DPRINTF(PuEngine4, "PuCore4::Start DMA WR to C at %d \n", curTick());
        pue4->dmaWrite(memC, sizeC, event, (uint8_t *)&bufferC[0]);
        schedule (event, curTick());

    } else if (cmd->getStatus() == STS_DMAWR_C) {
        cmd->setStatus(STS_COMPLETED), pue4->setStatus(STS_COMPLETED);
        DPRINTF(PuEngine4, "PuCore4::Complete Add  at %d \n", curTick());
        // No more schedule

    } else {
        cmd->setStatus(STS_ERROR), pue4->setStatus(STS_ERROR);
        DPRINTF(PuEngine4, "PuCore4: Unimplemented status = %#x\n",
                    cmd->getStatus());
    }
}

Tick
PuCore4::doAdd(PuCmd *cmd) // C = A + B
{
    Tick latency = 0;

    DPRINTF(PuEngine4, "PuCore4: doAdd()\n");
    cmd->print();

    int count = 0;
    int COLS = cmd->get(REG_AROWS);
    int ROWS = cmd->get(REG_ACOLS);

    double **dbA = (double **)&bufferA[0];
    double **dbB = (double **)&bufferB[0];
    double **dbC = (double **)&bufferC[0];

    for (int i = 0; i < ROWS; i++) {
        for (int j =0 ; j < COLS; j++) {
            dbC[i][j] = dbA[i][j] + dbB[i][j];
            count++;

            assert(dbC[i][j] == i+j); //just for debugging
        }
    }

    latency = count * params().fu_add_latency;
    return latency;
}

} // namespace gem5
