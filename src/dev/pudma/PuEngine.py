# -*- coding: utf-8 -*-
#  Copyright (c) 2022 H.S.Song
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.Device import BasicPioDevice, DmaDevice
from m5.params import *
from m5.SimObject import SimObject

class PuCore(SimObject):
    type = 'PuCore'
    cxx_header = "dev/pudma/pu_core.hh"
    cxx_class = 'gem5::PuCore'

    buffer_size = Param.MemorySize('100B',
                                   "Size of buffer to fill with goodbye")
    write_bandwidth = Param.MemoryBandwidth('100MB/s', "Bandwidth to fill "
                                            "the buffer")
class PuEngine(SimObject):
    type = 'PuEngine'
    cxx_header = "dev/pudma/pu_engine.hh"
    cxx_class = 'gem5::PuEngine'

    time_to_wait = Param.Latency("Time before firing the event")
    number_of_fires = Param.Int(1, "Number of times to fire the event before "
                                   "goodbye")
    pucore_object = Param.PuCore("A PuCore object")

    inst_port = ResponsePort("CPU side port, receve request from CPU side")
    data_port = ResponsePort("CPU side port, receve request from CPU side")
    mem_side = RequestPort("Memory side port, sends requests")

class PuEngine2(SimObject):
    type = 'PuEngine2'
    cxx_header = "dev/pudma/pu_engine2.hh"
    cxx_class = 'gem5::PuEngine2'

    inst_port = ResponsePort("CPU side port, receives requests")
    data_port = ResponsePort("CPU side port, receives requests")
    mem_side = RequestPort("Memory side port, sends requests")

    pucore = Param.PuCore("Actual compute core")

class PuCore3(SimObject):
    type = 'PuCore3'
    cxx_header = "dev/pudma/pu_core3.hh"
    cxx_class = 'gem5::PuCore3'

    buffer_size = Param.MemorySize('100B',
                                   "Size of buffer to fill with goodbye")
    write_bandwidth = Param.MemoryBandwidth('100MB/s', "Bandwidth to fill "
                                            "the buffer")

class PuEngine3(BasicPioDevice): # IsaFake instead of BasicPioDevice??
    type = 'PuEngine3'
    cxx_header = "dev/pudma/pu_engine3.hh"
    cxx_class = 'gem5::PuEngine3'

    pio_addr = Param.Addr(0x300, "Pio Port start Address")
    pio_latency = Param.Latency('100ns', "Programmed IO latency")

    pucore3 = Param.PuCore3("Actual compute core")
    devicename = Param.String('PuEngine3:v1',"User defined device name")

    pio_size = Param.Addr(0x800, "Size of address range")
    ret_data8 = Param.UInt8(0xFF, "Default data to return")
    ret_data16 = Param.UInt16(0xFFFF, "Default data to return")
    ret_data32 = Param.UInt32(0xFFFFFFFF, "Default data to return")
    ret_data64 = Param.UInt64(0xFFFFFFFFFFFFFFFF, "Default data to return")

    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")
    update_data = Param.Bool(False, "Update the data returned on writes")
    warn_access = Param.String("", "String to print when device is accessed")
    fake_mem = Param.Bool(False,
    "Is this device acting like a memory thus may get a cache line sized req")

class PuCore4(SimObject):
    type = 'PuCore4'
    cxx_header = "dev/pudma/pu_core4.hh"
    cxx_class = 'gem5::PuCore4'
    buffer_size_a = Param.MemorySize('1MiB',"Size of A buffer")
    buffer_size_b = Param.MemorySize('1MiB',"Size of B buffer")
    buffer_size_c = Param.MemorySize('1MiB',"Size of C buffer")
    opcode = Param.String("add","PU OP + matrix Addition")
    core_latency = Param.Latency('100ns', "PuCore initial latency")

    buffer_size = Param.MemorySize('100B',
                                   "Size of buffer to fill with message")
    write_bandwidth = Param.MemoryBandwidth('100MB/s', "Bandwidth to fill "
                                            "the buffer")

class PuEngine4(DmaDevice):
    type = 'PuEngine4'
    cxx_header = "dev/pudma/pu_engine4.hh"
    cxx_class = 'gem5::PuEngine4'

    pio_addr = Param.Addr('0x300', "Pio Port start Address")
    pio_size = Param.Addr('0x800', "Size of address range")
    pio_latency = Param.Latency('100ns', "Programmed IO latency")
    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")
    warn_access = Param.String("","String to print when device is accessed")

    x86_pio_base =  Param.Addr('0x8000000000000000',"X86 PIO base Address")
    pucore4 = Param.PuCore4("Actual compute core")
    devicename = Param.String('PuEngine4:v1',"User defined device name")
    range = Param.AddrRange("Pio Address AddrRange")

    rom1 = Param.SimpleMemory("Data A memory (RO)")
    rom1_base = Param.Addr("Data A Bmemory base address")
    rom1_size = Param.MemorySize("Data A memory size")
    rom1_file = Param.String("Data A binary file name")

    rom2 = Param.SimpleMemory("Data B memory (RO)")
    rom2_base = Param.Addr("Data B Bmemory base address")
    rom2_size = Param.MemorySize("Data B memory size")
    rom2_file = Param.String("Data B binary file name")

