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

from m5.objects.Device import BasicPioDevice
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
    pio_size = Param.Addr(0x800, "Size of address range")
    pio_latency = Param.Latency('100ns', "Programmed IO latency")

    pucore3 = Param.PuCore3("Actual compute core")
    devicename = Param.String('PuEngine3:v1',"User defined device name")