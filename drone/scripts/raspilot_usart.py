# Copyright (c) 2012-2015, Netforce Co., Ltd.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import serial
import array

_sp = serial.Serial(
        port="/dev/ttyAMA0",
        baudrate=115200,
        bytesize=8,
        parity=serial.PARITY_NONE,
        stopbits=1,
        xonxoff=0,
        rtscts=0,
        timeout=5
    )

def send_bytes(buf):
    msg=array.array("B",buf).tostring()
    _sp.write(msg)

class Reader:
    def __init__(self,buf,pos=0):
        self.buf=buf
        self.pos=pos

    def read_uint8(self):
        v=self.buf[self.pos]
        self.pos+=1
        return v

    def read_uint16(self):
        v=(self.buf[self.pos]<<8)|self.buf[self.pos+1]
        self.pos+=2
        return v

    def read_int16(self):
        v=(self.buf[self.pos]<<8)|self.buf[self.pos+1]
        if v>=(1<<15):
            v-=(1<<16)
        self.pos+=2
        return v

    def read_uint32(self):
        v=(self.buf[self.pos]<<24)|(self.buf[self.pos+1]<<16)|(self.buf[self.pos+2]<<8)|self.buf[self.pos+3]
        self.pos+=4
        return v

    def read_int32(self):
        v=(self.buf[self.pos]<<24)|(self.buf[self.pos+1]<<16)|(self.buf[self.pos+2]<<8)|self.buf[self.pos+3]
        if v>=(1<<31):
            v-=(1<<32)
        self.pos+=4
        return v

class Writer:
    def __init__(self):
        self.buf=[]

    def write_uint8(self,v):
        self.buf.append(v&0xff)

    def write_uint16(self,v):
        self.buf.append((v>>8)&0xff)
        self.buf.append(v&0xff)

    def write_uint32(self,v):
        self.buf.append((v>>24)&0xff)
        self.buf.append((v>>16)&0xff)
        self.buf.append((v>>8)&0xff)
        self.buf.append(v&0xff)

def set_pwm(chan,dur):
    w=Writer()
    w.write_uint8(chan)
    w.write_uint16(dur)
    send_bytes([ord("C"),1+len(w.buf),ord("P")]+w.buf)

def set_led(led_no,state):
    w=Writer()
    w.write_uint8(led_no)
    w.write_uint8(state)
    send_bytes([ord("C"),1+len(w.buf),ord("L")]+w.buf)
