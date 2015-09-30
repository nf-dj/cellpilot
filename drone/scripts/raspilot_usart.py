# Copyright (c) 2015 Netforce Co. Ltd.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
# OR OTHER DEALINGS IN THE SOFTWARE.

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
