#!/usr/bin/env python
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

import socket
import time
import sys

IP=""
PORT=int(sys.argv[1])

addr1=None
addr2=None

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((IP,PORT))
sock.settimeout(10)

print("relaying...")
last_print=None
last_pkt1_t=None
last_pkt2_t=None
while True:
    try:
        data,addr=sock.recvfrom(2048)
        t=time.time()
        #print("[%s] %s %s"%(t,addr,data))
        if addr1 and (not last_pkt1_t or t-last_pkt1_t>10):
            print("[%s] reset addr1"%t)
            addr1=None
        if addr2 and (not last_pkt2_t or t-last_pkt2_t>10):
            print("[%s] reset addr2"%t)
            addr2=None
        if not addr1 and addr!=addr2:
            print("[%s] set addr1 %s:%s"%(t,addr[0],addr[1]))
            addr1=addr
        elif not addr2 and addr!=addr1:
            print("[%s] set addr2 %s:%s"%(t,addr[0],addr[1]))
            addr2=addr
        if addr==addr1:
            last_pkt1_t=t
            if addr2:
                #print("[%s] relay %s:%s->%s:%s"%(t,addr1[0],addr1[1],addr2[0],addr2[1]))
                sock.sendto(data,addr2)
        elif addr==addr2:
            last_pkt2_t=t
            if addr1:
                #print("[%s] relay %s:%s->%s:%s"%(t,addr2[0],addr2[1],addr1[0],addr1[1]))
                sock.sendto(data,addr1)
    except socket.timeout:
        continue
