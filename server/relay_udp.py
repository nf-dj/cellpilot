#!/usr/bin/env python
# Copyright (c) 2015, Netforce Co., Ltd.
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
