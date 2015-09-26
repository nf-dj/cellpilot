-----------------------------------------------------------------------------
-- Copyright (c) 2015, Netforce Co. Ltd.
-- All rights reserved.
-- 
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
-- 
-- 1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--     this list of conditions and the following disclaimer in the documentation
--    and/or other materials provided with the distribution.
--  
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
-- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------------

socket=require("socket")

NUM_PWM=8

LOCAL_PORT=5099

sock=socket.udp()
sock:setsockname("0.0.0.0",LOCAL_PORT)
sock:settimeout(0)

drone_ip=nil
drone_port=nil

function set_drone_address(host,port)
    drone_ip=socket.dns.toip(host)
    drone_port=port
end

function send_to_drone(msg)
    if drone_ip==nil or drone_port==nil then 
        print("Unkown drone address")
        return
    end
    print("send_to_drone "..drone_ip..":"..drone_port.." '"..msg.."'")
    sock:sendto(msg,drone_ip,drone_port)
end

time0=socket.gettime()

max_pkt_size=1200
last_pkt_no=-1
split_pkt_data=""

function parse_instruments(data)
    ins={}
    ins.bat_volt=data:byte(1)*256+data:byte(2)
    return ins
end

function process_video_data(data)
    ins=parse_instruments(data)
    video_data=data:sub(2,#data)
    update_instruments(ins.bat_volt)
    decode_video_packet(video_data)
    render_screen()
end

function receive_pkt()
    pkt,from_ip,from_port=sock:receivefrom()
    if pkt==nil then
        return nil
    end
    if from_ip=="127.0.0.1" then
        return nil
    end
    return pkt,from_ip,from_port
end

t=0

function on_timer()
    t=t+1
    while true do
        pkt,from_ip,from_port=receive_pkt()
        if pkt==nil then break end
        time=socket.gettime()
        cmd=pkt:sub(1,1)
        if cmd=="v" then
            pkt_no=pkt:byte(2)*256*256*256+pkt:byte(3)*256*256+pkt:byte(4)*256+pkt:byte(5)
            if pkt_no>last_pkt_no then
                print("received video packet "..pkt_no.." ("..#pkt.." bytes)")
                last_pkt_no=pkt_no
                --decode_video_packet(pkt:sub(26,#pkt))
                data=pkt:sub(6,#pkt)
                process_video_data(data)
            else
                print("WARNING: ignored out-of-order packet!")
            end
        elseif cmd=="V" then
            pkt_no=pkt:byte(2)*256*256*256+pkt:byte(3)*256*256+pkt:byte(4)*256+pkt:byte(5)
            if pkt_no>last_pkt_no then
                print("received jumbo video packet "..pkt_no.." ("..#pkt.." bytes)")
                last_pkt_no=pkt_no
                data=pkt:sub(6,#pkt)
                split_pkt_data=split_pkt_data..data
                if #pkt<max_pkt_size then
                    --print("##############################################################")
                    --print("got jumbo video data: "..#split_pkt_data.." bytes")
                    process_video_data(split_pkt_data)
                    split_pkt_data=""
                end
            else
                print("WARNING: ignored out-of-order packet!")
            end
        elseif cmd=="M" then
            msg=pkt:sub(3,#pkt)
            draw_text("Drone says: "..msg);
        end
    end
end

pwms={}
for chan=0,NUM_PWM-1 do
    pwms[chan]=0
end

function send_pwms()
    print("########################################")
    print("send_pwms")
    msg="P"
    for chan=0,NUM_PWM-1 do
        pwm=pwms[chan]
        msg=msg.." "..chan..","..pwm
    end
    print("msg "..msg)
    send_to_drone(msg)
end

function send_pwms_delay(actions)
    print("########################################")
    print("send_pwms_delay")
    msg="P"
    for _,action in ipairs(actions) do
        chan=action[1]
        pwm=action[2]
        delay=action[3]
        next_pwm=action[4]
        msg=msg.." "..chan..","..pwm..","..delay..","..next_pwm
    end
    print("msg "..msg)
    send_to_drone(msg)
end

function on_key(key,x,y)
    if key=="k" then
        send_pulse_forward()
    elseif key=="j" then
        send_pulse_backward()
    elseif key=="h" then
        send_pulse_left()
    elseif key=="l" then
        send_pulse_right()
    elseif key=="a" then
        send_pulse_up()
    elseif key=="z" then
        send_pulse_down()
    elseif key=="s" then
        camera_up()
    elseif key=="x" then
        camera_down()
    elseif key=="q" then -- XXX: change this key
        start_stop()
    elseif key=="1" then
        set_mode_manual()
    elseif key=="2" then
        set_mode_alt()
    elseif key=="3" then
        set_mode_gps()
    elseif key=="m" then
        switch_hat_mode()
    end
end

function on_joy_axis(x1,y1,x2,y2)
    print("on_joy_axis "..x1.." "..y1.." "..x2.." "..y2)
    pwms[0]=math.floor(1000+(x2+32768)*1000/65536)+trim_vals[2] -- roll (A)
    pwms[1]=math.floor(1000+(y2+32768)*1000/65536)+trim_vals[3] -- pitch (E)
    pwms[2]=math.floor(1000+(y1+32768)*1000/65536)+trim_vals[1] -- throttle (T)
    pwms[3]=math.floor(1000+(x1+32768)*1000/65536)+trim_vals[0] -- yaw (R)
    for i=0,3 do
        pwms[i]=math.max(1000,math.min(2000,pwms[i]))
    end
    send_pwms()
end

function start_stop()
    print("SEND "..msg)
    actions={{0,1000,500,1500},{1,1000,500,1500},{2,1000,500,1500},{3,1000,500,1500}}
    send_pwms_delay(actions)
    draw_text("start/stop")
end

function set_mode_manual()
    pwms[4]=1180
    send_pwms()
    draw_text("mode manual")
end

function set_mode_alt()
    pwms[4]=1520
    send_pwms()
    draw_text("mode alt")
end

function set_mode_gps()
    pwms[4]=1860
    send_pwms()
    draw_text("mode gps")
end

function send_pulse_up()
    chan=2
    pwm=1500+trim_vals[1]+pulse_vals[0]
    next_pwm=1500+trim_vals[1]
    actions={{chan,pwm,pulse_durs[0],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse up")
end

function send_pulse_down()
    chan=2
    pwm=1500+trim_vals[1]-pulse_vals[0]
    next_pwm=1500+trim_vals[1]
    actions={{chan,pwm,pulse_durs[0],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse down")
end

function send_pulse_forward()
    chan=1
    pwm=1500+trim_vals[3]+pulse_vals[2]
    next_pwm=1500+trim_vals[3]
    actions={{chan,pwm,pulse_durs[2],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse forward")
end

function send_pulse_backward()
    chan=1
    pwm=1500+trim_vals[3]-pulse_vals[2]
    next_pwm=1500+trim_vals[3]
    actions={{chan,pwm,pulse_durs[2],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse backward")
end

function send_pulse_right()
    chan=0
    pwm=1500+trim_vals[2]+pulse_vals[1]
    next_pwm=1500+trim_vals[2]
    actions={{chan,pwm,pulse_durs[1],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse right")
end

function send_pulse_left()
    chan=0
    pwm=1500+trim_vals[2]-pulse_vals[1]
    next_pwm=1500+trim_vals[2]
    actions={{chan,pwm,pulse_durs[1],next_pwm}}
    send_pwms_delay(actions)
    draw_text("pulse left")
end

function switch_hat_mode()
    if hat_mode=="settings" then
        hat_mode="pulse"
    elseif hat_mode=="pulse" then
        hat_mode="settings"
    end
    draw_text("hat_mode "..hat_mode)
end

function on_joy_button(button)
    print("on_joy_button "..button)
    if button==0 then -- A
        camera_down()
    elseif button==1 then -- B
        camera_up()
    elseif button==2 then -- X
        set_mode_alt()
    elseif button==3 then -- Y
        set_mode_gps()
    elseif button==4 then -- LB
        send_pulse_down()
    elseif button==5 then -- RB
        send_pulse_up()
    elseif button==6 then -- back
    elseif button==7 then -- start
        start_stop()
    elseif button==999 then
        switch_hat_mode()
    end
end

cur_setting=0
num_settings=10

trim_vals={}
trim_vals[0]=0
trim_vals[1]=0
trim_vals[2]=0
trim_vals[3]=0

pulse_vals={}
pulse_vals[0]=200
pulse_vals[1]=200
pulse_vals[2]=200

pulse_durs={}
pulse_durs[0]=500
pulse_durs[1]=500
pulse_durs[2]=500

hat_mode="pulse"

function camera_up()
    pwms[5]=pwms[5]+250
    pwms[5]=math.max(1000,math.min(2000,pwms[5]))
    send_pwms()
    draw_text("camera "..pwms[5])
end

function camera_down()
    pwms[5]=pwms[5]-250
    pwms[5]=math.max(1000,math.min(2000,pwms[5]))
    send_pwms()
    draw_text("camera "..pwms[5])
end

function hat_up()
    print("hat_up")
    if hat_mode=="settings" then
        cur_setting=(cur_setting+1)%num_settings
        draw_cur_setting()
    elseif hat_mode=="pulse" then
        send_pulse_forward()
    end
end

function hat_down()
    print("hat_down")
    if hat_mode=="settings" then
        cur_setting=(cur_setting-1)%num_settings
        draw_cur_setting()
    elseif hat_mode=="pulse" then
        send_pulse_backward()
    end
end

function hat_left()
    print("hat_left")
    if hat_mode=="settings" then
        if cur_setting<4 then
            trim_vals[cur_setting]=trim_vals[cur_setting]-10
        elseif cur_setting<7 then
            i=cur_setting-4
            pulse_vals[i]=pulse_vals[i]-10
            if pulse_vals[i]<0 then
                pulse_vals[i]=0
            end
        elseif cur_setting<10 then
            i=cur_setting-7
            pulse_durs[i]=pulse_durs[i]-10
            if pulse_durs[i]<0 then
                pulse_durs[i]=0
            end
        end
        draw_cur_setting()
    elseif hat_mode=="pulse" then
        send_pulse_left()
    end
end

function hat_right()
    print("hat_right")
    if hat_mode=="settings" then
        if cur_setting<4 then
            trim_vals[cur_setting]=trim_vals[cur_setting]+10
        elseif cur_setting<7 then
            i=cur_setting-4
            pulse_vals[i]=pulse_vals[i]+10
            if pulse_vals[i]>1000 then
                pulse_vals[i]=1000
            end
        elseif cur_setting<10 then
            i=cur_setting-7
            pulse_durs[i]=pulse_durs[i]+10
            if pulse_durs[i]>1000 then
                pulse_durs[i]=1000
            end
        end
        draw_cur_setting()
    elseif hat_mode=="pulse" then
        send_pulse_right()
    end
end

function draw_cur_setting()
    if cur_setting<4 then
        draw_text("trim "..cur_setting.." "..trim_vals[cur_setting])
    elseif cur_setting<7 then
        i=cur_setting-4
        draw_text("pulse_val "..i.." "..pulse_vals[i])
    elseif cur_setting<10 then
        i=cur_setting-7
        draw_text("pulse_dur "..i.." "..pulse_durs[i])
    end
end

function on_joy_hat(button)
    print("on_joy_hat "..button)
    if button==1 then
        hat_up()
    elseif button==4 then
        hat_down()
    elseif button==8 then
        hat_left()
    elseif button==2 then
        hat_right()
    end
end
