#!/bin/sh
gpio -g mode 25 out
gpio -g write 25 0
sleep 1
gpio -g write 25 1
