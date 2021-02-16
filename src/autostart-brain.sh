#!/bin/bash
nmap -sn 192.168.1.0/24
raspberrypi=$(arp -n | grep -w -i 'B8:27:EB:B0:E7:9C' | awk '{print $1}')
fpga=$(arp -n | grep -w -i '00:00:F3:BE:EF:01' | awk '{print $1}')
projector=$(arp -n | grep -w -i '00:0a:cd:32:ae:b7' | awk '{print $1}')

tmux list-sessions | awk 'BEGIN{FS=":"}{print $1}' | xargs -n 1 tmux kill-session -t
tmux new-session -s roboy -d \; \
  send-keys 'killall roscore; roscore&' C-m \; \
  split-window -h \; \
  split-window -v \; \
  split-window -v \; \
  split-window -v \; \
  select-pane -t 0 \; \
  split-window -v \; \
  send-keys -t 2 'node-red' C-m\; \
  send-keys -t 3 'websocket --wait' C-m\; \
  send-keys -t 4 'face' C-m\; \
  send-keys -t 5 "ssh root@$fpga -t \"bash -lic 'plexus' \"" C-m\; \
  send-keys -t 0 'brain-kindyn --wait' C-m\; \
  select-pane -t 2 \; \
  split-window -v \; \
  send-keys "ssh pi@$raspberrypi -t \"bash -lic  'leds' \"" C-m\; 
tmux attach
