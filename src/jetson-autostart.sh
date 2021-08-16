#!/bin/bash
nmap -sn 192.168.1.0/24

raspberrypi=$(arp -n | grep -w -i 'b8:27:eb:ab:aa:26' |  awk 'NR == 1' | awk '{print $1}') #arp -n | grep -w -i 'b8:27:eb:ab:aa:26' | awk '{print $1}')
fpga=$(arp -n | grep -w -i '00:00:F3:BE:EF:02' | awk '{print $1}')
projector=$(arp -n | grep -w -i 'cc:4b:73:b5:4b:da' | awk '{print $1}')

tmux list-sessions | awk 'BEGIN{FS=":"}{print $1}' | xargs -n 1 tmux kill-session -t
tmux new-session -s roboy -d \; \
  send-keys 'echo' C-m \; \
  split-window -h \; \
  split-window -h \; \
  split-window -v \; \
  split-window -v \; \
  split-window -v \; \
  select-pane -t 0 \; \
  split-window -v \; \
  split-window -v \; \
  send-keys -t 3 'animus' C-m\; \
  send-keys -t 4 'websocket --wait' C-m\; \
  send-keys -t 5 'face' C-m\; \
  send-keys -t 6 "ssh root@$fpga -t \"bash -lic 'plexus' \"" C-m\; \
  send-keys -t 0 'rosrun rosserial_python serial_node.py tcp' C-m\; \
  send-keys -t 1 'node-red' C-m \; \
  send-keys -t 2 /home/roboy/restart_face.sh C-m \;
  #send-keys -t 7 'wheel-commander' C-m\; \
  #select-pane -t 5\; \
  #split-window -v \; \
  #send-keys "ssh pi@$raspberrypi -t \"bash -lic 'leds' \"" C-m\;   

tmux attach
