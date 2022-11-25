#!/bin/bash
nmap -sn 192.168.1.0/24

#nmap -sn 10.180.1.0/24

raspberrypi=$(arp -n | grep -w -i 'b8:27:eb:ab:aa:26' |  awk 'NR == 1' | awk '{print $1}') #arp -n | grep -w -i 'b8:27:eb:ab:aa:26' | awk '{print $1}')
fpga=$(arp -n | grep -w -i '00:00:F3:BE:EF:02' | awk '{print $1}')
projector=$(arp -n | grep -w -i '00:0e:c6:ac:ca:2b' | awk '{print $1}')

echo $fpga
echo $projector

/home/roboy/kill_nodered.sh

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
  send-keys -t 0 'rosrun rosserial_python serial_node.py tcp' C-m\; \
  send-keys -t 1 '/home/roboy/run_nodered.sh' C-m \; \
  send-keys -t 2 /home/roboy/restart_face.sh C-m \;\
  send-keys -t 6 'ik' C-m\; 
  #send-keys -t 7 'wheel-commander' C-m\; \
  #select-pane -t 5\; \
  #split-window -v \; \
  #send-keys "ssh pi@$raspberrypi -t \"bash -lic 'leds' \"" C-m\;   
  #"ssh root@$fpga -t \"bash -lic 'plexus' \"" C-m\; \
#tmux attach
