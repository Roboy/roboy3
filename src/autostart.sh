#!/bin/bash
tmux list-sessions | awk 'BEGIN{FS=":"}{print $1}' | xargs -n 1 tmux kill-session -t

tmux new-session -s roboy -d \; \
  send-keys 'killall roscore; roscore&' C-m \; \
  send-keys 'sleep 2; pinky-kindyn --wait' C-m\;\
  split-window -h \; \
  send-keys 'sleep 5; ik --wait' C-m\;\
  split-window -v \; \
  send-keys 'sleep 2; roscd ball_in_socket_estimator/python && 
  ~/anaconda3/envs/roboy/bin/python3 predict.py --wait' C-m\;\
  # split-window -v \; \
  # send-keys 'sleep 45; /home/roboy/auto_initializing.sh' C-m\;\

tmux attach

# tmux new-session -s roboy -d \; \
#   send-keys 'killall roscore; roscore&' C-m \; \
#   split-window -h \; \
#   split-window -v \; \
#   split-window -v \; \
#   split-window -v \; \
#   select-pane -t 0 \; \
#   split-window -v \; \
#   split-window -v \; \
#   send-keys -t 3 'node-red' C-m\; \
#   send-keys -t 4 'websocket --wait' C-m\; \
#   send-keys -t 5 'face' C-m\; \
#   send-keys -t 6 'ssh root@192.168.0.110 -t "bash -lic  'plexus' "' C-m\; \
#   send-keys -t 0 'pinky-kindyn --wait' C-m\; \
#   send-keys -t 1 'ssh roboy@192.168.0.104' C-m\; \
#   send-keys -t 2 'wheel-driver' C-m\; \
#   split-window -v \; \
#   send-keys 'rosrun rosserial_python serial_node.py tcp ' C-m \;\
#   select-pane -t 4\;\
#   split-window -v\; \
#   send-keys 'rviz' C-m\;\
#   split-window -v \;\
#   send-keys 'ik --wait'\;

#   # send-keys -t 2 /home/roboy/restart_face.sh C-m \;
# tmux attach
