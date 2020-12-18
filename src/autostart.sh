#!/bin/bash
tmux new-session \; \
  send-keys 'roscore&' C-m \; \
  split-window -h \; \
  split-window -v \; \
  split-window -v \; \
  split-window -v \; \
  select-pane -t 0 \; \
  split-window -v \; \
  send-keys -t 2 'node-red' C-m\; \
  send-keys -t 3 'websocket --wait' C-m\; \
  send-keys -t 4 'face' C-m\; \
  send-keys -t 5 'ssh root@192.168.0.124 -t "bash -lic  'plexus' "' C-m\; \
  send-keys -t 0 'pinky-kindyn --wait' C-m\; \
  send-keys -t 1 'animus' \;
