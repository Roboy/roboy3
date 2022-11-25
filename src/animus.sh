#!/bin/bash
source ~/.bashrc
rosparam set /kill_animus 1 && rosnode kill /animus_server_jetson 
tmux send-keys -t 3 animus C-m
#sudo jetson_clocks && /home/roboy/echocancel.sh && source /home/roboy/.virtualenvs/animus/bin/activate && PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp rosrun animus_server animus_server.py &
#echo $animus # &
pid=$!
echo $pid >> /home/roboy/animuspid
echo "animus is running on $pid"
