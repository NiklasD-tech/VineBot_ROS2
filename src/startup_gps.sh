#!/bin/bash

current_datetime=$(date +"%Y-%m-%d_%H-%M-%S")
mkdir -p /home/irobot/Desktop/Infos/Tests/$current_datetime

# Start a new tmux session named 'gps'
tmux new-session -d -s gps_session

# Pane 0
tmux rename-window -t gps_session:0 'Window 1'
tmux split-window -v -t gps_session:0
tmux split-window -h -t gps_session:0
tmux select-pane -t 0
tmux split-window -h -t gps_session:0
tmux select-pane -t 0
tmux split-window -h -t gps_session:0
tmux select-pane -t 2
tmux split-window -h -t gps_session:0
tmux select-pane -t 3
tmux split-window -v -t gps_session:0
tmux select-pane -t 0
tmux split-window -v -t gps_session:0

# Skript

#Roscore
tmux select-pane -t 0
tmux send-keys -t gps_session:0 'noetic' C-m


#nichts
tmux select-pane -t 1
tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0  'google-chrome /home/irobot/ros2_ws/src/geofence/geofence/subscrMap.html' C-m



#start GPS publisher
tmux select-pane -t 2
tmux send-keys -t gps_session:0 'sleep 3' C-m
tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0 'ros2 topic echo /navstatus' C-m


#Ros1 bridge
tmux select-pane -t 3
tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0 'ros2 topic echo /fix' C-m



#Rosbag2
tmux select-pane -t 4
tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0 'ros2 topic list' C-m

#nothing
tmux select-pane -t 5
tmux send-keys -t gps_session:0 'tmux kill-serv5' C-m

#gps Node
tmux select-pane -t 6

tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0 'cd ros2_ws/src/ublox/ublox_gps/launch' C-m
tmux send-keys -t gps_session:0 'ros2 launch ublox_gps_vineslam.py' C-m

#Localisation Node
tmux select-pane -t 7
tmux send-keys -t gps_session:0 'foxy' C-m
tmux send-keys -t gps_session:0 'sleep 3' C-m
tmux send-keys -t gps_session:0 'ros2 run geofence geofence_node .py' C-m
 



# Attach to the session
tmux attach-session -t gps_session
