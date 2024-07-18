#!/bin/bash

current_datetime=$(date +"%Y-%m-%d_%H-%M-%S")
mkdir -p /home/irobot/Desktop/Infos/Tests/$current_datetime

# Start a new tmux session named 'Vineslam'
tmux new-session -d -s Vineslam_session

# Pane 0
tmux rename-window -t Vineslam_session:0 'Window 1'
tmux split-window -v -t Vineslam_session:0
tmux split-window -h -t Vineslam_session:0
tmux select-pane -t 0
tmux split-window -h -t Vineslam_session:0
tmux select-pane -t 0
tmux split-window -h -t Vineslam_session:0
tmux select-pane -t 2
tmux split-window -h -t Vineslam_session:0
tmux select-pane -t 3
tmux split-window -v -t Vineslam_session:0
tmux select-pane -t 0
tmux split-window -v -p 99 -t Vineslam_session:0
tmux select-pane -t 7
tmux split-window -h -p 30 -t Vineslam_session:0
tmux select-pane -t 7
tmux split-window -v -p 99 -t Vineslam_session:0 



# Skript

#Roscore
tmux select-pane -t 0
tmux send-keys -t Vineslam_session:0 'noetic' C-m
tmux send-keys -t Vineslam_session:0 'roscore' C-m

#Playstation controller
tmux select-pane -t 1
tmux send-keys -t Vineslam_session:0 'sleep 1' C-m
tmux send-keys -t Vineslam_session:0 'noetic' C-m
tmux send-keys -t Vineslam_session:0 './startjoystick.sh' C-m

#Odometry & Steuerung
tmux select-pane -t 2
tmux send-keys -t Vineslam_session:0 'sleep 3' C-m
tmux send-keys -t Vineslam_session:0 'noetic' C-m
tmux send-keys -t Vineslam_session:0 'roslaunch roboteq_control bringup_vineslam.launch ' C-m

#Ros1 bridge
tmux select-pane -t 3
tmux send-keys -t Vineslam_session:0 'sleep 10' C-m
tmux send-keys -t Vineslam_session:0 'noetic' C-m
tmux send-keys -t Vineslam_session:0 'foxy' C-m
tmux send-keys -t Vineslam_session:0 'ros2 run ros1_bridge dynamic_bridge --bridge-1to2-topics' C-m

#Rosbag2
tmux select-pane -t 4
tmux send-keys -t Vineslam_session:0 'foxy' C-m
tmux send-keys -t Vineslam_session:0 'sleep 14' C-m
tmux send-keys -t Vineslam_session:0 'cd /home/irobot/Desktop/' C-m
tmux send-keys -t Vineslam_session:0 'ros2 bag record -o ~/Desktop/VineslamOutput/rosbag2  /clicked_point /detections_topic /diagnostics /emergency /goal_pose /gps_heading_topic /gps_topic /imu/magnetometer /imu_data_topic /imu_topic /initialpose /joint_states /joy /livox/lidar /move_base_simple/goal /odometry/filtered /parameter_events /rosout /rosout_agg /tf /tf_static ' C-m

#nothing
tmux select-pane -t 5
tmux send-keys -t Vineslam_session:0 'tmux kill-serv5' C-m

#Vineslam Node
tmux select-pane -t 6
tmux send-keys -t Vineslam_session:0 'sleep 17' C-m
tmux send-keys -t Vineslam_session:0 'foxy' C-m
tmux send-keys -t Vineslam_session:0 'cd ~/ros2_ws/src/vineslam1/test/test_slam_node' C-m
tmux send-keys -t Vineslam_session:0 'ros2 launch run.launch.py' C-m

#Localisation Node
tmux select-pane -t 7
tmux send-keys -t Vineslam_session:0 'foxy' C-m
tmux send-keys -t Vineslam_session:0 'ros2 run geofence geofence_node' C-m


#Localisation Node
tmux select-pane -t 8
tmux send-keys -t Vineslam_session:0 'foxy' C-m
tmux send-keys -t Vineslam_session:0 'cd ros2_ws/src/ublox/ublox_gps/launch' C-m
tmux send-keys -t Vineslam_session:0 'ros2 launch ublox_gps_vineslam.py' C-m

#Localisation Node
tmux select-pane -t 9
tmux send-keys -t Vineslam_session:0 'sleep 20' C-m
tmux send-keys -t Vineslam_session:0 'google-chrome /home/irobot/ros2_ws/src/geofence/geofence/subscrMap.html' C-m


# Attach to the session
tmux attach-session -t Vineslam_session

