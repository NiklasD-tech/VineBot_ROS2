# Startup Instructions

## This repository contains the status of the Vinebot in the Ros 2 section. It must be used in conjunction with the VineBot repository.

The VineSLAM and the whole robot can be started via a .bash script. <br>
`./ros2_ws/src/startup_vineslam.sh`


## Alias definition in ~/.bashrc
If ROS 1 and ROS 2 are on the same PC, you must source different environment variables. The best way is to use the alias

alias "noetic"="source /opt/ros/noetic/setup.bash;echo 'noetic is active' && source ~/vinebot_ws/devel/setup.bash;echo 'source vinebot_ws' "
alias "foxy"="source /opt/ros/foxy/setup.bash;echo 'Foxy is active' && source ~/ros2_ws/install/setup.bash;echo 'source ros2_ws' "

The order of the sources is important, first source noetic then vinebot_ws. 

## ROSbridge
https://roboticsknowledgebase.com/wiki/interfacing/ros1_ros2_bridge/  <br>
`sudo apt install ros-foxy-ros1-bridge` <br>
https://www.youtube.com/watch?v=4uKWhfW3_1s <br>
`ros2 run ros1_bridge dynamic_bridge --bridge-all-topics `  bridge all topics (both directions) <br>
                                    --bridge-1to2-topics  bridge one direction

## ROS 2 create RViz launch file
https://www.youtube.com/watch?v=WA3ynlo30vw


## Plot TF Tree
`ros2 run tf2_tools view_frames.py`

## ROS 2 livox installation
https://github.com/Livox-SDK/livox_ros2_driver <br>
You do not need to install the SDK, just the livox_ros2_driver. <br>
but there must be no /usr/local/lib/livox_sdk_static.a!!!!! <br>

Start command:  <br>
`cd ~/ros2_ws/src/ws_livox/src/livox_ros2_driver/launch` <br>
`ros2 launch livox_ros2_driver livox_lidar_launch.py` <br>
`ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py` 

## rosbag1
`rosbag record -a -o ~/Desktop/Infos/rosbag1/test_bag_1`<br>
`rosbag info test_bag_1/`<br>

## rosbag2
rosbag must always be started first, then all other nodes <br>
Record:<br>
`ros2 bag record -a -o ~/Desktop/Infos/rosbag2/test_bag_1`<br>
`ros2 bag record - o ~/Desktop/Infos/rosbag2/test_bag_1 /clicked_point /detections_topic /diagnostics /emergency /goal_pose /gps_heading_topic /gps_topic /imu/data /imu/magnetometer /imu_data_topic /imu_topic /initialpose /joint_states /joy /livox/lidar /move_base_simple/goal /odometry/filtered /parameter_events /rosout /rosout_agg /tf /tf_static `<br>
<br>
Info:<br>
`cd ~/Desktop/Infos/rosbag2/`<br>
`ros2 bag info test_bag_1/`<br>
 <br>
Play:<br>
`ros2 bag play rosbag2_testlab_4/ --topics /odometry/filtered /livox/lidar /tf_static /joint_states /tf`<br>
lidar has to be disabled!<br>

## Troubleshooting:
Bug1: vineslam node spams: <br>
[rviz2-3] Warning: Invalid frame ID "map_nn2" passed to canTransform argument target_frame - frame does not exist at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp<br>
Solution: No lidar data? Is the lidar node started? Ethernet cable connected?<br>
<br>
Bug 2: ros2_ws does not build with cb (colcon build -symlink install)<br>
Solution: source before!   (Foxy)<br>


## Ardusimple simpleRTK2B - 4G NTRIP Starter Kit setup<br>

https://www.ardusimple.com/use-pointperfect-with-u-blox-receivers/<br>
https://www.ardusimple.com/4g-ntrip-client-hookup-guide/#overview<br>
https://www.ardusimple.com/how-to-use-ardusimple-rtk-receivers-and-get-gps-data-in-ros/<br>
https://www.ardusimple.com/visualise-real-time-data-in-google-earth-with-simplertk2b/<br>
Please make sure that your GNSS receiver has the correct configuration (Rover 1Hz with 4G NTRIP Client (sending NMEA-GGA)) from here:<br>
https://www.ardusimple.com/configuration-files/<br>


`~/ros2_ws/src/ublox/ublox_gps/config`     <br> 
`ros2 launch ublox_gps ublox_gps_node-launch.py` <br>


## Launching U-Center on Linux
`cd /Desktop`<br>
`wine ./u-center_v24.04.exe`<br>

## Check the antenna signal in Ublox
```
cd /Desktop <br>
wine ./u-center_v24.04.exe <br>
view → chart view → at the bottom enter "SV C" in search <br>
```
