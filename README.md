Startup Instructions

The VineSLAM and the whole robot can be started via a .bash script. 
./ros2_ws/src/startup_vineslam.sh


Alias definition in ~/.bashrc
If ROS 1 and ROS 2 are on the same PC, then different sources. The best way is to use the alias

alias "noetic"="source /opt/ros/noetic/setup.bash;echo 'noetic is active' && source ~/vinebot_ws/devel/setup.bash;echo 'source vinebot_ws' "
alias "foxy"="source /opt/ros/foxy/setup.bash;echo 'Foxy is active' && source ~/ros2_ws/install/setup.bash;echo 'source ros2_ws' "

The order of the sources is important, first noetic sourcen and then vinebot_ws. 

Rosbridge
https://roboticsknowledgebase.com/wiki/interfacing/ros1_ros2_bridge/
sudo apt install ros-foxy-ros1-bridge
https://www.youtube.com/watch?v=4uKWhfW3_1s
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics #bridge all topics (both directions)
				      --bridge-1to2-topics # one direction

Ros2 create Rviz launch file
https://www.youtube.com/watch?v=WA3ynlo30vw


Plot TF Tree
ros2 run tf2_tools view_frames.py

ros2 livox installation
https://github.com/Livox-SDK/livox_ros2_driver
You do not need to install the SDK, just the livox_ros2_driver. 
but there must be no /usr/local/lib/livox_sdk_static.a!!!!!

Start command:
cd ~/ros2_ws/src/ws_livox/src/livox_ros2_driver/launch
ros2 launch livox_ros2_driver livox_lidar_launch.py
ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py 

rosbag1
rosbag record -a -o ~/Desktop/Infos/rosbag1/test_bag_1
rosbag info test_bag_1/

rosbag2
rosbag must always be started first, then all other nodes
Record:
ros2 bag record -a -o ~/Desktop/Infos/rosbag2/test_bag_1

ros2 bag record - o ~/Desktop/Infos/rosbag2/test_bag_1 /clicked_point /detections_topic /diagnostics /emergency /goal_pose /gps_heading_topic /gps_topic /imu/data /imu/magnetometer /imu_data_topic /imu_topic /initialpose /joint_states /joy /livox/lidar /move_base_simple/goal /odometry/filtered /parameter_events /rosout /rosout_agg /tf /tf_static 


Info:
cd ~/Desktop/Infos/rosbag2/
ros2 bag info test_bag_1/
Play:
ros2 bag play rosbag2_testlab_4/ --topics /odometry/filtered /livox/lidar /tf_static /joint_states /tf
lidar has to be disabled!

Troubleshooting:
Bug1: vineslam node spams:
[rviz2-3] Warning: Invalid frame ID "map_nn2" passed to canTransform argument target_frame - frame does not exist at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
Solution: No lidar data? Is the lidar node started? Ethernet cable connected?

Bug 2: ros2_ws does not build with cb (colcon build -symlink install)
Solution: source before!   (Foxy)




Ardusimple simpleRTK2B - 4G NTRIP Starter Kit setup

https://www.ardusimple.com/use-pointperfect-with-u-blox-receivers/
https://www.ardusimple.com/4g-ntrip-client-hookup-guide/#overview
https://www.ardusimple.com/how-to-use-ardusimple-rtk-receivers-and-get-gps-data-in-ros/
https://www.ardusimple.com/visualise-real-time-data-in-google-earth-with-simplertk2b/
Please make sure that your GNSS receiver has the correct configuration (Rover 1Hz with 4G NTRIP Client (sending NMEA-GGA)) from here:
https://www.ardusimple.com/configuration-files/


~/ros2_ws/src/ublox/ublox_gps/config$ ros2 launch ublox_gps ublox_gps_node-launch.py


Launching U-Center on Linux
cd /Desktop
wine ./u-center_v24.04.exe

Check the antenna signal in Ublox
cd /Desktop
wine ./u-center_v24.04.exe
view → chart view → at the bottom enter "SV C" in search
