# VineSLAM

**A multi-layer and multi-sensor Localization and Mapping approach for agricultural environments.**

## Table of Contents

1. [Overview](#overview)
3. [Hardware requirements](#hw_requirements)
4. [Installation](#installation)
5. [Configuration](#ros)
5. [How to run](#run)

## <a name="overview"/> Overview

**VineSLAM** is a localization and mapping algorithm designed for challenging agricultural environments such as mountain vineyards and flat orchards.
This algorithm is based on two main performance optimizers: **(1)** topological mapping to manage the memory resources and enable large-scale and long-term operation; and **(2)** CUDA-based GPU optimizations to improve runtime performance.

This algorithm supports two operation modes: **SLAM**, where a multi-layer map is built while the robot is simultaneously localized; and **localization-only**, using a pre-built map to localize the robot.

This implementation is currently supported in **ROS2 Foxy** and **ROS2 Eloquent**.

<div align="center">
<img align="center" width="600" height="400" src="./docs/vineslam_aosta_seq_robosense_speed.gif" alt=""> 
</div>

## <a name="hw_requirements"/> Requirements

### Processing requirements

**VineSLAM** can be executed in **CPU-only** and **GPU-enabled** modes.

To use the GPU optimization, you will need an **NVIDIA**-compatible GPU on your computer.

### Sensors

Mandatory sensors for execution:
- Wheel odometry
- 3D LiDAR

Optional sensors for improved accuracy:
- IMU
- GNSS

**VineSLAM** has been successfully tested with **Velodyne VLP-16**, **Robosense RS-LiDAR-32**, and **Livox**.

**VineSLAM** supports GNSS-based heading input using the Ublox receivers and drivers. Thus, [Ublox ROS package](https://github.com/KumarRobotics/ublox) is a VineSLAM dependency, but it is not mandatory to use.

## <a name="installation"/> Installation

### Dependencies

```
# General dependencies
sudo apt-get install libopencv-dev
sudo apt-get install libboost-dev
sudo apt-get install libasio-dev

# ROS dependencies (considering a ros-foxy-full installation)
sudo apt-get install ros-foxy-diagnostic-updater
sudo apt-get install ros-foxy-vision-msgs
```

To use the GPU optimized version, you must **install CUDA**, from [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads).

### Compilation

Before compiling, a definition should be changed to set the corresponding code for you LiDAR.

To do that, please change the [following line](https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam/-/blob/master/vineslam_ros/include/vineslam_ros.hpp#L67) in the file [vineslam_ros/include/vineslam_ros.hpp](./vineslam_ros/include/vineslam_ros.hpp) as follows:

````
#define LIDAR_TYPE 0  // for velodyne
#define LIDAR_TYPE 1  // for robosense
#define LIDAR_TYPE 2  // for livox
````

After that, you are ready to compile VineSLAM:

````
cd <path_to_ros2_ws>/src
git clone https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam -b master
git clone https://github.com/KumarRobotics/ublox -b foxy-devel
cd <path_to_ros2_ws>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
````

Note that if you have CUDA installed, VineSLAM will automatically build CUDA kernels and use GPU optimizations.

### Docker

To install and build VineSLAM you can also use the provided [Dockerfile](./docker/Dockerfile).
Note that the Docker environment does not consider CUDA installation.

````
git clone https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam -b master
cd vineslam/docker
docker build -t vineslam .
````

This will create a docker container with VineSLAM ready to compile and execute. Now you just have to...
````
docker run -it vineslam bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
````

## <a name="ros"/> Configuration

To access the ROS structure implemented by VineSLAM and the parameters tuning, please see the following [README file](./docs/interfaces.md).

## <a name="run"/> How to run

In the [test folder](./test) you can find two folders containing all the necessary files to execute both the SLAM and Localization nodes.

After performing the **topic remappings** and **parameters tuning** you can simply execute, for example the SLAM Node, as follows:

````
cd <path_to_ros2_ws>/src/vineslam/test/test_slam_node
ros2 launch run.launch.py
````

This will automatically open a rviz visualization panel properly configured.

The same can be done for the localization node in the respective test folder.

## Acknowledges 

<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<p float="left">
<img src="https://scorpion-h2020.eu/wp-content/uploads/2021/03/logo-scorpion-simple.png" alt="scorpion" height="120" >
</p>

*The developement of this project at this moment is being support by the [SCORPION project](https://scorpion-h2020.eu/):*
This project has received funding from the European Union’s Horizon 2020  research and innovation programme under grant agreement no. 101004085.



The development of this project had the support of the following projects:


* Supported by  [ROSIN](rosin-project.eu) - ROS-Industrial Quality-Assured Robot Software Components.  
* VineSLAM was started to be developed under [ROMOVI project](https://www.inesctec.pt/en/projects/romovi#about), a P2020 project.

<img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="60" >
<p float="left">
     <img src="https://repositorio.inesctec.pt/logos/compete.png" alt="eu_flag" height="45" align="left" >
     <img src="https://repositorio.inesctec.pt/logos/norte2020/portugal2020.svg" alt="eu_flag" height="45" align="left" >
     <img src="https://repositorio.inesctec.pt/logos/ue-feder_cor.jpg" alt="eu_flag" height="45" align="left" >
     <img src="https://repositorio.inesctec.pt/logos/logo_cores.jpg" alt="eu_flag" height="45" align="left" >
</p>