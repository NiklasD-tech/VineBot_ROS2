import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    dirname, filename = os.path.split(os.path.realpath(__file__))
    config_path = os.path.join(dirname, 'config.yaml')
    rviz_path = os.path.join(dirname, 'visualization.rviz')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    ld = LaunchDescription()

        #Lidar
    lidar=LaunchDescription([
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("livox_ros2_driver"), '/launch', '/livox_lidar_launch.py'])
                )
    ])
    ld.add_action(lidar)

    # VineSLAM node
    vineslam = Node(
        package='vineslam_ros',
        executable='slam_node',
        name='slam_node',
        parameters=[config],
        remappings=[
            ('/odom_topic', '/odometry/filtered'),
            #('/odom_topic', '/rtabmap/odom'),
            #('/odom_topic', '/vinebot_controller/odom'),
            ('/scan_topic', '/livox/lidar'),
            ('/gps_topic', '/fix'),
            ('/gps_heading_topic', '/navrelposned'),
            #('/imu_topic', '/imu/data'),
                    ],
    )
    ld.add_action(vineslam)

    # Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path, '--ros-args', '--log-level', 'INFO'],
    )
    ld.add_action(rviz)

    return ld
