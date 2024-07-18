#pragma once

#include "vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
class SLAMNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  SLAMNode();

  // Class destructor - saves the map to an output xml file
  ~SLAMNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Load topological map
  void loadMaps();

  // Get static transformations
  void getTfs();

  // Automatically generate a topological map
  void genTopologicalMap();

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // Publish the tfs exported by the localization node
  void broadcastTfsWithTopic();
  void broadcastTfs();

  // ROS subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr landmark_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr gps_heading_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscriber_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
};

}  // namespace vineslam