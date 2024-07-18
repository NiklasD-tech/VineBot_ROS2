#include "../include/vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
void VineSLAM_ros::publishDenseInfo()
{
  if (init_flag_)
  {
    return;
  }

  if (params_.publish_level_ == 0)
  {
    false;  // do not publish anything
  }
  else if (params_.publish_level_ == 1)
  {
    publish3DMap();
  }
  else if (params_.publish_level_ == 2)
  {
    publish3DMap();
    publishTopologicalMap();
  }
}

void VineSLAM_ros::publishGridMapLimits() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Publish the map
  visualization_msgs::msg::Marker grid_map_cube;
  grid_map_cube.header.frame_id = params_.world_frame_id_;
  grid_map_cube.header.stamp = rclcpp::Time();
  grid_map_cube.id = 0;
  grid_map_cube.type = visualization_msgs::msg::Marker::CUBE;
  grid_map_cube.action = visualization_msgs::msg::Marker::ADD;
  grid_map_cube.color.a = 0.7;
  grid_map_cube.color.r = 0;
  grid_map_cube.color.g = 1;
  grid_map_cube.color.b = 0;
  grid_map_cube.pose.position.x = grid_map_->origin_.x_ + grid_map_->width_ / 2;
  grid_map_cube.pose.position.y = grid_map_->origin_.y_ + grid_map_->lenght_ / 2;
  grid_map_cube.pose.position.z = grid_map_->origin_.z_ + grid_map_->height_ / 2;
  grid_map_cube.pose.orientation.x = 0;
  grid_map_cube.pose.orientation.y = 0;
  grid_map_cube.pose.orientation.z = 0;
  grid_map_cube.pose.orientation.w = 1;
  grid_map_cube.scale.x = grid_map_->width_;
  grid_map_cube.scale.y = grid_map_->lenght_;
  grid_map_cube.scale.z = grid_map_->height_;

  marker_array.markers.push_back(grid_map_cube);
  grid_map_publisher_->publish(marker_array);
}

void VineSLAM_ros::publishRobotBox(const Pose& robot_pose) const
{
  // Robot pose orientation to quaternion
  tf2::Quaternion q;
  q.setRPY(robot_pose.R_, robot_pose.P_, robot_pose.Y_);

  // Set robot original box corners
  visualization_msgs::msg::Marker robot_cube;
  robot_cube.header.frame_id = params_.world_frame_id_;
  robot_cube.header.stamp = rclcpp::Time();
  robot_cube.id = 0;
  robot_cube.type = visualization_msgs::msg::Marker::CUBE;
  robot_cube.action = visualization_msgs::msg::Marker::ADD;
  robot_cube.color.a = 0.7;
  robot_cube.color.r = 0;
  robot_cube.color.g = 0;
  robot_cube.color.b = 1;
  robot_cube.scale.x = params_.robot_dim_x_;
  robot_cube.scale.y = params_.robot_dim_y_;
  robot_cube.scale.z = params_.robot_dim_z_;
  robot_cube.pose.position.x = robot_pose.x_;
  robot_cube.pose.position.y = robot_pose.y_;
  robot_cube.pose.position.z = robot_pose.z_ + params_.robot_dim_z_ / 2;
  robot_cube.pose.orientation.x = q.getX();
  robot_cube.pose.orientation.y = q.getY();
  robot_cube.pose.orientation.z = q.getZ();
  robot_cube.pose.orientation.w = q.getW();

  robot_box_publisher_->publish(robot_cube);
}

void VineSLAM_ros::publishLocalSemanticMap(const Pose& origin, const std::vector<SemanticFeature>& landmarks) const
{
}

void VineSLAM_ros::publishSemanticMap() const
{
}

void VineSLAM_ros::publishSemanticMapFromArray(const std::map<int, SemanticFeature>& landmarks) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  visualization_msgs::msg::Marker ellipse;

  // Define marker layout
  marker.ns = "/markers";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration(970000000);

  // Define marker layout
  ellipse.ns = "/ellipses";
  ellipse.type = visualization_msgs::msg::Marker::CYLINDER;
  ellipse.action = visualization_msgs::msg::Marker::ADD;
  ellipse.scale.z = 0.01f;
  ellipse.pose.orientation.x = 0.0f;
  ellipse.pose.orientation.y = 0.0f;
  ellipse.color.r = 0.0f;
  ellipse.color.g = 1.0f;
  ellipse.color.b = 0.0f;
  ellipse.color.a = 1.0f;
  ellipse.lifetime = rclcpp::Duration(970000000);

  // Publish markers
  int id = 1;
  for (const auto& l_sfeature : landmarks)
  {
    // Draw sfeature mean
    marker.ns = "/marker";
    marker.id = id;
    marker.header.stamp = rclcpp::Time();
    marker.header.frame_id = params_.world_frame_id_;
    marker.pose.position.x = l_sfeature.second.pos_.x_;
    marker.pose.position.y = l_sfeature.second.pos_.y_;
    if (l_sfeature.second.label_ == 1)  // trunk
    {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.50;
      marker.pose.position.z = l_sfeature.second.pos_.z_;
    }
    else  // not a trunk
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.7;
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;
      marker.pose.position.z = l_sfeature.second.pos_.z_;
    }

    marker_array.markers.push_back(marker);

    id++;
  }

  semantic_map_publisher_->publish(marker_array);
}

void VineSLAM_ros::publishElevationMap() const
{
  visualization_msgs::msg::MarkerArray elevation_map_marker;
  visualization_msgs::msg::Marker cube;

  float min_height = grid_map_->origin_.z_;
  float max_height = grid_map_->origin_.z_ + grid_map_->height_;

  // Define marker layout
  cube.ns = "/elevation_cube";
  cube.header.stamp = rclcpp::Time();
  cube.header.frame_id = params_.world_frame_id_;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;
  cube.color.a = 1.0;
  cube.lifetime = rclcpp::Duration(970000000);

  // Compute map layer bounds
  float xmin = elevation_map_->origin_.x_;
  float xmax = xmin + elevation_map_->width_;
  float ymin = elevation_map_->origin_.y_;
  float ymax = ymin + elevation_map_->lenght_;
  for (float i = xmin; i < xmax - elevation_map_->resolution_;)
  {
    for (float j = ymin; j < ymax - elevation_map_->resolution_;)
    {
      float z = (*elevation_map_)(i, j);
      if (z == 0)
      {
        j += elevation_map_->resolution_;
        continue;
      }

      float r, g, b;
      float h = (static_cast<float>(1) - std::min(std::max((std::fabs(z) - min_height) / (max_height - min_height), static_cast<float>(0)), static_cast<float>(1)));
      vineslam::ElevationMap::color(h, r, g, b);

      cube.pose.position.x = i;
      cube.pose.position.y = j;
      cube.pose.position.z = z / 2;
      cube.scale.x = elevation_map_->resolution_;
      cube.scale.y = elevation_map_->resolution_;
      cube.scale.z = z;
      cube.color.r = r;
      cube.color.g = g;
      cube.color.b = b;
      cube.id = elevation_map_marker.markers.size();

      elevation_map_marker.markers.push_back(cube);

      j += elevation_map_->resolution_;
    }
    i += elevation_map_->resolution_;
  }

  elevation_map_publisher_->publish(elevation_map_marker);
}

void VineSLAM_ros::publish3DMap()
{
  std::vector<Corner> corner_features = topological_map_->getCorners();
  std::vector<Planar> planar_features = topological_map_->getPlanars();

  publish3DMap(Pose(0, 0, 0, 0, 0, 0), topological_map_->planes_, map3D_planes_publisher_);

  sensor_msgs::msg::PointCloud2 corner_cloud2;
  Convertions::toROSMsg(corner_features, corner_cloud2);
  corner_cloud2.header.frame_id = params_.world_frame_id_;
  map3D_corners_publisher_->publish(corner_cloud2);

  sensor_msgs::msg::PointCloud2 planar_cloud2;
  Convertions::toROSMsg(planar_features, planar_cloud2);
  planar_cloud2.header.frame_id = params_.world_frame_id_;
  map3D_planars_publisher_->publish(planar_cloud2);
}

void VineSLAM_ros::publish3DMap(const std::vector<Plane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::msg::Marker::POINTS;
  viz_pts.action = visualization_msgs::msg::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.lifetime = rclcpp::Duration(970000000);
  viz_pts.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  float i = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      viz_pts.color.r = 0.0f;
      viz_pts.color.g = 0.0f;
      viz_pts.color.b = i / 3;
      viz_pts.color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
    }
    i += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub->publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::msg::Marker::POINTS;
  viz_pts.action = visualization_msgs::msg::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.lifetime = rclcpp::Duration(970000000);
  viz_pts.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  float i = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      viz_pts.color.r = 0.0f;
      viz_pts.color.g = 0.0f;
      viz_pts.color.b = i * 10;
      viz_pts.color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
    }
    i += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub->publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const std::vector<SemiPlane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker viz_pts;
  visualization_msgs::msg::Marker viz_line, viz_line1;
  visualization_msgs::msg::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::msg::Marker::POINTS;
  viz_pts.action = visualization_msgs::msg::Marker::ADD;
  viz_pts.lifetime = rclcpp::Duration(970000000);
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::msg::Marker::ADD;
  viz_line.scale.x = 0.1;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.lifetime = rclcpp::Duration(970000000);
  viz_line.header.frame_id = params_.world_frame_id_;

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::msg::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::msg::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.lifetime = rclcpp::Duration(970000000);
  viz_normal.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  float i = 0;
  float l = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      std_msgs::msg::ColorRGBA color;
      color.r = i / 10.;
      color.g = i / 10.;
      color.b = i / 10.;
      color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
      viz_pts.colors.push_back(color);
    }
    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      for (size_t k = 1; k < plane.extremas_.size(); k++)
      {
        Point l_pt = plane.extremas_[k] * robot_tf;

        std_msgs::msg::ColorRGBA color;
        color.r = static_cast<int>(i + 1) % 2;
        color.g = static_cast<int>(i + 1) % 3;
        color.b = static_cast<int>(i + 1) % 4;
        color.a = 1.0;

        geometry_msgs::msg::Point viz_pt;
        viz_pt.x = p_pt.x_;
        viz_pt.y = p_pt.y_;
        viz_pt.z = p_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_pt.x = l_pt.x_;
        viz_pt.y = l_pt.y_;
        viz_pt.z = l_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_line.color = color;
        viz_line.id = l;

        viz_pts.points.push_back(viz_pt);
        viz_pts.colors.push_back(color);
        marker_array.markers.push_back(viz_line);
        viz_line.points.clear();

        p_pt = l_pt;
        l += 1;
      }
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      Point l_pt = plane.extremas_[plane.extremas_.size() - 1] * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      viz_pt.x = p_pt.x_;
      viz_pt.y = p_pt.y_;
      viz_pt.z = p_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_line.id = l++;
      marker_array.markers.push_back(viz_line);
      viz_line.points.clear();
    }

    Point centroid = plane.centroid_ * robot_tf;
    geometry_msgs::msg::Point p1, p2;
    p1.x = centroid.x_;
    p1.y = centroid.y_;
    p1.z = centroid.z_;
    p2.x = p1.x + plane.a_;
    p2.y = p1.y + plane.b_;
    p2.z = p1.z + plane.c_;
    viz_normal.id = i;
    viz_normal.points.push_back(p1);
    viz_normal.points.push_back(p2);
    marker_array.markers.push_back(viz_normal);
    viz_normal.points.clear();

    i += 1;
    l += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub->publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker viz_pts;
  visualization_msgs::msg::Marker viz_line;
  visualization_msgs::msg::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::msg::Marker::POINTS;
  viz_pts.action = visualization_msgs::msg::Marker::ADD;
  viz_pts.lifetime = rclcpp::Duration(970000000);
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::msg::Marker::ADD;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.scale.x = 0.1;
  viz_line.scale.y = 0.1;
  viz_line.lifetime = rclcpp::Duration(970000000);
  viz_line.header.frame_id = params_.world_frame_id_;

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::msg::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::msg::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.lifetime = rclcpp::Duration(970000000);
  viz_normal.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  float i = 0;
  float l = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      std_msgs::msg::ColorRGBA color;
      color.r = i / 10.;
      color.g = i / 10.;
      color.b = i / 10.;
      color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
      viz_pts.colors.push_back(color);
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      for (size_t k = 1; k < plane.extremas_.size(); k++)
      {
        Point l_pt = plane.extremas_[k] * robot_tf;

        std_msgs::msg::ColorRGBA color;
        color.r = static_cast<int>(i + 1) % 2;
        color.g = static_cast<int>(i + 1) % 3;
        color.b = static_cast<int>(i + 1) % 4;
        color.a = 1.0;

        geometry_msgs::msg::Point viz_pt;
        viz_pt.x = p_pt.x_;
        viz_pt.y = p_pt.y_;
        viz_pt.z = p_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_pt.x = l_pt.x_;
        viz_pt.y = l_pt.y_;
        viz_pt.z = l_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_line.color = color;
        viz_line.id = l;

        viz_pts.points.push_back(viz_pt);
        viz_pts.colors.push_back(color);
        marker_array.markers.push_back(viz_line);
        viz_line.points.clear();

        p_pt = l_pt;
        l += 1;
      }
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      Point l_pt = plane.extremas_[plane.extremas_.size() - 1] * robot_tf;

      geometry_msgs::msg::Point viz_pt;
      viz_pt.x = p_pt.x_;
      viz_pt.y = p_pt.y_;
      viz_pt.z = p_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_line.id = l;
      marker_array.markers.push_back(viz_line);
      viz_line.points.clear();
    }

    Point centroid = plane.centroid_ * robot_tf;
    geometry_msgs::msg::Point p1, p2;
    p1.x = centroid.x_;
    p1.y = centroid.y_;
    p1.z = centroid.z_;
    p2.x = p1.x + plane.a_;
    p2.y = p1.y + plane.b_;
    p2.z = p1.z + plane.c_;
    viz_normal.id = i;
    viz_normal.points.push_back(p1);
    viz_normal.points.push_back(p2);
    marker_array.markers.push_back(viz_normal);
    viz_normal.points.clear();

    i += 1;
    l += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub->publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  std::vector<Corner> cloud_out;
  for (const auto& corner : corners)
  {
    Point l_pt = corner.pos_ * robot_tf;
    Corner c(l_pt, 0);
    cloud_out.push_back(c);
  }

  sensor_msgs::msg::PointCloud2 cloud_out2;
  Convertions::toROSMsg(cloud_out, cloud_out2);
  cloud_out2.header.frame_id = params_.world_frame_id_;
  pub->publish(cloud_out2);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  std::vector<Corner> cloud_out;
  for (const auto& corner : corners)
  {
    Point l_pt = corner.pos_ * robot_tf;
    Corner c(l_pt, 0);
    cloud_out.push_back(c);
  }

  sensor_msgs::msg::PointCloud2 cloud_out2;
  Convertions::toROSMsg(cloud_out, cloud_out2);
  cloud_out2.header.frame_id = params_.world_frame_id_;
  pub->publish(cloud_out2);
}

void VineSLAM_ros::publish3DMap(const std::vector<Planar>& planars, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  std::vector<Planar> cloud_out;
  for (const auto& planar : planars)
  {
    Point l_pt = planar.pos_ * robot_tf;
    Planar p(l_pt, 0);
    cloud_out.push_back(p);
  }

  sensor_msgs::msg::PointCloud2 planar_out2;
  Convertions::toROSMsg(cloud_out, planar_out2);
  planar_out2.header.frame_id = params_.world_frame_id_;
  pub->publish(planar_out2);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  std::vector<Planar> cloud_out;
  for (const auto& planar : planars)
  {
    Point l_pt = planar.pos_ * robot_tf;
    Planar p(l_pt, 0);
    cloud_out.push_back(p);
  }

  sensor_msgs::msg::PointCloud2 planar_out2;
  Convertions::toROSMsg(cloud_out, planar_out2);
  planar_out2.header.frame_id = params_.world_frame_id_;
  pub->publish(planar_out2);
}

void VineSLAM_ros::make6DofMarker(visualization_msgs::msg::InteractiveMarker& imarker, Pose pose, std::string marker_name)
{
  // Convert euler to quaternion
  tf2::Quaternion q;
  q.setRPY(pose.R_, pose.P_, pose.Y_);

  // Create and initialize the interactive marker
  imarker.header.frame_id = params_.world_frame_id_;
  imarker.pose.position.x = pose.x_;
  imarker.pose.position.y = pose.y_;
  imarker.pose.position.z = pose.z_;
  imarker.pose.orientation.x = q.getX();
  imarker.pose.orientation.y = q.getY();
  imarker.pose.orientation.z = q.getZ();
  imarker.pose.orientation.w = q.getW();
  imarker.scale = 2;
  imarker.name = marker_name;
  imarker.description = "6-DoF interactive marker";

  // Create and set controls
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;

  visualization_msgs::msg::Marker marker_box;
  marker_box.type = visualization_msgs::msg::Marker::SPHERE;
  marker_box.scale.x = imarker.scale * 0.3;
  marker_box.scale.y = imarker.scale * 0.3;
  marker_box.scale.z = imarker.scale * 0.3;
  marker_box.color.r = 1;
  marker_box.color.b = 0;
  marker_box.color.g = 0;
  marker_box.color.a = 0.9;

  control.markers.push_back(marker_box);
  imarker.controls.push_back(control);
  imarker.controls[0].interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);
}

void VineSLAM_ros::publishTopologicalMap()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker line_strip, aligned_line_strip;
  visualization_msgs::msg::Marker circle;

  // Define markers layout
  circle.header.stamp = rclcpp::Time();
  circle.header.frame_id = params_.world_frame_id_;
  circle.ns = "/circles";
  circle.type = visualization_msgs::msg::Marker::SPHERE;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.scale.x = 0.8;
  circle.scale.y = 0.8;
  circle.scale.z = 0.8;
  circle.pose.orientation.x = 0.0;
  circle.pose.orientation.y = 0.0;
  circle.pose.orientation.z = 0.0;
  circle.pose.orientation.w = 1.0;
  circle.color.r = 0.0f;
  circle.color.g = 0.0f;
  circle.color.b = 1.0f;
  circle.color.a = 1.0f;
  circle.header.stamp = rclcpp::Time();
  line_strip.header.stamp = rclcpp::Time();
  line_strip.header.frame_id = params_.world_frame_id_;
  line_strip.ns = "/lines";
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;

  aligned_line_strip.header.stamp = rclcpp::Time();
  aligned_line_strip.header.frame_id = params_.world_frame_id_;
  aligned_line_strip.ns = "/alinged_lines";
  aligned_line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  aligned_line_strip.action = visualization_msgs::msg::Marker::ADD;

  int id = 0;                    // marker identifier
  std::vector<int> connections;  // array to store the drawn connections
  for (size_t i = 0; i < topological_map_->graph_vertexes_.size(); i++)
  {
    // Draw circle at the vertexes
    circle.id = id++;
    circle.pose.position.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].center_.x_;
    circle.pose.position.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].center_.y_;

    // Draw rectangles to observe the area of each node (using line strips)
    line_strip.id = id++;
    geometry_msgs::msg::Point c1, c2, c3, c4;
    c1.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[0].x_;
    c1.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[0].y_;
    c2.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[1].x_;
    c2.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[1].y_;
    c3.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[2].x_;
    c3.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[2].y_;
    c4.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[3].x_;
    c4.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].rectangle_[3].y_;
    line_strip.points.push_back(c1);
    line_strip.points.push_back(c2);
    line_strip.points.push_back(c3);
    line_strip.points.push_back(c4);
    line_strip.points.push_back(c1);

    // Draw aligned rectangles by undoing the orientation of each one
    if (topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_.size() == 4)
    {
      aligned_line_strip.id = id;
      geometry_msgs::msg::Point aligned_c1, aligned_c2, aligned_c3, aligned_c4;
      aligned_c1.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[0].x_;
      aligned_c1.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[0].y_;
      aligned_c2.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[1].x_;
      aligned_c2.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[1].y_;
      aligned_c3.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[2].x_;
      aligned_c3.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[2].y_;
      aligned_c4.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[3].x_;
      aligned_c4.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].aligned_rectangle_[3].y_;
      aligned_line_strip.color.b = (static_cast<float>(i) / static_cast<float>(topological_map_->graph_vertexes_.size()));
      aligned_line_strip.color.g = 0.5;
      aligned_line_strip.color.r = 0.3;
      aligned_line_strip.color.a = 1.0;
      aligned_line_strip.scale.x = 0.3;
      aligned_line_strip.points.push_back(aligned_c1);
      aligned_line_strip.points.push_back(aligned_c2);
      aligned_line_strip.points.push_back(aligned_c3);
      aligned_line_strip.points.push_back(aligned_c4);
      aligned_line_strip.points.push_back(aligned_c1);
      // marker_array.markers.push_back(aligned_line_strip);
      aligned_line_strip.points.clear();
    }

    // Select color of the rectangles - highligh active and allocated nodes (!)
    if (std::find(topological_map_->active_nodes_vertexes_.begin(), topological_map_->active_nodes_vertexes_.end(), topological_map_->graph_vertexes_[i]) == topological_map_->active_nodes_vertexes_.end())
    {
      line_strip.color.b = (static_cast<float>(i) / static_cast<float>(topological_map_->graph_vertexes_.size()));
      line_strip.color.g = 0.5;
      line_strip.color.r = 0.3;
      line_strip.color.a = 0.9;
      line_strip.scale.x = 0.3;
    }
    else
    {
      if (std::find(topological_map_->allocated_nodes_vertexes_.begin(), topological_map_->allocated_nodes_vertexes_.end(), topological_map_->graph_vertexes_[i]) == topological_map_->allocated_nodes_vertexes_.end())
      {
        line_strip.color.b = 0.0;
        line_strip.color.g = 0.0;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
        line_strip.scale.x = 0.5;
      }
      else
      {
        line_strip.color.b = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.a = 1.0;
        line_strip.scale.x = 0.5;
      }
    }

    // Save markers
    marker_array.markers.push_back(circle);
    marker_array.markers.push_back(line_strip);

    // Clear the line strip to use in the next iteration
    line_strip.points.clear();

    // Compute the adjacent vertexes and draw the connections
    std::vector<uint32_t> adj_list;
    topological_map_->getAdjacentList(topological_map_->graph_vertexes_[i], &adj_list);
    for (auto it : adj_list)
    {
      // We do not re-draw connections, so we check if this connection was already drawn
      int v1_index = topological_map_->map_[it].index_;
      int v2_index = topological_map_->map_[topological_map_->graph_vertexes_[i]].index_;
      int connection = std::stoi(std::to_string(v1_index) + std::to_string(v2_index));
      if (std::find(connections.begin(), connections.end(), connection) == connections.end())
      {
        connections.push_back(connection);

        line_strip.id = id++;
        geometry_msgs::msg::Point v1, v2;
        v1.x = topological_map_->map_[it].center_.x_;
        v1.y = topological_map_->map_[it].center_.y_;
        v2.x = topological_map_->map_[topological_map_->graph_vertexes_[i]].center_.x_;
        v2.y = topological_map_->map_[topological_map_->graph_vertexes_[i]].center_.y_;
        line_strip.points.push_back(v1);
        line_strip.points.push_back(v2);

        line_strip.color.b = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.r = 1.0;
        line_strip.color.a = 0.6;
        line_strip.scale.x = 0.2;

        // Save line strip
        // marker_array.markers.push_back(line_strip);

        // Clear the line strip to use in the next iteration
        line_strip.points.clear();
      }
    }

    // Clear the line strip to use in the next iteration
    line_strip.points.clear();
  }

  // for (int i = 0; i < topological_map_->allocated_nodes_vertexes_.size(); i++)
  // {
  //   OccupancyMap* grid_map = topological_map_->map_[topological_map_->allocated_nodes_vertexes_[i]].grid_map_;
  //   // Publish the map
  //   visualization_msgs::msg::Marker grid_map_cube;
  //   grid_map_cube.header.frame_id = params_.world_frame_id_;
  //   grid_map_cube.header.stamp = rclcpp::Time();
  //   grid_map_cube.id = id++;
  //   grid_map_cube.type = visualization_msgs::msg::Marker::CUBE;
  //   grid_map_cube.action = visualization_msgs::msg::Marker::ADD;
  //   grid_map_cube.color.a = 0.7;
  //   grid_map_cube.color.r = (static_cast<float>(i) /
  //   static_cast<float>(topological_map_->allocated_nodes_vertexes_.size())); grid_map_cube.color.g = 1;
  //   grid_map_cube.color.b = 0;
  //   grid_map_cube.pose.position.x = grid_map->origin_.x_ + grid_map->width_ / 2;
  //   grid_map_cube.pose.position.y = grid_map->origin_.y_ + grid_map->lenght_ / 2;
  //   grid_map_cube.pose.position.z = -0.05;
  //   grid_map_cube.pose.orientation.x = 0;
  //   grid_map_cube.pose.orientation.y = 0;
  //   grid_map_cube.pose.orientation.z = 0;
  //   grid_map_cube.pose.orientation.w = 1;
  //   grid_map_cube.scale.x = grid_map->width_;
  //   grid_map_cube.scale.y = grid_map->lenght_;
  //   grid_map_cube.scale.z = 0.1;

  //   marker_array.markers.push_back(grid_map_cube);
  // }

  topological_map_publisher_->publish(marker_array);
}

}  // namespace vineslam