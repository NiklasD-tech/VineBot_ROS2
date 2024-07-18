#include "../../include/vineslam/mapping/landmark_mapping.hpp"

namespace vineslam
{
LandmarkMapper::LandmarkMapper(Parameters params) : params_(std::move(params))
{
  filter_frequency_ = 5;
  stdev_threshold_ = 0.2;
}

void LandmarkMapper::init(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
                          const std::vector<float>& depths, const std::vector<int>& labels, OccupancyMap& grid_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss;
  pose.getDist(robot_gauss);

  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  Point trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  // Initialize landmark identifier
  id_ = 0;

  // Compute initial covariance matrix
  // - proportional to the pose signal and the distance to the robot
  for (int i = 0; i < n_obsv; i++)
  {
    if (labels[i] != 1)  // not a trunk - 3D case
    {
      // Construct the observations vector
      Eigen::VectorXf z(3, 1);
      z << depths[i], bearings[i], pitches[i];

      // Calculate
      // - the initial estimation of the landmark on map's referential frame
      float yaw = Const::normalizeAngle(bearings[i]);
      float pitch = Const::normalizeAngle(pitches[i]);
      Point X_cam;
      X_cam.x_ = (depths[i] * (std::cos(yaw) * std::cos(pitch)));
      X_cam.y_ = (depths[i] * (std::sin(yaw) * std::cos(pitch)));
      X_cam.z_ = (depths[i] * (std::sin(pitch)));

      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = X_cam.x_ * Rot[6] + X_cam.y_ * Rot[7] + X_cam.z_ * Rot[8] + trans.z_;

      // Push back a Kalman Filter object for the respective landmark
      KF kf;
      kf.init3D(params_, X.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      // and get the correspondent standard deviation
      id_++;
      Point pos(X.x_, X.y_, X.z_);
      Gaussian<Point, Point> std = filters[filters.size() - 1].getStdev();
      grid_map.insert(SemanticFeature(pos, std, labels[i]), id_);
    }
    else  // trunk - 2D case
    {
      // Construct the observations vector
      Eigen::VectorXf z(2, 1);
      z << depths[i], bearings[i];

      // Calculate
      // - the initial estimation of the landmark on map's referential frame
      float th = Const::normalizeAngle(bearings[i]);
      Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = 0.;

      // Push back a Kalman Filter object for the respective landmark
      KF kf;
      kf.init2D(params_, X.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      // and get the correspondent standard deviation
      id_++;
      Point pos(X.x_, X.y_, 0.);
      Gaussian<Point, Point> std = filters[filters.size() - 1].getStdev();
      grid_map.insert(SemanticFeature(pos, std, labels[i]), id_);
    }
  }
}

void LandmarkMapper::init(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
                          const std::vector<float>& depths, const std::vector<int>& labels,
                          TopologicalMap& topological_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss;
  pose.getDist(robot_gauss);

  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  Point trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  // Initialize landmark identifier
  id_ = 0;

  // Compute initial covariance matrix
  // - proportional to the pose signal and the distance to the robot
  for (int i = 0; i < n_obsv; i++)
  {
    if (labels[i] != 1)  // not a trunk - 3D case
    {
      // Construct the observations vector
      Eigen::VectorXf z(3, 1);
      z << depths[i], bearings[i], pitches[i];

      // Calculate
      // - the initial estimation of the landmark on map's referential frame
      float yaw = Const::normalizeAngle(bearings[i]);
      float pitch = Const::normalizeAngle(pitches[i]);
      Point X_cam;
      X_cam.x_ = (depths[i] * (std::cos(yaw) * std::cos(pitch)));
      X_cam.y_ = (depths[i] * (std::sin(yaw) * std::cos(pitch)));
      X_cam.z_ = (depths[i] * (std::sin(pitch)));

      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = X_cam.x_ * Rot[6] + X_cam.y_ * Rot[7] + X_cam.z_ * Rot[8] + trans.z_;

      // Push back a Kalman Filter object for the respective landmark
      KF kf;
      kf.init3D(params_, X.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      // and get the correspondent standard deviation
      id_++;
      Point pos(X.x_, X.y_, X.z_);
      Gaussian<Point, Point> std = filters[filters.size() - 1].getStdev();
      topological_map.insert(SemanticFeature(pos, std, labels[i]), id_);
    }
    else  // trunk - 2D case
    {
      // Construct the observations vector
      Eigen::VectorXf z(2, 1);
      z << depths[i], bearings[i];

      // Calculate
      // - the initial estimation of the landmark on map's referential frame
      float th = Const::normalizeAngle(bearings[i]);
      Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = 0.;

      // Push back a Kalman Filter object for the respective landmark
      KF kf;
      kf.init2D(params_, X.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      // and get the correspondent standard deviation
      id_++;
      Point pos(X.x_, X.y_, 0.);
      Gaussian<Point, Point> std = filters[filters.size() - 1].getStdev();
      topological_map.insert(SemanticFeature(pos, std, labels[i]), id_);
    }
  }
}

void LandmarkMapper::process(const Pose& pose, const std::vector<SemanticFeature>& landmarks,
                             const std::vector<int>& labels, OccupancyMap& grid_map)
{
  std::cout << "LandmakrMapper::process() !!! \n\n";
  // Convert local map to bearings and depths
  std::vector<float> bearings(landmarks.size());
  std::vector<float> pitches(landmarks.size());
  std::vector<float> depths(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); i++)
  {
    Point pt = landmarks[i].pos_;
    depths[i] = std::sqrt(std::pow(pt.x_, 2) + std::pow(pt.y_, 2) + std::pow(pt.z_, 2));
    bearings[i] = std::atan2(pt.y_, pt.x_) - pose.Y_;
    pitches[i] = std::atan2(pt.z_, std::sqrt(std::pow(pt.x_, 2) + std::pow(pt.y_, 2))) - pose.P_;
  }
  // Estimate global map
  predict(pose, bearings, pitches, depths, labels, grid_map);
}

void LandmarkMapper::process(const Pose& pose, const std::vector<SemanticFeature>& landmarks,
                             const std::vector<int>& labels, TopologicalMap& topological_map)
{
  // Convert local map to bearings and depths
  std::vector<float> bearings(landmarks.size());
  std::vector<float> pitches(landmarks.size());
  std::vector<float> depths(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); i++)
  {
    Point pt = landmarks[i].pos_;
    depths[i] = std::sqrt(std::pow(pt.x_, 2) + std::pow(pt.y_, 2) + std::pow(pt.z_, 2));
    bearings[i] = std::atan2(pt.y_, pt.x_) - pose.Y_;
    pitches[i] = std::atan2(pt.z_, std::sqrt(std::pow(pt.x_, 2) + std::pow(pt.y_, 2))) - pose.P_;
  }
  // Estimate global map
  predict(pose, bearings, pitches, depths, labels, topological_map);
}

void LandmarkMapper::predict(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
                             const std::vector<float>& depths, const std::vector<int>& labels, OccupancyMap& grid_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss;
  pose.getDist(robot_gauss);

  // Convert 6DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  pose.toRotMatrix(Rot);
  Point trans = pose.getXYZ();

  for (int i = 0; i < n_obsv; i++)
  {
    if (labels[i] != 1)  // not a trunk - 3D case
    {
      // Calculate the landmark position on map's referential frame
      // based on the ith observation
      float yaw = Const::normalizeAngle(bearings[i]);
      float pitch = Const::normalizeAngle(pitches[i]);
      Point X_cam;
      X_cam.x_ = (depths[i] * (std::cos(yaw) * std::cos(pitch)));
      X_cam.y_ = (depths[i] * (std::sin(yaw) * std::cos(pitch)));
      X_cam.z_ = (depths[i] * (std::sin(pitch)));

      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = X_cam.x_ * Rot[6] + X_cam.y_ * Rot[7] + X_cam.z_ * Rot[8] + trans.z_;

      // Construct the observations vector
      VectorXf z(3, 1);
      z << depths[i], bearings[i], pitches[i];

      // Check if the landmark already exists in the map
      std::pair<int, SemanticFeature> correspondence = findCorr(X, labels[i], grid_map);
      // If not, initialize the landmark on the map, as well as the
      // correspondent Kalman Filter
      if (correspondence.first < 0)
      {
        // Initialize the Kalman Filter
        KF kf;
        kf.init3D(params_, X.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
        filters.push_back(kf);

        // Insert the landmark on the map, with a single observation
        id_++;
        Gaussian<Point, Point> gauss = filters[filters.size() - 1].getStdev();
        grid_map.insert(SemanticFeature(X, gauss, labels[i]), id_);
      }
      // If so, update the landmark position estimation using a Kalman
      // Filter call
      else
      {
        // Invocate the Kalman Filter
        filters[correspondence.first - 1].process3D(pose.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
        // Get the state vector and the standard deviation associated
        // with the estimation
        Point X_out = filters[correspondence.first - 1].getState3D();
        Gaussian<Point, Point> gauss = filters[correspondence.first - 1].getStdev();

        // Update the estimation on the map
        grid_map.update(SemanticFeature(X_out, gauss, labels[i]), correspondence.second, correspondence.first);
      }
    }
    else  // trunk - 2D case
    {
      // Calculate the landmark position on map's referential frame
      // based on the ith observation
      float th = Const::normalizeAngle(bearings[i]);
      Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = 0.;

      // Construct the observations vector
      VectorXf z(2, 1);
      z << depths[i], bearings[i];

      // Check if the landmark already exists in the map
      std::pair<int, SemanticFeature> correspondence = findCorr(X, labels[i], grid_map);
      // If not, initialize the landmark on the map, as well as the
      // correspondent Kalman Filter
      if (correspondence.first < 0)
      {
        Eigen::MatrixXd R(2, 2);

        // Initialize the Kalman Filter
        KF kf;
        kf.init2D(params_, X.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
        filters.push_back(kf);

        // Insert the landmark on the map, with a single observation
        id_++;
        Gaussian<Point, Point> gauss = filters[filters.size() - 1].getStdev();
        grid_map.insert(SemanticFeature(X, gauss, labels[i]), id_);
      }
      // If so, update the landmark position estimation using a Kalman
      // Filter call
      else
      {
        // Invocate the Kalman Filter
        filters[correspondence.first - 1].process2D(pose.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
        // Get the state vector and the standard deviation associated
        // with the estimation
        Point X_out = filters[correspondence.first - 1].getState2D();
        Gaussian<Point, Point> gauss = filters[correspondence.first - 1].getStdev();

        // Update the estimation on the map
        grid_map.update(SemanticFeature(X_out, gauss, labels[i]), correspondence.second, correspondence.first);
      }
    }
  }
}

void LandmarkMapper::predict(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
                             const std::vector<float>& depths, const std::vector<int>& labels,
                             TopologicalMap& topological_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss;
  pose.getDist(robot_gauss);

  // Convert 6DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  pose.toRotMatrix(Rot);
  Point trans = pose.getXYZ();

  for (int i = 0; i < n_obsv; i++)
  {
    if (labels[i] != 1)  // not a trunk - 3D case
    {
      // Calculate the landmark position on map's referential frame
      // based on the ith observation
      float yaw = Const::normalizeAngle(bearings[i]);
      float pitch = Const::normalizeAngle(pitches[i]);
      Point X_cam;
      X_cam.x_ = (depths[i] * (std::cos(yaw) * std::cos(pitch)));
      X_cam.y_ = (depths[i] * (std::sin(yaw) * std::cos(pitch)));
      X_cam.z_ = (depths[i] * (std::sin(pitch)));

      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = X_cam.x_ * Rot[6] + X_cam.y_ * Rot[7] + X_cam.z_ * Rot[8] + trans.z_;

      // Construct the observations vector
      VectorXf z(3, 1);
      z << depths[i], bearings[i], pitches[i];

      // Check if the landmark already exists in the map
      std::pair<int, SemanticFeature> correspondence = findCorr(X, labels[i], topological_map);
      // If not, initialize the landmark on the map, as well as the
      // correspondent Kalman Filter
      if (correspondence.first < 0)
      {
        // Initialize the Kalman Filter
        KF kf;
        kf.init3D(params_, X.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
        filters.push_back(kf);

        // Insert the landmark on the map, with a single observation
        id_++;
        Gaussian<Point, Point> gauss = filters[filters.size() - 1].getStdev();
        topological_map.insert(SemanticFeature(X, gauss, labels[i]), id_);
      }
      // If so, update the landmark position estimation using a Kalman
      // Filter call
      else
      {
        // Invocate the Kalman Filter
        filters[correspondence.first - 1].process3D(pose.toEig3D(), robot_gauss.stdev_.toEig3D(), z);
        // Get the state vector and the standard deviation associated
        // with the estimation
        Point X_out = filters[correspondence.first - 1].getState3D();
        Gaussian<Point, Point> gauss = filters[correspondence.first - 1].getStdev();

        // Update the estimation on the map
        topological_map.update(SemanticFeature(X_out, gauss, labels[i]), correspondence.second, correspondence.first);
      }
    }
    else  // trunk - 2D case
    {
      // Calculate the landmark position on map's referential frame
      // based on the ith observation
      float th = Const::normalizeAngle(bearings[i]);
      Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
      Point X;
      X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
      X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
      X.z_ = 0.;

      // Construct the observations vector
      VectorXf z(2, 1);
      z << depths[i], bearings[i];

      // Check if the landmark already exists in the map
      std::pair<int, SemanticFeature> correspondence = findCorr(X, labels[i], topological_map);
      // If not, initialize the landmark on the map, as well as the
      // correspondent Kalman Filter
      if (correspondence.first < 0)
      {
        Eigen::MatrixXd R(2, 2);

        // Initialize the Kalman Filter
        KF kf;
        kf.init2D(params_, X.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
        filters.push_back(kf);

        // Insert the landmark on the map, with a single observation
        id_++;
        Gaussian<Point, Point> gauss = filters[filters.size() - 1].getStdev();
        topological_map.insert(SemanticFeature(X, gauss, labels[i]), id_);
      }
      // If so, update the landmark position estimation using a Kalman
      // Filter call
      else
      {
        // Invocate the Kalman Filter
        filters[correspondence.first - 1].process2D(pose.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
        // Get the state vector and the standard deviation associated
        // with the estimation
        Point X_out = filters[correspondence.first - 1].getState2D();
        Gaussian<Point, Point> gauss = filters[correspondence.first - 1].getStdev();

        // Update the estimation on the map
        topological_map.update(SemanticFeature(X_out, gauss, labels[i]), correspondence.second, correspondence.first);
      }
    }
  }
}

std::pair<int, SemanticFeature> LandmarkMapper::findCorr(const Point& pos, const int& label, OccupancyMap& grid_map)
{
  float best_aprox = (label == 1) ? 0.5 : 0.25;  // for trunks we support a larger correspondence
                                                 // tolerance (0.5 meters in this case)
  std::pair<int, SemanticFeature> correspondence;
  correspondence.first = -1;

  std::map<int, SemanticFeature>* l_landmarks = nullptr;

  if (grid_map(pos.x_, pos.y_, pos.z_).data != nullptr)
  {
    l_landmarks = grid_map(pos.x_, pos.y_, pos.z_).data->landmarks_;
  }

  if (l_landmarks != nullptr)
  {
    // Search on current cell first
    for (const auto& l_landmark : *l_landmarks)
    {
      float dist = pos.distance(l_landmark.second.pos_);

      if (dist < best_aprox && l_landmark.second.label_ == label)
      {
        correspondence = l_landmark;
        best_aprox = dist;
      }
    }
  }

  // Search on adjacent cells then
  int number_layers = (correspondence.first == -1) ? 2 : 1;
  std::vector<Cell> adjacents;
  grid_map.getAdjacent(pos.x_, pos.y_, pos.z_, number_layers, adjacents);
  for (const auto& l_cell : adjacents)
  {
    std::map<int, SemanticFeature>* ll_landmarks = nullptr;

    if (l_cell.data != nullptr)
    {
      ll_landmarks = l_cell.data->landmarks_;
    }

    if (ll_landmarks != nullptr)
    {
      for (const auto& l_landmark : *ll_landmarks)
      {
        float dist = pos.distance(l_landmark.second.pos_);
        if (dist < best_aprox && l_landmark.second.label_ == label)
        {
          correspondence = l_landmark;
          best_aprox = dist;
        }
      }
    }
  }

  return correspondence;
}

std::pair<int, SemanticFeature> LandmarkMapper::findCorr(const Point& pos, const int& label,
                                                         TopologicalMap& topological_map)
{
  float best_aprox = (label == 1) ? 0.5 : 0.25;  // for trunks we support a larger correspondence
                                                 // tolerance (0.5 meters in this case)
  std::pair<int, SemanticFeature> correspondence;
  correspondence.first = -1;

  std::map<int, SemanticFeature>* l_landmarks = nullptr;

  Cell cell;
  vertex_t node;
  topological_map.getCell(pos, cell, node);
  if (cell.data != nullptr)
  {
    l_landmarks = cell.data->landmarks_;
  }

  if (l_landmarks != nullptr)
  {
    // Search on current cell first
    for (const auto& l_landmark : *l_landmarks)
    {
      float dist = pos.distance(l_landmark.second.pos_);

      if (dist < best_aprox && l_landmark.second.label_ == label)
      {
        correspondence = l_landmark;
        best_aprox = dist;
      }
    }
  }

  // Search on adjacent cells then
  int number_layers = (correspondence.first == -1) ? 2 : 1;
  std::vector<Cell> adjacents;
  topological_map.getAdjacent(pos.x_, pos.y_, pos.z_, number_layers, node, adjacents);
  for (const auto& l_cell : adjacents)
  {
    std::map<int, SemanticFeature>* ll_landmarks = nullptr;

    if (l_cell.data != nullptr)
    {
      ll_landmarks = l_cell.data->landmarks_;
    }

    if (ll_landmarks != nullptr)
    {
      for (const auto& l_landmark : *ll_landmarks)
      {
        float dist = pos.distance(l_landmark.second.pos_);
        if (dist < best_aprox && l_landmark.second.label_ == label)
        {
          correspondence = l_landmark;
          best_aprox = dist;
        }
      }
    }
  }

  return correspondence;
}

void LandmarkMapper::localMap(const Pose& cam_origin_pose, const std::vector<int>& labels,
                              const std::vector<float>& bearings, const std::vector<float>& pitches,
                              std::vector<SemanticFeature>& landmarks, OccupancyMap& grid_map, Pose robot_pose) const
{
  // Transform the landmark orientations into the map referential frame
  uint32_t n_obsv = labels.size();
  for (uint32_t i = 0; i < n_obsv; i++)
  {
    int label = labels[i];
    float bearing = bearings[i];
    float pitch = pitches[i];

    // Transform the landmark orientation to the map reference frame
    Tf l_measurement_base = Pose(0., 0., 0., 0., pitch, bearing).toTf();
    Tf l_measurement_map = robot_pose.toTf() * l_measurement_base;
    Pose l_measurement(l_measurement_map.R_array_, l_measurement_map.t_array_);

    // Find the position of the landmark by intersection with the satellite occupancy grid map (saved in 'grid_map')
    float multiplier = 0.2;
    float increment = 0.1;
    bool found_occupied_cell = false;
    while (!found_occupied_cell && multiplier < 1.75)
    {
      // Point projection along a line
      // ... x = cos(yaw)*cos(pitch)
      // ... y = sin(yaw)*cos(pitch)
      // ... z = sin(pitch)
      Point l_pt;
      l_pt.x_ = cam_origin_pose.x_ + (multiplier * (std::cos(l_measurement.Y_) * std::cos(l_measurement.P_)));
      l_pt.y_ = cam_origin_pose.y_ + (multiplier * (std::sin(l_measurement.Y_) * std::cos(l_measurement.P_)));
      l_pt.z_ = cam_origin_pose.z_ + (multiplier * (std::sin(l_measurement.P_)));

      // Save landmark info if we found an intersection with the satellite occupancy grid map
      if (grid_map.isInside(l_pt.x_, l_pt.y_, 0.) && grid_map.isInside(l_pt.x_, l_pt.y_, l_pt.z_))
      {
        if (grid_map(l_pt.x_, l_pt.y_, 0).data != nullptr && grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
        {
          if (grid_map(l_pt.x_, l_pt.y_, 0).data->is_occupied_ != nullptr)
          {
            if (*(grid_map(l_pt.x_, l_pt.y_, 0).data->is_occupied_) == true)
            {
              if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_ != nullptr)
              {
                if (!grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->empty())
                {
                  // We found an occupied cell, so our landmark will be inside this voxel
                  // To compute an approximation of its position we compute the average of all the points
                  // inside of it
                  found_occupied_cell = true;
                  Point mean_pt(0., 0., 0.);
                  for (const auto& pt : *grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_)
                  {
                    mean_pt.x_ += pt.pos_.x_;
                    mean_pt.y_ += pt.pos_.y_;
                    mean_pt.z_ += pt.pos_.z_;
                  }
                  mean_pt.x_ /= grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();
                  mean_pt.y_ /= grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();
                  mean_pt.z_ /= grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();

                  SemanticFeature l_landmark;
                  l_landmark.pos_ =
                      mean_pt - robot_pose.getXYZ();  // we want the landmark in the local robot referential
                  l_landmark.info_ = SemanticInfo(label);
                  l_landmark.label_ = label;

                  landmarks.push_back(l_landmark);
                }
              }
            }
          }
        }
      }

      multiplier += increment;
    }
  }
}

void LandmarkMapper::localMap(const Pose& cam_origin_pose, const std::vector<int>& labels,
                              const std::vector<float>& bearings, const std::vector<float>& pitches,
                              std::vector<SemanticFeature>& landmarks, TopologicalMap& topological_map,
                              Pose robot_pose) const
{
  // Transform the landmark orientations into the map referential frame
  uint32_t n_obsv = labels.size();
  for (uint32_t i = 0; i < n_obsv; i++)
  {
    int label = labels[i];
    float bearing = bearings[i];
    float pitch = pitches[i];

    // Transform the landmark orientation to the map reference frame
    Tf l_measurement_base = Pose(0., 0., 0., 0., pitch, bearing).toTf();
    Tf l_measurement_map = robot_pose.toTf() * l_measurement_base;
    Pose l_measurement(l_measurement_map.R_array_, l_measurement_map.t_array_);

    // Find the position of the landmark by intersection with the satellite occupancy grid map (saved in 'grid_map')
    float multiplier = 0.2;
    float increment = 0.1;
    bool found_occupied_cell = false;
    while (!found_occupied_cell && multiplier < 1.75)
    {
      // Point projection along a line
      // ... x = cos(yaw)*cos(pitch)
      // ... y = sin(yaw)*cos(pitch)
      // ... z = sin(pitch)
      Point l_pt;
      l_pt.x_ = cam_origin_pose.x_ + (multiplier * (std::cos(l_measurement.Y_) * std::cos(l_measurement.P_)));
      l_pt.y_ = cam_origin_pose.y_ + (multiplier * (std::sin(l_measurement.Y_) * std::cos(l_measurement.P_)));
      l_pt.z_ = cam_origin_pose.z_ + (multiplier * (std::sin(l_measurement.P_)));

      // Save landmark info if we found an intersection with the satellite occupancy grid map
      vertex_t node;
      topological_map.getNode(Point(l_pt.x_, l_pt.y_, 0), node);
      OccupancyMap* grid_map = topological_map.map_[node].grid_map_;
      if (grid_map != nullptr)
      {
        if (grid_map->isInside(l_pt.x_, l_pt.y_, 0.) && grid_map->isInside(l_pt.x_, l_pt.y_, l_pt.z_))
        {
          if ((*grid_map)(l_pt.x_, l_pt.y_, 0).data != nullptr && (*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
          {
            if ((*grid_map)(l_pt.x_, l_pt.y_, 0).data->is_occupied_ != nullptr)
            {
              if (*((*grid_map)(l_pt.x_, l_pt.y_, 0).data->is_occupied_) == true)
              {
                if ((*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_ != nullptr)
                {
                  if (!(*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->empty())
                  {
                    // We found an occupied cell, so our landmark will be inside this voxel
                    // To compute an approximation of its position we compute the average of all the points
                    // inside of it
                    found_occupied_cell = true;
                    Point mean_pt(0., 0., 0.);
                    for (const auto& pt : *(*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_)
                    {
                      mean_pt.x_ += pt.pos_.x_;
                      mean_pt.y_ += pt.pos_.y_;
                      mean_pt.z_ += pt.pos_.z_;
                    }
                    mean_pt.x_ /= (*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();
                    mean_pt.y_ /= (*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();
                    mean_pt.z_ /= (*grid_map)(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_->size();

                    SemanticFeature l_landmark;
                    l_landmark.pos_ =
                        mean_pt - robot_pose.getXYZ();  // we want the landmark in the local robot referential
                    l_landmark.info_ = SemanticInfo(label);
                    l_landmark.label_ = label;

                    landmarks.push_back(l_landmark);
                  }
                }
              }
            }
          }
        }
      }

      multiplier += increment;
    }
  }
}

void LandmarkMapper::localMap(const Pose& cam_origin_pose, const std::vector<int>& labels,
                              const std::vector<float>& bearings, const std::vector<float>& pitches,
                              std::vector<SemanticFeature>& landmarks, const std::vector<SemiPlane>& planes,
                              Pose robot_pose) const
{
  // Transform the landmark orientations into the map referential frame
  uint32_t n_obsv = labels.size();
  for (uint32_t i = 0; i < n_obsv; i++)
  {
    int label = labels[i];
    float bearing = bearings[i];
    float pitch = pitches[i];

    // Transform the landmark orientation to the map reference frame
    Tf l_measurement_base = Pose(0., 0., 0., 0., pitch, bearing).toTf();
    Tf l_measurement_map = robot_pose.toTf() * l_measurement_base;
    Pose l_measurement(l_measurement_map.R_array_, l_measurement_map.t_array_);

    // Compute the intersection between the landmark ray and the semiplanes
    for (const auto& plane : planes)
    {
      // Make sure that the plane is not horizontal
      float dot = Vec(plane.a_, plane.b_, plane.c_).dot(Vec(0., 0., 1.));
      if (std::fabs(dot) > 1.0)
      {
        continue;
      }

      // Compute the ray vector of the landmark
      Vec ray_vector(std::cos(l_measurement.Y_) * std::cos(l_measurement.P_),
                     std::sin(l_measurement.Y_) * std::cos(l_measurement.P_), std::sin(l_measurement.P_));

      // Calculate the intersection between the ray and the plane
      Point diff = cam_origin_pose.getXYZ() - plane.centroid_;
      double prod1 = Vec(diff.x_, diff.y_, diff.z_).dot(Vec(plane.a_, plane.b_, plane.c_));
      double prod2 = ray_vector.dot(Vec(plane.a_, plane.b_, plane.c_));
      double prod3 = prod1 / prod2;
      Vec ray_vector_multiplied(ray_vector.x_ * prod3, ray_vector.y_ * prod3, ray_vector.z_ * prod3);
      Point intersection = (cam_origin_pose.getXYZ() + ray_vector_multiplied);

      // Validate the intersection and insert the landmark
      if (intersection.norm3D() < 1.5)
      {
        SemanticFeature l_landmark;
        l_landmark.pos_ = intersection - robot_pose.getXYZ();  // we want the landmark in the local robot referential
        l_landmark.info_ = SemanticInfo(label);
        l_landmark.label_ = label;

        landmarks.push_back(l_landmark);
      }
    }
  }
}

}  // namespace vineslam
