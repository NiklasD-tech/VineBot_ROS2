#pragma once

#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/params.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/mapping/static/occupancy_map_static.hpp>

#include <iostream>
#include <vector>
#include <map>
#include <set>

namespace vineslam
{
struct CellData
{
  // List of landmarks, features, and points at each cell
  std::map<int, SemanticFeature>* landmarks_{ nullptr };
  std::vector<ImageFeature>* surf_features_{ nullptr };
  std::vector<Corner>* corner_features_{ nullptr };
  std::vector<Planar>* planar_features_{ nullptr };

  // List of candidate landmarks, features, and points at each cell
  std::vector<Corner>* candidate_corner_features_{ nullptr };
  std::vector<Planar>* candidate_planar_features_{ nullptr };

  // Occupation flags
  bool* is_occupied_{ nullptr };  // true if the occupancy grid cell is occupied
};

struct Cell
{
  CellData* data{ nullptr };  // this is a trick: a pointer occupies 64-bit of memory. thus, for the cells not occupied,
                              // we only have 64 bits of memory. if the struct Cell contained all the members present in
                              // Data, we would have 6 * 64 bits of memory per non-occupied cell.
};

class MapLayer
{
public:
  MapLayer() = default;
  ~MapLayer();

  // Class constructor
  // - initializes the grid map given the input parameters
  MapLayer(const Parameters& params, const Pose& origin_offset);

  // Copy contructor
  explicit MapLayer(const MapLayer& grid_map);

  // Assignment operator
  MapLayer& operator=(const MapLayer& grid_map);

  // 2D grid map direct access to cell coordinates
  Cell& operator()(int i, int j)
  {
    // Verify validity of indexing
    try
    {
      check(i, j);
    }
    catch (char const* msg)
    {
#if VERBOSE == 1
      std::cout << msg;
      std::cout << "Returning last grid element ..." << std::endl;
#endif

      return cell_vec_[cell_vec_.size() - 1];
    }

    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));

    return cell_vec_[idx];
  }

  // 2D grid map access given a Feature/Landmark location
  Cell& operator()(float i, float j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(i / resolution_ + .49));
    int l_j = static_cast<int>(std::round(j / resolution_ + .49));

    return (*this)(l_i, l_j);
  }

  // Check out of bounds indexing
  void check(const int& i, const int& j)
  {
    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int index = l_i + (l_j * static_cast<int>(std::round(width_ / resolution_ + .49)));

    // Trough exception if out of bounds indexing
    if (index > (static_cast<int>(cell_vec_.size()) - 1) || index < 0)
      throw "Exception: Access to grid map out of bounds\n";
  }

  // Check if a point if inside the map
  bool isInside(const float& i, const float& j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(i / resolution_ + .49));
    int l_j = static_cast<int>(std::round(j / resolution_ + .49));

    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int ll_i = l_i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int ll_j = l_j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int index = ll_i + (ll_j * static_cast<int>(std::round(width_ / resolution_ + .49)));

    if (index > (static_cast<int>(cell_vec_.size()) - 1) || index < 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<Cell>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin()
  {
    return cell_vec_.begin();
  }
  iterator end()
  {
    return cell_vec_.end();
  }

  // Insert a Landmark using the direct grid coordinates
  bool insert(const SemanticFeature& l_landmark, const int& id, const int& i, const int& j);

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& l_landmark, const int& id);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const ImageFeature& l_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& l_feature);

  // Insert a Corner Feature using the direct grid coordinates
  bool insert(const Corner& l_feature, const int& i, const int& j);
  bool directInsert(const Corner& l_feature, const int& i, const int& j);
  // Insert a Corner Feature given a Feature/Landmark location
  bool insert(const Corner& l_feature);
  bool directInsert(const Corner& l_feature);

  // Insert a Planar Feature using the direct grid coordinates
  bool insert(const Planar& l_feature, const int& i, const int& j);
  bool directInsert(const Planar& l_feature, const int& i, const int& j);
  // Insert a Planar Feature given a Feature/Landmark location
  bool insert(const Planar& l_feature);
  bool directInsert(const Planar& l_feature);

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark, const SemanticFeature& old_landmark, const int& old_landmark_id);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(/*const ImageFeature& old_image_feature, const ImageFeature& new_image_feature*/);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Method to get all the adjacent cells to a given cell
  bool getAdjacent(const int& i, const int& j, const int& layers, std::vector<Cell>& adjacent);
  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float& i, const float& j, const int& layers, std::vector<Cell>& adjacent);

  // Find nearest neighbor of an image feature considering adjacent cells
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);

  // Find nearest neighbor of a corner feature considering adjacent cells
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);

  // Find nearest neighbor of a planar feature considering adjacent cells
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);

  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);

  // Deallocate all the map memory
  void deallocateAllMem();

  // Getter functions
  std::map<int, SemanticFeature> getLandmarks() const
  {
    std::map<int, SemanticFeature> out_landmarks;
    for (const auto& i : landmark_set_)
      for (const auto& landmark : *cell_vec_[i].data->landmarks_)
        CellRoutines::insert(landmark.first, landmark.second, &out_landmarks);

    return out_landmarks;
  }
  std::vector<Corner> getCorners() const
  {
    std::vector<Corner> out_corners;
    for (const auto& i : corner_set_)
      for (const auto& corner : *cell_vec_[i].data->corner_features_)
        out_corners.push_back(corner);

    return out_corners;
  }
  std::vector<Planar> getPlanars() const
  {
    std::vector<Planar> out_planars;
    for (const auto& i : planar_set_)
      for (const auto& planar : *cell_vec_[i].data->planar_features_)
        out_planars.push_back(planar);

    return out_planars;
  }
  std::vector<ImageFeature> getImageFeatures() const
  {
    std::vector<ImageFeature> out_surf_features;
    for (const auto& i : surf_set_)
      for (const auto& img_feature : *cell_vec_[i].data->surf_features_)
        out_surf_features.push_back(img_feature);

    return out_surf_features;
  }

  // Returns true if the map has no features or landmarks
  bool empty() const
  {
    return (n_surf_features_ == 0 && n_landmarks_ == 0 && n_corner_features_ == 0 && n_planar_features_ == 0);
  }

  // Delete all features in the map
  void clear()
  {
    // ************************ WARNING ********************** //
    // ************** This function is very slow ************* //
    // ******************************************************* //
    for (auto& cell : cell_vec_)
    {
      cell.data->corner_features_->shrink_to_fit();
      cell.data->planar_features_->shrink_to_fit();
      cell.data->surf_features_->shrink_to_fit();
      cell.data->landmarks_->clear();
    }

    n_corner_features_ = 0;
    n_planar_features_ = 0;
    n_surf_features_ = 0;
    n_landmarks_ = 0;
  }

  // Number of features, landmarks, and points in the map
  int n_surf_features_{};
  int n_corner_features_{};
  int n_planar_features_{};
  int n_landmarks_{};

  // Minimum number of observations to add a corners or planar feature to the map
  uint32_t min_planar_obsvs_;
  uint32_t min_corner_obsvs_;

  // Grid map dimensions
  Point origin_;
  float resolution_;
  float width_;
  float lenght_;

private:
  // Private grid map to store all the cells
  std::vector<Cell> cell_vec_;

  // Pointer arrays to occupied cells with each feature class
  std::set<int> surf_set_;
  std::set<int> corner_set_;
  std::set<int> planar_set_;
  std::set<int> landmark_set_;
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the multi-layer grid map given the input parameters
  OccupancyMap(const Parameters& params, const Pose& origin_offset, const uint32_t& min_planar_obsvs,
               const uint32_t& min_corner_obsvs);

  ~OccupancyMap();

  // Copy constructor
  OccupancyMap(const OccupancyMap& grid_map);

  // Access map layer
  MapLayer& operator()(float z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    layer_num = (layer_num < zmin_) ? zmin_ : layer_num;
    layer_num = (layer_num > zmax_) ? zmax_ : layer_num;

    return layers_map_[layer_num];
  }

  // 3D grid map access to cell coordinates
  Cell& operator()(float x, float y, float z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    layer_num = (layer_num < zmin_) ? zmin_ : layer_num;
    layer_num = (layer_num > zmax_) ? zmax_ : layer_num;

    return layers_map_[layer_num](x, y);
  }

  // Check if a point is inside the map
  bool isInside(const float& x, const float& y, const float& z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    if (layer_num > zmax_ || layer_num < zmin_)
    {
      return false;
    }
    else
    {
      if (!layers_map_[layer_num].isInside(x, y))
      {
        return false;
      }
      else
      {
        return true;
      }
    }
  }

  // Define iterator to provide access to the layers array
  typedef std::map<int, MapLayer>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin()
  {
    return layers_map_.begin();
  }
  iterator end()
  {
    return layers_map_.end();
  }

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& l_landmark, const int& id);

  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& l_feature);

  // Insert a corner given a Feature/Landmark location
  bool insert(const Corner& l_feature);
  bool directInsert(const Corner& l_feature);

  // Insert a planar feature given a Feature/Landmark location
  bool insert(const Planar& l_feature);
  bool directInsert(const Planar& l_feature);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark, const SemanticFeature& old_landmark, const int& old_landmark_id);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature);

  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float& x, const float& y, const float& z, const int& layers, std::vector<Cell>& adjacent);

  // Find nearest neighbor of a feature considering adjacent cells
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);

  // Find nearest neighbor of a corner feature considering adjacent cells
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);

  // Find nearest neighbor of a planar feature considering adjacent cells
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);

  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);
  // Recover the layer number from the feature z component
  bool getLayerNumber(const float& z, int& layer_num) const;

  // Deallocate all the map memory
  void deallocateAllMem();

  // Getter functions
  std::map<int, SemanticFeature> getLandmarks()
  {
    std::map<int, SemanticFeature> out_landmarks;
    for (const auto& layer : layers_map_)
    {
      std::map<int, SemanticFeature> l_landmarks = layer.second.getLandmarks();
      for (const auto landmark : l_landmarks)
      {
        out_landmarks[landmark.first] = landmark.second;
      }
    }

    return out_landmarks;
  }
  std::vector<Corner> getCorners()
  {
    std::vector<Corner> out_corners;
    for (const auto& layer : layers_map_)
    {
      std::vector<Corner> l_corners = layer.second.getCorners();
      out_corners.insert(out_corners.end(), l_corners.begin(), l_corners.end());
    }

    return out_corners;
  }
  std::vector<Planar> getPlanars()
  {
    std::vector<Planar> out_planars;
    for (const auto& layer : layers_map_)
    {
      std::vector<Planar> l_planars = layer.second.getPlanars();
      out_planars.insert(out_planars.end(), l_planars.begin(), l_planars.end());
    }

    return out_planars;
  }
  std::vector<ImageFeature> getImageFeatures()
  {
    std::vector<ImageFeature> out_surf_features;
    for (const auto& layer : layers_map_)
    {
      std::vector<ImageFeature> l_surf_features = layer.second.getImageFeatures();
      out_surf_features.insert(out_surf_features.end(), l_surf_features.begin(), l_surf_features.end());
    }

    return out_surf_features;
  }

  // Returns true is none layer has features/landmarks
  bool empty() const
  {
    bool is_empty = false;
    for (const auto& layer : layers_map_)
      is_empty |= layer.second.empty();

    return is_empty;
  }

  // Delete all layers in the map
  void clear()
  {
    for (auto& layer : layers_map_)
      layer.second.clear();
  }

  // Grid map dimensions
  Point origin_;
  float width_;
  float lenght_;
  float height_;
  float resolution_;
  float resolution_z_;
  int zmin_;
  int zmax_;

  // Global planes handler
  std::vector<SemiPlane> planes_;

private:
  // Private grid map to store all the individual layers
  // (int, MapLayer): (layer number, layer class)
  std::map<int, MapLayer> layers_map_;
};

}  // namespace vineslam