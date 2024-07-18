#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include <vineslam/interface/localization_mapping_interface.hpp>
#include <vineslam/localization/pf.hpp>
#include <vineslam/feature/three_dimensional.hpp>

#include "../gpu_timer.h"

namespace vineslam
{
__device__ void alignPoint(const float& in_x, const float& in_y, const float& cx, const float& cy, const float& angle, float& out_x, float& out_y)
{
  float x1 = in_x - cx;
  float y1 = in_y - cy;

  out_x = x1 * cos(angle) - y1 * sin(angle);
  out_y = x1 * sin(angle) + y1 * cos(angle);

  out_x += cx;
  out_y += cy;
}

__device__ int to1D(const float& x, const float& y, const float& z, const float& origin_x, const float& origin_y, const float& origin_z,
                    const float& resolution, const float& resolution_z, const float& width, const float& length)
{
  int ii = static_cast<int>(std::round(x / resolution + 0.49));
  int jj = static_cast<int>(std::round(y / resolution + 0.49));
  int xx = ii - static_cast<int>(std::round(origin_x / resolution + 0.49));
  int yy = jj - static_cast<int>(std::round(origin_y / resolution + 0.49));
  int zz = static_cast<int>(std::round((z - origin_z) / resolution_z));
  return (xx + (static_cast<int>(std::round(width / resolution + 0.49)) * (yy + (zz * static_cast<int>(std::round(length / resolution + 0.49))))));
}

__device__ uint8_t getBit(const int& pos, const uint8_t& byte)
{
  return ((byte >> pos) & 1U);
}

// Kernel function for corner-based weight calculation
__global__ void cornerWeightCalculation(uint8_t** map, uint32_t* sizes, float* infos, float* ts, float* features, LookUpTable look_up_table, int particles_size,
                                        int corners_size, float* weights)
{
  int particle_idx = threadIdx.x + blockIdx.x * blockDim.x;
  int feature_idx = threadIdx.y + blockIdx.y * blockDim.y;

  if (particle_idx < particles_size)
  {
    if (feature_idx < corners_size)
    {
      // Transform feature to particles' reference frame
      float ft_x = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 0] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 1] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 2] + ts[particle_idx * 12 + 9];
      float ft_y = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 3] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 4] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 5] + ts[particle_idx * 12 + 10];
      float ft_z = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 6] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 7] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 8] + ts[particle_idx * 12 + 11];

      // Compute the topological node where the feature lies
      int i_look_up_idx = (ft_x - look_up_table.look_up_table_min_x_) / look_up_table.look_up_table_res_;
      int j_look_up_idx = (ft_y - look_up_table.look_up_table_min_y_) / look_up_table.look_up_table_res_;
      int index = i_look_up_idx + j_look_up_idx * look_up_table.look_up_table_cols_;

      if (index < look_up_table.look_up_table_size_ && index > 0)
      {
        int node_number = look_up_table.look_up_table_[index];

        if (sizes[node_number] > 0)
        {
          // Align point to match the local grid map reference frame
          float aligned_x, aligned_y;
          alignPoint(ft_x, ft_y, infos[node_number * 10 + 0], infos[node_number * 10 + 1], infos[node_number * 10 + 2], aligned_x, aligned_y);

          // Get the index to access the sub map stored in the topological node
          int local_map_idx = to1D(aligned_x, aligned_y, ft_z, infos[node_number * 10 + 3], infos[node_number * 10 + 4], infos[node_number * 10 + 5],
                                   infos[node_number * 10 + 6], infos[node_number * 10 + 7], infos[node_number * 10 + 8], infos[node_number * 10 + 9]);

          // Get the byte index where the feature boolean is stored
          int byte_number = static_cast<int>(local_map_idx / 8);

          // Get the occupancy result from the obtained index
          if (byte_number < sizes[node_number] && byte_number >= 0)
          {
            int bit_number = local_map_idx % 8;
            float val = (float)getBit(bit_number, map[node_number][byte_number]);
            atomicAdd(&weights[particle_idx], val);
          }
        }
      }
    }
  }
}

// Kernel function for planar-based weight calculation
__global__ void planarWeightCalculation(uint8_t** map, uint32_t* sizes, float* infos, float* ts, float* features, LookUpTable look_up_table, int particles_size,
                                        int planars_size, float* weights)
{
  int particle_idx = threadIdx.x + blockIdx.x * blockDim.x;
  int feature_idx = threadIdx.y + blockIdx.y * blockDim.y;

  if (particle_idx < particles_size)
  {
    if (feature_idx < planars_size)
    {
      // Transform feature to particles' reference frame
      float ft_x = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 0] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 1] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 2] + ts[particle_idx * 12 + 9];
      float ft_y = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 3] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 4] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 5] + ts[particle_idx * 12 + 10];
      float ft_z = features[feature_idx * 3 + 0] * ts[particle_idx * 12 + 6] + features[feature_idx * 3 + 1] * ts[particle_idx * 12 + 7] +
                   features[feature_idx * 3 + 2] * ts[particle_idx * 12 + 8] + ts[particle_idx * 12 + 11];

      // Compute the topological node where the feature lies
      int i_look_up_idx = (ft_x - look_up_table.look_up_table_min_x_) / look_up_table.look_up_table_res_;
      int j_look_up_idx = (ft_y - look_up_table.look_up_table_min_y_) / look_up_table.look_up_table_res_;
      int index = i_look_up_idx + j_look_up_idx * look_up_table.look_up_table_cols_;

      if (index < look_up_table.look_up_table_size_ && index > 0)
      {
        int node_number = look_up_table.look_up_table_[index];

        if (sizes[node_number] > 0)
        {
          // Align point to match the local grid map reference frame
          float aligned_x, aligned_y;
          alignPoint(ft_x, ft_y, infos[node_number * 10 + 0], infos[node_number * 10 + 1], infos[node_number * 10 + 2], aligned_x, aligned_y);

          // Get the index to access the sub map stored in the topological node
          int local_map_idx = to1D(aligned_x, aligned_y, ft_z, infos[node_number * 10 + 3], infos[node_number * 10 + 4], infos[node_number * 10 + 5],
                                   infos[node_number * 10 + 6], infos[node_number * 10 + 7], infos[node_number * 10 + 8], infos[node_number * 10 + 9]);

          // Get the byte index where the feature boolean is stored
          int byte_number = static_cast<int>(sizes[node_number] / 2) + static_cast<int>(local_map_idx / 8);

          // Get the occupancy result from the obtained index
          if (byte_number >= static_cast<int>(sizes[node_number] / 2) && byte_number < sizes[node_number])
          {
            int bit_number = local_map_idx % 8;
            float val = (float)getBit(bit_number, map[node_number][byte_number]);
            atomicAdd(&weights[particle_idx], val);
          }
        }
      }
    }
  }
}

__global__ void weightRefinement(float* corner_weights, float* planar_weights, int particles_size, float sigma_corner_matching, float sigma_planar_matching)
{
  int particle_idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (particle_idx < particles_size)
  {
    float normalizer_corner = 1.0 / (sigma_corner_matching * sqrt(M_2PI));
    float w_corners = corner_weights[particle_idx];
    corner_weights[particle_idx] = (normalizer_corner * (exp((1.0 / sigma_corner_matching) * w_corners)));

    float normalizer_planar = 1.0 / (sigma_planar_matching * sqrt(M_2PI));
    float w_planars = planar_weights[particle_idx];
    planar_weights[particle_idx] = (normalizer_planar * (exp((1.0 / sigma_planar_matching) * w_planars)));
  }
}

void pfUpdate(const std::vector<Corner>& corners, const std::vector<Planar>& planars, std::vector<Particle>& particles,
              LocalizationMappingInterface* localization_mapping_interface)
{
  // Declare timer
  GpuTimer timer;

  // Error code to check return values for CUDA calls
  cudaError_t err = cudaSuccess;

  // Kernel settings
  int block_dim_per_component = 16;
  int particles_size = particles.size();
  int corners_size = corners.size();
  int planars_size = planars.size();
  float simple_weight_factor = 20.0;
  float sigma_corner_matching = static_cast<float>(corners_size) / simple_weight_factor;
  float sigma_planar_matching = static_cast<float>(planars_size) / simple_weight_factor;

  // Allocate memory on the device
  // uint8_t** d_array: localization and mapping interface structure that store the topological map into a matrix
  // uint32_t* d_sizes: array that stores the size of each topological node (allows knowing if they are allocated)
  // float*    d_infos: information required for the localization process
  //                    organized as a 1D array with 10 data fields for each node
  //                    [center_x, center_y, angle, origin_x, origin_y, origin_z, res, res_z, width, length]
  // float* d_transformations: array that stores a homogeneous transformation for each particle (12 fields per particle)
  // float* d_corners:         array that stores the corners features [x1, y1, z1, x2, y2, z2, ..., xn, yn, zn]
  // float* d_planars:         array that stores the planars features [x1, y1, z1, x2, y2, z2, ..., xn, yn, zn]
  // LookUpTable* d_look_up_table: structure that holds the necessary data to index the topological nodes from 3D
  //                               coordinates
  // float*       d_weights: output particle weights (1-to-1)

  // ****************************
  // *** Input Map
  // ****************************
  timer.Start();
  uint8_t** tmp = (uint8_t**)malloc(localization_mapping_interface->number_of_nodes_ * sizeof(uint8_t*));
  uint8_t** d_array = NULL;
  for (int i = 0; i < localization_mapping_interface->number_of_nodes_; i++)
  {
    if (localization_mapping_interface->sizes_[i] > 0)
    {
      cudaMalloc((void**)&tmp[i], localization_mapping_interface->sizes_[i] * sizeof(uint8_t));
    }
  }
  cudaMalloc((void**)&d_array, localization_mapping_interface->number_of_nodes_ * sizeof(uint8_t*));
  for (int i = 0; i < localization_mapping_interface->number_of_nodes_; i++)
  {
    if (localization_mapping_interface->sizes_[i] > 0)
    {
      cudaMemcpy(tmp[i], localization_mapping_interface->array_[i], localization_mapping_interface->sizes_[i] * sizeof(uint8_t), cudaMemcpyHostToDevice);
    }
  }
  cudaMemcpy(d_array, tmp, localization_mapping_interface->number_of_nodes_ * sizeof(uint8_t*), cudaMemcpyHostToDevice);

  // ****************************
  // *** Input Sizes
  // ****************************
  uint32_t* d_sizes = NULL;
  err = cudaMalloc((void**)&d_sizes, localization_mapping_interface->number_of_nodes_ * sizeof(uint32_t));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_sizes (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err =
      cudaMemcpy(d_sizes, localization_mapping_interface->sizes_, localization_mapping_interface->number_of_nodes_ * sizeof(uint32_t), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector sizes from host to device (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // ****************************
  // *** Input Infos
  // ****************************
  float* d_info = NULL;
  err = cudaMalloc((void**)&d_info, localization_mapping_interface->number_of_nodes_ * 10 * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_info (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err =
      cudaMemcpy(d_info, localization_mapping_interface->info_, localization_mapping_interface->number_of_nodes_ * 10 * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector info from host to device (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // ****************************
  // *** Input Transformations
  // ****************************
  float* h_transformations = (float*)malloc(particles_size * 12 * sizeof(float));
  for (int i = 0; i < particles_size; i++)
  {
    h_transformations[i * 12 + 0] = particles[i].tf_.R_array_[0];
    h_transformations[i * 12 + 1] = particles[i].tf_.R_array_[1];
    h_transformations[i * 12 + 2] = particles[i].tf_.R_array_[2];
    h_transformations[i * 12 + 3] = particles[i].tf_.R_array_[3];
    h_transformations[i * 12 + 4] = particles[i].tf_.R_array_[4];
    h_transformations[i * 12 + 5] = particles[i].tf_.R_array_[5];
    h_transformations[i * 12 + 6] = particles[i].tf_.R_array_[6];
    h_transformations[i * 12 + 7] = particles[i].tf_.R_array_[7];
    h_transformations[i * 12 + 8] = particles[i].tf_.R_array_[8];
    h_transformations[i * 12 + 9] = particles[i].tf_.t_array_[0];
    h_transformations[i * 12 + 10] = particles[i].tf_.t_array_[1];
    h_transformations[i * 12 + 11] = particles[i].tf_.t_array_[2];
  }
  float* d_transformations = NULL;
  err = cudaMalloc((void**)&d_transformations, particles_size * 12 * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_transformations (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_transformations, h_transformations, particles_size * 12 * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector transformations from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // ****************************
  // *** Input Features
  // ****************************
  // Corners
  float* h_corners = (float*)malloc(corners_size * 3 * sizeof(float));
  for (int i = 0; i < corners_size; i++)
  {
    h_corners[i * 3 + 0] = corners[i].pos_.x_;
    h_corners[i * 3 + 1] = corners[i].pos_.y_;
    h_corners[i * 3 + 2] = corners[i].pos_.z_;
  }
  float* d_corners = NULL;
  err = cudaMalloc((void**)&d_corners, corners_size * 3 * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_corners (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_corners, h_corners, corners_size * 3 * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector corners from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  // Planars
  float* h_planars = (float*)malloc(planars_size * 3 * sizeof(float));
  for (int i = 0; i < planars_size; i++)
  {
    h_planars[i * 3 + 0] = planars[i].pos_.x_;
    h_planars[i * 3 + 1] = planars[i].pos_.y_;
    h_planars[i * 3 + 2] = planars[i].pos_.z_;
  }
  float* d_planars = NULL;
  err = cudaMalloc((void**)&d_planars, planars_size * 3 * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_planars (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_planars, h_planars, planars_size * 3 * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector planars from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // ****************************
  // *** Lookup table
  // ****************************
  LookUpTable d_look_up_table;
  err = cudaMalloc((void**)&d_look_up_table.look_up_table_, localization_mapping_interface->look_up_table_data_.look_up_table_size_ * sizeof(uint32_t));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_look_up_table (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_look_up_table.look_up_table_, localization_mapping_interface->look_up_table_data_.look_up_table_,
                   localization_mapping_interface->look_up_table_data_.look_up_table_size_ * sizeof(uint32_t), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector lookup_table from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  d_look_up_table.look_up_table_size_ = localization_mapping_interface->look_up_table_data_.look_up_table_size_;
  d_look_up_table.look_up_table_min_x_ = localization_mapping_interface->look_up_table_data_.look_up_table_min_x_;
  d_look_up_table.look_up_table_min_y_ = localization_mapping_interface->look_up_table_data_.look_up_table_min_y_;
  d_look_up_table.look_up_table_res_ = localization_mapping_interface->look_up_table_data_.look_up_table_res_;
  d_look_up_table.look_up_table_cols_ = localization_mapping_interface->look_up_table_data_.look_up_table_cols_;

  // ****************************
  // *** Output weights
  // ****************************
  float* h_corner_weights = (float*)malloc(particles_size * sizeof(float));
  float* h_planar_weights = (float*)malloc(particles_size * sizeof(float));
  for (int i = 0; i < particles_size; i++)
  {
    h_corner_weights[i] = 0.0;
    h_planar_weights[i] = 0.0;
  }
  // Corners
  float* d_corner_weights = NULL;
  err = cudaMalloc((void**)&d_corner_weights, particles_size * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_corner_weights (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_corner_weights, h_corner_weights, particles_size * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector corner weights from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  // Planars
  float* d_planar_weights = NULL;
  err = cudaMalloc((void**)&d_planar_weights, particles_size * sizeof(float));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector d_planar_weights (localization and mapping interface) (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_planar_weights, h_planar_weights, particles_size * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector planar weights from host to device (localization and mapping interface) (error "
            "code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  timer.Stop();

  // Call the kernel
  // ----------------------------
  // --- Kernel call (!!!)
  // ----------------------------
  timer.Start();
  dim3 block_dim(block_dim_per_component, block_dim_per_component);

  dim3 grid_dim_for_corners((particles_size + block_dim.x - 1) / block_dim.x, (corners_size + block_dim.y - 1) / block_dim.y);
  cornerWeightCalculation<<<grid_dim_for_corners, block_dim>>>(d_array, d_sizes, d_info, d_transformations, d_corners,
                                                               d_look_up_table, particles_size, corners_size,
                                                               d_corner_weights);

  dim3 grid_dim_for_planars((particles_size + block_dim.x - 1) / block_dim.x, (planars_size + block_dim.y - 1) / block_dim.y);
  planarWeightCalculation<<<grid_dim_for_planars, block_dim>>>(d_array, d_sizes, d_info, d_transformations, d_planars, d_look_up_table, particles_size,
                                                               planars_size, d_planar_weights);

  weightRefinement<<<(particles_size + block_dim_per_component - 1) / block_dim_per_component,
                     block_dim_per_component>>>(d_corner_weights, d_planar_weights, particles_size,
                                                sigma_corner_matching, sigma_planar_matching);
  cudaDeviceSynchronize();

  err = cudaMemcpy(h_corner_weights, d_corner_weights, particles_size * sizeof(float), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector output corners from device to host (localization and mapping interface) (error code "
            "%s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(h_planar_weights, d_planar_weights, particles_size * sizeof(float), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
    fprintf(stderr,
            "Failed to copy vector output planars from device to host (localization and mapping interface) (error code "
            "%s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Save particles' weights
  for (int i = 0; i < particles_size; i++)
  {
    particles[i].w_ = h_corner_weights[i] * h_planar_weights[i];
  }

  // Free allocated memory
  for (int i = 0; i < localization_mapping_interface->number_of_nodes_; i++)
  {
    if (localization_mapping_interface->sizes_[i] > 0)
    {
      cudaFree(tmp[i]);
    }
  }
  cudaFree(d_array);
  cudaFree(d_sizes);
  cudaFree(d_info);
  cudaFree(d_transformations);
  cudaFree(d_corners);
  cudaFree(d_planars);
  cudaFree(d_corner_weights);
  cudaFree(d_planar_weights);
  cudaFree(d_look_up_table.look_up_table_);
  free(tmp);
  free(h_transformations);
  free(h_corner_weights);
  free(h_planar_weights);
  free(h_corners);
  free(h_planars);
}
}  // namespace vineslam