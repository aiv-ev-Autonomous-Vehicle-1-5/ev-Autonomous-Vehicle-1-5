#ifndef LIDAR_CLUSTERING__DBSCAN_GPU_KERNELS_H_
#define LIDAR_CLUSTERING__DBSCAN_GPU_KERNELS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPU resource handle (opaque pointer)
typedef struct GpuResources* GpuResourcesHandle;

// Lifecycle
GpuResourcesHandle gpu_resources_create(size_t max_points, size_t max_cells, size_t max_neighbors);
void gpu_resources_destroy(GpuResourcesHandle h);

// Upload points (SoA) to GPU
int gpu_upload_points(GpuResourcesHandle h, const float* hx, const float* hy, const float* hz, int n);

// Compute cell indices on GPU, download to host
int gpu_compute_cell_indices(
  GpuResourcesHandle h, int n,
  float origin_x, float origin_y, float origin_z,
  float inv_cell_size,
  int grid_x, int grid_y, int grid_z,
  int* h_cell_indices_out);

// Upload sorted indices + sorted cell indices, build cell bounds
int gpu_upload_sorted_and_build_bounds(
  GpuResourcesHandle h,
  const int* h_sorted_indices, const int* h_sorted_cells,
  int n, size_t num_cells);

// Count neighbors and mark core points
int gpu_count_neighbors(
  GpuResourcesHandle h, int n,
  float eps_sq, int min_pts,
  int grid_x, int grid_y, int grid_z,
  int* h_neighbor_counts_out, int* h_is_core_out);

// Write neighbor list using precomputed offsets
int gpu_write_neighbors(
  GpuResourcesHandle h, int n,
  float eps_sq,
  int grid_x, int grid_y, int grid_z,
  const int* h_neighbor_offsets,
  int* h_neighbor_list_out, size_t total_neighbors);

// Perform DSU Union on GPU (avoids neighbor list transfer)
// labels_in_out must be initialized to 0..n-1 on device (or host and uploaded)
int gpu_dbscan_dsu_union(
  GpuResourcesHandle h, int n,
  float eps_sq,
  int grid_x, int grid_y, int grid_z,
  int* h_labels_in_out);

// Resize neighbor list buffer if needed
int gpu_resize_neighbor_list(GpuResourcesHandle h, size_t new_max_neighbors);

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_CLUSTERING__DBSCAN_GPU_KERNELS_H_
