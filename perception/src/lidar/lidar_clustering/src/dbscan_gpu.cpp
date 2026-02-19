// Host-side GPU DBSCAN orchestration
// Compiled by g++ (NOT nvcc) — safe to use all C++ STL headers
// Calls GPU kernels via C API defined in dbscan_gpu_kernels.h

#include "lidar_clustering/dbscan_gpu.hpp"
#include "lidar_clustering/dbscan_gpu_kernels.h"

#include <algorithm>
#include <numeric>
#include <vector>
#include <queue>
#include <stdexcept>

namespace lidar_clustering
{

struct DBSCANGpu::Impl
{
  GpuResourcesHandle gpu_;
  size_t max_points_;
  size_t max_cells_;
  size_t max_neighbors_;

  explicit Impl(size_t max_points)
  : max_points_(max_points),
    max_cells_(1000000),
    max_neighbors_(max_points * 30)
  {
    gpu_ = gpu_resources_create(max_points_, max_cells_, max_neighbors_);
    if (!gpu_) {
      throw std::runtime_error("Failed to create GPU resources for DBSCAN");
    }
  }

  ~Impl()
  {
    gpu_resources_destroy(gpu_);
  }

  void ensure_capacity(size_t n, size_t num_cells)
  {
    bool recreate = false;
    if (n > max_points_) {
      max_points_ = n * 2;
      max_neighbors_ = max_points_ * 30;
      recreate = true;
    }
    if (num_cells > max_cells_) {
      max_cells_ = num_cells * 2;
      recreate = true;
    }
    if (recreate) {
      gpu_resources_destroy(gpu_);
      gpu_ = gpu_resources_create(max_points_, max_cells_, max_neighbors_);
      if (!gpu_) {
        throw std::runtime_error("Failed to recreate GPU resources");
      }
    }
  }
};

// ============================================================
// DBSCANGpu public interface
// ============================================================

DBSCANGpu::DBSCANGpu(size_t max_points)
: impl_(std::make_unique<Impl>(max_points))
{
}

DBSCANGpu::~DBSCANGpu() = default;
DBSCANGpu::DBSCANGpu(DBSCANGpu &&) noexcept = default;
DBSCANGpu & DBSCANGpu::operator=(DBSCANGpu &&) noexcept = default;

std::vector<int32_t> DBSCANGpu::cluster(
  const std::vector<Point3D> & points, double eps, int min_pts)
{
  const int n = static_cast<int>(points.size());
  std::vector<int32_t> labels(n, -1);

  if (n == 0) return labels;

  // ---- SoA conversion + AABB ----
  std::vector<float> hx(n), hy(n), hz(n);
  float min_x = points[0].x, max_x = points[0].x;
  float min_y = points[0].y, max_y = points[0].y;
  float min_z = points[0].z, max_z = points[0].z;

  for (int i = 0; i < n; i++) {
    hx[i] = points[i].x;
    hy[i] = points[i].y;
    hz[i] = points[i].z;
    if (hx[i] < min_x) min_x = hx[i];
    if (hx[i] > max_x) max_x = hx[i];
    if (hy[i] < min_y) min_y = hy[i];
    if (hy[i] > max_y) max_y = hy[i];
    if (hz[i] < min_z) min_z = hz[i];
    if (hz[i] > max_z) max_z = hz[i];
  }

  const float cell_size = static_cast<float>(eps);
  const float inv_cell_size = 1.0f / cell_size;
  const int grid_x = static_cast<int>((max_x - min_x) * inv_cell_size) + 2;
  const int grid_y = static_cast<int>((max_y - min_y) * inv_cell_size) + 2;
  const int grid_z = static_cast<int>((max_z - min_z) * inv_cell_size) + 2;
  const size_t num_cells = static_cast<size_t>(grid_x) * grid_y * grid_z;

  // Grid too large — fall back (caller detects all -1 labels)
  if (num_cells > 10000000) {
    return labels;
  }

  impl_->ensure_capacity(n, num_cells);

  // ---- Upload points to GPU ----
  if (gpu_upload_points(impl_->gpu_, hx.data(), hy.data(), hz.data(), n) != 0) {
    throw std::runtime_error("gpu_upload_points failed");
  }

  // ---- Compute cell indices on GPU, download to host ----
  std::vector<int> h_cell_indices(n);
  if (gpu_compute_cell_indices(impl_->gpu_, n,
    min_x, min_y, min_z, inv_cell_size,
    grid_x, grid_y, grid_z,
    h_cell_indices.data()) != 0)
  {
    throw std::runtime_error("gpu_compute_cell_indices failed");
  }

  // ---- CPU sort by cell index ----
  std::vector<int> sorted_indices(n);
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(),
    [&h_cell_indices](int a, int b) {
      return h_cell_indices[a] < h_cell_indices[b];
    });

  std::vector<int> sorted_cells(n);
  for (int i = 0; i < n; i++) {
    sorted_cells[i] = h_cell_indices[sorted_indices[i]];
  }

  // ---- Upload sorted data, build cell bounds on GPU ----
  if (gpu_upload_sorted_and_build_bounds(impl_->gpu_,
    sorted_indices.data(), sorted_cells.data(),
    n, num_cells) != 0)
  {
    throw std::runtime_error("gpu_upload_sorted_and_build_bounds failed");
  }

  // ---- Count neighbors on GPU ----
  const float eps_sq = static_cast<float>(eps * eps);
  std::vector<int> h_neighbor_counts(n);
  std::vector<int> h_is_core(n);

  if (gpu_count_neighbors(impl_->gpu_, n, eps_sq, min_pts,
    grid_x, grid_y, grid_z,
    h_neighbor_counts.data(), h_is_core.data()) != 0)
  {
    throw std::runtime_error("gpu_count_neighbors failed");
  }

  // ---- DSU Connectivity on GPU ----
  // We avoid downloading the massive neighbor list.
  // Instead, we compute connected components on the GPU.
  std::vector<int> h_dsu_labels(n);
  
  if (gpu_dbscan_dsu_union(impl_->gpu_, n, eps_sq, 
    grid_x, grid_y, grid_z,
    h_dsu_labels.data()) != 0)
  {
    throw std::runtime_error("gpu_dbscan_dsu_union failed");
  }

  // ---- CPU Post-Processing (Flatten DSU & Assign IDs) ----
  
  // 1. Path Compression (Flattening)
  for (int i = 0; i < n; i++) {
    int root = i;
    while (h_dsu_labels[root] != root) {
      root = h_dsu_labels[root];
    }
    // Simple path compression
    int curr = i;
    while (curr != root) {
      int next = h_dsu_labels[curr];
      h_dsu_labels[curr] = root;
      curr = next;
    }
  }

  // 2. Identify Valid Clusters (must have at least one Core point)
  // map: root_idx -> cluster_id (or -1 if invalid/noise)
  std::vector<int> root_to_cluster_id(n, -2); // -2: unknown
  int32_t cluster_counter = 0;

  // We need to check if a tree has a core point.
  // Since we only union if source is core, all points in a non-trivial tree
  // are connected to a core point.
  // However, single points (noise) will have root == i.
  // If is_core[i] is true, it's a cluster of 1 (or more).
  // If is_core[i] is false, and it's a root (labels[i]==i), it's noise.
  
  // Wait, what if a border point Q was unioned to P? 
  // P is core. P -> Q. 
  // AtomicMin(labels[Q], P). labels[Q] = P.
  // Root of Q is P. P is core. Valid.
  
  // What if two border points? No union.
  
  // So: for each ROOT, check if it is core OR if it absorbed a core point?
  // Actually, because we ONLY union if `is_core[src]`, a non-core point
  // can only be part of a tree if a core point pulled it in.
  // So if `labels[i] == i`, it is a cluster ONLY if `is_core[i]`.
  // If `labels[i] == i` and `!is_core[i]`, it is NOISE.
  // If `labels[i] != i`, it belongs to a root. That root MUST be core
  // (or connected to core) because only core points initiate unions.
  // Exception: root could be a border point that was 'min' of the set?
  // `atomicMin(&labels[max], min)`.
  // If Core P (idx 100) connects to Border Q (idx 50).
  // `atomicMin(labels[100], 50)`.
  // `labels[100] = 50`.
  // Root is 50. Q is NOT core.
  // But 50 is connected to 100 (which is core).
  // So we need to track if a tree contains a core point.

  std::vector<bool> tree_has_core(n, false);
  for (int i = 0; i < n; i++) {
    if (h_is_core[i]) {
      int root = h_dsu_labels[i];
      tree_has_core[root] = true;
    }
  }

  for (int i = 0; i < n; i++) {
    int root = h_dsu_labels[i];
    if (root_to_cluster_id[root] == -2) {
      if (tree_has_core[root]) {
        root_to_cluster_id[root] = cluster_counter++;
      } else {
        root_to_cluster_id[root] = -1; // Noise
      }
    }
    labels[i] = root_to_cluster_id[root];
  }

  return labels;
}

}  // namespace lidar_clustering
