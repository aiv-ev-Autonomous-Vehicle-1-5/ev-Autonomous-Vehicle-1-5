// Pure CUDA kernels + C API wrappers
// NO STL, NO CUB, NO Thrust - only cuda_runtime.h and C headers
// This avoids nvcc 11.5 + GCC 11 incompatibility with <functional>

#include "lidar_clustering/dbscan_gpu_kernels.h"

#include <cuda_runtime.h>
#include <stdlib.h>
#include <stdio.h>

// ============================================================
// GpuResources struct (opaque in the C header)
// ============================================================
struct GpuResources {
  float* d_px;
  float* d_py;
  float* d_pz;
  int* d_cell_indices;
  int* d_sorted_indices;
  int* d_sorted_cells;
  int* d_cell_start;
  int* d_cell_end;
  int* d_neighbor_counts;
  int* d_is_core;
  int* d_neighbor_offsets;
  int* d_neighbor_list;
  size_t max_points;
  size_t max_cells;
  size_t max_neighbors;
};

// ============================================================
// CUDA Kernels
// ============================================================

__device__ int device_find(int* labels, int i) {
  int p = labels[i];
  if (p != i) {
    int next = labels[p];
    if (next != p) {
      // Path halving (simple compression)
      p = next; 
      // Note: Full path compression is hard with concurrent modifications
    }
  }
  return p;
}

__device__ void device_union(int* labels, int i, int j) {
  int root_i = i;
  int root_j = j;
  
  while (true) {
    // Find roots
    while (labels[root_i] != root_i) root_i = labels[root_i];
    while (labels[root_j] != root_j) root_j = labels[root_j];

    if (root_i == root_j) return;

    int small = (root_i < root_j) ? root_i : root_j;
    int large = (root_i < root_j) ? root_j : root_i;

    // Attempt to hook large -> small
    if (atomicCAS(&labels[large], large, small) == large) {
      return; // Success
    }
    // CAS failed, retry (graph structure changed)
    // Re-read roots starting from the nodes we had
    root_i = labels[large]; // Start from where we failed
    root_j = small;
  }
}

__global__ void kernel_compute_cell_indices(
  const float* px, const float* py, const float* pz,
  int* cell_indices, int n,
  float origin_x, float origin_y, float origin_z,
  float inv_cell_size,
  int grid_x, int grid_y, int grid_z)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n) return;

  int cx = (int)((px[idx] - origin_x) * inv_cell_size);
  int cy = (int)((py[idx] - origin_y) * inv_cell_size);
  int cz = (int)((pz[idx] - origin_z) * inv_cell_size);

  if (cx < 0) cx = 0;
  if (cx >= grid_x) cx = grid_x - 1;
  if (cy < 0) cy = 0;
  if (cy >= grid_y) cy = grid_y - 1;
  if (cz < 0) cz = 0;
  if (cz >= grid_z) cz = grid_z - 1;

  cell_indices[idx] = cx + cy * grid_x + cz * grid_x * grid_y;
}

__global__ void kernel_find_cell_bounds(
  const int* cell_indices, int* cell_start, int* cell_end, int n)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n) return;

  int cell = cell_indices[idx];
  if (idx == 0 || cell != cell_indices[idx - 1]) {
    cell_start[cell] = idx;
  }
  if (idx == n - 1 || cell != cell_indices[idx + 1]) {
    cell_end[cell] = idx + 1;
  }
}

__global__ void kernel_count_neighbors(
  const float* px, const float* py, const float* pz,
  const int* sorted_indices,
  const int* cell_indices_sorted,
  const int* cell_start, const int* cell_end,
  int n, float eps_sq, int min_pts,
  int* neighbor_counts, int* is_core,
  int grid_x, int grid_y, int grid_z)
{
  int sorted_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (sorted_idx >= n) return;

  int pid = sorted_indices[sorted_idx];
  float qx = px[pid], qy = py[pid], qz = pz[pid];

  int cell = cell_indices_sorted[sorted_idx];
  int cz_val = cell / (grid_x * grid_y);
  int cy_val = (cell % (grid_x * grid_y)) / grid_x;
  int cx_val = cell % grid_x;

  int count = 0;
  for (int dz = -1; dz <= 1; dz++) {
    int nz = cz_val + dz;
    if (nz < 0 || nz >= grid_z) continue;
    for (int dy = -1; dy <= 1; dy++) {
      int ny = cy_val + dy;
      if (ny < 0 || ny >= grid_y) continue;
      for (int dx = -1; dx <= 1; dx++) {
        int nx = cx_val + dx;
        if (nx < 0 || nx >= grid_x) continue;

        int ncell = nx + ny * grid_x + nz * grid_x * grid_y;
        int start = cell_start[ncell];
        if (start < 0) continue;
        int end_val = cell_end[ncell];

        for (int j = start; j < end_val; j++) {
          int qid = sorted_indices[j];
          float ddx = qx - px[qid];
          float ddy = qy - py[qid];
          float ddz = qz - pz[qid];
          float dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
          if (dist_sq <= eps_sq) {
            count++;
          }
        }
      }
    }
  }

  neighbor_counts[pid] = count;
  is_core[pid] = (count >= min_pts) ? 1 : 0;
}

__global__ void kernel_write_neighbors(
  const float* px, const float* py, const float* pz,
  const int* sorted_indices,
  const int* cell_indices_sorted,
  const int* cell_start, const int* cell_end,
  int n, float eps_sq,
  const int* neighbor_offsets,
  int* neighbor_list,
  int grid_x, int grid_y, int grid_z)
{
  int sorted_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (sorted_idx >= n) return;

  int pid = sorted_indices[sorted_idx];
  float qx = px[pid], qy = py[pid], qz = pz[pid];

  int cell = cell_indices_sorted[sorted_idx];
  int cz_val = cell / (grid_x * grid_y);
  int cy_val = (cell % (grid_x * grid_y)) / grid_x;
  int cx_val = cell % grid_x;

  int write_pos = neighbor_offsets[pid];

  for (int dz = -1; dz <= 1; dz++) {
    int nz = cz_val + dz;
    if (nz < 0 || nz >= grid_z) continue;
    for (int dy = -1; dy <= 1; dy++) {
      int ny = cy_val + dy;
      if (ny < 0 || ny >= grid_y) continue;
      for (int dx = -1; dx <= 1; dx++) {
        int nx = cx_val + dx;
        if (nx < 0 || nx >= grid_x) continue;

        int ncell = nx + ny * grid_x + nz * grid_x * grid_y;
        int start = cell_start[ncell];
        if (start < 0) continue;
        int end_val = cell_end[ncell];

        for (int j = start; j < end_val; j++) {
          int qid = sorted_indices[j];
          float ddx = qx - px[qid];
          float ddy = qy - py[qid];
          float ddz = qz - pz[qid];
          float dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
          if (dist_sq <= eps_sq) {
            neighbor_list[write_pos++] = qid;
          }
        }
      }
    }
  }
}

// ============================================================
// C API Implementation
// ============================================================

static int check_cuda(cudaError_t err) {
  if (err != cudaSuccess) {
    fprintf(stderr, "CUDA error: %s\n", cudaGetErrorString(err));
    return -1;
  }
  return 0;
}

GpuResourcesHandle gpu_resources_create(
  size_t max_points, size_t max_cells, size_t max_neighbors)
{
  GpuResources* r = (GpuResources*)calloc(1, sizeof(GpuResources));
  if (!r) return NULL;

  r->max_points = max_points;
  r->max_cells = max_cells;
  r->max_neighbors = max_neighbors;

  if (cudaMalloc(&r->d_px, max_points * sizeof(float)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_py, max_points * sizeof(float)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_pz, max_points * sizeof(float)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_cell_indices, max_points * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_sorted_indices, max_points * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_sorted_cells, max_points * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_cell_start, max_cells * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_cell_end, max_cells * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_neighbor_counts, max_points * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_is_core, max_points * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_neighbor_offsets, (max_points + 1) * sizeof(int)) != cudaSuccess) goto fail;
  if (cudaMalloc(&r->d_neighbor_list, max_neighbors * sizeof(int)) != cudaSuccess) goto fail;

  return r;

fail:
  gpu_resources_destroy(r);
  return NULL;
}

void gpu_resources_destroy(GpuResourcesHandle h) {
  if (!h) return;
  cudaFree(h->d_px);
  cudaFree(h->d_py);
  cudaFree(h->d_pz);
  cudaFree(h->d_cell_indices);
  cudaFree(h->d_sorted_indices);
  cudaFree(h->d_sorted_cells);
  cudaFree(h->d_cell_start);
  cudaFree(h->d_cell_end);
  cudaFree(h->d_neighbor_counts);
  cudaFree(h->d_is_core);
  cudaFree(h->d_neighbor_offsets);
  cudaFree(h->d_neighbor_list);
  free(h);
}

int gpu_upload_points(
  GpuResourcesHandle h,
  const float* hx, const float* hy, const float* hz, int n)
{
  if (!h || n <= 0) return -1;
  if (check_cuda(cudaMemcpy(h->d_px, hx, n * sizeof(float), cudaMemcpyHostToDevice))) return -1;
  if (check_cuda(cudaMemcpy(h->d_py, hy, n * sizeof(float), cudaMemcpyHostToDevice))) return -1;
  if (check_cuda(cudaMemcpy(h->d_pz, hz, n * sizeof(float), cudaMemcpyHostToDevice))) return -1;
  return 0;
}

int gpu_compute_cell_indices(
  GpuResourcesHandle h, int n,
  float origin_x, float origin_y, float origin_z,
  float inv_cell_size,
  int grid_x, int grid_y, int grid_z,
  int* h_cell_indices_out)
{
  if (!h || n <= 0) return -1;

  int block_size = 256;
  int grid_dim = (n + block_size - 1) / block_size;

  kernel_compute_cell_indices<<<grid_dim, block_size>>>(
    h->d_px, h->d_py, h->d_pz,
    h->d_cell_indices, n,
    origin_x, origin_y, origin_z, inv_cell_size,
    grid_x, grid_y, grid_z);

  if (check_cuda(cudaGetLastError())) return -1;
  if (check_cuda(cudaDeviceSynchronize())) return -1;

  if (check_cuda(cudaMemcpy(h_cell_indices_out, h->d_cell_indices,
    n * sizeof(int), cudaMemcpyDeviceToHost))) return -1;

  return 0;
}

int gpu_upload_sorted_and_build_bounds(
  GpuResourcesHandle h,
  const int* h_sorted_indices, const int* h_sorted_cells,
  int n, size_t num_cells)
{
  if (!h || n <= 0) return -1;

  if (check_cuda(cudaMemcpy(h->d_sorted_indices, h_sorted_indices,
    n * sizeof(int), cudaMemcpyHostToDevice))) return -1;
  if (check_cuda(cudaMemcpy(h->d_sorted_cells, h_sorted_cells,
    n * sizeof(int), cudaMemcpyHostToDevice))) return -1;

  if (check_cuda(cudaMemset(h->d_cell_start, -1, num_cells * sizeof(int)))) return -1;
  if (check_cuda(cudaMemset(h->d_cell_end, 0, num_cells * sizeof(int)))) return -1;

  int block_size = 256;
  int grid_dim = (n + block_size - 1) / block_size;

  kernel_find_cell_bounds<<<grid_dim, block_size>>>(
    h->d_sorted_cells, h->d_cell_start, h->d_cell_end, n);

  if (check_cuda(cudaGetLastError())) return -1;
  if (check_cuda(cudaDeviceSynchronize())) return -1;

  return 0;
}

int gpu_count_neighbors(
  GpuResourcesHandle h, int n,
  float eps_sq, int min_pts,
  int grid_x, int grid_y, int grid_z,
  int* h_neighbor_counts_out, int* h_is_core_out)
{
  if (!h || n <= 0) return -1;

  if (check_cuda(cudaMemset(h->d_neighbor_counts, 0, n * sizeof(int)))) return -1;

  int block_size = 256;
  int grid_dim = (n + block_size - 1) / block_size;

  kernel_count_neighbors<<<grid_dim, block_size>>>(
    h->d_px, h->d_py, h->d_pz,
    h->d_sorted_indices,
    h->d_sorted_cells,
    h->d_cell_start, h->d_cell_end,
    n, eps_sq, min_pts,
    h->d_neighbor_counts, h->d_is_core,
    grid_x, grid_y, grid_z);

  if (check_cuda(cudaGetLastError())) return -1;
  if (check_cuda(cudaDeviceSynchronize())) return -1;

  if (check_cuda(cudaMemcpy(h_neighbor_counts_out, h->d_neighbor_counts,
    n * sizeof(int), cudaMemcpyDeviceToHost))) return -1;
  if (check_cuda(cudaMemcpy(h_is_core_out, h->d_is_core,
    n * sizeof(int), cudaMemcpyDeviceToHost))) return -1;

  return 0;
}

int gpu_write_neighbors(
  GpuResourcesHandle h, int n,
  float eps_sq,
  int grid_x, int grid_y, int grid_z,
  const int* h_neighbor_offsets,
  int* h_neighbor_list_out, size_t total_neighbors)
{
  if (!h || n <= 0) return -1;

  if (check_cuda(cudaMemcpy(h->d_neighbor_offsets, h_neighbor_offsets,
    n * sizeof(int), cudaMemcpyHostToDevice))) return -1;

  int block_size = 256;
  int grid_dim = (n + block_size - 1) / block_size;

  kernel_write_neighbors<<<grid_dim, block_size>>>(
    h->d_px, h->d_py, h->d_pz,
    h->d_sorted_indices,
    h->d_sorted_cells,
    h->d_cell_start, h->d_cell_end,
    n, eps_sq,
    h->d_neighbor_offsets,
    h->d_neighbor_list,
    grid_x, grid_y, grid_z);

  if (check_cuda(cudaGetLastError())) return -1;
  if (check_cuda(cudaDeviceSynchronize())) return -1;

  if (total_neighbors > 0) {
    if (check_cuda(cudaMemcpy(h_neighbor_list_out, h->d_neighbor_list,
      total_neighbors * sizeof(int), cudaMemcpyDeviceToHost))) return -1;
  }

  return 0;
}

int gpu_resize_neighbor_list(GpuResourcesHandle h, size_t new_max_neighbors) {
  if (!h) return -1;
  cudaFree(h->d_neighbor_list);
  h->d_neighbor_list = NULL;
  h->max_neighbors = new_max_neighbors;
  if (check_cuda(cudaMalloc(&h->d_neighbor_list,
    new_max_neighbors * sizeof(int)))) return -1;
  return 0;
}

__global__ void kernel_init_labels(int* labels, int n) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n) labels[idx] = idx;
}

__global__ void kernel_dsu_union(
  const float* px, const float* py, const float* pz,
  const int* sorted_indices,
  const int* cell_indices_sorted,
  const int* cell_start, const int* cell_end,
  int n, float eps_sq,
  int* labels,
  const int* is_core,
  int grid_x, int grid_y, int grid_z)
{
  int sorted_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (sorted_idx >= n) return;

  int pid = sorted_indices[sorted_idx];
  
  // Only core points initiate connections
  if (!is_core[pid]) return;

  float qx = px[pid], qy = py[pid], qz = pz[pid];
  int cell = cell_indices_sorted[sorted_idx];
  
  int cz_val = cell / (grid_x * grid_y);
  int cy_val = (cell % (grid_x * grid_y)) / grid_x;
  int cx_val = cell % grid_x;

  for (int dz = -1; dz <= 1; dz++) {
    int nz = cz_val + dz;
    if (nz < 0 || nz >= grid_z) continue;
    for (int dy = -1; dy <= 1; dy++) {
      int ny = cy_val + dy;
      if (ny < 0 || ny >= grid_y) continue;
      for (int dx = -1; dx <= 1; dx++) {
        int nx = cx_val + dx;
        if (nx < 0 || nx >= grid_x) continue;

        int ncell = nx + ny * grid_x + nz * grid_x * grid_y;
        int start = cell_start[ncell];
        if (start < 0) continue;
        int end_val = cell_end[ncell];

        for (int j = start; j < end_val; j++) {
          int qid = sorted_indices[j];
          float ddx = qx - px[qid];
          float ddy = qy - py[qid];
          float ddz = qz - pz[qid];
          float dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
          
          if (dist_sq <= eps_sq) {
            // Found a neighbor. Union pid and qid.
            // Note: device_union handles the loop and atomics.
            device_union(labels, pid, qid);
          }
        }
      }
    }
  }
}

// Perform DSU Union on GPU
int gpu_dbscan_dsu_union(
  GpuResourcesHandle h, int n,
  float eps_sq,
  int grid_x, int grid_y, int grid_z,
  int* h_labels_out)
{
  if (!h || n <= 0) return -1;

  // We reuse d_neighbor_counts as labels array to save memory/allocation
  int* d_labels = h->d_neighbor_counts;

  // 1. Initialize labels: labels[i] = i
  int block_size = 256;
  int grid_dim = (n + block_size - 1) / block_size;
  kernel_init_labels<<<grid_dim, block_size>>>(d_labels, n);
  if (check_cuda(cudaGetLastError())) return -1;

  // 2. Run DSU kernel
  kernel_dsu_union<<<grid_dim, block_size>>>(
    h->d_px, h->d_py, h->d_pz,
    h->d_sorted_indices,
    h->d_sorted_cells,
    h->d_cell_start, h->d_cell_end,
    n, eps_sq,
    d_labels,
    h->d_is_core,
    grid_x, grid_y, grid_z);

  if (check_cuda(cudaGetLastError())) return -1;
  if (check_cuda(cudaDeviceSynchronize())) return -1;

  // 3. Download labels to host
  if (check_cuda(cudaMemcpy(h_labels_out, d_labels,
    n * sizeof(int), cudaMemcpyDeviceToHost))) return -1;

  return 0;
}
