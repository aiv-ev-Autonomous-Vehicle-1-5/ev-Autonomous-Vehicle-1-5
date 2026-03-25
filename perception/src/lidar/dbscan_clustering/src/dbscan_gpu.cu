#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include "dbscan_clustering/dbscan_gpu.cuh"

// Check CUDA call; on failure log and jump to cleanup label
#define CUDA_TRY(call) \
  do { \
    err = (call); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "CUDA error: %s %s %d\n", cudaGetErrorString(err), __FILE__, __LINE__); \
      goto cleanup; \
    } \
  } while(0)

// Cooldown after CUDA failure (seconds)
static constexpr double GPU_COOLDOWN_SEC = 3.0;
static std::chrono::steady_clock::time_point s_last_failure{};
static bool s_in_cooldown = false;

__global__ void neighbor_kernel(const float* xyz, int N, float eps2, int max_neighbors, int* neighbors, int* neighbor_counts)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= N) return;
  const float xi = xyz[3*i + 0];
  const float yi = xyz[3*i + 1];
  const float zi = xyz[3*i + 2];
  int count = 0;
  for (int j = 0; j < N && count < max_neighbors; ++j) {
    const float dx = xi - xyz[3*j + 0];
    const float dy = yi - xyz[3*j + 1];
    const float dz = zi - xyz[3*j + 2];
    const float d2 = dx*dx + dy*dy + dz*dz;
    if (d2 <= eps2) {
      neighbors[i * max_neighbors + count] = j;
      ++count;
    }
  }
  neighbor_counts[i] = count;
  for (int k = count; k < max_neighbors; ++k) neighbors[i * max_neighbors + k] = -1;
}

bool dbscan_gpu_query_neighbors(const float* xyz_host, int N, float eps, int max_neighbors, int* neighbors_host, int* neighbor_counts_host)
{
  if (N <= 0) return true;

  // Skip CUDA calls during cooldown period after a failure
  if (s_in_cooldown) {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - s_last_failure).count();
    if (elapsed < GPU_COOLDOWN_SEC) {
      return false;  // still cooling down, skip silently
    }
    s_in_cooldown = false;
    fprintf(stderr, "GPU cooldown elapsed (%.1fs) — retrying CUDA.\n", elapsed);
  }

  size_t xyz_bytes = sizeof(float) * 3 * N;
  size_t neighbors_bytes = sizeof(int) * N * max_neighbors;
  size_t counts_bytes = sizeof(int) * N;

  float* d_xyz = nullptr;
  int* d_neighbors = nullptr;
  int* d_counts = nullptr;
  bool success = false;
  cudaError_t err;

  CUDA_TRY(cudaMalloc((void**)&d_xyz, xyz_bytes));
  CUDA_TRY(cudaMalloc((void**)&d_neighbors, neighbors_bytes));
  CUDA_TRY(cudaMalloc((void**)&d_counts, counts_bytes));

  CUDA_TRY(cudaMemcpy(d_xyz, xyz_host, xyz_bytes, cudaMemcpyHostToDevice));

  {
    int threads = 256;
    int blocks = (N + threads - 1) / threads;
    neighbor_kernel<<<blocks, threads>>>(d_xyz, N, eps * eps, max_neighbors, d_neighbors, d_counts);
  }
  CUDA_TRY(cudaGetLastError());
  CUDA_TRY(cudaDeviceSynchronize());

  CUDA_TRY(cudaMemcpy(neighbors_host, d_neighbors, neighbors_bytes, cudaMemcpyDeviceToHost));
  CUDA_TRY(cudaMemcpy(neighbor_counts_host, d_counts, counts_bytes, cudaMemcpyDeviceToHost));

  success = true;

cleanup:
  if (d_xyz) cudaFree(d_xyz);
  if (d_neighbors) cudaFree(d_neighbors);
  if (d_counts) cudaFree(d_counts);
  if (!success) {
    // Clear sticky CUDA error without destroying the context
    // (cudaDeviceReset() is unsafe in component_container_mt — it destroys
    //  the CUDA context for ALL components in the process)
    cudaGetLastError();
    fprintf(stderr, "CUDA error — cleared error state. Cooldown %.0fs before retry.\n", GPU_COOLDOWN_SEC);
    s_last_failure = std::chrono::steady_clock::now();
    s_in_cooldown = true;
  }
  return success;
}
