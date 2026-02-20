#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include "dbscan_clustering/dbscan_gpu.cuh"

// Simple error check
#define CUDA_CHECK(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
  if (code != cudaSuccess) {
    fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}

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

void dbscan_gpu_query_neighbors(const float* xyz_host, int N, float eps, int max_neighbors, int* neighbors_host, int* neighbor_counts_host)
{
  if (N <= 0) return;
  size_t xyz_bytes = sizeof(float) * 3 * N;
  size_t neighbors_bytes = sizeof(int) * N * max_neighbors;
  size_t counts_bytes = sizeof(int) * N;

  float* d_xyz = nullptr;
  int* d_neighbors = nullptr;
  int* d_counts = nullptr;

  CUDA_CHECK(cudaMalloc((void**)&d_xyz, xyz_bytes));
  CUDA_CHECK(cudaMalloc((void**)&d_neighbors, neighbors_bytes));
  CUDA_CHECK(cudaMalloc((void**)&d_counts, counts_bytes));

  CUDA_CHECK(cudaMemcpy(d_xyz, xyz_host, xyz_bytes, cudaMemcpyHostToDevice));

  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  neighbor_kernel<<<blocks, threads>>>(d_xyz, N, eps * eps, max_neighbors, d_neighbors, d_counts);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(neighbors_host, d_neighbors, neighbors_bytes, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(neighbor_counts_host, d_counts, counts_bytes, cudaMemcpyDeviceToHost));

  cudaFree(d_xyz);
  cudaFree(d_neighbors);
  cudaFree(d_counts);
}
