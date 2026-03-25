#pragma once

extern "C" {
  // xyz: pointer to float array of size 3*N (x,y,z)
  // neighbors: int array of size N * max_neighbors (filled with neighbor indices or -1)
  // neighbor_counts: int array of size N (number of neighbors found, capped by max_neighbors)
  // Returns false if CUDA context is corrupted (e.g. after laptop suspend/resume); resets GPU and caller should skip this frame
  bool dbscan_gpu_query_neighbors(const float* xyz, int N, float eps, int max_neighbors, int* neighbors, int* neighbor_counts);
}
