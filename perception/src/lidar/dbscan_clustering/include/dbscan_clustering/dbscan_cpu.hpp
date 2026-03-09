#ifndef DBSCAN_CLUSTERING__DBSCAN_CPU_HPP_
#define DBSCAN_CLUSTERING__DBSCAN_CPU_HPP_

/// CPU fallback for neighbor search using nanoflann KD-Tree.
/// Same interface as dbscan_gpu_query_neighbors().
void dbscan_cpu_query_neighbors(
    const float* xyz, int N, float eps, int max_neighbors,
    int* neighbors, int* neighbor_counts);

#endif  // DBSCAN_CLUSTERING__DBSCAN_CPU_HPP_
