#include "dbscan_clustering/dbscan_cpu.hpp"
#include "nanoflann.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace
{

/// nanoflann adaptor for a flat float array [x0,y0,z0, x1,y1,z1, ...]
struct PointCloudAdaptor
{
  const float * pts;
  int count;

  inline size_t kdtree_get_point_count() const
  {
    return static_cast<size_t>(count);
  }

  inline float kdtree_get_pt(size_t idx, size_t dim) const
  {
    return pts[3 * idx + dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX &) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
    PointCloudAdaptor, 3>;

}  // anonymous namespace

void dbscan_cpu_query_neighbors(
    const float * xyz, int N, float eps, int max_neighbors,
    int * neighbors, int * neighbor_counts)
{
  if (N <= 0) return;

  // Initialize output
  std::memset(neighbor_counts, 0, sizeof(int) * N);
  std::memset(neighbors, -1, sizeof(int) * N * max_neighbors);

  // Build KD-Tree
  PointCloudAdaptor cloud{xyz, N};
  KDTree tree(3, cloud, nanoflann::KDTreeSingleIndexParams{32});

  const float eps2 = eps * eps;
  nanoflann::SearchParameters params;
  params.sorted = false;

  // Reusable result buffer
  std::vector<nanoflann::ResultItem<uint32_t, float>> matches;

  for (int i = 0; i < N; ++i) {
    matches.clear();
    tree.radiusSearch(&xyz[3 * i], eps2, matches, params);

    int count = std::min(static_cast<int>(matches.size()), max_neighbors);
    neighbor_counts[i] = count;

    int * row = neighbors + i * max_neighbors;
    for (int k = 0; k < count; ++k) {
      row[k] = static_cast<int>(matches[k].first);
    }
  }
}
