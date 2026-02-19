#ifndef LIDAR_CLUSTERING__DBSCAN_GPU_HPP_
#define LIDAR_CLUSTERING__DBSCAN_GPU_HPP_

#include <cstdint>
#include <memory>
#include <vector>

namespace lidar_clustering
{

struct Point3D {
  float x, y, z;
};

class DBSCANGpu
{
public:
  explicit DBSCANGpu(size_t max_points = 50000);
  ~DBSCANGpu();

  DBSCANGpu(const DBSCANGpu &) = delete;
  DBSCANGpu & operator=(const DBSCANGpu &) = delete;
  DBSCANGpu(DBSCANGpu &&) noexcept;
  DBSCANGpu & operator=(DBSCANGpu &&) noexcept;

  std::vector<int32_t> cluster(
    const std::vector<Point3D> & points, double eps, int min_pts);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar_clustering

#endif  // LIDAR_CLUSTERING__DBSCAN_GPU_HPP_
