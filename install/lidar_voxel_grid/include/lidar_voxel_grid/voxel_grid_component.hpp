#ifndef LIDAR_VOXEL_GRID__VOXEL_GRID_COMPONENT_HPP_
#define LIDAR_VOXEL_GRID__VOXEL_GRID_COMPONENT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_voxel_grid
{

class VoxelGridComponent : public rclcpp::Node
{
public:
  explicit VoxelGridComponent(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  double leaf_size_x_;
  double leaf_size_y_;
  double leaf_size_z_;
  double near_range_;   // XY distance threshold (m): within → voxel grid, beyond → passthrough
};

}  // namespace lidar_voxel_grid

#endif  // LIDAR_VOXEL_GRID__VOXEL_GRID_COMPONENT_HPP_
