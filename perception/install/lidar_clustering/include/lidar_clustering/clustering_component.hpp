#ifndef LIDAR_CLUSTERING__CLUSTERING_COMPONENT_HPP_
#define LIDAR_CLUSTERING__CLUSTERING_COMPONENT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_clustering/dbscan_gpu.hpp"

namespace lidar_clustering
{

class ClusteringComponent : public rclcpp::Node
{
public:
  explicit ClusteringComponent(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  double eps_;
  int min_pts_;
  double adaptive_range_ref_;
  std::unique_ptr<DBSCANGpu> dbscan_gpu_;
};

}  // namespace lidar_clustering

#endif  // LIDAR_CLUSTERING__CLUSTERING_COMPONENT_HPP_
