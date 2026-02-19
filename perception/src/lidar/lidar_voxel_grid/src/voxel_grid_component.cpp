#include "lidar_voxel_grid/voxel_grid_component.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_voxel_grid
{

VoxelGridComponent::VoxelGridComponent(const rclcpp::NodeOptions & options)
: Node("voxel_grid_component", options)
{
  leaf_size_x_ = this->declare_parameter("leaf_size_x", 0.05);
  leaf_size_y_ = this->declare_parameter("leaf_size_y", 0.05);
  leaf_size_z_ = this->declare_parameter("leaf_size_z", 0.05);
  near_range_  = this->declare_parameter("near_range", 10.0);

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&VoxelGridComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "VoxelGrid initialized: leaf=%.2fx%.2fx%.2f, near_range=%.1fm (beyond=passthrough)",
    leaf_size_x_, leaf_size_y_, leaf_size_z_, near_range_);
}

void VoxelGridComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS2 msg → PCL XYZ cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  const float r2_thresh = static_cast<float>(near_range_ * near_range_);

  // Split into near / far zones by XY distance
  pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  near_cloud->reserve(cloud->size());
  far_cloud->reserve(cloud->size());

  for (const auto & p : cloud->points) {
    float r2 = p.x * p.x + p.y * p.y;
    if (r2 <= r2_thresh) {
      near_cloud->push_back(p);
    } else {
      far_cloud->push_back(p);
    }
  }

  // Apply voxel grid only to near zone
  pcl::PointCloud<pcl::PointXYZ>::Ptr near_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  if (!near_cloud->empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(near_cloud);
    vg.setLeafSize(
      static_cast<float>(leaf_size_x_),
      static_cast<float>(leaf_size_y_),
      static_cast<float>(leaf_size_z_));
    vg.filter(*near_filtered);
  }

  // Merge: filtered near + raw far
  pcl::PointCloud<pcl::PointXYZ> merged;
  merged.reserve(near_filtered->size() + far_cloud->size());
  merged += *near_filtered;
  merged += *far_cloud;

  // Convert PCL → ROS2 msg
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(merged, output_msg);
  output_msg.header = msg->header;

  publisher_->publish(output_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "VoxelGrid: %zu total -> near %zu->%zu (filtered), far %zu (passthrough) -> %zu output",
    cloud->size(),
    near_cloud->size(), near_filtered->size(),
    far_cloud->size(), merged.size());
}

}  // namespace lidar_voxel_grid

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_voxel_grid::VoxelGridComponent)
