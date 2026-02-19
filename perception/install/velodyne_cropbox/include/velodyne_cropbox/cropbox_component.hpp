#ifndef VELODYNE_CROPBOX__CROPBOX_COMPONENT_HPP_
#define VELODYNE_CROPBOX__CROPBOX_COMPONENT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace velodyne_cropbox
{

class CropBoxComponent : public rclcpp::Node
{
public:
  explicit CropBoxComponent(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  // CropBox parameters
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
  bool negative_;  // If true, remove points inside the box instead of outside
};

}  // namespace velodyne_cropbox

#endif  // VELODYNE_CROPBOX__CROPBOX_COMPONENT_HPP_
