#ifndef MAKE_CYLINDER__MAKE_CYLINDER_NODE_HPP_
#define MAKE_CYLINDER__MAKE_CYLINDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ev_msgs/msg/cone_array.hpp>

namespace make_cylinder
{

class MakeCylinderNode : public rclcpp::Node
{
public:
  explicit MakeCylinderNode(const rclcpp::NodeOptions & options);

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<ev_msgs::msg::ConeArray>::SharedPtr pub_cones_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  double sigmoid_k_;
  double sigmoid_n_mid_;
};

}  // namespace make_cylinder

#endif  // MAKE_CYLINDER__MAKE_CYLINDER_NODE_HPP_
