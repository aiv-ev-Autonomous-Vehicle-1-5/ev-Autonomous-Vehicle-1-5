#ifndef MAKE_BBOX__MAKE_BBOX_NODE_HPP_
#define MAKE_BBOX__MAKE_BBOX_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ev_msgs/msg/b_box_array.hpp>

namespace make_bbox
{

class MakeBBoxNode : public rclcpp::Node
{
public:
  explicit MakeBBoxNode(const rclcpp::NodeOptions & options);

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<ev_msgs::msg::BBoxArray>::SharedPtr pub_bboxes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  double sigmoid_k_;
  double sigmoid_n_mid_;
  float max_size_x_;
  float max_size_y_;
  float max_size_z_;
};

}  // namespace make_bbox

#endif  // MAKE_BBOX__MAKE_BBOX_NODE_HPP_
