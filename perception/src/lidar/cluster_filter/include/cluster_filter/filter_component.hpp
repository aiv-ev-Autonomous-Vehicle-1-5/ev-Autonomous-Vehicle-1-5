#ifndef CLUSTER_FILTER__FILTER_COMPONENT_HPP_
#define CLUSTER_FILTER__FILTER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_interfaces/msg/cone_array.hpp"



namespace cluster_filter

{

class FilterComponent : public rclcpp::Node

{

public:

  explicit FilterComponent(const rclcpp::NodeOptions & options);



private:

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);



  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_pub_;

  rclcpp::Publisher<lidar_interfaces::msg::ConeArray>::SharedPtr cone_data_pub_;



  int min_cluster_size_;

  int max_cluster_size_;

  double flatness_threshold_;

  double min_height_, max_height_;

  double min_width_, max_width_;

};

}  // namespace cluster_filter



#endif  // CLUSTER_FILTER__FILTER_COMPONENT_HPP_
