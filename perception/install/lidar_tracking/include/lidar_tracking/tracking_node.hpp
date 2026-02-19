#ifndef LIDAR_TRACKING_TRACKING_NODE_HPP_
#define LIDAR_TRACKING_TRACKING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lidar_interfaces/msg/cone_array.hpp>
#include <lidar_interfaces/msg/tracked_cone_array.hpp>
#include "lidar_tracking/byte_tracker.hpp"

namespace lidar_tracking
{

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options);

private:
  void callback(const lidar_interfaces::msg::ConeArray::SharedPtr msg);

  // Subscribers & Publishers
  rclcpp::Subscription<lidar_interfaces::msg::ConeArray>::SharedPtr sub_;
  rclcpp::Publisher<lidar_interfaces::msg::TrackedConeArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Tracker
  std::unique_ptr<ByteTracker> tracker_;
  rclcpp::Time last_time_;
  bool initialized_ = false;

  // Parameters
  float dt_default_;
};

} // namespace lidar_tracking

#endif // LIDAR_TRACKING_TRACKING_NODE_HPP_
