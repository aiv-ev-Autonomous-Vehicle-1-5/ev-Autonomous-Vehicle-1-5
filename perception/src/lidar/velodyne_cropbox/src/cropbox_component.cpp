#include "velodyne_cropbox/cropbox_component.hpp"

#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace velodyne_cropbox
{

CropBoxComponent::CropBoxComponent(const rclcpp::NodeOptions & options)
: Node("cropbox_component", options)
{
  // Declare parameters with default values
  x_min_ = this->declare_parameter("x_min", -10.0);
  x_max_ = this->declare_parameter("x_max", 10.0);
  y_min_ = this->declare_parameter("y_min", -10.0);
  y_max_ = this->declare_parameter("y_max", 10.0);
  z_min_ = this->declare_parameter("z_min", -2.0);
  z_max_ = this->declare_parameter("z_max", 2.0);
  negative_ = this->declare_parameter("negative", false);

  // Create subscription with QoS profile for sensor data
  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&CropBoxComponent::pointCloudCallback, this, std::placeholders::_1));

  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "CropBox initialized with range: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f], Negative: %s",
    x_min_, x_max_, y_min_, y_max_, z_min_, z_max_, negative_ ? "true" : "false");
}

void CropBoxComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const uint32_t num_points = msg->width * msg->height;
  if (num_points == 0) {
    publisher_->publish(*msg);
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // First pass: determine which points to keep
  std::vector<uint32_t> kept_indices;
  kept_indices.reserve(num_points);

  for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    bool inside = (x >= x_min_ && x <= x_max_ &&
                   y >= y_min_ && y <= y_max_ &&
                   z >= z_min_ && z <= z_max_);

    // negative=false: keep inside, negative=true: keep outside
    if (inside != negative_) {
      kept_indices.push_back(i);
    }
  }

  // Build output message preserving all original fields (ring, time, etc.)
  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.fields = msg->fields;
  output_msg.point_step = msg->point_step;
  output_msg.height = 1;
  output_msg.width = static_cast<uint32_t>(kept_indices.size());
  output_msg.row_step = output_msg.width * msg->point_step;
  output_msg.is_bigendian = msg->is_bigendian;
  output_msg.is_dense = msg->is_dense;
  output_msg.data.resize(output_msg.row_step);

  for (uint32_t out = 0; out < kept_indices.size(); ++out) {
    std::memcpy(
      &output_msg.data[out * msg->point_step],
      &msg->data[kept_indices[out] * msg->point_step],
      msg->point_step);
  }

  publisher_->publish(output_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "CropBox: Input %u pts -> Output %zu pts (%.1f%%)",
    num_points, kept_indices.size(),
    num_points > 0 ? (kept_indices.size() * 100.0 / num_points) : 0.0);
}

}  // namespace velodyne_cropbox

// Register the component with the ROS system to allow it to be dynamically loaded
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_cropbox::CropBoxComponent)
