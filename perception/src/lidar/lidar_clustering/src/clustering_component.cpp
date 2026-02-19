#include "lidar_clustering/clustering_component.hpp"

#include <cmath>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_clustering
{

// label → 고유 RGB 색상 변환 (HSV hue 기반)
static uint32_t labelToRGB(int32_t label)
{
  if (label < 0) {
    // noise: 어두운 회색
    return (80u << 16) | (80u << 8) | 80u;
  }

  // Golden angle(137.5°)로 hue 분산 → 인접 클러스터 색 차이 극대화
  float hue = std::fmod(label * 137.5f, 360.0f);
  float s = 0.9f, v = 0.95f;

  // HSV → RGB 변환
  float c = v * s;
  float x = c * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  float r, g, b;

  if (hue < 60)       { r = c; g = x; b = 0; }
  else if (hue < 120) { r = x; g = c; b = 0; }
  else if (hue < 180) { r = 0; g = c; b = x; }
  else if (hue < 240) { r = 0; g = x; b = c; }
  else if (hue < 300) { r = x; g = 0; b = c; }
  else                { r = c; g = 0; b = x; }

  auto R = static_cast<uint8_t>((r + m) * 255);
  auto G = static_cast<uint8_t>((g + m) * 255);
  auto B = static_cast<uint8_t>((b + m) * 255);

  return (static_cast<uint32_t>(R) << 16) |
         (static_cast<uint32_t>(G) << 8) |
          static_cast<uint32_t>(B);
}


ClusteringComponent::ClusteringComponent(const rclcpp::NodeOptions & options)
: Node("clustering_component", options)
{
  eps_ = this->declare_parameter("eps", 0.5);
  min_pts_ = this->declare_parameter("min_pts", 5);
  adaptive_range_ref_ = this->declare_parameter("adaptive_range_ref", 10.0);

  try {
    dbscan_gpu_ = std::make_unique<DBSCANGpu>(50000);
    RCLCPP_INFO(this->get_logger(), "GPU DBSCAN initialized (max 50000 points)");
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(this->get_logger(),
      "GPU DBSCAN init failed: %s. Shutting down.", e.what());
    rclcpp::shutdown();
    return;
  }

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&ClusteringComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "DBSCAN Clustering initialized: eps=%.2f, min_pts=%d, adaptive_range_ref=%.1fm (GPU)",
    eps_, min_pts_, adaptive_range_ref_);
}

void ClusteringComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const uint32_t num_points = msg->width * msg->height;
  if (num_points == 0) {
    publisher_->publish(*msg);
    return;
  }

  // Extract x, y, z from input PointCloud2
  std::vector<Point3D> points;
  points.reserve(num_points);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }

  // Adaptive eps: 거리에 비례하여 좌표 축소 → 고정 eps로 동일 효과
  // eps_effective(r) = eps * (1 + r / adaptive_range_ref)
  // 구현: 좌표를 1/(1 + r/ref)로 스케일 → 먼 포인트 간격이 좁아짐
  std::vector<Point3D> scaled_points(num_points);
  const float ref = static_cast<float>(adaptive_range_ref_);

  for (uint32_t i = 0; i < num_points; ++i) {
    float r = std::sqrt(points[i].x * points[i].x + points[i].y * points[i].y);
    float scale = 1.0f / (1.0f + r / ref);
    scaled_points[i] = {points[i].x * scale, points[i].y * scale, points[i].z * scale};
  }

  // Run GPU DBSCAN (스케일된 좌표 사용, 고정 eps)
  std::vector<int32_t> labels = dbscan_gpu_->cluster(scaled_points, eps_, min_pts_);

  // Build output PointCloud2 with x, y, z (FLOAT32) + label (INT32) + rgb (FLOAT32)
  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.height = 1;
  output_msg.width = num_points;
  output_msg.is_bigendian = false;
  output_msg.is_dense = true;

  sensor_msgs::msg::PointField field;
  field.count = 1;

  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  output_msg.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  output_msg.fields.push_back(field);

  field.name = "label";
  field.offset = 12;
  field.datatype = sensor_msgs::msg::PointField::INT32;
  output_msg.fields.push_back(field);

  // rgb 필드 (PCL packed float format - RViz2 "RGB8" Color Transformer 호환)
  field.name = "rgb";
  field.offset = 16;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);

  output_msg.point_step = 20;  // 5 * 4 bytes (x,y,z,label,rgb)
  output_msg.row_step = output_msg.point_step * num_points;
  output_msg.data.resize(output_msg.row_step);

  for (uint32_t i = 0; i < num_points; ++i) {
    const size_t offset = static_cast<size_t>(i) * 20;
    std::memcpy(&output_msg.data[offset + 0], &points[i].x, 4);
    std::memcpy(&output_msg.data[offset + 4], &points[i].y, 4);
    std::memcpy(&output_msg.data[offset + 8], &points[i].z, 4);
    std::memcpy(&output_msg.data[offset + 12], &labels[i], 4);

    // label → packed RGB float (PCL 호환 format)
    uint32_t rgb_packed = labelToRGB(labels[i]);
    float rgb_float;
    std::memcpy(&rgb_float, &rgb_packed, 4);
    std::memcpy(&output_msg.data[offset + 16], &rgb_float, 4);
  }

  publisher_->publish(output_msg);

  int32_t max_label = -1;
  int32_t noise_count = 0;
  for (const auto & l : labels) {
    if (l == -1) {
      ++noise_count;
    } else if (l > max_label) {
      max_label = l;
    }
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "DBSCAN[GPU]: %u pts -> %d clusters, %d noise",
    num_points, max_label + 1, noise_count);
}

}  // namespace lidar_clustering

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_clustering::ClusteringComponent)
