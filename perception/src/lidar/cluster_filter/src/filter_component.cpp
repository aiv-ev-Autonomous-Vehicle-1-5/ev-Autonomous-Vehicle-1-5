#include "cluster_filter/filter_component.hpp"

#include <cstring>
#include <unordered_map>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace cluster_filter
{

// Cluster label → bright RGB color (20-color palette)
static float labelToRgb(int32_t label)
{
  static const uint32_t palette[] = {
    0xFF4444, 0x44FF44, 0x44AAFF, 0xFFFF44, 0xFF44FF,
    0x44FFFF, 0xFFAA44, 0xAA88FF, 0x88FFAA, 0xFF88CC,
    0xCCFF44, 0x44CCFF, 0xFF8888, 0x88FF88, 0xAAAAFF,
    0xFFCC44, 0xCC88FF, 0x88FFCC, 0xFF88FF, 0xCCFF88,
  };
  static const int palette_size = sizeof(palette) / sizeof(palette[0]);
  uint32_t packed = palette[label % palette_size];
  float rgb;
  std::memcpy(&rgb, &packed, sizeof(float));
  return rgb;
}

// Unpack palette color to r,g,b [0,1]
static void paletteColor(int32_t label, double & r, double & g, double & b)
{
  static const uint32_t palette[] = {
    0xFF4444, 0x44FF44, 0x44AAFF, 0xFFFF44, 0xFF44FF,
    0x44FFFF, 0xFFAA44, 0xAA88FF, 0x88FFAA, 0xFF88CC,
    0xCCFF44, 0x44CCFF, 0xFF8888, 0x88FF88, 0xAAAAFF,
    0xFFCC44, 0xCC88FF, 0x88FFCC, 0xFF88FF, 0xCCFF88,
  };
  static const int palette_size = sizeof(palette) / sizeof(palette[0]);
  uint32_t packed = palette[label % palette_size];
  r = ((packed >> 16) & 0xFF) / 255.0;
  g = ((packed >>  8) & 0xFF) / 255.0;
  b = ( packed        & 0xFF) / 255.0;
}

FilterComponent::FilterComponent(const rclcpp::NodeOptions & options)
: Node("filter_component", options)
{
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 5);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 5000);
  flatness_threshold_ = this->declare_parameter("flatness_threshold", 0.05);
  min_height_ = this->declare_parameter("min_height", 0.3);
  max_height_ = this->declare_parameter("max_height", 0.9);
  min_width_ = this->declare_parameter("min_width", 0.1);
  max_width_ = this->declare_parameter("max_width", 0.5);

  auto qos = rclcpp::SensorDataQoS();
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&FilterComponent::callback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos); // 뭉쳐진 점들 색깔별로
  cone_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cones", 10); // 원기둥
  cone_data_pub_ = this->create_publisher<lidar_interfaces::msg::ConeArray>("/lidar/cones_detected", 10); // tracking node로 넘겨주는 msg

  RCLCPP_INFO(this->get_logger(),
    "ClusterFilter initialized: size=[%d,%d], height=[%.2f,%.2f], width=[%.2f,%.2f], flatness<%.3f",
    min_cluster_size_, max_cluster_size_,
    min_height_, max_height_, min_width_, max_width_, flatness_threshold_);
}

void FilterComponent::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const uint32_t num_points = msg->width * msg->height;
  if (num_points == 0) {
    pub_->publish(*msg);
    return;
  }

  // Read x, y, z, label from input
  struct PointLabel { float x, y, z; int32_t label; };
  std::vector<PointLabel> points(num_points);

  sensor_msgs::PointCloud2ConstIterator<float>   it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float>   it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float>   it_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<int32_t> it_l(*msg, "label");

  for (uint32_t i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z, ++it_l) {
    points[i] = {*it_x, *it_y, *it_z, *it_l};
  }

  // Group point indices by cluster label
  std::unordered_map<int32_t, std::vector<uint32_t>> cluster_indices;
  int32_t noise_count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (points[i].label == -1) {
      ++noise_count;
    } else {
      cluster_indices[points[i].label].push_back(i);
    }
  }

  // AABB per valid cluster
  struct BBox {
    float x_min, y_min, z_min;
    float x_max, y_max, z_max;
  };

  std::unordered_map<int32_t, bool>  valid_clusters;
  std::unordered_map<int32_t, BBox>  valid_bboxes;
  int valid_cluster_count   = 0;
  int filtered_cluster_count = 0;

  for (const auto & [label, indices] : cluster_indices) {
    const int size = static_cast<int>(indices.size());

    // 1. Size filter
    if (size < min_cluster_size_ || size > max_cluster_size_) {
      ++filtered_cluster_count;
      continue;
    }

    // 2. Bounding box computation
    float x_min = std::numeric_limits<float>::max();
    float y_min = x_min, z_min = x_min;
    float x_max = std::numeric_limits<float>::lowest();
    float y_max = x_max, z_max = x_max;

    for (uint32_t idx : indices) {
      const auto & p = points[idx];
      x_min = std::min(x_min, p.x); x_max = std::max(x_max, p.x);
      y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y);
      z_min = std::min(z_min, p.z); z_max = std::max(z_max, p.z);
    }

    float height  = z_max - z_min;
    float width_x = x_max - x_min;
    float width_y = y_max - y_min;

    if (height < min_height_ || height > max_height_ ||
        width_x > max_width_ || width_y > max_width_) {
      ++filtered_cluster_count;
      continue;
    }

    // 3. Flatness filter (PCA) — ground remnant detection
    if (size >= 3) {
      Eigen::MatrixXf mat(size, 3);
      for (int j = 0; j < size; ++j) {
        const auto & p = points[indices[j]];
        mat(j, 0) = p.x;
        mat(j, 1) = p.y;
        mat(j, 2) = p.z;
      }
      Eigen::Vector3f mean = mat.colwise().mean();
      mat.rowwise() -= mean.transpose();
      Eigen::Matrix3f cov = (mat.transpose() * mat) / static_cast<float>(size - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
      Eigen::Vector3f eigenvalues = solver.eigenvalues();

      float lambda_sum = eigenvalues.sum();
      if (lambda_sum > 0.0f) {
        float flatness = eigenvalues(0) / lambda_sum;
        if (flatness < flatness_threshold_) {
          Eigen::Vector3f normal = solver.eigenvectors().col(0);
          if (std::abs(normal.z()) > 0.7f) {
            ++filtered_cluster_count;
            continue;  // ground remnant
          }
        }
      }
    }

    valid_clusters[label] = true;
    valid_bboxes[label]   = {x_min, y_min, z_min, x_max, y_max, z_max};
    ++valid_cluster_count;
  }

  // --- Publish filtered PointCloud2 ---
  uint32_t out_count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (points[i].label >= 0 && valid_clusters.count(points[i].label)) {
      ++out_count;
    }
  }

  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header      = msg->header;
  output_msg.height      = 1;
  output_msg.width       = out_count;
  output_msg.is_bigendian = false;
  output_msg.is_dense    = true;

  sensor_msgs::msg::PointField field;
  field.count = 1;
  field.name = "x";   field.offset = 0;  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);
  field.name = "y";   field.offset = 4;
  output_msg.fields.push_back(field);
  field.name = "z";   field.offset = 8;
  output_msg.fields.push_back(field);
  field.name = "label"; field.offset = 12; field.datatype = sensor_msgs::msg::PointField::INT32;
  output_msg.fields.push_back(field);
  field.name = "rgb"; field.offset = 16; field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);

  output_msg.point_step = 20;
  output_msg.row_step   = output_msg.point_step * out_count;
  output_msg.data.resize(output_msg.row_step);

  uint32_t out_idx = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (points[i].label < 0 || !valid_clusters.count(points[i].label)) {
      continue;
    }
    const size_t off = static_cast<size_t>(out_idx) * 20;
    float rgb = labelToRgb(points[i].label);
    std::memcpy(&output_msg.data[off + 0],  &points[i].x,     4);
    std::memcpy(&output_msg.data[off + 4],  &points[i].y,     4);
    std::memcpy(&output_msg.data[off + 8],  &points[i].z,     4);
    std::memcpy(&output_msg.data[off + 12], &points[i].label, 4);
    std::memcpy(&output_msg.data[off + 16], &rgb,             4);
    ++out_idx;
  }
  pub_->publish(output_msg);

  // --- Publish AABB MarkerArray & ConeArray Data ---
  visualization_msgs::msg::MarkerArray marker_array;
  lidar_interfaces::msg::ConeArray cone_array_msg;
  cone_array_msg.header = msg->header;

  // Delete all previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);

  int marker_id = 0;
  for (const auto & [label, bbox] : valid_bboxes) {
    float cx = (bbox.x_min + bbox.x_max) * 0.5f;
    float cy = (bbox.y_min + bbox.y_max) * 0.5f;
    float cz = (bbox.z_min + bbox.z_max) * 0.5f;
    float sx = bbox.x_max - bbox.x_min;
    float sy = bbox.y_max - bbox.y_min;
    float sz = bbox.z_max - bbox.z_min;

    // Create Cone message
    lidar_interfaces::msg::Cone cone;
    cone.header = msg->header;
    cone.position.x = cx;
    cone.position.y = cy;
    cone.position.z = cz;
    cone.dimensions.x = sx;
    cone.dimensions.y = sy;
    cone.dimensions.z = sz;
    cone.label = label;
    // Simple confidence assignment based on cluster properties (placeholder logic)
    // In real app, you might use point density or model matching score.
    cone.confidence = 1.0f; 
    cone_array_msg.cones.push_back(cone);

    double r, g, b;
    paletteColor(label, r, g, b);

    // Cylinder marker — diameter = max(sx, sy), height = sz
    float diameter = std::max(sx, sy);
    float z1 = bbox.z_max;

    visualization_msgs::msg::Marker m;
    m.header    = msg->header;
    m.ns        = "cone";
    m.id        = marker_id++;
    m.type      = visualization_msgs::msg::Marker::CYLINDER;
    m.action    = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x  = cx;
    m.pose.position.y  = cy;
    m.pose.position.z  = cz;
    m.pose.orientation.w = 1.0;
    m.scale.x   = diameter;
    m.scale.y   = diameter;
    m.scale.z   = sz;
    m.color.r   = static_cast<float>(r);
    m.color.g   = static_cast<float>(g);
    m.color.b   = static_cast<float>(b);
    m.color.a   = 0.5f;  // semi-transparent
    m.lifetime  = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(m);

    // Text label above the cylinder
    visualization_msgs::msg::Marker text;
    text.header   = msg->header;
    text.ns       = "cone_text";
    text.id       = marker_id++;
    text.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action   = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = cx;
    text.pose.position.y = cy;
    text.pose.position.z = z1 + 0.1f;
    text.pose.orientation.w = 1.0;
    text.scale.z  = 0.15;
    text.color.r  = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
    text.lifetime = rclcpp::Duration::from_seconds(0.2);
    text.text     = "C" + std::to_string(label) +
                    " [" + std::to_string(static_cast<int>(sx * 100)) + "x" +
                           std::to_string(static_cast<int>(sy * 100)) + "x" +
                           std::to_string(static_cast<int>(sz * 100)) + "cm]";
    marker_array.markers.push_back(text);
  }

  cone_pub_->publish(marker_array);
  cone_data_pub_->publish(cone_array_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Filter: %u pts -> %d clusters (%d filtered), %d noise, %u output",
    num_points, valid_cluster_count, filtered_cluster_count,
    noise_count, out_count);
}

}  // namespace cluster_filter

RCLCPP_COMPONENTS_REGISTER_NODE(cluster_filter::FilterComponent)
