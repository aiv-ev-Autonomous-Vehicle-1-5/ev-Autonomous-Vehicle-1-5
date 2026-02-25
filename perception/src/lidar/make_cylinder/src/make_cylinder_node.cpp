#include "make_cylinder/make_cylinder_node.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace make_cylinder
{

namespace
{

// cluster_splitter와 동일한 HSV 색상 hash
std_msgs::msg::ColorRGBA id_to_color(int32_t cid)
{
  std_msgs::msg::ColorRGBA c;
  c.a = 0.7F;
  if (cid < 0) {
    c.r = c.g = c.b = 0.5F;
    return c;
  }
  const int hue = (cid * 137 + 43) % 360;
  const float hf = static_cast<float>(hue) / 60.0F;
  const int sector = static_cast<int>(hf) % 6;
  const float frac = hf - static_cast<float>(static_cast<int>(hf));
  const float q = 1.0F - frac;
  const float t = frac;
  switch (sector) {
    case 0: c.r = 1.0F; c.g = t;    c.b = 0.0F; break;
    case 1: c.r = q;    c.g = 1.0F; c.b = 0.0F; break;
    case 2: c.r = 0.0F; c.g = 1.0F; c.b = t;    break;
    case 3: c.r = 0.0F; c.g = q;    c.b = 1.0F; break;
    case 4: c.r = t;    c.g = 0.0F; c.b = 1.0F; break;
    default: c.r = 1.0F; c.g = 0.0F; c.b = q;   break;
  }
  return c;
}

}  // namespace

MakeCylinderNode::MakeCylinderNode(const rclcpp::NodeOptions & options)
: Node("make_cylinder", options)
{
  auto input_topic  = declare_parameter<std::string>("input_topic", "/pointcloud/clustered_split");
  auto output_topic = declare_parameter<std::string>("output_topic", "/perception/cones");
  auto marker_topic = declare_parameter<std::string>("marker_topic", "/perception/cones_marker");
  sigmoid_k_    = declare_parameter<double>("sigmoid_k", 0.3);
  sigmoid_n_mid_ = declare_parameter<double>("sigmoid_n_mid", 15.0);

  auto qos = rclcpp::SensorDataQoS();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    std::bind(&MakeCylinderNode::callback, this, std::placeholders::_1));

  pub_cones_   = create_publisher<ev_msgs::msg::ConeArray>(output_topic, qos);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, qos);

  RCLCPP_INFO(get_logger(), "MakeCylinderNode initialized: %s -> %s + %s",
    input_topic.c_str(), output_topic.c_str(), marker_topic.c_str());
}

void MakeCylinderNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const size_t total = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (total == 0U) {
    auto out = std::make_unique<ev_msgs::msg::ConeArray>();
    out->header = msg->header;
    pub_cones_->publish(std::move(out));
    return;
  }

  // 클러스터별 포인트 집계
  struct ClusterStats {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    size_t count = 0;
    std::vector<std::pair<float, float>> xy_points;  // centroid 계산 후 radius용
  };

  std::unordered_map<int32_t, ClusterStats> clusters;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int32_t> it_cid(*msg, "cluster_id");

    for (size_t i = 0; i < total; ++i, ++it_x, ++it_y, ++it_z, ++it_cid) {
      const int32_t cid = *it_cid;
      if (cid < 0) continue;  // 노이즈 제외

      auto & s = clusters[cid];
      s.sum_x += static_cast<double>(*it_x);
      s.sum_y += static_cast<double>(*it_y);
      s.sum_z += static_cast<double>(*it_z);
      if (*it_z < s.min_z) s.min_z = *it_z;
      if (*it_z > s.max_z) s.max_z = *it_z;
      s.xy_points.emplace_back(*it_x, *it_y);
      ++s.count;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "PointCloud2 field error: %s", e.what());
    return;
  }

  // ConeArray 생성
  auto cone_msg = std::make_unique<ev_msgs::msg::ConeArray>();
  cone_msg->header = msg->header;
  cone_msg->cones.reserve(clusters.size());

  // MarkerArray 생성
  auto marker_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // DELETEALL 마커 (이전 프레임 잔상 제거)
  visualization_msgs::msg::Marker del;
  del.header = msg->header;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_msg->markers.push_back(del);

  int marker_id = 0;
  for (const auto & [cid, s] : clusters) {
    if (s.count == 0) continue;

    const double inv_n = 1.0 / static_cast<double>(s.count);
    const float cx = static_cast<float>(s.sum_x * inv_n);
    const float cy = static_cast<float>(s.sum_y * inv_n);
    const float cz = static_cast<float>(s.sum_z * inv_n);

    // radius: 수평 최대 거리
    float max_r2 = 0.0F;
    for (const auto & [px, py] : s.xy_points) {
      const float dx = px - cx;
      const float dy = py - cy;
      const float r2 = dx * dx + dy * dy;
      if (r2 > max_r2) max_r2 = r2;
    }
    const float radius = std::sqrt(max_r2);
    const float height = s.max_z - s.min_z;

    // sigmoid confidence
    const double n = static_cast<double>(s.count);
    const float confidence = static_cast<float>(
      1.0 / (1.0 + std::exp(-sigmoid_k_ * (n - sigmoid_n_mid_))));

    // Cone 메시지
    ev_msgs::msg::Cone cone;
    cone.position.x = static_cast<double>(cx);
    cone.position.y = static_cast<double>(cy);
    cone.position.z = static_cast<double>(cz);
    cone.radius = radius;
    cone.height = height;
    cone.confidence = confidence;
    cone.label = cid;
    cone_msg->cones.push_back(cone);

    // Cylinder 마커
    visualization_msgs::msg::Marker m;
    m.header = msg->header;
    m.ns = "cones";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = static_cast<double>(cx);
    m.pose.position.y = static_cast<double>(cy);
    m.pose.position.z = static_cast<double>(s.min_z) + static_cast<double>(height) * 0.5;
    m.pose.orientation.w = 1.0;
    m.scale.x = static_cast<double>(radius) * 2.0;  // 지름
    m.scale.y = static_cast<double>(radius) * 2.0;
    m.scale.z = static_cast<double>(height);
    m.color = id_to_color(cid);
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_msg->markers.push_back(m);
  }

  pub_cones_->publish(std::move(cone_msg));
  pub_markers_->publish(std::move(marker_msg));
}

}  // namespace make_cylinder

RCLCPP_COMPONENTS_REGISTER_NODE(make_cylinder::MakeCylinderNode)
