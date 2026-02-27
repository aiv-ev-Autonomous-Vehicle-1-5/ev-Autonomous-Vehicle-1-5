#include "make_bbox/make_bbox_node.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace make_bbox
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

MakeBBoxNode::MakeBBoxNode(const rclcpp::NodeOptions & options)
: Node("make_bbox", options)
{
  auto input_topic  = declare_parameter<std::string>("input_topic", "/pointcloud/clustered");
  auto output_topic = declare_parameter<std::string>("output_topic", "/perception/bboxes");
  auto marker_topic = declare_parameter<std::string>("marker_topic", "/perception/bboxes_marker");
  sigmoid_k_     = declare_parameter<double>("sigmoid_k", 0.3);
  sigmoid_n_mid_ = declare_parameter<double>("sigmoid_n_mid", 15.0);

  auto qos = rclcpp::SensorDataQoS();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    std::bind(&MakeBBoxNode::callback, this, std::placeholders::_1));

  pub_bboxes_  = create_publisher<ev_msgs::msg::BBoxArray>(output_topic, qos);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, qos);

  RCLCPP_INFO(get_logger(), "MakeBBoxNode initialized: %s -> %s + %s",
    input_topic.c_str(), output_topic.c_str(), marker_topic.c_str());
}

void MakeBBoxNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const size_t total = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (total == 0U) {
    auto out = std::make_unique<ev_msgs::msg::BBoxArray>();
    out->header = msg->header;
    pub_bboxes_->publish(std::move(out));
    return;
  }

  // 클러스터별 바운딩박스 통계
  struct ClusterStats {
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    size_t count = 0;
  };

  std::unordered_map<int32_t, ClusterStats> clusters;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int32_t> it_cid(*msg, "cluster_id");

    for (size_t i = 0; i < total; ++i, ++it_x, ++it_y, ++it_z, ++it_cid) {
      const int32_t cid = *it_cid;
      if (cid < 0) continue;

      auto & s = clusters[cid];
      if (*it_x < s.min_x) s.min_x = *it_x;
      if (*it_x > s.max_x) s.max_x = *it_x;
      if (*it_y < s.min_y) s.min_y = *it_y;
      if (*it_y > s.max_y) s.max_y = *it_y;
      if (*it_z < s.min_z) s.min_z = *it_z;
      if (*it_z > s.max_z) s.max_z = *it_z;
      ++s.count;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "PointCloud2 field error: %s", e.what());
    return;
  }

  // BBoxArray 생성
  auto bbox_msg = std::make_unique<ev_msgs::msg::BBoxArray>();
  bbox_msg->header = msg->header;
  bbox_msg->bboxes.reserve(clusters.size());

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

    // 바운딩박스 크기
    const float size_x = s.max_x - s.min_x;
    const float size_y = s.max_y - s.min_y;
    const float size_z = s.max_z - s.min_z;

    // 바운딩박스 중심
    const float cx = (s.min_x + s.max_x) * 0.5F;
    const float cy = (s.min_y + s.max_y) * 0.5F;
    const float cz = (s.min_z + s.max_z) * 0.5F;

    // sigmoid confidence
    const double n = static_cast<double>(s.count);
    const float confidence = static_cast<float>(
      1.0 / (1.0 + std::exp(-sigmoid_k_ * (n - sigmoid_n_mid_))));

    // BBox 메시지 (바운딩박스)
    ev_msgs::msg::BBox bbox;
    bbox.position.x = static_cast<double>(cx);
    bbox.position.y = static_cast<double>(cy);
    bbox.position.z = static_cast<double>(cz);
    bbox.size_x = size_x;
    bbox.size_y = size_y;
    bbox.size_z = size_z;
    bbox.confidence = confidence;
    bbox.label = cid;
    bbox_msg->bboxes.push_back(bbox);

    // LINE_LIST 마커 (와이어프레임 바운딩박스)
    visualization_msgs::msg::Marker m;
    m.header = msg->header;
    m.ns = "bboxes";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;  // 선 두께 (3cm)
    m.color = id_to_color(cid);
    m.color.a = 1.0F;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);

    // 8개 꼭짓점
    const double x0 = static_cast<double>(s.min_x), x1 = static_cast<double>(s.max_x);
    const double y0 = static_cast<double>(s.min_y), y1 = static_cast<double>(s.max_y);
    const double z0 = static_cast<double>(s.min_z), z1 = static_cast<double>(s.max_z);

    auto pt = [](double x, double y, double z) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = z;
      return p;
    };

    // 12개 모서리 (LINE_LIST: 점 2개가 한 쌍)
    // 바닥면
    m.points.push_back(pt(x0, y0, z0)); m.points.push_back(pt(x1, y0, z0));
    m.points.push_back(pt(x1, y0, z0)); m.points.push_back(pt(x1, y1, z0));
    m.points.push_back(pt(x1, y1, z0)); m.points.push_back(pt(x0, y1, z0));
    m.points.push_back(pt(x0, y1, z0)); m.points.push_back(pt(x0, y0, z0));
    // 윗면
    m.points.push_back(pt(x0, y0, z1)); m.points.push_back(pt(x1, y0, z1));
    m.points.push_back(pt(x1, y0, z1)); m.points.push_back(pt(x1, y1, z1));
    m.points.push_back(pt(x1, y1, z1)); m.points.push_back(pt(x0, y1, z1));
    m.points.push_back(pt(x0, y1, z1)); m.points.push_back(pt(x0, y0, z1));
    // 수직 기둥
    m.points.push_back(pt(x0, y0, z0)); m.points.push_back(pt(x0, y0, z1));
    m.points.push_back(pt(x1, y0, z0)); m.points.push_back(pt(x1, y0, z1));
    m.points.push_back(pt(x1, y1, z0)); m.points.push_back(pt(x1, y1, z1));
    m.points.push_back(pt(x0, y1, z0)); m.points.push_back(pt(x0, y1, z1));

    marker_msg->markers.push_back(m);
  }

  pub_bboxes_->publish(std::move(bbox_msg));
  pub_markers_->publish(std::move(marker_msg));
}

}  // namespace make_bbox

RCLCPP_COMPONENTS_REGISTER_NODE(make_bbox::MakeBBoxNode)
