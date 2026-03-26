#include "make_bbox/make_bbox_node.hpp"

#include <algorithm>
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
  max_size_x_    = static_cast<float>(declare_parameter<double>("max_size_x", 0.55));
  max_size_y_    = static_cast<float>(declare_parameter<double>("max_size_y", 0.55));
  max_size_z_    = static_cast<float>(declare_parameter<double>("max_size_z", 0.89));
  min_size_z_    = static_cast<float>(declare_parameter<double>("min_size_z", 0.12));
  max_center_z_  = static_cast<float>(declare_parameter<double>("max_center_z", 0.5));
  min_center_z_  = static_cast<float>(declare_parameter<double>("min_center_z", -1.2));
  max_base_height_ratio_ = static_cast<float>(declare_parameter<double>("max_base_height_ratio", 0.9));

  // 클러스터 분할 파라미터
  enable_split_          = declare_parameter<bool>("enable_cluster_split", true);
  split_cone_diameter_   = static_cast<float>(declare_parameter<double>("split_cone_diameter_m", 0.5));
  split_kmeans_max_iter_ = declare_parameter<int>("split_kmeans_max_iter", 15);
  split_min_points_      = declare_parameter<int>("split_min_points", 5);

  // 분할 ROI (base_link 기준)
  split_roi_x_min_ = static_cast<float>(declare_parameter<double>("split_roi_x_min", 0.8));
  split_roi_x_max_ = static_cast<float>(declare_parameter<double>("split_roi_x_max", 6.8));
  split_roi_y_min_ = static_cast<float>(declare_parameter<double>("split_roi_y_min", -2.0));
  split_roi_y_max_ = static_cast<float>(declare_parameter<double>("split_roi_y_max", 2.0));

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

  // ── 1단계: 클러스터별 포인트 수집 ──
  struct Point3 { float x, y, z; };
  std::unordered_map<int32_t, std::vector<Point3>> cluster_points;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int32_t> it_cid(*msg, "cluster_id");

    for (size_t i = 0; i < total; ++i, ++it_x, ++it_y, ++it_z, ++it_cid) {
      const int32_t cid = *it_cid;
      if (cid < 0) continue;
      cluster_points[cid].push_back({*it_x, *it_y, *it_z});
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "PointCloud2 field error: %s", e.what());
    return;
  }

  // ── 2단계: oversized 클러스터 K-means 분할 ──
  // 분할 결과를 최종 클러스터 리스트에 담는다
  struct ClusterData {
    std::vector<Point3> points;
    int32_t label;  // 시각화용 label
  };
  std::vector<ClusterData> final_clusters;
  final_clusters.reserve(cluster_points.size());

  int32_t next_label = 0;

  for (auto & [cid, pts] : cluster_points) {
    if (pts.size() < 2 || !enable_split_) {
      // 분할 불필요
      final_clusters.push_back({std::move(pts), next_label++});
      continue;
    }

    // XY 바운딩박스 계산
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
    for (const auto & p : pts) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    const float range_x = max_x - min_x;
    const float range_y = max_y - min_y;
    const float longest = std::max(range_x, range_y);

    // ROI 체크: 클러스터 중심이 분할 ROI 밖이면 분할 스킵
    const float cent_x = (min_x + max_x) * 0.5F;
    const float cent_y = (min_y + max_y) * 0.5F;
    if (cent_x < split_roi_x_min_ || cent_x > split_roi_x_max_ ||
        cent_y < split_roi_y_min_ || cent_y > split_roi_y_max_) {
      final_clusters.push_back({std::move(pts), next_label++});
      continue;
    }

    // 분할이 필요한지 판단: 최장축이 콘 지름보다 커야 함
    const int k = static_cast<int>(std::round(longest / split_cone_diameter_));
    if (k < 2) {
      final_clusters.push_back({std::move(pts), next_label++});
      continue;
    }

    // K-means (XY만 사용)
    // 초기 centroid: 최장축을 따라 k등분 위치에 배치
    const bool split_along_x = (range_x >= range_y);
    std::vector<float> cx(k), cy(k);
    for (int j = 0; j < k; ++j) {
      const float t = (static_cast<float>(j) + 0.5F) / static_cast<float>(k);
      cx[j] = split_along_x ? (min_x + t * range_x) : ((min_x + max_x) * 0.5F);
      cy[j] = split_along_x ? ((min_y + max_y) * 0.5F) : (min_y + t * range_y);
    }

    const size_t n = pts.size();
    std::vector<int> assign(n, 0);

    for (int iter = 0; iter < split_kmeans_max_iter_; ++iter) {
      // 할당: 각 포인트를 가장 가까운 centroid에
      bool changed = false;
      for (size_t i = 0; i < n; ++i) {
        float best_d2 = std::numeric_limits<float>::max();
        int best_j = 0;
        for (int j = 0; j < k; ++j) {
          const float dx = pts[i].x - cx[j];
          const float dy = pts[i].y - cy[j];
          const float d2 = dx * dx + dy * dy;
          if (d2 < best_d2) { best_d2 = d2; best_j = j; }
        }
        if (assign[i] != best_j) { assign[i] = best_j; changed = true; }
      }
      if (!changed) break;

      // centroid 재계산
      std::vector<float> sum_x(k, 0.0F), sum_y(k, 0.0F);
      std::vector<int> cnt(k, 0);
      for (size_t i = 0; i < n; ++i) {
        sum_x[assign[i]] += pts[i].x;
        sum_y[assign[i]] += pts[i].y;
        ++cnt[assign[i]];
      }
      for (int j = 0; j < k; ++j) {
        if (cnt[j] > 0) {
          cx[j] = sum_x[j] / static_cast<float>(cnt[j]);
          cy[j] = sum_y[j] / static_cast<float>(cnt[j]);
        }
      }
    }

    // 서브클러스터를 final_clusters에 추가
    std::vector<std::vector<Point3>> sub(k);
    for (size_t i = 0; i < n; ++i) {
      sub[assign[i]].push_back(pts[i]);
    }
    for (int j = 0; j < k; ++j) {
      if (static_cast<int>(sub[j].size()) >= split_min_points_) {
        final_clusters.push_back({std::move(sub[j]), next_label++});
      }
    }
  }

  // ── 3단계: 바운딩박스 & 마커 생성 ──
  auto bbox_msg = std::make_unique<ev_msgs::msg::BBoxArray>();
  bbox_msg->header = msg->header;
  bbox_msg->bboxes.reserve(final_clusters.size());

  auto marker_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker del;
  del.header = msg->header;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_msg->markers.push_back(del);

  int marker_id = 0;
  for (const auto & cl : final_clusters) {
    // 바운딩박스 통계
    float bmin_x = std::numeric_limits<float>::max(), bmax_x = std::numeric_limits<float>::lowest();
    float bmin_y = std::numeric_limits<float>::max(), bmax_y = std::numeric_limits<float>::lowest();
    float bmin_z = std::numeric_limits<float>::max(), bmax_z = std::numeric_limits<float>::lowest();
    for (const auto & p : cl.points) {
      bmin_x = std::min(bmin_x, p.x); bmax_x = std::max(bmax_x, p.x);
      bmin_y = std::min(bmin_y, p.y); bmax_y = std::max(bmax_y, p.y);
      bmin_z = std::min(bmin_z, p.z); bmax_z = std::max(bmax_z, p.z);
    }

    const float size_x = bmax_x - bmin_x;
    const float size_y = bmax_y - bmin_y;
    const float size_z = bmax_z - bmin_z;

    // 크기 필터: 납작한 바닥 노이즈 제거 + 라바콘보다 큰 물체 탈락
    if (size_z < min_size_z_ ||
        size_x > max_size_x_ || size_y > max_size_y_ || size_z > max_size_z_) {
      continue;
    }

    // 밑변/높이 비율 필터: 라바콘은 높이 > 밑변이므로 ratio가 낮음
    // 넓적한 비-라바콘 클러스터(ratio >= threshold)를 제거
    const float base = std::max(size_x, size_y);
    const float ratio = base / (size_z + 1e-6F);
    if (ratio > max_base_height_ratio_) {
      continue;
    }

    const float cx = (bmin_x + bmax_x) * 0.5F;
    const float cy = (bmin_y + bmax_y) * 0.5F;
    const float cz = (bmin_z + bmax_z) * 0.5F;

    // 클러스터 중심 z 필터: 너무 높거나 낮은 물체 제거
    if (cz > max_center_z_ || cz < min_center_z_) {
      continue;
    }

    ev_msgs::msg::BBox bbox;
    bbox.position.x = static_cast<double>(cx);
    bbox.position.y = static_cast<double>(cy);
    bbox.position.z = static_cast<double>(cz);
    bbox.size_x = size_x;
    bbox.size_y = size_y;
    bbox.size_z = size_z;
    bbox.label = cl.label;
    bbox_msg->bboxes.push_back(bbox);

    // LINE_LIST 마커 (와이어프레임 바운딩박스)
    visualization_msgs::msg::Marker m;
    m.header = msg->header;
    m.ns = "bboxes";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;
    m.color = id_to_color(cl.label);
    m.color.a = 1.0F;


    const double x0 = static_cast<double>(bmin_x), x1 = static_cast<double>(bmax_x);
    const double y0 = static_cast<double>(bmin_y), y1 = static_cast<double>(bmax_y);
    const double z0 = static_cast<double>(bmin_z), z1 = static_cast<double>(bmax_z);

    auto pt = [](double x, double y, double z) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = z;
      return p;
    };

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
