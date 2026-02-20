#include "cluster_splitter/cluster_splitter_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace cluster_splitter
{

namespace
{

double squared_distance_xy(
  const ClusterSplitterNode::PointRecord & p,
  const std::pair<double, double> & c)
{
  const double dx = static_cast<double>(p.x) - c.first;
  const double dy = static_cast<double>(p.y) - c.second;
  return dx * dx + dy * dy;
}

std::pair<double, double> centroid_xy(
  const std::vector<ClusterSplitterNode::PointRecord> & points,
  const std::vector<size_t> & indices)
{
  if (indices.empty()) {
    return {0.0, 0.0};
  }

  double sx = 0.0;
  double sy = 0.0;
  for (const size_t idx : indices) {
    sx += static_cast<double>(points[idx].x);
    sy += static_cast<double>(points[idx].y);
  }
  const double inv = 1.0 / static_cast<double>(indices.size());
  return {sx * inv, sy * inv};
}

}  // namespace

ClusterSplitterNode::ClusterSplitterNode(const rclcpp::NodeOptions & options)
: Node("cluster_splitter", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/pointcloud/clustered");
  output_topic_ = declare_parameter<std::string>("output_topic", "/pointcloud/clustered_split");

  min_points_to_consider_split_ = declare_parameter<int>("min_points_to_consider_split", 24);
  oversized_point_count_ = declare_parameter<int>("oversized_point_count", 80);
  min_subcluster_points_ = declare_parameter<int>("min_subcluster_points", 8);
  max_recursive_depth_ = declare_parameter<int>("max_recursive_depth", 4);
  kmeans_max_iterations_ = declare_parameter<int>("kmeans_max_iterations", 20);

  oversized_axis_length_m_ = declare_parameter<double>("oversized_axis_length_m", 0.9);
  valley_min_gap_m_ = declare_parameter<double>("valley_min_gap_m", 0.20);
  kmeans_min_centroid_distance_m_ =
    declare_parameter<double>("kmeans_min_centroid_distance_m", 0.25);
  preserve_noise_points_ = declare_parameter<bool>("preserve_noise_points", true);
  enable_kmeans_fallback_ = declare_parameter<bool>("enable_kmeans_fallback", true);

  enable_cone_validation_ = declare_parameter<bool>("enable_cone_validation", true);
  cone_diameter_m_ = declare_parameter<double>("cone_diameter_m", 0.607);
  cone_max_width_m_ = declare_parameter<double>("cone_max_width_m", 0.75);
  cone_min_width_m_ = declare_parameter<double>("cone_min_width_m", 0.10);
  cone_max_height_m_ = declare_parameter<double>("cone_max_height_m", 1.3);
  cone_min_height_m_ = declare_parameter<double>("cone_min_height_m", 0.15);
  cone_min_points_ = declare_parameter<int>("cone_min_points", 5);

  auto qos = rclcpp::SensorDataQoS();
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    qos,
    std::bind(&ClusterSplitterNode::callback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  RCLCPP_INFO(
    get_logger(),
    "cluster_splitter started in:%s out:%s axis_th:%.2f point_th:%d gap_th:%.2f kmeans:%s cone_validate:%s",
    input_topic_.c_str(), output_topic_.c_str(), oversized_axis_length_m_, oversized_point_count_,
    valley_min_gap_m_,
    enable_kmeans_fallback_ ? "on" : "off",
    enable_cone_validation_ ? "on" : "off");
}

void ClusterSplitterNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const size_t total_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (total_points == 0U) {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    pub_->publish(out);
    return;
  }

  std::vector<PointRecord> points;
  points.reserve(total_points);

  std::vector<int32_t> original_labels;
  original_labels.reserve(total_points);

  std::unordered_map<int32_t, std::vector<size_t>> cluster_to_indices;
  cluster_to_indices.reserve(total_points / 8U + 1U);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int32_t> it_cluster(*msg, "cluster_id");

    for (size_t i = 0; i < total_points; ++i, ++it_x, ++it_y, ++it_z, ++it_cluster) {
      points.push_back(PointRecord{*it_x, *it_y, *it_z});
      original_labels.push_back(*it_cluster);
      if (*it_cluster >= 0) {
        cluster_to_indices[*it_cluster].push_back(i);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "PointCloud2 missing required fields (x,y,z,cluster_id): %s", e.what());
    return;
  }

  std::vector<int32_t> relabeled(points.size(), -1);

  std::vector<int32_t> source_cluster_ids;
  source_cluster_ids.reserve(cluster_to_indices.size());
  for (const auto & kv : cluster_to_indices) {
    source_cluster_ids.push_back(kv.first);
  }
  std::sort(source_cluster_ids.begin(), source_cluster_ids.end());

  int32_t next_cluster_id = 0;
  for (const int32_t source_id : source_cluster_ids) {
    const auto & indices = cluster_to_indices[source_id];
    if (indices.empty()) {
      continue;
    }

    std::vector<std::vector<size_t>> split_groups;
    split_groups.reserve(4);
    split_cluster_recursive(points, indices, 0, split_groups);

    for (const auto & group : split_groups) {
      if (group.empty()) {
        continue;
      }
      for (const size_t idx : group) {
        relabeled[idx] = next_cluster_id;
      }
      ++next_cluster_id;
    }
  }

  if (enable_cone_validation_) {
    validate_cone_clusters(points, relabeled, next_cluster_id);
  }

  std::vector<size_t> output_indices;
  output_indices.reserve(points.size());
  if (preserve_noise_points_) {
    for (size_t i = 0; i < points.size(); ++i) {
      output_indices.push_back(i);
    }
  } else {
    for (size_t i = 0; i < points.size(); ++i) {
      if (original_labels[i] >= 0) {
        output_indices.push_back(i);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 out;
  out.header = msg->header;
  out.height = 1;
  out.width = static_cast<uint32_t>(output_indices.size());
  out.is_bigendian = msg->is_bigendian;
  out.is_dense = false;

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1;

  sensor_msgs::msg::PointField fy;
  fy.name = "y";
  fy.offset = 4;
  fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fy.count = 1;

  sensor_msgs::msg::PointField fz;
  fz.name = "z";
  fz.offset = 8;
  fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fz.count = 1;

  sensor_msgs::msg::PointField fcluster;
  fcluster.name = "cluster_id";
  fcluster.offset = 12;
  fcluster.datatype = sensor_msgs::msg::PointField::INT32;
  fcluster.count = 1;

  sensor_msgs::msg::PointField frgb;
  frgb.name = "rgb";
  frgb.offset = 16;
  frgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
  frgb.count = 1;

  out.fields = {fx, fy, fz, fcluster, frgb};
  out.point_step = 20;
  out.row_step = out.point_step * out.width;
  out.data.resize(static_cast<size_t>(out.row_step));

  // cluster_id → RGB (간단 hash, near_radius 없음)
  auto id_to_packed_rgb = [](int32_t cid) -> float {
    uint32_t rgb;
    if (cid < 0) {
      rgb = (128U << 16) | (128U << 8) | 128U;
    } else {
      const int hue = (cid * 137 + 43) % 360;
      const float hf = static_cast<float>(hue) / 60.0F;
      const int sector = static_cast<int>(hf) % 6;
      const float frac = hf - static_cast<float>(static_cast<int>(hf));
      const float q = 1.0F - frac;
      const float t = frac;
      float r, g, b;
      switch (sector) {
        case 0: r = 1.0F; g = t;    b = 0.0F; break;
        case 1: r = q;    g = 1.0F; b = 0.0F; break;
        case 2: r = 0.0F; g = 1.0F; b = t;    break;
        case 3: r = 0.0F; g = q;    b = 1.0F; break;
        case 4: r = t;    g = 0.0F; b = 1.0F; break;
        default: r = 1.0F; g = 0.0F; b = q;   break;
      }
      rgb = (static_cast<uint32_t>(r * 255.0F) << 16) |
            (static_cast<uint32_t>(g * 255.0F) << 8) |
             static_cast<uint32_t>(b * 255.0F);
    }
    float f;
    std::memcpy(&f, &rgb, sizeof(float));
    return f;
  };

  // 클러스터별 색 미리 계산
  std::vector<float> cluster_rgb(static_cast<size_t>(next_cluster_id));
  for (int32_t cid = 0; cid < next_cluster_id; ++cid) {
    cluster_rgb[static_cast<size_t>(cid)] = id_to_packed_rgb(cid);
  }
  const float noise_rgb = id_to_packed_rgb(-1);

  sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
  sensor_msgs::PointCloud2Iterator<int32_t> out_cluster(out, "cluster_id");
  sensor_msgs::PointCloud2Iterator<float> out_rgb(out, "rgb");

  for (const size_t idx : output_indices) {
    const auto & p = points[idx];
    const int32_t label = relabeled[idx];

    *out_x = p.x;
    ++out_x;
    *out_y = p.y;
    ++out_y;
    *out_z = p.z;
    ++out_z;
    *out_cluster = label;
    ++out_cluster;
    *out_rgb = (label >= 0 && label < next_cluster_id)
      ? cluster_rgb[static_cast<size_t>(label)] : noise_rgb;
    ++out_rgb;
  }

  pub_->publish(out);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "splitter in_clusters:%zu out_clusters:%d points_in:%zu points_out:%zu",
    source_cluster_ids.size(), static_cast<int>(next_cluster_id), points.size(), output_indices.size());
}

void ClusterSplitterNode::split_cluster_recursive(
  const std::vector<PointRecord> & points,
  const std::vector<size_t> & indices,
  int depth,
  std::vector<std::vector<size_t>> & split_groups) const
{
  if (indices.empty()) {
    return;
  }

  if (
    depth >= max_recursive_depth_ ||
    indices.size() < static_cast<size_t>(min_points_to_consider_split_))
  {
    split_groups.push_back(indices);
    return;
  }

  const AxisStats stats = compute_axis_stats(points, indices);
  if (!is_oversized_cluster(indices, stats)) {
    split_groups.push_back(indices);
    return;
  }

  std::vector<size_t> left;
  std::vector<size_t> right;

  bool split_success = split_by_projection_gap(indices, stats, left, right);
  if (!split_success && enable_kmeans_fallback_) {
    split_success = split_by_kmeans(points, indices, left, right);
  }

  if (
    !split_success ||
    left.size() < static_cast<size_t>(min_subcluster_points_) ||
    right.size() < static_cast<size_t>(min_subcluster_points_))
  {
    split_groups.push_back(indices);
    return;
  }

  split_cluster_recursive(points, left, depth + 1, split_groups);
  split_cluster_recursive(points, right, depth + 1, split_groups);
}

bool ClusterSplitterNode::split_by_projection_gap(
  const std::vector<size_t> & indices,
  const AxisStats & stats,
  std::vector<size_t> & left,
  std::vector<size_t> & right) const
{
  if (indices.size() < 2U) {
    return false;
  }

  std::vector<std::pair<double, size_t>> sorted = stats.projection;

  std::sort(
    sorted.begin(), sorted.end(),
    [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

  size_t best_split_idx = std::numeric_limits<size_t>::max();
  double best_gap = 0.0;

  for (size_t i = 0; i + 1U < sorted.size(); ++i) {
    const size_t left_count = i + 1U;
    const size_t right_count = sorted.size() - left_count;
    if (
      left_count < static_cast<size_t>(min_subcluster_points_) ||
      right_count < static_cast<size_t>(min_subcluster_points_))
    {
      continue;
    }

    const double gap = sorted[i + 1U].first - sorted[i].first;
    if (gap >= valley_min_gap_m_ && gap > best_gap) {
      best_gap = gap;
      best_split_idx = i;
    }
  }

  if (best_split_idx == std::numeric_limits<size_t>::max()) {
    return false;
  }

  left.clear();
  right.clear();
  left.reserve(best_split_idx + 1U);
  right.reserve(sorted.size() - (best_split_idx + 1U));

  for (size_t i = 0; i < sorted.size(); ++i) {
    if (i <= best_split_idx) {
      left.push_back(sorted[i].second);
    } else {
      right.push_back(sorted[i].second);
    }
  }

  return true;
}

bool ClusterSplitterNode::split_by_kmeans(
  const std::vector<PointRecord> & points,
  const std::vector<size_t> & indices,
  std::vector<size_t> & left,
  std::vector<size_t> & right) const
{
  if (indices.size() < static_cast<size_t>(2 * min_subcluster_points_)) {
    return false;
  }

  std::pair<double, double> c0{
    static_cast<double>(points[indices.front()].x),
    static_cast<double>(points[indices.front()].y)};

  size_t farthest_idx = indices.front();
  double farthest_d2 = -1.0;
  for (const size_t idx : indices) {
    const double d2 = squared_distance_xy(points[idx], c0);
    if (d2 > farthest_d2) {
      farthest_d2 = d2;
      farthest_idx = idx;
    }
  }

  std::pair<double, double> c1{
    static_cast<double>(points[farthest_idx].x),
    static_cast<double>(points[farthest_idx].y)};

  if (farthest_d2 <= 1e-8) {
    return false;
  }

  left.clear();
  right.clear();

  for (int iter = 0; iter < kmeans_max_iterations_; ++iter) {
    left.clear();
    right.clear();

    for (const size_t idx : indices) {
      const double d0 = squared_distance_xy(points[idx], c0);
      const double d1 = squared_distance_xy(points[idx], c1);
      if (d0 <= d1) {
        left.push_back(idx);
      } else {
        right.push_back(idx);
      }
    }

    if (left.empty() || right.empty()) {
      return false;
    }

    const auto new_c0 = centroid_xy(points, left);
    const auto new_c1 = centroid_xy(points, right);

    const double shift0 = (new_c0.first - c0.first) * (new_c0.first - c0.first) +
      (new_c0.second - c0.second) * (new_c0.second - c0.second);
    const double shift1 = (new_c1.first - c1.first) * (new_c1.first - c1.first) +
      (new_c1.second - c1.second) * (new_c1.second - c1.second);

    c0 = new_c0;
    c1 = new_c1;

    if (shift0 + shift1 < 1e-8) {
      break;
    }
  }

  if (
    left.size() < static_cast<size_t>(min_subcluster_points_) ||
    right.size() < static_cast<size_t>(min_subcluster_points_))
  {
    return false;
  }

  const double dx = c0.first - c1.first;
  const double dy = c0.second - c1.second;
  const double centroid_distance = std::sqrt(dx * dx + dy * dy);
  if (centroid_distance < kmeans_min_centroid_distance_m_) {
    return false;
  }

  return true;
}

ClusterSplitterNode::AxisStats ClusterSplitterNode::compute_axis_stats(
  const std::vector<PointRecord> & points,
  const std::vector<size_t> & indices) const
{
  AxisStats stats;
  stats.axis_length = 0.0;
  stats.ux = 1.0;
  stats.uy = 0.0;
  stats.projection.clear();
  stats.projection.reserve(indices.size());

  if (indices.empty()) {
    return stats;
  }

  double mx = 0.0;
  double my = 0.0;
  for (const size_t idx : indices) {
    mx += static_cast<double>(points[idx].x);
    my += static_cast<double>(points[idx].y);
  }
  const double inv_n = 1.0 / static_cast<double>(indices.size());
  mx *= inv_n;
  my *= inv_n;

  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  for (const size_t idx : indices) {
    const double dx = static_cast<double>(points[idx].x) - mx;
    const double dy = static_cast<double>(points[idx].y) - my;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  cov_xx *= inv_n;
  cov_xy *= inv_n;
  cov_yy *= inv_n;

  const double theta = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
  stats.ux = std::cos(theta);
  stats.uy = std::sin(theta);

  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();

  for (const size_t idx : indices) {
    const double proj =
      static_cast<double>(points[idx].x) * stats.ux +
      static_cast<double>(points[idx].y) * stats.uy;
    stats.projection.emplace_back(proj, idx);
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }

  if (std::isfinite(min_proj) && std::isfinite(max_proj)) {
    stats.axis_length = max_proj - min_proj;
  }

  return stats;
}

void ClusterSplitterNode::validate_cone_clusters(
  const std::vector<PointRecord> & points,
  std::vector<int32_t> & relabeled,
  int32_t & next_cluster_id) const
{
  if (next_cluster_id <= 0) {
    return;
  }

  const size_t n_clusters = static_cast<size_t>(next_cluster_id);

  // Step A: 클러스터별 bounding box + centroid 계산
  struct ClusterStats {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    int count = 0;
  };

  std::vector<ClusterStats> stats(n_clusters);
  for (size_t i = 0; i < points.size(); ++i) {
    const int32_t lbl = relabeled[i];
    if (lbl < 0 || lbl >= next_cluster_id) {
      continue;
    }
    auto & s = stats[static_cast<size_t>(lbl)];
    const auto & p = points[i];
    s.sum_x += static_cast<double>(p.x);
    s.sum_y += static_cast<double>(p.y);
    s.sum_z += static_cast<double>(p.z);
    s.min_x = std::min(s.min_x, p.x);
    s.max_x = std::max(s.max_x, p.x);
    s.min_y = std::min(s.min_y, p.y);
    s.max_y = std::max(s.max_y, p.y);
    s.min_z = std::min(s.min_z, p.z);
    s.max_z = std::max(s.max_z, p.z);
    s.count += 1;
  }

  // Step B: 가까운 클러스터 병합 (Union-Find, 병합 후 규격 초과 검사)
  std::vector<int32_t> parent(n_clusters);
  for (size_t i = 0; i < n_clusters; ++i) {
    parent[i] = static_cast<int32_t>(i);
  }

  // Union-Find: find with path compression
  std::function<int32_t(int32_t)> find = [&](int32_t x) -> int32_t {
    if (parent[static_cast<size_t>(x)] != x) {
      parent[static_cast<size_t>(x)] = find(parent[static_cast<size_t>(x)]);
    }
    return parent[static_cast<size_t>(x)];
  };

  // 그룹별 병합 bounding box 추적 (root 기준)
  // 초기에는 각자의 stats를 복사
  std::vector<float> group_min_x(n_clusters), group_max_x(n_clusters);
  std::vector<float> group_min_y(n_clusters), group_max_y(n_clusters);
  for (size_t i = 0; i < n_clusters; ++i) {
    group_min_x[i] = stats[i].min_x;
    group_max_x[i] = stats[i].max_x;
    group_min_y[i] = stats[i].min_y;
    group_max_y[i] = stats[i].max_y;
  }

  const double merge_dist2 = cone_diameter_m_ * cone_diameter_m_;

  for (size_t i = 0; i < n_clusters; ++i) {
    if (stats[i].count == 0) {
      continue;
    }
    const double cx_i = stats[i].sum_x / static_cast<double>(stats[i].count);
    const double cy_i = stats[i].sum_y / static_cast<double>(stats[i].count);

    for (size_t j = i + 1; j < n_clusters; ++j) {
      if (stats[j].count == 0) {
        continue;
      }
      const int32_t ri = find(static_cast<int32_t>(i));
      const int32_t rj = find(static_cast<int32_t>(j));
      if (ri == rj) {
        continue;  // 이미 같은 그룹
      }

      const double cx_j = stats[j].sum_x / static_cast<double>(stats[j].count);
      const double cy_j = stats[j].sum_y / static_cast<double>(stats[j].count);
      const double dx = cx_i - cx_j;
      const double dy = cy_i - cy_j;
      if ((dx * dx + dy * dy) > merge_dist2) {
        continue;  // 너무 멀리 있음
      }

      // 병합 시 bounding box 시뮬레이션
      const float merged_min_x = std::min(
        group_min_x[static_cast<size_t>(ri)], group_min_x[static_cast<size_t>(rj)]);
      const float merged_max_x = std::max(
        group_max_x[static_cast<size_t>(ri)], group_max_x[static_cast<size_t>(rj)]);
      const float merged_min_y = std::min(
        group_min_y[static_cast<size_t>(ri)], group_min_y[static_cast<size_t>(rj)]);
      const float merged_max_y = std::max(
        group_max_y[static_cast<size_t>(ri)], group_max_y[static_cast<size_t>(rj)]);

      const double merged_span_x = static_cast<double>(merged_max_x - merged_min_x);
      const double merged_span_y = static_cast<double>(merged_max_y - merged_min_y);

      // 병합 후 XY span이 cone_max_width_m 초과하면 병합 안 함
      if (merged_span_x > cone_max_width_m_ || merged_span_y > cone_max_width_m_) {
        continue;
      }

      // 병합 실행
      parent[static_cast<size_t>(rj)] = ri;
      group_min_x[static_cast<size_t>(ri)] = merged_min_x;
      group_max_x[static_cast<size_t>(ri)] = merged_max_x;
      group_min_y[static_cast<size_t>(ri)] = merged_min_y;
      group_max_y[static_cast<size_t>(ri)] = merged_max_y;
    }
  }

  // relabeled 배열에 Union-Find 결과 반영
  for (size_t i = 0; i < points.size(); ++i) {
    const int32_t lbl = relabeled[i];
    if (lbl >= 0 && lbl < next_cluster_id) {
      relabeled[i] = find(lbl);
    }
  }

  // Step C: 병합 후 통계 재계산 + 콘 규격 검증
  // root id 기준으로 재집계
  std::unordered_map<int32_t, ClusterStats> merged_stats;
  for (size_t i = 0; i < points.size(); ++i) {
    const int32_t lbl = relabeled[i];
    if (lbl < 0) {
      continue;
    }
    auto & s = merged_stats[lbl];
    const auto & p = points[i];
    s.min_x = std::min(s.min_x, p.x);
    s.max_x = std::max(s.max_x, p.x);
    s.min_y = std::min(s.min_y, p.y);
    s.max_y = std::max(s.max_y, p.y);
    s.min_z = std::min(s.min_z, p.z);
    s.max_z = std::max(s.max_z, p.z);
    s.count += 1;
  }

  // 규격 밖 클러스터 → noise(-1)
  std::unordered_map<int32_t, bool> valid_cluster;
  int rejected = 0;
  int merged_count = 0;

  // 병합 카운트: root가 자기 자신이 아닌 것의 수
  for (size_t i = 0; i < n_clusters; ++i) {
    if (stats[i].count > 0 && find(static_cast<int32_t>(i)) != static_cast<int32_t>(i)) {
      ++merged_count;
    }
  }

  for (const auto & kv : merged_stats) {
    const auto & s = kv.second;
    const double span_x = static_cast<double>(s.max_x - s.min_x);
    const double span_y = static_cast<double>(s.max_y - s.min_y);
    const double span_z = static_cast<double>(s.max_z - s.min_z);
    const double xy_span = std::max(span_x, span_y);

    bool ok = true;
    if (s.count < cone_min_points_) {
      ok = false;
    } else if (xy_span > cone_max_width_m_ || xy_span < cone_min_width_m_) {
      ok = false;
    } else if (span_z > cone_max_height_m_ || span_z < cone_min_height_m_) {
      ok = false;
    }

    valid_cluster[kv.first] = ok;
    if (!ok) {
      ++rejected;
    }
  }

  // 무효 클러스터 → -1
  for (size_t i = 0; i < points.size(); ++i) {
    const int32_t lbl = relabeled[i];
    if (lbl >= 0) {
      auto it = valid_cluster.find(lbl);
      if (it != valid_cluster.end() && !it->second) {
        relabeled[i] = -1;
      }
    }
  }

  // next_cluster_id 재정렬 (0부터 연속)
  std::unordered_map<int32_t, int32_t> remap;
  int32_t new_id = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const int32_t lbl = relabeled[i];
    if (lbl < 0) {
      continue;
    }
    auto it = remap.find(lbl);
    if (it == remap.end()) {
      remap[lbl] = new_id;
      relabeled[i] = new_id;
      ++new_id;
    } else {
      relabeled[i] = it->second;
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "cone_validate merged:%d rejected:%d valid:%d",
    merged_count, rejected, static_cast<int>(new_id));

  next_cluster_id = new_id;
}

bool ClusterSplitterNode::is_oversized_cluster(
  const std::vector<size_t> & indices,
  const AxisStats & stats) const
{
  return
    indices.size() >= static_cast<size_t>(oversized_point_count_) ||
    stats.axis_length >= oversized_axis_length_m_;
}

}  // namespace cluster_splitter

RCLCPP_COMPONENTS_REGISTER_NODE(cluster_splitter::ClusterSplitterNode)
