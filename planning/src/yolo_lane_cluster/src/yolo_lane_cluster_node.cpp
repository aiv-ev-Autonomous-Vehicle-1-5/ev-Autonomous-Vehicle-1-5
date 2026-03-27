/**
 * @file yolo_lane_cluster_node.cpp
 * @brief YoloLaneClusterNode 생성자 + 콜백 오케스트레이션
 *
 * [처리 흐름]
 *   1. /perception/raw_lane_boundaries 수신
 *   2. 모든 클러스터 포인트를 LanePoint 풀로 flat화
 *   3. LEFT/RIGHT seed로 가장 가까운 클러스터 선택
 *   4. 선택된 클러스터의 x_min 포인트를 chaining seed로 backbone chaining
 *   5. Backtracking으로 overlap 해소
 *   6. 클러스터의 x_min (x,y)를 다음 프레임 seed 중심으로 저장
 *   7. 긴 쪽 채택 + 가상 반대편 차선 생성
 *   8. /perception/lane_boundaries 발행
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace yolo_lane_cluster
{

YoloLaneClusterNode::YoloLaneClusterNode(const rclcpp::NodeOptions & options)
: Node("yolo_lane_cluster_node", options)
{
  // ── 파라미터 선언 ──
  params_.seed_left_y        = declare_parameter("seed.left_y", 0.8);
  params_.seed_right_y       = declare_parameter("seed.right_y", -0.8);
  params_.seed_timeout_sec   = declare_parameter("seed.timeout_sec", 3.0);
  params_.holdover_frames    = declare_parameter("holdover.frames", 3);
  params_.track_width        = declare_parameter("virtual_lane.track_width", 1.6);

  auto & cp = params_.chainer;
  cp.d_max             = declare_parameter("chainer.d_max", 2.5);
  cp.forward_cone_deg  = declare_parameter("chainer.forward_cone_deg", 120.0);
  cp.lateral_gate      = declare_parameter("chainer.lateral_gate", 1.3);
  cp.alpha             = declare_parameter("chainer.alpha", 1.2);
  cp.beta              = declare_parameter("chainer.beta", 1.2);
  cp.gamma             = declare_parameter("chainer.gamma", 0.7);
  cp.lambda_side       = declare_parameter("chainer.lambda_side", 0.5);
  cp.max_backtrack_count = declare_parameter("chainer.max_backtrack_count", 5);
  cp.backtrack_w_curv  = declare_parameter("chainer.backtrack_w_curv", 1.0);
  cp.backtrack_w_dist  = declare_parameter("chainer.backtrack_w_dist", 1.0);
  cp.max_chain_len     = declare_parameter("chainer.max_chain_len", 300);

  // ── Seed 초기화 ──
  left_seed_.center_x  = 0.0;
  left_seed_.center_y  = params_.seed_left_y;
  right_seed_.center_x = 0.0;
  right_seed_.center_y = params_.seed_right_y;
  left_seed_last_seen_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
  right_seed_last_seen_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ── QoS: Best Effort, depth=1 ──
  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  sub_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/raw_lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::SharedPtr msg) {
      on_lane_boundaries(msg);
    });

  pub_ = create_publisher<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be);

  rclcpp::QoS qos_debug(1);
  qos_debug.best_effort();
  debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/yolo_lane_cluster/debug/lane_points", qos_debug);

  RCLCPP_INFO(get_logger(), "Yolo Lane Cluster 노드 가동! (backbone chaining 모드)");
}

void YoloLaneClusterNode::on_lane_boundaries(
  const ev_msgs::msg::LaneBoundaryArray::SharedPtr msg)
{
  ev_msgs::msg::LaneBoundaryArray output;
  output.header = msg->header;

  if (msg->boundaries.empty()) {
    pub_->publish(output);
    return;
  }

  // ── Step 1: 포인트 풀 생성 ──
  std::vector<LanePoint> points;
  for (int bi = 0; bi < static_cast<int>(msg->boundaries.size()); ++bi) {
    for (const auto & pt : msg->boundaries[bi].points) {
      points.push_back({pt.x, pt.y, bi});
    }
  }

  if (points.empty()) {
    pub_->publish(output);
    return;
  }

  // ── Seed 타임아웃: 일정 시간 chaining 미성공 시 초기 위치로 리셋 ──
  const rclcpp::Time now = msg->header.stamp;
  const rclcpp::Duration timeout =
    rclcpp::Duration::from_seconds(params_.seed_timeout_sec);

  if (left_seed_last_seen_.nanoseconds() > 0 &&
      (now - left_seed_last_seen_) > timeout) {
    left_seed_.center_x = 0.0;
    left_seed_.center_y = params_.seed_left_y;
    RCLCPP_WARN(get_logger(), "LEFT seed 타임아웃 → 초기 위치로 리셋");
  }
  if (right_seed_last_seen_.nanoseconds() > 0 &&
      (now - right_seed_last_seen_) > timeout) {
    right_seed_.center_x = 0.0;
    right_seed_.center_y = params_.seed_right_y;
    RCLCPP_WARN(get_logger(), "RIGHT seed 타임아웃 → 초기 위치로 리셋");
  }

  // ── Step 2: 클러스터별 하단 K개 포인트의 평균 y로 좌/우 판별 ──
  // 단일 x_min 포인트 대신 x가 가장 작은 K개의 평균 y를 사용하여
  // 노이즈에 의한 좌/우 판별 플리핑 방지
  const int num_clusters = static_cast<int>(msg->boundaries.size());
  constexpr int K_BOTTOM = 5;  // 하단 포인트 수

  std::vector<double> cluster_bottom_avg_y(num_clusters, 0.0);
  for (int c = 0; c < num_clusters; ++c) {
    const auto & bpts = msg->boundaries[c].points;
    if (bpts.empty()) continue;

    // x 기준 정렬된 인덱스 생성 (작은 순)
    std::vector<int> sorted_idx(bpts.size());
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
    std::partial_sort(sorted_idx.begin(),
      sorted_idx.begin() + std::min(K_BOTTOM, static_cast<int>(bpts.size())),
      sorted_idx.end(),
      [&](int a, int b) { return bpts[a].x < bpts[b].x; });

    // 하단 K개 평균 y
    int k = std::min(K_BOTTOM, static_cast<int>(bpts.size()));
    double sum_y = 0.0;
    for (int i = 0; i < k; ++i) {
      sum_y += bpts[sorted_idx[i]].y;
    }
    cluster_bottom_avg_y[c] = sum_y / k;
  }

  // dead zone: |avg_y| < 이 값이면 양쪽 seed 모두 후보 허용 (seed 거리로 결정)
  // 커브에서 클러스터 하단이 y≈0 근처일 때 플리핑 방지
  constexpr double SIDE_DEAD_ZONE = 0.15;

  auto find_cluster_for_seed = [&](double sx, double sy, int exclude_cluster, bool want_left) -> int {
    double best_sq = std::numeric_limits<double>::max();
    int best_cluster = -1;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      int cl = points[i].label;
      if (cl == exclude_cluster) continue;
      // dead zone 밖에서만 hard y-sign 체크, 안에서는 양쪽 허용
      double avg_y = cluster_bottom_avg_y[cl];
      if (want_left && avg_y < -SIDE_DEAD_ZONE) continue;
      if (!want_left && avg_y > SIDE_DEAD_ZONE) continue;
      double dx = points[i].x - sx;
      double dy = points[i].y - sy;
      double d_sq = dx * dx + dy * dy;
      if (d_sq < best_sq) {
        best_sq = d_sq;
        best_cluster = cl;
      }
    }
    return best_cluster;
  };

  int left_cluster  = find_cluster_for_seed(
    left_seed_.center_x, left_seed_.center_y, -1, true);
  int right_cluster = find_cluster_for_seed(
    right_seed_.center_x, right_seed_.center_y, left_cluster, false);

  // ── Step 3: 각 클러스터의 x_min 포인트 → chaining seed ──
  auto find_xmin_in_cluster = [&](int cluster_label) -> int {
    int best = -1;
    double min_x = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      if (points[i].label != cluster_label) continue;
      if (points[i].x < min_x) {
        min_x = points[i].x;
        best = i;
      }
    }
    return best;
  };

  int left_chain_seed  = (left_cluster >= 0)  ? find_xmin_in_cluster(left_cluster)  : -1;
  int right_chain_seed = (right_cluster >= 0) ? find_xmin_in_cluster(right_cluster) : -1;

  // ── Step 4: Backbone Chaining ──
  // chaining 결과 ≤ 5이면 버리고 holdover에 맡김
  std::vector<int> left_bb, right_bb;

  if (left_chain_seed >= 0) {
    left_bb = extract_backbone(points, left_chain_seed, true);
    if (left_bb.size() <= 5) left_bb.clear();
  }
  if (right_chain_seed >= 0) {
    right_bb = extract_backbone(points, right_chain_seed, false);
    if (right_bb.size() <= 5) right_bb.clear();
  }

  // ── Step 5: Overlap 해소 ──
  if (!left_bb.empty() && !right_bb.empty()) {
    resolve_overlaps(points, left_bb, right_bb);
  }

  // ── Step 6: Seed 추적 업데이트 — 매칭 클러스터의 x_min (x,y) ──
  bool left_chained  = (left_bb.size() >= 2);
  bool right_chained = (right_bb.size() >= 2);

  // seed x는 ego 근처에 머물러야 함 — 앞으로 drift 방지
  constexpr double SEED_X_MAX = 1.0;

  if (left_chained && left_chain_seed >= 0) {
    left_seed_.center_x = std::min(points[left_chain_seed].x, SEED_X_MAX);
    left_seed_.center_y = points[left_chain_seed].y;
    left_seed_last_seen_ = now;
    // y가 반대쪽으로 drift하면 초기값으로 리셋
    if (left_seed_.center_y <= 0.0) {
      left_seed_.center_x = 0.0;
      left_seed_.center_y = params_.seed_left_y;
    }
  }
  if (right_chained && right_chain_seed >= 0) {
    right_seed_.center_x = std::min(points[right_chain_seed].x, SEED_X_MAX);
    right_seed_.center_y = points[right_chain_seed].y;
    right_seed_last_seen_ = now;
    if (right_seed_.center_y >= 0.0) {
      right_seed_.center_x = 0.0;
      right_seed_.center_y = params_.seed_right_y;
    }
  }

  // ── Step 7: 출력 구성 ──
  bool left_virtual  = false;
  bool right_virtual = false;

  if (left_chained && right_chained) {
    auto left_bd  = backbone_to_boundary(points, left_bb, msg->header);
    auto right_bd = backbone_to_boundary(points, right_bb, msg->header);
    // 실제 차선 backbone에도 outlier 필터 적용
    filter_virtual_lane_outliers(left_bd);
    filter_virtual_lane_outliers(right_bd);
    double left_len  = path_length(left_bd);
    double right_len = path_length(right_bd);

    if (left_len >= right_len) {
      auto virtual_right = generate_virtual_lane(left_bd, LaneSide::LEFT);
      filter_virtual_lane_outliers(virtual_right);
      left_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
      output.boundaries.push_back(left_bd);
      if (!virtual_right.points.empty()) {
        virtual_right.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
        output.boundaries.push_back(virtual_right);
        right_virtual = true;
      }
    } else {
      auto virtual_left = generate_virtual_lane(right_bd, LaneSide::RIGHT);
      filter_virtual_lane_outliers(virtual_left);
      if (!virtual_left.points.empty()) {
        virtual_left.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
        output.boundaries.push_back(virtual_left);
        left_virtual = true;
      }
      right_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
      output.boundaries.push_back(right_bd);
    }

  } else if (left_chained) {
    auto left_bd = backbone_to_boundary(points, left_bb, msg->header);
    filter_virtual_lane_outliers(left_bd);
    left_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
    auto virtual_right = generate_virtual_lane(left_bd, LaneSide::LEFT);
    filter_virtual_lane_outliers(virtual_right);
    output.boundaries.push_back(left_bd);
    if (!virtual_right.points.empty()) {
      virtual_right.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
      output.boundaries.push_back(virtual_right);
      right_virtual = true;
    }

  } else if (right_chained) {
    auto right_bd = backbone_to_boundary(points, right_bb, msg->header);
    filter_virtual_lane_outliers(right_bd);
    right_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
    auto virtual_left = generate_virtual_lane(right_bd, LaneSide::RIGHT);
    filter_virtual_lane_outliers(virtual_left);
    if (!virtual_left.points.empty()) {
      virtual_left.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
      output.boundaries.push_back(virtual_left);
      left_virtual = true;
    }
    output.boundaries.push_back(right_bd);
  }

  // ── Holdover: 결과가 없으면 이전 프레임 결과 재발행 ──
  if (output.boundaries.empty()) {
    if (holdover_remaining_ > 0) {
      last_output_.header = msg->header;  // 타임스탬프만 갱신
      pub_->publish(last_output_);
      --holdover_remaining_;
      publish_debug_markers(last_output_, false, false, false, false);
      return;
    }
  } else {
    // 유효 결과 → 버퍼 저장 + holdover 카운터 리셋
    last_output_ = output;
    holdover_remaining_ = params_.holdover_frames;
  }

  pub_->publish(output);

  publish_debug_markers(output, left_chained, right_chained,
                        left_virtual, right_virtual);
}

double YoloLaneClusterNode::path_length(const ev_msgs::msg::LaneBoundary & bd)
{
  double total = 0.0;
  for (size_t i = 1; i < bd.points.size(); ++i) {
    double dx = bd.points[i].x - bd.points[i - 1].x;
    double dy = bd.points[i].y - bd.points[i - 1].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

}  // namespace yolo_lane_cluster

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yolo_lane_cluster::YoloLaneClusterNode)
