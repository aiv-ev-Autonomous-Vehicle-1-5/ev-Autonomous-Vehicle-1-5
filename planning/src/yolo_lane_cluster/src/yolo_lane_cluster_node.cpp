/**
 * @file yolo_lane_cluster_node.cpp
 * @brief YoloLaneClusterNode 생성자 + 콜백 오케스트레이션
 *
 * [처리 흐름]
 *   1. /perception/raw_lane_boundaries 수신
 *   2. 왼쪽/오른쪽 시드로 클러스터 매칭
 *   3. 양쪽 매칭 시 긴 쪽 선택 → track_width 안쪽 오프셋으로 반대편 가상 차선 생성
 *      한쪽만 매칭 시 가상 반대편 차선 생성 (기존 로직)
 *   4. 모든 boundary에 lane_side 라벨 (LEFT/RIGHT) 설정
 *   5. /perception/lane_boundaries 발행
 *   6. 디버그 마커 발행 (lazy)
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

#include <cmath>

namespace yolo_lane_cluster
{

YoloLaneClusterNode::YoloLaneClusterNode(const rclcpp::NodeOptions & options)
: Node("yolo_lane_cluster_node", options)
{
  // ── 파라미터 선언 ──
  params_.seed_init_x        = declare_parameter("seed.init_x", 0.0);
  params_.seed_left_y        = declare_parameter("seed.left_y", 0.75);
  params_.seed_right_y       = declare_parameter("seed.right_y", -0.75);
  params_.search_rect_width  = declare_parameter("seed.search_rect_width", 2.0);
  params_.search_rect_height = declare_parameter("seed.search_rect_height", 8.0);
  params_.track_width        = declare_parameter("virtual_lane.track_width", 1.5);

  // ── 시드 초기화 ──
  reset_seed(left_seed_, LaneSide::LEFT);
  reset_seed(right_seed_, LaneSide::RIGHT);

  // ── QoS: Best Effort, depth=1 ──
  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  // ── 구독: 카메라 raw 차선 ──
  sub_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/raw_lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::SharedPtr msg) {
      on_lane_boundaries(msg);
    });

  // ── 발행: 가공된 차선 → planning ──
  pub_ = create_publisher<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be);

  // ── 디버그 발행 (lazy) ──
  rclcpp::QoS qos_debug(1);
  qos_debug.best_effort();
  debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/yolo_lane_cluster/debug/lane_points", qos_debug);

  RCLCPP_INFO(get_logger(), "Yolo Lane Cluster 노드 가동!");
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

  // ── Step 1: 시드 매칭 ──
  int left_idx  = match_cluster_to_seed(left_seed_, *msg, -1);
  int right_idx = match_cluster_to_seed(right_seed_, *msg, left_idx);

  // 둘 다 같은 클러스터 매칭 방지 (left가 이미 exclude)
  // right_idx에서 left_idx를 제외했으므로 충돌 없음

  // ── Step 2: 출력 구성 ──
  bool left_matched  = (left_idx >= 0);
  bool right_matched = (right_idx >= 0);
  bool left_virtual  = false;
  bool right_virtual = false;

  if (left_matched && right_matched) {
    // 양쪽 다 보임 → 긴 쪽 선택, 반대편은 가상 생성
    const auto & left_bd  = msg->boundaries[left_idx];
    const auto & right_bd = msg->boundaries[right_idx];
    double left_len  = path_length(left_bd);
    double right_len = path_length(right_bd);

    if (left_len >= right_len) {
      // 왼쪽이 길거나 같음 → 왼쪽 채택, 가상 오른쪽 생성
      auto virtual_right = generate_virtual_lane(left_bd, LaneSide::LEFT);
      auto real_left = left_bd;
      real_left.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
      output.boundaries.push_back(real_left);
      if (!virtual_right.points.empty()) {
        virtual_right.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
        output.boundaries.push_back(virtual_right);
        right_virtual = true;
      }
    } else {
      // 오른쪽이 길음 → 오른쪽 채택, 가상 왼쪽 생성
      auto virtual_left = generate_virtual_lane(right_bd, LaneSide::RIGHT);
      if (!virtual_left.points.empty()) {
        virtual_left.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
        output.boundaries.push_back(virtual_left);
        left_virtual = true;
      }
      auto real_right = right_bd;
      real_right.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
      output.boundaries.push_back(real_right);
    }
    update_seed(left_seed_, left_bd);
    update_seed(right_seed_, right_bd);

  } else if (left_matched) {
    // 왼쪽만 보임 → 가상 오른쪽 차선 생성
    auto left_bd = msg->boundaries[left_idx];
    left_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
    auto virtual_right = generate_virtual_lane(left_bd, LaneSide::LEFT);
    output.boundaries.push_back(left_bd);
    if (!virtual_right.points.empty()) {
      virtual_right.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
      output.boundaries.push_back(virtual_right);
      right_virtual = true;
    }
    update_seed(left_seed_, left_bd);

  } else if (right_matched) {
    // 오른쪽만 보임 → 가상 왼쪽 차선 생성
    auto right_bd = msg->boundaries[right_idx];
    right_bd.lane_side = ev_msgs::msg::LaneBoundary::SIDE_RIGHT;
    auto virtual_left = generate_virtual_lane(right_bd, LaneSide::RIGHT);
    if (!virtual_left.points.empty()) {
      virtual_left.lane_side = ev_msgs::msg::LaneBoundary::SIDE_LEFT;
      output.boundaries.push_back(virtual_left);
      left_virtual = true;
    }
    output.boundaries.push_back(right_bd);
    update_seed(right_seed_, right_bd);
  }
  // else: 매칭 없음 → 빈 배열 발행

  pub_->publish(output);

  // ── Step 3: 디버그 ──
  publish_debug_markers(output, left_matched, right_matched,
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
