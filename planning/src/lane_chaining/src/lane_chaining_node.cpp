/**
 * @file lane_chaining_node.cpp
 * @brief LaneChainingNode 생성자 + 콜백 오케스트레이션
 *
 * [처리 흐름]
 *   1. /perception/raw_lane_boundaries 수신
 *   2. 왼쪽/오른쪽 시드로 클러스터 매칭
 *   3. 한쪽만 매칭 시 가상 차선 생성
 *   4. /perception/lane_boundaries 발행
 *   5. 디버그 마커 발행 (lazy)
 */
#include "lane_chaining/lane_chaining_node.hpp"

namespace lane_chaining
{

LaneChainingNode::LaneChainingNode(const rclcpp::NodeOptions & options)
: Node("lane_chaining_node", options)
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
    "/lane_chaining/debug/lane_points", qos_debug);

  RCLCPP_INFO(get_logger(), "Lane Chaining 노드 가동!");
}

void LaneChainingNode::on_lane_boundaries(
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
    // 양쪽 다 보임 → 그대로 전달
    output.boundaries.push_back(msg->boundaries[left_idx]);
    output.boundaries.push_back(msg->boundaries[right_idx]);
    update_seed(left_seed_, msg->boundaries[left_idx]);
    update_seed(right_seed_, msg->boundaries[right_idx]);

  } else if (left_matched) {
    // 왼쪽만 보임 → 가상 오른쪽 차선 생성
    const auto & left_bd = msg->boundaries[left_idx];
    auto virtual_right = generate_virtual_lane(left_bd, LaneSide::LEFT);
    output.boundaries.push_back(left_bd);
    if (!virtual_right.points.empty()) {
      output.boundaries.push_back(virtual_right);
      right_virtual = true;
    }
    update_seed(left_seed_, left_bd);

  } else if (right_matched) {
    // 오른쪽만 보임 → 가상 왼쪽 차선 생성
    const auto & right_bd = msg->boundaries[right_idx];
    auto virtual_left = generate_virtual_lane(right_bd, LaneSide::RIGHT);
    if (!virtual_left.points.empty()) {
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

}  // namespace lane_chaining

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lane_chaining::LaneChainingNode)
