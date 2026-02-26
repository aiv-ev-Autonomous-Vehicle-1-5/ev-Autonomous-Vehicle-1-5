/**
 * @file lc_planner_node.cpp
 * @brief LC Planner 메인 노드 — 구현부
 *
 * 10Hz 타이머 콜백에서 8단계 파이프라인을 실행한다.
 *
 * 파이프라인:
 *   Stage 0: Stale Gate — perception 데이터 타임아웃 검사
 *   Stage 1: Input Parse — 콘/차선 → 단일 ChainedPoint 벡터
 *   Stage 2: LineChainer — DFS 체이닝 + 노이즈 할당 + 리샘플
 *   Stage 3: Costmap Generation — 체인 기반 자력 costmap
 *   Stage 4: Magnetic Planner — Greedy 전진 탐색 → raw_path
 *   Stage 5: Postprocess — prune → smooth → resample → yaw
 *   Stage 6: Safety Check — 곡률/속도 검사
 *   Stage 7: Publish — 경로, 상태, 디버그 토픽 발행
 */
#include "planning_lc_ver/nodes/lc_planner_node.hpp"
#include "planning_lc_ver/common/geometry.hpp"
#include "planning_lc_ver/common/debug_publish.hpp"
#include "planning_lc_ver/safety/safety_checker.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace planning_lc_ver
{

LCPlannerNode::LCPlannerNode(const rclcpp::NodeOptions & options)
: Node("lc_planner_node", options),
  stamp_lanes_(0, 0, RCL_ROS_TIME),
  stamp_cones_(0, 0, RCL_ROS_TIME)
{
  params_.load(this);

  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  // ── 구독 설정 ──
  sub_lanes_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();
      last_lanes_ = std::move(msg);
    });

  sub_cones_ = create_subscription<ev_msgs::msg::ConeArray>(
    "/perception/cones", qos_be,
    [this](ev_msgs::msg::ConeArray::UniquePtr msg) {
      stamp_cones_ = now();
      last_cones_ = std::move(msg);
    });

  // ── Core publishers ──
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/path", qos_be);
  pub_status_ = create_publisher<std_msgs::msg::String>(
    "/planning/status", qos_be);

  // ── Debug publishers ──
  pub_dbg_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/debug/costmap", qos_be);
  pub_dbg_raw_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/raw_path", qos_be);
  pub_dbg_left_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/left_chain", qos_be);
  pub_dbg_right_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/right_chain", qos_be);

  // ── 10Hz 타이머 ──
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LCPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "LCPlannerNode initialized (10 Hz, LineChainer + MR costmap)");
}

/**
 * @brief 콘/차선 메시지를 단일 ChainedPoint 벡터로 수집
 *
 * 콘은 PointType::CONE, 차선은 PointType::LANE 태그만 붙임.
 * 좌/우 구분은 하지 않음 — LineChainer가 seed 선택으로 결정.
 */
void LCPlannerNode::parse_input(
  std::vector<ChainedPoint> & all_pts) const
{
  // 콘 수집 (velodyne → base_link 오프셋 보정)
  if (last_cones_) {
    const double ox = params_.sensor_tf.tf_x;
    const double oy = params_.sensor_tf.tf_y;
    for (const auto & c : last_cones_->cones) {
      all_pts.push_back({
        c.position.x + ox,
        c.position.y + oy,
        PointType::CONE});
    }
  }

  // 차선 수집
  if (last_lanes_) {
    for (const auto & b : last_lanes_->boundaries) {
      for (const auto & p : b.points) {
        all_pts.push_back({p.x, p.y, PointType::LANE});
      }
    }
  }
}

bool LCPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_cones_) {
    const double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  return !have_perception;
}

void LCPlannerNode::on_timer()
{
  const auto stamp = now();
  const std::string frame_id = "base_link";

  // ======== Stage 0: Stale Gate ========
  if (check_stale()) {
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = "STALE";
    pub_status_->publish(std::move(status_msg));
    return;
  }

  // ======== Stage 1: Input Parse ========
  // 모든 콘/차선을 단일 ChainedPoint 벡터로 수집 (L/R 분리 없음)
  std::vector<ChainedPoint> all_pts;
  parse_input(all_pts);

  // ======== Stage 2: LineChainer ========
  // DFS 체이닝 → 노이즈 할당 → 리샘플링
  auto chain_result = line_chainer_.chain(all_pts, params_);

  // ======== Stage 3: Costmap Generation ========
  // 체인 기반 costmap 생성 (ChainedPoint.type에 따라 콘/차선 차별 적용)
  auto costmap = costmap_generator_.generate(
    chain_result.left_chain, chain_result.right_chain, params_);

  // ======== Stage 4: Magnetic Planner ========
  auto raw_path = magnetic_planner_.plan(costmap, params_);

  // ======== Stage 5: Postprocess ========
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds);

  // ======== Stage 6: Safety Check ========
  auto safety = safety_checker::check(pp_result, params_);

  // ======== Stage 7: Publish ========

  // ── Core: 최종 경로 발행 ──
  auto path_msg = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(pp_result.path, frame_id, stamp));
  pub_path_->publish(std::move(path_msg));

  // ── Core: 플래너 상태 발행 ──
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // ── Debug: costmap ──
  if (pub_dbg_costmap_->get_subscription_count() > 0 && costmap.valid) {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.stamp = stamp;
    grid_msg->header.frame_id = frame_id;
    grid_msg->info.resolution = static_cast<float>(costmap.resolution);
    grid_msg->info.width = costmap.cols;
    grid_msg->info.height = costmap.rows;
    grid_msg->info.origin.position.x = costmap.origin_x;
    grid_msg->info.origin.position.y = costmap.origin_y;
    grid_msg->info.origin.orientation.w = 1.0;
    grid_msg->data.resize(costmap.rows * costmap.cols);
    for (size_t i = 0; i < costmap.data.size(); ++i) {
      grid_msg->data[i] = static_cast<int8_t>(
        std::clamp(costmap.data[i], 0.0, 100.0));
    }
    pub_dbg_costmap_->publish(std::move(grid_msg));
  }

  // ── Debug: raw path ──
  if (pub_dbg_raw_path_->get_subscription_count() > 0) {
    pub_dbg_raw_path_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(raw_path, frame_id, stamp)));
  }

  // ── Debug: left/right chain ──
  if (pub_dbg_left_chain_->get_subscription_count() > 0) {
    std::vector<Point2D> left_pts;
    left_pts.reserve(chain_result.left_chain.size());
    for (const auto & p : chain_result.left_chain) {
      left_pts.push_back(p.to_point2d());
    }
    pub_dbg_left_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(left_pts, frame_id, stamp)));
  }

  if (pub_dbg_right_chain_->get_subscription_count() > 0) {
    std::vector<Point2D> right_pts;
    right_pts.reserve(chain_result.right_chain.size());
    for (const auto & p : chain_result.right_chain) {
      right_pts.push_back(p.to_point2d());
    }
    pub_dbg_right_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(right_pts, frame_id, stamp)));
  }
}

}  // namespace planning_lc_ver

RCLCPP_COMPONENTS_REGISTER_NODE(planning_lc_ver::LCPlannerNode)
