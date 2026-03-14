/**
 * @file lc_planner_node.cpp
 * @brief LC Planner 메인 노드 구현 — DirectionChainer v2 7단계 파이프라인
 *
 * ══════════════════════════════════════════════════════════════
 *  이 파일은 lc_planner_node.hpp에 선언된 LCPlannerNode를 구현한다.
 *
 *  핵심 구조:
 *    생성자 — 파라미터 로드, QoS 설정, 구독/발행 생성, 타이머 시작
 *    on_timer() — 10Hz로 호출되는 메인 루프 (7단계 파이프라인)
 *                 perception 메시지의 원본 센서 타임스탬프를 모든 출력에 전파
 *
 *  7단계 파이프라인 (on_timer 콜백):
 *    Stage 0: Stale Gate — perception 데이터 타임아웃 검사
 *    Stage 1: Input Parse — bbox/차선 ROS 메시지 → 단일 ChainPoint 벡터
 *             (input_parser.hpp의 parse_input() 사용)
 *    Stage 2: DirectionChainer — Component→Backbone→Branch + 리샘플
 *    Stage 3: Costmap + A* — 가우시안 코스트맵 생성 + A* 경로 탐색
 *             (goal_calculator.hpp의 calculate_goal() 사용)
 *    Stage 5: Postprocess — prune → smooth → curvature_clamp → resample → yaw
 *    Stage 6: Safety Check — 경로 유효성 + 곡률 검사
 *    Stage 7: Publish — 경로, 상태, 디버그 토픽 발행
 *             (debug_publisher.hpp의 헬퍼 함수들 사용)
 *
 *  [모듈 분리 구조]
 *    - nodes/input_parser.hpp:    Stage 1 입력 파싱 로직
 *    - nodes/goal_calculator.hpp: Stage 3c goal 계산 로직
 *    - nodes/debug_publisher.hpp: Stage 7 디버그 시각화 로직
 * ══════════════════════════════════════════════════════════════
 */
// ── 프로젝트 내부 헤더 ──
#include "chaining_costmap_ver/nodes/chaining_costmap_ver_node.hpp"
#include "chaining_costmap_ver/nodes/input_parser.hpp"        // parse_input() — Stage 1
#include "chaining_costmap_ver/nodes/goal_calculator.hpp"     // calculate_goal() — Stage 3c
#include "chaining_costmap_ver/nodes/debug_publisher.hpp"     // 디버그 시각화 헬퍼 — Stage 7
#include "chaining_costmap_ver/common/geometry.hpp"           // dist(), cross2() 등 기하 유틸
#include "chaining_costmap_ver/common/debug_publish.hpp"      // to_path_msg(), to_points_marker()
#include "chaining_costmap_ver/safety/safety_checker.hpp"     // safety_checker::check()

// ── ROS 2 컴포넌트 등록 매크로 ──
#include <rclcpp_components/register_node_macro.hpp>

// ── 표준 라이브러리 ──
#include <chrono>
#include <cmath>
#include <algorithm>

namespace chaining_costmap_ver
{

// ══════════════════════════════════════════════════════════════
//  생성자 — 파라미터 로드, QoS 설정, 구독/발행/타이머 초기화
// ══════════════════════════════════════════════════════════════
LCPlannerNode::LCPlannerNode(const rclcpp::NodeOptions & options)
: Node("lc_planner_node", options),
  stamp_lanes_(0, 0, RCL_ROS_TIME),
  stamp_bboxes_(0, 0, RCL_ROS_TIME)
{
  params_.load(this);

  // ── QoS 설정: Best Effort, depth=1 ──
  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  // ── 구독 설정 ──
  sub_lanes_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();
      last_lanes_ = std::move(msg);
    });

  sub_bboxes_ = create_subscription<ev_msgs::msg::BBoxArray>(
    "/perception/bboxes", qos_be,
    [this](ev_msgs::msg::BBoxArray::UniquePtr msg) {
      stamp_bboxes_ = now();
      last_bboxes_ = std::move(msg);
    });

  // ── Core 퍼블리셔 ──
  pub_path_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planning/path", qos_be);
  pub_status_ = create_publisher<std_msgs::msg::String>(
    "/planning/status", qos_be);

  // ── Debug 퍼블리셔 (Reliable QoS, lazy publishing) ──
  rclcpp::QoS qos_dbg(1);
  qos_dbg.reliable();

  pub_dbg_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/debug/costmap", qos_dbg);
  pub_dbg_raw_path_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planning/debug/raw_path", qos_dbg);
  pub_dbg_pruned_path_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planning/debug/pruned_path", qos_dbg);
  pub_dbg_left_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/left_chain", qos_dbg);
  pub_dbg_right_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/right_chain", qos_dbg);
  pub_dbg_left_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/left_branches", qos_dbg);
  pub_dbg_right_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/right_branches", qos_dbg);
  pub_dbg_seeds_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/seeds", qos_dbg);
  pub_dbg_local_goal_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/local_goal", qos_dbg);
  pub_dbg_obstacle_wall_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/obstacle_wall", qos_dbg);
  pub_dbg_curvature_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/curvature", qos_dbg);
  pub_dbg_lane_points_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/lane_points", qos_dbg);

  // ── 10Hz 타이머 ──
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LCPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "LCPlannerNode initialized (10 Hz, DirectionChainer v2 + Costmap + A*)");
}

// ══════════════════════════════════════════════════════════════
//  Stage 0: Stale Gate — 인지 데이터 신선도 검사
// ══════════════════════════════════════════════════════════════
bool LCPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_bboxes_) {
    const double dt = (t - stamp_bboxes_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  return !have_perception;
}

// ══════════════════════════════════════════════════════════════
//  on_timer() — 10Hz 메인 루프: 7단계 파이프라인
// ══════════════════════════════════════════════════════════════
void LCPlannerNode::on_timer()
{
  // 원본 센서 타임스탬프를 전파 — topic delay 측정 가능
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  if (last_bboxes_) {
    stamp = rclcpp::Time(last_bboxes_->header.stamp);
  }
  if (last_lanes_) {
    rclcpp::Time t(last_lanes_->header.stamp);
    if (t > stamp) stamp = t;
  }
  const std::string frame_id = "base_link";

  // ======== Stage 0: Stale Gate ========
  if (check_stale()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[Stage0] STALE — perception timeout");
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = "STALE";
    pub_status_->publish(std::move(status_msg));
    return;
  }

  // ======== Stage 1: Input Parse ========
  // input_parser.hpp의 free function 사용 (ROS 메시지 → ChainPoint 벡터)
  std::vector<ChainPoint> all_pts;
  parse_input(last_bboxes_.get(), last_lanes_.get(), params_, all_pts);

  // ======== Stage 2: DirectionChainer ========
  auto dc_result = direction_chainer_.chain(all_pts, params_);

  // ======== Stage 2.5: Seed Gate ========
  if (!dc_result.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[Stage2.5] FAIL — not enough seeds (both backbones empty)");
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = "FAIL - not enough seeds";
    pub_status_->publish(std::move(status_msg));

    auto path_msg = std::make_unique<visualization_msgs::msg::Marker>(
      to_points_marker({}, frame_id, stamp,
                       "final_path", 0.0f, 1.0f, 0.0f, 1.0f, 0.08));
    pub_path_->publish(std::move(path_msg));
    return;
  }

  // ======== Stage 3: Costmap Generation + A* Path Planning ========
  // 3a. ChainPoint → ChainedPoint 변환
  std::vector<ChainedPoint> left_chained, right_chained, unchained_chained;
  for (const auto & p : dc_result.left.component)
    left_chained.push_back(p.to_chained_point());
  for (const auto & p : dc_result.right.component)
    right_chained.push_back(p.to_chained_point());
  for (const auto & p : dc_result.unchained)
    unchained_chained.push_back(p.to_chained_point());

  // 3b. Costmap 생성
  auto costmap = costmap_generator_.generate(
    left_chained, right_chained, unchained_chained, params_);

  // 3b-2. Entry walls
  if (costmap.valid &&
      !dc_result.left.backbone.empty() && !dc_result.right.backbone.empty()) {
    Point2D left_seed = {
      dc_result.left.backbone.front().x,
      dc_result.left.backbone.front().y};
    Point2D right_seed = {
      dc_result.right.backbone.front().x,
      dc_result.right.backbone.front().y};
    CostmapGenerator::apply_entry_walls(costmap, left_seed, right_seed, params_);
  }

  // 3c. Goal 계산 (goal_calculator.hpp)
  auto goal_result = calculate_goal(dc_result, costmap, params_);

  // 3d. Goal을 costmap 경계 안쪽으로 clamp
  if (goal_result.have_goal && costmap.valid) {
    clamp_goal_to_costmap(goal_result.goal, costmap);
  }

  // 3e. A* 경로 탐색
  std::vector<Point2D> raw_path;
  if (goal_result.have_goal && costmap.valid) {
    raw_path = astar_planner_.plan(costmap, {0.0, 0.0}, goal_result.goal, params_);
  }

  // ======== Stage 5: Postprocess ========
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds,
    1.0 / params_.vehicle.r_min(),
    params_.postprocess.curvature_clamp_max_iter);

  // ======== Stage 6: Safety Check ========
  auto safety = safety_checker::check(pp_result, params_);

  // ── 상태 로그 ──
  const double r_min = params_.vehicle.r_min();
  const double kappa_limit = 1.0 / r_min;
  const double r_actual = (safety.max_curvature > 1e-6) ? 1.0 / safety.max_curvature : 999.0;
  if (safety.state == PlannerState::OK) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Planner] OK — path:%zu pts, kappa=%.3f (r=%.2fm), limit=%.3f (r_min=%.2fm, delta_max=%.1f°)",
      pp_result.path.size(), safety.max_curvature, r_actual,
      kappa_limit, r_min, params_.vehicle.delta_max * 180.0 / M_PI);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Planner] %s", safety.reason.c_str());
  }

  // ======== Stage 7: Publish ========
  // ── Core: 최종 경로 발행 ──
  auto path_msg = std::make_unique<visualization_msgs::msg::Marker>(
    to_points_marker(pp_result.path, frame_id, stamp,
                     "final_path", 0.0f, 1.0f, 0.0f, 1.0f, 0.08));
  pub_path_->publish(std::move(path_msg));

  // ── Core: 플래너 상태 발행 ──
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // ── Debug: lane points (수신된 차선점 시각화, 분홍색 SPHERE) ──
  if (pub_dbg_lane_points_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray lane_ma;
    visualization_msgs::msg::Marker del;
    del.header.stamp = stamp;
    del.header.frame_id = frame_id;
    del.ns = "lane_points";
    del.id = -1;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    lane_ma.markers.push_back(del);

    int lane_id = 0;
    for (const auto & pt : all_pts) {
      if (pt.type != PointType::LANE) continue;
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = "lane_points";
      m.id = lane_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pt.x;
      m.pose.position.y = pt.y;
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.15;
      m.color.r = 1.0f;
      m.color.g = 0.41f;
      m.color.b = 0.71f;
      m.color.a = 1.0f;
      m.lifetime = rclcpp::Duration::from_seconds(0.2);
      lane_ma.markers.push_back(m);
    }
    pub_dbg_lane_points_->publish(
      std::make_unique<visualization_msgs::msg::MarkerArray>(lane_ma));
  }

  // ── Debug: costmap (debug_publisher.hpp) ──
  publish_debug_costmap(pub_dbg_costmap_, costmap, frame_id, stamp);

  // ── Debug: obstacle_wall (debug_publisher.hpp) ──
  publish_debug_obstacle_wall(
    pub_dbg_obstacle_wall_, costmap, params_.astar.obstacle_cost, frame_id, stamp);

  // ── Debug: curvature (debug_publisher.hpp) ──
  publish_debug_curvature(
    pub_dbg_curvature_, pp_result, params_.vehicle.r_min(), frame_id, stamp);

  // ── Debug: raw_path (A* 원시 경로) — 흰색 점 ──
  if (pub_dbg_raw_path_->get_subscription_count() > 0) {
    pub_dbg_raw_path_->publish(std::make_unique<visualization_msgs::msg::Marker>(
      to_points_marker(raw_path, frame_id, stamp,
                       "raw_path", 1.0f, 1.0f, 1.0f, 1.0f, 0.06)));
  }

  // ── Debug: pruned_path (prune 직후) — 노란색 점 ──
  if (pub_dbg_pruned_path_->get_subscription_count() > 0 &&
      !pp_result.pruned.empty()) {
    pub_dbg_pruned_path_->publish(std::make_unique<visualization_msgs::msg::Marker>(
      to_points_marker(pp_result.pruned, frame_id, stamp,
                       "pruned_path", 1.0f, 1.0f, 0.0f, 1.0f, 0.10)));
  }

  // ── Debug: chainer 시각화 (backbone, branches, seeds, local_goal) ──
  if (params_.chainer.publish_debug) {

    // left backbone (Path)
    if (pub_dbg_left_chain_->get_subscription_count() > 0) {
      std::vector<Point2D> left_pts;
      left_pts.reserve(dc_result.left.backbone.size());
      for (const auto & p : dc_result.left.backbone)
        left_pts.push_back(p.to_point2d());
      pub_dbg_left_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
        to_path_msg(left_pts, frame_id, stamp)));
    }

    // right backbone (Path)
    if (pub_dbg_right_chain_->get_subscription_count() > 0) {
      std::vector<Point2D> right_pts;
      right_pts.reserve(dc_result.right.backbone.size());
      for (const auto & p : dc_result.right.backbone)
        right_pts.push_back(p.to_point2d());
      pub_dbg_right_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
        to_path_msg(right_pts, frame_id, stamp)));
    }

    // left branches (debug_publisher.hpp)
    if (pub_dbg_left_branches_->get_subscription_count() > 0) {
      auto ma = make_branch_markers(
        dc_result.left.branches, dc_result.left.backbone,
        0.5f, 1.0f, 0.5f, "left_branches", frame_id, stamp);
      pub_dbg_left_branches_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // right branches (debug_publisher.hpp)
    if (pub_dbg_right_branches_->get_subscription_count() > 0) {
      auto ma = make_branch_markers(
        dc_result.right.branches, dc_result.right.backbone,
        1.0f, 0.5f, 0.5f, "right_branches", frame_id, stamp);
      pub_dbg_right_branches_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // seeds & goals (debug_publisher.hpp)
    if (pub_dbg_seeds_->get_subscription_count() > 0) {
      auto ma = make_seeds_markers(dc_result, frame_id, stamp);
      pub_dbg_seeds_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // local_goal 마커 (노란색 구체)
    if (pub_dbg_local_goal_->get_subscription_count() > 0 && goal_result.have_goal) {
      visualization_msgs::msg::MarkerArray goal_ma;
      visualization_msgs::msg::Marker gm;
      gm.header.frame_id = frame_id;
      gm.header.stamp = stamp;
      gm.ns = "local_goal";
      gm.id = 0;
      gm.type = visualization_msgs::msg::Marker::SPHERE;
      gm.action = visualization_msgs::msg::Marker::ADD;
      gm.scale.x = gm.scale.y = gm.scale.z = 0.3;
      gm.color.r = 1.0f;
      gm.color.g = 1.0f;
      gm.color.b = 0.0f;
      gm.color.a = 1.0f;
      gm.pose.position.x = goal_result.goal.x;
      gm.pose.position.y = goal_result.goal.y;
      gm.pose.position.z = 0.0;
      goal_ma.markers.push_back(gm);
      pub_dbg_local_goal_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(goal_ma));
    }
  }
}

}  // namespace chaining_costmap_ver

// ══════════════════════════════════════════════════════════════
//  ROS 2 컴포넌트 등록
// ══════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(chaining_costmap_ver::LCPlannerNode)
