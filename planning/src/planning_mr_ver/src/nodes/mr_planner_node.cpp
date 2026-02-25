/**
 * @file mr_planner_node.cpp
 * @brief MR Planner 메인 노드 — 구현부
 *
 * 10Hz 타이머 콜백에서 6단계 파이프라인을 실행한다.
 *
 * 파이프라인:
 *   Stage 0: Stale Gate — perception 데이터 타임아웃 검사
 *   Stage 1: Input Parse — 콘/차선 메시지 → Point2D 벡터
 *   Stage 2: Costmap Generation — 자력 costmap 생성
 *   Stage 3: Magnetic Planner — Greedy 전진 탐색 → raw_path
 *   Stage 4: Postprocess — prune → smooth → resample → yaw
 *   Stage 5: Safety Check — 곡률/속도 검사
 *   Stage 6: Publish — 경로, 상태, 디버그 토픽 발행
 */
#include "planning_mr_ver/nodes/mr_planner_node.hpp"
#include "planning_mr_ver/common/geometry.hpp"
#include "planning_mr_ver/common/debug_publish.hpp"
#include "planning_mr_ver/safety/safety_checker.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace planning_mr_ver
{

/**
 * @brief 생성자 — 파라미터 로드, 구독/발행/타이머 초기화
 *
 * 초기화 순서:
 *   1. params_.load(): planning_mr.yaml에서 파라미터 로드
 *   2. 구독 설정: /perception/lane_boundaries, /perception/cones
 *   3. 발행 설정: /planning/path, /planning/status, 디버그 토픽들
 *   4. 타이머 설정: 100ms (10Hz) 주기
 */
MRPlannerNode::MRPlannerNode(const rclcpp::NodeOptions & options)
: Node("mr_planner_node", options),
  stamp_lanes_(0, 0, RCL_ROS_TIME),   // 초기값: 에포크 0 (아직 수신 안 됨)
  stamp_cones_(0, 0, RCL_ROS_TIME)
{
  params_.load(this);  // YAML → 파라미터 구조체

  // ── QoS 설정: Best Effort ──
  // Best Effort: 메시지 유실 시 재전송하지 않고 최신 데이터만 사용
  // 센서 데이터(10Hz)는 이전 프레임 재전송보다 최신 프레임이 더 유용하므로 Best Effort 사용
  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  // ── 구독 설정 ──
  // track_planning(CDT 버전)과 동일한 토픽 이름 → drop-in replacement
  // UniquePtr 콜백: Intra-process Zero-copy를 위해 소유권 이전 방식 사용
  sub_lanes_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();            // 수신 시각 기록 (stale 판정용)
      last_lanes_ = std::move(msg);    // 소유권 이전 (메모리 복사 없음)
    });

  sub_cones_ = create_subscription<ev_msgs::msg::ConeArray>(
    "/perception/cones", qos_be,
    [this](ev_msgs::msg::ConeArray::UniquePtr msg) {
      stamp_cones_ = now();
      last_cones_ = std::move(msg);
    });

  // ── Core publishers ──
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/path", qos_be);           // control 노드가 구독
  pub_status_ = create_publisher<track_msgs::msg::PlannerStatus>(
    "/planning/status", qos_be);          // 상태 모니터링

  // ── Debug publishers ──
  pub_dbg_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/debug/costmap", qos_be);   // RViz2에서 costmap 시각화
  pub_dbg_raw_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/raw_path", qos_be);  // 후처리 전 경로 시각화

  // ── 10Hz 타이머 ──
  // wall_timer: 시스템 시간 기준 (sim_time과 무관하게 10Hz 보장)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MRPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "MRPlannerNode initialized (10 Hz, MR costmap)");
}

/**
 * @brief 차선 메시지를 파싱하여 Point2D 벡터로 수집
 *
 * CDT 버전에서는 좌/우 차선을 분리했지만,
 * MR 버전에서는 모든 차선 점이 동일한 S극 자석이므로 단일 벡터로 수집한다.
 */
void MRPlannerNode::parse_lanes(
  std::vector<Point2D> & all_lane_pts) const
{
  if (!last_lanes_) return;  // 아직 차선 데이터를 받지 못함
  for (const auto & b : last_lanes_->boundaries) {
    for (const auto & p : b.points) {
      all_lane_pts.push_back({p.x, p.y});
    }
  }
}

/**
 * @brief 콘 메시지를 파싱하여 Point2D 벡터로 수집
 */
void MRPlannerNode::parse_cones(
  std::vector<Point2D> & all_cones) const
{
  if (!last_cones_) return;  // 아직 콘 데이터를 받지 못함
  // velodyne 프레임 → base_link 프레임 오프셋 보정
  const double ox = params_.sensor_tf.tf_x;
  const double oy = params_.sensor_tf.tf_y;
  for (const auto & c : last_cones_->cones) {
    all_cones.push_back({c.position.x + ox, c.position.y + oy});
  }
}

/**
 * @brief perception 데이터 타임아웃(stale) 검사
 *
 * 차선과 콘 모두 perception_ms(기본 300ms) 이내에 수신되지 않으면 STALE.
 * 둘 중 하나라도 유효하면 정상 (차선만 또는 콘만으로도 costmap 생성 가능).
 *
 * @return true: STALE (양쪽 다 타임아웃), false: 정상 (최소 한쪽 유효)
 */
bool MRPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  // 차선 타임아웃 검사
  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;  // ns → ms
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  // 콘 타임아웃 검사
  if (last_cones_) {
    const double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  return !have_perception;  // 둘 다 타임아웃이면 true (STALE)
}

/**
 * @brief 10Hz 타이머 콜백 — MR 플래너 전체 파이프라인 실행
 *
 * 매 100ms마다 호출되어 6단계 파이프라인을 순차 실행한다.
 */
void MRPlannerNode::on_timer()
{
  const auto stamp = now();
  const std::string frame_id = "base_link";  // ego 차량 좌표계

  // ======== Stage 0: Stale Gate ========
  // perception 데이터가 타임아웃이면 STALE 상태 발행 후 조기 반환
  if (check_stale()) {
    auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
    status_msg->header.stamp = stamp;
    status_msg->header.frame_id = frame_id;
    status_msg->status = track_msgs::msg::PlannerStatus::STALE;
    status_msg->reason = "input_stale";
    pub_status_->publish(std::move(status_msg));
    return;  // 파이프라인 중단
  }

  // ======== Stage 1: Input Parse ========
  // 콘과 차선 메시지를 Point2D 벡터로 변환 (L/R 분리 없이 단일 벡터)
  std::vector<Point2D> all_cones, all_lane_pts;
  parse_cones(all_cones);
  parse_lanes(all_lane_pts);

  // ======== Stage 2: Costmap Generation ========
  // 콘/차선을 S극 자석으로 모델링하여 10×10m costmap 생성
  auto costmap = costmap_generator_.generate(all_cones, all_lane_pts, params_);

  // ======== Stage 3: Magnetic Planner ========
  // costmap 위에서 ego(0,0) → heading 방향으로 Greedy 전진 탐색
  auto raw_path = magnetic_planner_.plan(costmap, params_);

  // ======== Stage 4: Postprocess ========
  // prune(가지치기) → smooth(스무딩) → resample(리샘플) → yaw(heading 계산)
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds);

  // ======== Stage 5: Safety Check ========
  // 최대 곡률 검사 → 차량 최소 회전반경 초과 시 INFEASIBLE
  // 곡률 기반 속도 제한: v_curve = sqrt(a_lat_max / curvature)
  auto safety = safety_checker::check(pp_result, params_);

  // ======== Stage 6: Publish ========

  // ── Core: 최종 경로 발행 ──
  auto path_msg = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(pp_result.path, frame_id, stamp));
  pub_path_->publish(std::move(path_msg));

  // ── Core: 플래너 상태 발행 ──
  auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
  status_msg->header.stamp = stamp;
  status_msg->header.frame_id = frame_id;
  status_msg->status = static_cast<uint8_t>(safety.state);
  status_msg->reason = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // ── Debug: costmap을 OccupancyGrid로 발행 (Lazy Publishing) ──
  // 구독자가 있을 때만 변환/발행 (불필요한 연산 절약)
  if (pub_dbg_costmap_->get_subscription_count() > 0 && costmap.valid) {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.stamp = stamp;
    grid_msg->header.frame_id = frame_id;
    // OccupancyGrid 메타데이터 설정
    grid_msg->info.resolution = static_cast<float>(costmap.resolution);
    grid_msg->info.width = costmap.cols;
    grid_msg->info.height = costmap.rows;
    grid_msg->info.origin.position.x = costmap.origin_x;
    grid_msg->info.origin.position.y = costmap.origin_y;
    grid_msg->info.origin.orientation.w = 1.0;
    // double(0~100) → int8_t(0~100) 변환
    grid_msg->data.resize(costmap.rows * costmap.cols);
    for (size_t i = 0; i < costmap.data.size(); ++i) {
      grid_msg->data[i] = static_cast<int8_t>(
        std::clamp(costmap.data[i], 0.0, 100.0));
    }
    pub_dbg_costmap_->publish(std::move(grid_msg));
  }

  // ── Debug: 후처리 전 raw path 발행 (Lazy Publishing) ──
  if (pub_dbg_raw_path_->get_subscription_count() > 0) {
    pub_dbg_raw_path_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(raw_path, frame_id, stamp)));
  }
}

}  // namespace planning_mr_ver

// ROS 2 ComposableNode 등록 매크로
// 이 매크로로 인해 component_container에서 동적 로딩이 가능하다.
// launch 파일에서 ComposableNode로 지정하여 Intra-process 통신을 활성화한다.
RCLCPP_COMPONENTS_REGISTER_NODE(planning_mr_ver::MRPlannerNode)
