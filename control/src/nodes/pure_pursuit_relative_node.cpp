// ============================================================================
// pure_pursuit_relative_node.cpp — Pure Pursuit 제어 노드 구현
// ============================================================================
// 이 파일은 노드 오케스트레이터 역할만 담당한다.
// 알고리즘 계산은 pursuit::* 함수에, 시각화는 DebugVisualizer에 위임한다.
//
// [제어 루프 파이프라인] (on_timer, 20Hz)
//   1) 경로 유효성 확인 → 없으면 정지
//   2) planning 상태 확인 → FAIL이면 정지
//   3) 최근접점 탐색
//   4) 전방 곡률 분석 (선감속)
//   5) 속도 적응형 lookahead 계산
//   6) lookahead 목표점 선택
//   7) 안전 조건 확인
//   8) Pure Pursuit 조향각 계산
//   9) 속도 결정 + rate limit
//  10) 제어 명령 발행 (T870 / ERP42)
//  11) 디버그 시각화 발행 (원본 센서 타임스탬프 전파 → topic delay 측정 가능)
// ============================================================================

#include "pp_controller_cpp/nodes/pure_pursuit_relative_node.hpp"
#include "pp_controller_cpp/pursuit/pursuit_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

using std::placeholders::_1;

namespace pp_controller_cpp
{

// ===========================================================================
// 생성자
// ===========================================================================
PurePursuitRelativeNode::PurePursuitRelativeNode(
  const rclcpp::NodeOptions & options)
  : Node("pure_pursuit_relative_node", options)
  , debug_viz_(
      // 디버그 퍼블리셔를 미리 생성하여 DebugVisualizer에 주입
      this->create_publisher<visualization_msgs::msg::Marker>(
        "/pp_debug/lookahead_point", rclcpp::QoS(1).best_effort()),
      this->create_publisher<visualization_msgs::msg::Marker>(
        "/pp_debug/pursuit_arc", rclcpp::QoS(1).best_effort())
    )
{
  // 1. 파라미터 로드
  params_.load(this);

  // 2. Subscriber 생성

  // [Subscriber] /planning/path (visualization_msgs/Marker, POINTS 타입)
  //   - planning 모듈이 발행하는 경로를 수신
  //   - 각 Point의 x/y는 base_link 기준 상대좌표 (x: 전방, y: 좌측이 +)
  //   - QoS: BestEffort, KeepLast(10) — planning publisher가 BestEffort이므로 맞춤
  path_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    params_.topics.path_topic,
    rclcpp::QoS(10).best_effort(),
    std::bind(&PurePursuitRelativeNode::on_path, this, _1)
  );

  // [Subscriber] /planning/status (std_msgs/String)
  //   - "OK": 정상, "FAIL - ...": 실패 (정지 조건), "WARNING - ...": 경고 (경로 추종 계속)
  status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/planning/status",
    rclcpp::QoS(10).best_effort(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      latest_status_ = msg->data;
    }
  );

  // 3. Publisher 생성

  // [Publisher] /t870/control_command — 실차 제어 명령
  cmd_pub_ = this->create_publisher<t870_msgs::msg::ControlCommand>(
    params_.topics.cmd_topic,
    rclcpp::QoS(1).best_effort()
  );

  // [Publisher] /erp42/control_command — Gazebo 시뮬레이션용 (lazy)
  cmd_erp42_pub_ = this->create_publisher<erp42_msgs::msg::ControlCommand>(
    "/erp42/control_command",
    rclcpp::QoS(10)
  );

  // 4. 제어 루프 타이머 (50Hz = 20ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&PurePursuitRelativeNode::on_timer, this)
  );

  // 초기화 완료 로그
  RCLCPP_INFO(
    this->get_logger(),
    "[PP Relative] path=%s cmd=%s L=%.2f Ld=[%.2f, %.2f] v=[%.2f, %.2f] a_lat=%.2f delta_max=%.3f",
    params_.topics.path_topic.c_str(),
    params_.topics.cmd_topic.c_str(),
    params_.vehicle.wheelbase,
    params_.lookahead.min,
    params_.lookahead.max,
    params_.speed.min,
    params_.speed.max,
    params_.speed.lateral_accel_limit,
    params_.vehicle.delta_max
  );
}

// ===========================================================================
// on_path: 경로 콜백
// ===========================================================================
void PurePursuitRelativeNode::on_path(
  const visualization_msgs::msg::Marker::SharedPtr msg)
{
  latest_points_ = msg->points;
  last_path_time_ = this->now();            // stale 검사용
  last_path_stamp_ = msg->header.stamp;     // 원본 센서 타임스탬프 전파
}

// ===========================================================================
// path_fresh: 경로 유효성 판단
// ===========================================================================
// planning 노드가 죽거나 LiDAR 데이터가 끊긴 경우를 감지하는 역할.
// ===========================================================================
bool PurePursuitRelativeNode::path_fresh() const
{
  if (latest_points_.empty()) {
    return false;
  }
  const double dt = (this->now() - last_path_time_).seconds();
  return dt <= params_.safety.path_timeout_sec;
}

// ===========================================================================
// publish_stop: 정지 명령 발행
// ===========================================================================
// 호출되는 상황:
//   1) 경로가 없거나 타임아웃
//   2) 목표점 계산 실패
//   3) 목표점이 너무 가깝거나 차량 뒤쪽에 위치
// ===========================================================================
void PurePursuitRelativeNode::publish_stop()
{
  t870_msgs::msg::ControlCommand cmd;
  cmd.speed = 0.0;
  cmd.steering = 0.0;
  cmd_pub_->publish(cmd);

  // Gazebo 시뮬레이션용 ERP42 명령 (구독자가 있을 때만)
  if (cmd_erp42_pub_->get_subscription_count() > 0) {
    erp42_msgs::msg::ControlCommand erp_cmd;
    erp_cmd.speed = 0.0;
    erp_cmd.steering = 0.0;
    erp_cmd.brake = 1;
    cmd_erp42_pub_->publish(erp_cmd);
  }

  last_cmd_speed_ = 0.0;
  last_control_time_ = this->now();
}

// ===========================================================================
// on_timer: 메인 제어 루프 (20Hz)
// ===========================================================================
//
// [정지 조건]
//   - 경로 미수신 또는 타임아웃 (path_timeout_sec 초과)
//   - planning 상태 FAIL
//   - 경로 점이 2개 미만
//   - 목표점까지 거리가 0에 가까움 (Ld < 1e-3)
//   - 목표점이 차량 뒤쪽 (tx ≤ min_x_target)
// ===========================================================================
void PurePursuitRelativeNode::on_timer()
{
  const auto now = this->now();
  double control_dt = 0.05;
  if (last_control_time_.nanoseconds() > 0) {
    control_dt = std::clamp((now - last_control_time_).seconds(), 1e-3, 0.2);
  }
  last_control_time_ = now;

  // ----- 경로 유효성 확인 -----
  if (!path_fresh()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Path is missing or stale. Stop.");
    publish_stop();
    return;
  }

  // ----- planning 상태 확인 -----
  if (latest_status_ == "FAIL - not enough seeds" ||
      latest_status_ == "FAIL - no valid path" ||
      latest_status_ == "FAIL - too short valid path")
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Planning status: %s. Stop.", latest_status_.c_str());
    publish_stop();
    return;
  }

  // ----- Pure Pursuit 파이프라인 -----
  const auto & pts = latest_points_;
  const auto & p = params_;

  // 1) 최근접점 탐색
  const size_t nearest_i = pursuit::find_nearest_index(pts);

  // 2) 전방 곡률 분석 (선감속)
  const double preview_kappa = pursuit::compute_preview_curvature(
    pts, nearest_i, p.speed.preview_distance);

  // 3) preview 기반 목표 속도 → 동적 lookahead 계산
  const double preview_speed_target = pursuit::compute_speed_target(
    preview_kappa, p.speed.min, p.speed.max, p.speed.lateral_accel_limit);
  const double lookahead_cmd = pursuit::compute_dynamic_lookahead(
    preview_speed_target, p.lookahead.min, p.lookahead.max, p.lookahead.speed_gain);

  // 4) lookahead 목표점 계산
  double tx = 0.0, ty = 0.0, Ld_used = 0.0;
  if (!pursuit::compute_target_relative(pts, nearest_i, lookahead_cmd, tx, ty, Ld_used)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Failed to compute target. Stop.");
    publish_stop();
    return;
  }

  // ----- 안전 조건 확인 -----

  // Ld가 0에 가까우면 kappa = 2*y/Ld^2 에서 0 나누기 발생 방지
  if (Ld_used < 1e-3) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Target distance too small. Stop.");
    publish_stop();
    return;
  }

  // 목표점이 차량 뒤쪽이면 비정상 동작 방지
  if (tx <= p.safety.min_x_target) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Target is behind or too close: tx=%.3f. Stop.", tx);
    publish_stop();
    return;
  }

  // ----- 조향각 계산 -----
  const double delta = pursuit::compute_steering(
    ty, Ld_used, p.vehicle.wheelbase, p.vehicle.delta_max);

  // kappa_pp는 디버그 시각화에 필요
  const double kappa_pp = (2.0 * ty) / (Ld_used * Ld_used);

  // ----- 속도 결정 -----
  const double effective_abs_kappa = std::max(std::abs(kappa_pp), preview_kappa);
  const double v_target = pursuit::compute_speed_target(
    effective_abs_kappa, p.speed.min, p.speed.max, p.speed.lateral_accel_limit);
  const double v_cmd = pursuit::rate_limit_speed(
    v_target, last_cmd_speed_, control_dt, p.speed.accel_rate, p.speed.decel_rate);
  last_cmd_speed_ = v_cmd;

  // ----- T870 제어 명령 발행 -----
  t870_msgs::msg::ControlCommand cmd;
  cmd.speed = v_cmd;
  cmd.steering = delta;
  cmd_pub_->publish(cmd);

  // ----- ERP42 시뮬레이션 명령 (lazy) -----
  if (cmd_erp42_pub_->get_subscription_count() > 0) {
    erp42_msgs::msg::ControlCommand erp_cmd;
    erp_cmd.speed = v_cmd;
    erp_cmd.steering = delta;
    erp_cmd.brake = 0;
    cmd_erp42_pub_->publish(erp_cmd);
  }

  // ----- 디버깅 로그 (500ms마다 throttle) -----
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[PP Relative] target=(%.2f, %.2f), Ld=%.2f, kappa_pp=%.3f, kappa_prev=%.3f, "
    "v_target=%.2f, v_cmd=%.2f, delta=%.3f",
    tx, ty, Ld_used, kappa_pp, preview_kappa, v_target, v_cmd, delta);

  // ----- 디버그 시각화 발행 (lazy) — 원본 센서 타임스탬프 전파 -----
  debug_viz_.publish_lookahead_point(tx, ty, last_path_stamp_);
  debug_viz_.publish_pursuit_arc(tx, ty, kappa_pp, last_path_stamp_);
}

}  // namespace pp_controller_cpp

// ===========================================================================
// ROS 2 컴포넌트 등록
// ===========================================================================
// ComposableNode로 등록하여 런치 파일에서 ComposableNodeContainer로 로드 가능.
// rclcpp_components_register_node()가 자동으로 standalone 실행 파일도 생성한다.
// ===========================================================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pp_controller_cpp::PurePursuitRelativeNode)
