// ============================================================================
// pure_pursuit_relative_node.cpp — Pure Pursuit 제어 노드 구현
// ============================================================================
// 이 파일은 노드 오케스트레이터 역할만 담당한다.
// 알고리즘 계산은 pursuit::* 함수에, 시각화는 DebugVisualizer에 위임한다.
//
// [제어 루프 파이프라인] (on_timer, 50Hz)
//   1) 경로 유효성 확인 → 없으면 정지
//   2) planning 상태 확인 → FAIL이면 정지
//   3) 최근접점 탐색
//   4) 전방 곡률 분석 (선감속)
//   5) 속도 적응형 lookahead 계산
//   6) lookahead 목표점 선택
//   7) 안전 조건 확인
//   8) Pure Pursuit 조향각 계산
//   9) 속도 결정 + rate limit
//  10) 제어 명령 발행 (T870 / ERP42) — header에 원본 센서 타임스탬프(velodyne_points) 전파
//  11) 디버그 시각화 발행 (원본 센서 타임스탬프 전파 → topic delay 측정 가능)
// ============================================================================

#include "pp_controller_cpp/nodes/pure_pursuit_relative_node.hpp"
#include "pp_controller_cpp/pursuit/path_query.hpp"
#include "pp_controller_cpp/pursuit/speed_planning.hpp"
#include "pp_controller_cpp/pursuit/steering.hpp"

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
  , cmd_pub_(
      this->create_publisher<t870_msgs::msg::ControlCommand>(
        "/t870/control_command", rclcpp::QoS(1).best_effort()),
      this->create_publisher<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command", rclcpp::QoS(10))
    )
  , debug_viz_(
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

  // [Subscriber] /perception/bboxes (ev_msgs/BBoxArray) — CREEP 모드용 장애물 판정
  bbox_sub_ = this->create_subscription<ev_msgs::msg::BBoxArray>(
    "/perception/bboxes",
    rclcpp::QoS(10).best_effort(),
    [this](ev_msgs::msg::BBoxArray::SharedPtr msg) {
      latest_bboxes_ = msg;
    }
  );

  // CREEP ROI 디버그 퍼블리셔
  pub_dbg_creep_roi_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/pp_debug/creep_roi", rclcpp::QoS(1).best_effort());

  // 3. 제어 루프 타이머 (50Hz = 20ms)
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
// publish_emergency_decel: 점진적 긴급 감속 명령 발행
// ===========================================================================
// emergency_decel_rate로 rate limit을 적용하여 점진적으로 감속한다.
// 역전기력(back-EMF)에 의한 하드웨어 손상을 방지한다.
//
// 호출되는 상황:
//   1) FAIL 연속 카운터가 emergency_stop_count에 도달
//   2) 경로가 없거나 타임아웃
//   3) 목표점 계산 실패
//   4) 목표점이 너무 가깝거나 차량 뒤쪽에 위치
// ===========================================================================
void PurePursuitRelativeNode::publish_emergency_decel(double dt)
{
  const double v_cmd = pursuit::rate_limit_speed(
    0.0, last_cmd_speed_, dt,
    params_.speed.accel_rate,
    params_.safety.emergency_decel_rate);
  last_cmd_speed_ = v_cmd;
  cmd_pub_.publish(v_cmd, 0.0, last_path_stamp_);
  last_control_time_ = this->now();
}

// ===========================================================================
// compute_filtered_remaining: 경로 잔여 길이 이동 평균 필터
// ===========================================================================
// 이동 평균 필터 + 급락 확정 로직 (buffer 동결 방식)
//
// [동작]
//   1) buffer의 기존 avg 계산 (현재 raw는 아직 넣지 않음)
//   2) raw < avg × path_drop_ratio → 급락 의심
//      - pending_raws_에 보류, buffer 동결, avg 반환
//      - N프레임 연속 시 확정 → pending 전부 buffer push, raw 반환
//   3) 정상 → 밀린 pending + raw를 buffer에 push, avg 반환
//
// [설계 의도]
//   단발 노이즈에 급브레이크 방지 (buffer 동결로 avg 오염 없음)
//   진짜 급정지는 60ms(3프레임) 지연 후 즉시 반영
// ===========================================================================
double PurePursuitRelativeNode::compute_filtered_remaining(double raw_remaining)
{
  const auto & sp = params_.speed;

  // 초기화: buffer가 비어있으면 raw를 넣고 반환
  if (path_length_buffer_.empty()) {
    path_length_buffer_.push_back(raw_remaining);
    return raw_remaining;
  }

  // 기존 buffer로 avg 계산 (raw는 아직 넣지 않음)
  double avg = 0.0;
  for (const auto & l : path_length_buffer_) avg += l;
  avg /= static_cast<double>(path_length_buffer_.size());

  const double drop_threshold = avg * sp.path_drop_ratio;

  if (raw_remaining < drop_threshold) {
    // ── 급락 의심: buffer 동결, raw를 pending에 보류 ──
    pending_raws_.push_back(raw_remaining);
    ++path_drop_counter_;

    if (path_drop_counter_ >= sp.path_drop_confirm_count) {
      // 확정: pending 전부 buffer에 push
      for (const double r : pending_raws_) {
        path_length_buffer_.push_back(r);
      }
      while (static_cast<int>(path_length_buffer_.size()) > sp.path_length_filter_size) {
        path_length_buffer_.pop_front();
      }
      pending_raws_.clear();
      path_drop_counter_ = 0;
      return raw_remaining;
    }

    // 미확정: 동결된 avg 반환
    return avg;
  }

  // ── 정상: 밀린 pending + 현재 raw를 buffer에 push ──
  for (const double r : pending_raws_) {
    path_length_buffer_.push_back(r);
  }
  pending_raws_.clear();
  path_drop_counter_ = 0;

  path_length_buffer_.push_back(raw_remaining);
  while (static_cast<int>(path_length_buffer_.size()) > sp.path_length_filter_size) {
    path_length_buffer_.pop_front();
  }

  // 새 avg 계산하여 반환
  double new_avg = 0.0;
  for (const auto & l : path_length_buffer_) new_avg += l;
  new_avg /= static_cast<double>(path_length_buffer_.size());
  return new_avg;
}

// ===========================================================================
// is_forward_clear: 전방 ROI에 bbox 장애물이 없는지 판단
// ===========================================================================
// CREEP 모드 진입 조건: 전방 (0 ~ roi_x) × (±roi_y) 영역에 bbox가 없으면 true
// ===========================================================================
bool PurePursuitRelativeNode::is_forward_clear() const
{
  if (!latest_bboxes_) return false;  // bbox 데이터 없으면 안전하게 false

  const auto & cp = params_.creep;
  for (const auto & b : latest_bboxes_->bboxes) {
    if (b.position.x > 0.0 && b.position.x < cp.roi_x &&
        b.position.y > -cp.roi_y && b.position.y < cp.roi_y)
    {
      return false;  // ROI 내 장애물 있음
    }
  }
  return true;
}

// ===========================================================================
// on_timer: 메인 제어 루프 (50Hz)
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
  double control_dt = 0.02;
  if (last_control_time_.nanoseconds() > 0) {
    control_dt = std::clamp((now - last_control_time_).seconds(), 1e-3, 0.2);
  }
  last_control_time_ = now;

  // ----- CREEP ROI 디버그 마커 (lazy, 항상 발행) -----
  if (pub_dbg_creep_roi_->get_subscription_count() > 0) {
    const auto & cp = params_.creep;
    visualization_msgs::msg::Marker m;
    m.header.stamp = now;
    m.header.frame_id = "base_link";
    m.ns = "creep_roi";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.8f;
    auto pt = [](double x, double y) {
      geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = 0.0; return p;
    };
    m.points.push_back(pt(0.0, -cp.roi_y));
    m.points.push_back(pt(cp.roi_x, -cp.roi_y));
    m.points.push_back(pt(cp.roi_x,  cp.roi_y));
    m.points.push_back(pt(0.0,  cp.roi_y));
    m.points.push_back(pt(0.0, -cp.roi_y));
    pub_dbg_creep_roi_->publish(m);
  }

  // ----- 정지 조건 판정 + 연속 카운터 -----
  bool should_stop = false;
  std::string stop_reason;

  if (!path_fresh()) {
    should_stop = true;
    stop_reason = "Path is missing or stale";
  } else if (latest_status_ == "FAIL - not enough seeds" ||
             latest_status_ == "FAIL - backbone too short" ||
             latest_status_ == "FAIL - no valid path" ||
             latest_status_ == "WARNING - too short valid path")
  {
    should_stop = true;
    stop_reason = latest_status_;
  }

  if (should_stop) {
    ++fail_counter_;
    if (fail_counter_ >= params_.safety.emergency_stop_count) {
      // CREEP: 전방에 장애물 없으면 저속 직진
      if (is_forward_clear()) {
        const double v_cmd = pursuit::rate_limit_speed(
          params_.creep.speed, last_cmd_speed_, control_dt,
          params_.speed.accel_rate, params_.speed.decel_rate);
        last_cmd_speed_ = v_cmd;
        cmd_pub_.publish(v_cmd, 0.0, last_path_stamp_);
        last_control_time_ = this->now();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[PP Relative] CREEP — forward clear, v_cmd=%.2f", v_cmd);
        return;
      }

      // FAIL: 전방에 장애물 있음 → 긴급 감속
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[PP Relative] FAIL — %s, forward blocked (%d consecutive). Emergency decel.",
        stop_reason.c_str(), fail_counter_);
      publish_emergency_decel(control_dt);
    }
    // 카운터 미달 시: 이전 명령 유지 (노이즈 필터링)
    return;
  }

  // 정상 상태 → 카운터 리셋
  fail_counter_ = 0;

  // ----- Pure Pursuit 파이프라인 -----
  const auto & pts = latest_points_;
  const auto & p = params_;

  // 1) 최근접점 탐색
  const size_t nearest_i = pursuit::find_nearest_index(pts);

  // 2) 전방 곡률 분석 (선감속)
  const double preview_kappa = pursuit::compute_preview_curvature(
    pts, nearest_i, p.speed.preview_distance, p.speed.preview_curvature_percentile);

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
    publish_emergency_decel(control_dt);
    return;
  }

  // ----- 안전 조건 확인 -----

  // Ld가 0에 가까우면 kappa = 2*y/Ld^2 에서 0 나누기 발생 방지
  if (Ld_used < 1e-3) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Target distance too small. Stop.");
    publish_emergency_decel(control_dt);
    return;
  }

  // 목표점이 차량 뒤쪽이면 비정상 동작 방지
  if (tx <= p.safety.min_x_target) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PP Relative] Target is behind or too close: tx=%.3f. Stop.", tx);
    publish_emergency_decel(control_dt);
    return;
  }

  // ----- 조향각 계산 -----
  const double delta = pursuit::compute_steering(
    ty, Ld_used, p.vehicle.wheelbase, p.vehicle.delta_max);

  // kappa_pp는 디버그 시각화에 필요
  const double kappa_pp = (2.0 * ty) / (Ld_used * Ld_used);

  // ----- 속도 결정 -----
  // a) 곡률 기반 목표 속도
  const double effective_abs_kappa = std::max(std::abs(kappa_pp), preview_kappa);
  const double v_curvature = pursuit::compute_speed_target(
    effective_abs_kappa, p.speed.min, p.speed.max, p.speed.lateral_accel_limit);

  // b) 제동거리 기반 목표 속도 (경로 길이 이동 평균 필터)
  const double raw_remaining = pursuit::compute_remaining_length(pts, nearest_i);
  const double filtered_remaining = compute_filtered_remaining(raw_remaining);

  // 남은 거리에서 정지 여유거리를 빼서, stop_margin 지점에서 속도 0으로 정지
  const double effective_remaining = std::max(0.0, filtered_remaining - p.speed.stop_margin);
  const double v_path_end = pursuit::compute_path_end_speed(
    effective_remaining, p.speed.decel_rate, 0.0, p.speed.max);

  // c) 두 목표 속도 중 작은 값 선택
  const double v_target = std::min(v_curvature, v_path_end);
  const double v_cmd = pursuit::rate_limit_speed(
    v_target, last_cmd_speed_, control_dt, p.speed.accel_rate, p.speed.decel_rate);
  last_cmd_speed_ = v_cmd;

  // ----- 제어 명령 발행 (T870 + ERP42) -----
  cmd_pub_.publish(v_cmd, delta, last_path_stamp_);

  // ----- 디버깅 로그 (500ms마다 throttle) -----
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[PP Relative] target=(%.2f, %.2f), Ld=%.2f, kappa_pp=%.3f, kappa_prev=%.3f, "
    "v_target=%.2f, v_cmd=%.2f, delta=%.3f, "
    "remain=%.2f, filtered=%.2f, v_curv=%.2f, v_end=%.2f",
    tx, ty, Ld_used, kappa_pp, preview_kappa, v_target, v_cmd, delta,
    raw_remaining, filtered_remaining, v_curvature, v_path_end);

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
