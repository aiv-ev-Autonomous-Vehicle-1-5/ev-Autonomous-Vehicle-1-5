// ============================================================================
// pure_pursuit_relative_node.cpp
// ============================================================================
//
// [개요]
//   상대좌표(base_link 기준) Marker(POINTS)를 입력받아 Pure Pursuit 알고리즘으로
//   T870 차량의 조향각(steering)과 속도(speed)를 계산하는 제어 노드.
//
// [Pure Pursuit 알고리즘 요약]
//   Pure Pursuit는 차량 전방의 "목표점(lookahead point)"을 향해
//   원호(circular arc)를 그리며 이동하도록 조향하는 기하학적 경로 추종 방법이다.
//
//   핵심 공식:
//     곡률(curvature) kappa = 2 * y_target / Ld^2
//     조향각(steering)  delta = atan(L * kappa)
//
//   여기서:
//     y_target = 목표점의 차량 기준 횡방향 거리 (좌: +, 우: -)
//     Ld       = 차량 원점에서 목표점까지의 직선 거리
//     L        = 차량 축간거리 (wheelbase)
//
// [왜 "Relative" 버전인가?]
//   일반적인 Pure Pursuit는 글로벌 좌표계에서 차량 위치와 heading을 알아야 한다.
//   하지만 이 노드는 planning 모듈이 이미 base_link 기준 상대좌표로 경로를
//   생성해주므로, GPS/odometry 없이도 동작한다.
//   이는 대회 규정상 차선 구간에서 GPS 사용이 금지되어 있기 때문에 중요하다.
//
// [데이터 흐름]
//   /planning/path   (visualization_msgs/Marker POINTS, base_link 기준 상대좌표)
//   /planning/status (std_msgs/String — "OK", "FAIL - ..." 등 planning 상태)
//     → [이 노드: Pure Pursuit 계산]
//       → /t870/control_command  (t870_msgs/ControlCommand)  — 실차용
//       → /erp42/control_command (erp42_msgs/ControlCommand) — Gazebo 시뮬레이션용 (lazy)
//       → /pp_debug/lookahead_point (Marker SPHERE)          — 디버그: lookahead 목표점 (lazy)
//       → /pp_debug/pursuit_arc     (Marker LINE_STRIP)      — 디버그: PP 원호 궤적 (lazy)
//
// ============================================================================

#include <cmath>        // sqrt, atan 등 수학 함수
#include <memory>       // shared_ptr (ROS2 노드 공유 포인터용)
#include <string>       // std::string (토픽 이름 저장용)
#include <vector>       // vector (내부적으로 Path 메시지가 사용)
#include <algorithm>    // std::clamp (값 범위 제한용)
#include <limits>       // numeric_limits

#include "rclcpp/rclcpp.hpp"                      // ROS2 C++ 클라이언트 라이브러리
#include "visualization_msgs/msg/marker.hpp"       // 경로 메시지 (POINTS 마커)
#include "t870_msgs/msg/control_command.hpp"       // T870 제어 명령 메시지 (speed, steering)
#include "erp42_msgs/msg/control_command.hpp"      // ERP42 제어 명령 메시지 (Gazebo 시뮬레이션용)
#include "std_msgs/msg/string.hpp"                 // Planning 상태 메시지 (/planning/status)

using std::placeholders::_1;  // std::bind에서 콜백 인자 바인딩용

// -----------------------------------------------------------------------------
// norm2d: 2차원 유클리드 거리 계산
// -----------------------------------------------------------------------------
// 두 점 사이의 거리 또는 원점에서 (x, y)까지의 거리를 계산한다.
// 예: norm2d(3, 4) = 5.0
//
// Pure Pursuit에서는 두 가지 용도로 사용:
//   1) path 상의 연속된 두 점 사이의 거리 (누적 arc length 계산)
//   2) 차량 원점에서 목표점까지의 직선 거리 Ld 계산
// -----------------------------------------------------------------------------
static double norm2d(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

// =============================================================================
// PurePursuitRelativeNode 클래스
// =============================================================================
// rclcpp::Node를 상속하는 ROS2 노드.
//
// 동작 방식:
//   1) /planning/path 토픽을 구독하여 최신 경로를 저장한다.
//   2) 20Hz 타이머(50ms)로 제어 루프를 실행한다.
//   3) 매 제어 주기마다:
//      a) 경로가 유효한지 확인 (비어있지 않고 timeout 이내)
//      b) lookahead 거리만큼 전방의 목표점을 찾는다
//      c) Pure Pursuit 공식으로 조향각을 계산한다
//      d) 안전 조건을 확인하고 제어 명령을 발행한다
// =============================================================================
class PurePursuitRelativeNode : public rclcpp::Node
{
public:
  PurePursuitRelativeNode() : Node("pure_pursuit_relative_node")
  {
    // =========================================================================
    // 1. ROS2 파라미터 선언 (declare)
    // =========================================================================
    // ROS2에서는 파라미터를 먼저 declare 해야 get_parameter로 읽을 수 있다.
    // launch 파일이나 yaml에서 이 값들을 오버라이드할 수 있다.
    //
    // 예: ros2 run pp_controller_cpp pure_pursuit_relative_node
    //       --ros-args -p lookahead:=2.0 -p speed:=0.5
    // =========================================================================

    // --- 토픽 관련 파라미터 ---
    // path_topic: planning 모듈이 발행하는 base_link 기준 상대좌표 경로
    // cmd_topic:  T870 차량 인터페이스가 구독하는 제어 명령 토픽
    this->declare_parameter<std::string>("path_topic", "/planning/path");
    this->declare_parameter<std::string>("cmd_topic",  "/t870/control_command");

    // --- 차량/알고리즘 파라미터 ---
    // wheelbase (L): T870의 앞바퀴 축~뒷바퀴 축 간 거리 [m]
    //   - Ackermann 조향 기하학의 핵심 파라미터
    //   - 값이 크면 같은 곡률에서 조향각이 커짐
    this->declare_parameter<double>("wheelbase", 0.87);

    // lookahead: 이전 고정 lookahead 설정과의 호환용 파라미터
    this->declare_parameter<double>("lookahead", 1.2);
    this->declare_parameter<double>("lookahead_min", 0.8);
    this->declare_parameter<double>("lookahead_max", 1.6);
    this->declare_parameter<double>("lookahead_speed_gain", 0.6);

    // speed: 이전 고정 속도 설정과의 호환용 파라미터
    this->declare_parameter<double>("speed", 0.3);
    this->declare_parameter<double>("speed_min", 0.4);
    this->declare_parameter<double>("speed_max", 1.2);
    this->declare_parameter<double>("lateral_accel_limit", 0.9);
    this->declare_parameter<double>("preview_distance", 2.5);
    this->declare_parameter<double>("accel_rate", 0.8);
    this->declare_parameter<double>("decel_rate", 1.8);

    // delta_max: 최대 조향각 제한 [rad]
    //   - 0.314 rad ≈ 18도
    //   - T870 하드웨어의 물리적 조향 한계를 반영
    //   - 이 값을 초과하는 조향 명령은 clamp됨
    this->declare_parameter<double>("delta_max", 0.314);

    // path_timeout_sec: 경로 메시지 타임아웃 [초]
    //   - 마지막 경로 수신 후 이 시간이 지나면 경로를 "stale"로 판단하고 정지
    //   - planning 노드 장애 시 차량이 무한정 이전 경로를 따라가는 것을 방지
    this->declare_parameter<double>("path_timeout_sec", 0.5);

    // min_x_target: 목표점의 최소 전방 거리 [m]
    //   - 목표점의 x좌표가 이 값 이하이면 "뒤에 있거나 너무 가까움"으로 판단
    //   - 이 경우 조향 계산이 불안정해질 수 있으므로 정지
    this->declare_parameter<double>("min_x_target", 0.05);

    // =========================================================================
    // 2. 파라미터 값 읽기 (get)
    // =========================================================================
    // declare한 파라미터를 멤버 변수에 캐싱한다.
    // 이후 제어 루프에서 매번 get_parameter를 호출하지 않아도 됨.
    // (동적 파라미터 변경이 필요하면 on_set_parameters_callback 추가 필요)
    // =========================================================================
    path_topic_ = this->get_parameter("path_topic").as_string();
    cmd_topic_  = this->get_parameter("cmd_topic").as_string();

    const double legacy_lookahead = this->get_parameter("lookahead").as_double();
    const double legacy_speed = this->get_parameter("speed").as_double();

    L_                = this->get_parameter("wheelbase").as_double();
    delta_max_        = this->get_parameter("delta_max").as_double();
    path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
    min_x_target_     = this->get_parameter("min_x_target").as_double();

    lookahead_min_ = this->get_parameter("lookahead_min").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    lookahead_speed_gain_ = this->get_parameter("lookahead_speed_gain").as_double();
    if (lookahead_min_ <= 0.0) {
      lookahead_min_ = legacy_lookahead;
    }
    if (lookahead_max_ <= 0.0) {
      lookahead_max_ = legacy_lookahead;
    }
    if (lookahead_max_ < lookahead_min_) {
      std::swap(lookahead_min_, lookahead_max_);
    }

    v_max_ = this->get_parameter("speed_max").as_double();
    if (v_max_ <= 0.0) {
      v_max_ = std::max(0.0, legacy_speed);
    }
    v_min_ = std::clamp(this->get_parameter("speed_min").as_double(), 0.0, v_max_);
    lateral_accel_limit_ = std::max(1e-3, this->get_parameter("lateral_accel_limit").as_double());
    preview_distance_ = std::max(lookahead_min_, this->get_parameter("preview_distance").as_double());
    accel_rate_ = std::max(1e-3, this->get_parameter("accel_rate").as_double());
    decel_rate_ = std::max(1e-3, this->get_parameter("decel_rate").as_double());
    last_cmd_speed_ = 0.0;

    // =========================================================================
    // 3. Subscriber / Publisher 생성
    // =========================================================================

    // [Subscriber] /planning/path (visualization_msgs/Marker, POINTS 타입)
    //   - planning 모듈이 발행하는 경로를 수신
    //   - 각 Point의 x/y는 base_link 기준 상대좌표
    //     (x: 전방, y: 좌측이 +)
    //   - QoS: BestEffort, KeepLast(10) — planning publisher가 BestEffort이므로 맞춤
    //   - 콜백 on_path()에서 latest_points_에 저장
    path_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      path_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&PurePursuitRelativeNode::on_path, this, _1)
    );

    // [Subscriber] /planning/status (std_msgs/String)
    //   - planning 모듈이 발행하는 상태 문자열을 수신
    //   - "OK": 정상, "FAIL - ...": 실패 (정지 조건으로 사용)
    //   - 정지 대상 FAIL:
    //     "FAIL - not enough seeds"      — 체인 생성 실패
    //     "FAIL - no valid path"         — A* 경로 없음
    //     "FAIL - too short valid path"  — 경로 길이 < min_path_length
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/planning/status",
      rclcpp::QoS(10).best_effort(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        latest_status_ = msg->data;
      }
    );

    // [Publisher] /t870/control_command (t870_msgs/ControlCommand)
    //   - T870 차량 인터페이스(t870_ros 패키지)가 구독
    //   - 필드: speed (float64, m/s), steering (float64, rad)
    //   - QoS: BestEffort, KeepLast(1) — 제어 명령은 최신 값만 의미 있으므로
    cmd_pub_ = this->create_publisher<t870_msgs::msg::ControlCommand>(
      cmd_topic_,
      rclcpp::QoS(1).best_effort()
    );

    // [Debug Publisher] /pp_debug/lookahead_point (visualization_msgs/Marker, SPHERE)
    //   - PP가 선택한 lookahead 목표점을 RViz2에서 초록색 구로 시각화
    //   - base_link 프레임 기준 (tx, ty, 0) 위치에 표시
    dbg_lookahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pp_debug/lookahead_point",
      rclcpp::QoS(1).best_effort()
    );

    // [Debug Publisher] /pp_debug/pursuit_arc (visualization_msgs/Marker, LINE_STRIP)
    //   - PP가 계산한 곡률(kappa)로부터 차량이 따라갈 예상 원호 궤적을 시각화
    //   - 차량 원점에서 lookahead 목표점까지의 원호를 약 30개 점으로 샘플링
    //   - kappa ≈ 0 (직선)이면 직선 경로를 표시
    dbg_arc_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pp_debug/pursuit_arc",
      rclcpp::QoS(1).best_effort()
    );

    // [Lazy Publisher] /erp42/control_command (erp42_msgs/ControlCommand)
    //   - Gazebo 시뮬레이션의 gazebo_bridge가 구독
    //   - 구독자가 없으면 발행하지 않음 (lazy: get_subscription_count()로 확인)
    //   - 필드: speed (float64), steering (float64), brake (uint8)
    cmd_erp42_pub_ = this->create_publisher<erp42_msgs::msg::ControlCommand>(
      "/erp42/control_command",
      rclcpp::QoS(10)
    );

    // =========================================================================
    // 4. 제어 루프 타이머 (20Hz = 50ms 주기)
    // =========================================================================
    // wall_timer: 시뮬레이션 시간이 아닌 실제 시계 기준 타이머
    // 20Hz는 T870의 제어 주기에 적합한 빈도
    // (너무 빠르면 CAN 통신 부하, 너무 느리면 제어 지연)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitRelativeNode::on_timer, this)
    );

    // 초기화 완료 로그 출력
    RCLCPP_INFO(
      this->get_logger(),
      "[PP Relative] path=%s cmd=%s L=%.2f Ld=[%.2f, %.2f] v=[%.2f, %.2f] a_lat=%.2f delta_max=%.3f",
      path_topic_.c_str(),
      cmd_topic_.c_str(),
      L_,
      lookahead_min_,
      lookahead_max_,
      v_min_,
      v_max_,
      lateral_accel_limit_,
      delta_max_
    );
  }

private:
  // ===========================================================================
  // 콜백: on_path
  // ===========================================================================
  // /planning/path 토픽이 들어올 때마다 호출된다.
  //
  // 하는 일:
  //   1) 수신한 Path 메시지를 latest_path_에 복사 저장
  //   2) 수신 시각을 last_path_time_에 기록 (타임아웃 판단용)
  //
  // 주의:
  //   - 이전 경로는 덮어써진다 (최신 경로만 유지)
  //   - 제어 루프(on_timer)와 비동기적으로 호출되므로,
  //     single-threaded executor에서는 동시 접근 문제 없음
  // ===========================================================================
  void on_path(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    latest_points_ = msg->points;     // points 배열 복사
    last_path_time_ = this->now();    // 현재 ROS 시각 기록
  }

  // ===========================================================================
  // path_fresh: 경로가 유효(신선)한지 판단
  // ===========================================================================
  // 반환값:
  //   true  = 경로가 비어있지 않고, 마지막 수신 후 path_timeout_sec_ 이내
  //   false = 경로가 없거나 오래된 경우
  //
  // 이 함수가 false를 반환하면 차량은 안전하게 정지한다.
  // planning 노드가 죽거나 LiDAR 데이터가 끊긴 경우를 감지하는 역할.
  // ===========================================================================
  bool path_fresh() const
  {
    // 한 번도 경로를 받은 적 없거나 빈 경로
    if (latest_points_.empty()) {
      return false;
    }

    // 마지막 수신 시각으로부터 경과 시간 계산
    const double dt = (this->now() - last_path_time_).seconds();
    return dt <= path_timeout_sec_;
  }

  // ===========================================================================
  // publish_stop: 정지 명령 발행
  // ===========================================================================
  // 속도 0, 조향 0(직진)으로 정지 명령을 발행한다.
  //
  // 호출되는 상황:
  //   1) 경로가 없거나 타임아웃
  //   2) 목표점 계산 실패
  //   3) 목표점이 너무 가깝거나 차량 뒤쪽에 위치
  // ===========================================================================
  void publish_stop()
  {
    t870_msgs::msg::ControlCommand cmd;
    cmd.speed = 0.0;       // 속도 0
    cmd.steering = 0.0;    // 직진 유지
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

  size_t find_nearest_index() const
  {
    const auto & pts = latest_points_;
    size_t nearest_i = 0;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < pts.size(); ++i) {
      const double d2 = pts[i].x * pts[i].x + pts[i].y * pts[i].y;
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest_i = i;
      }
    }

    return nearest_i;
  }

  double compute_dynamic_lookahead(double speed) const
  {
    const double unclamped = lookahead_min_ + lookahead_speed_gain_ * std::max(0.0, speed);
    return std::clamp(unclamped, lookahead_min_, lookahead_max_);
  }

  double compute_preview_curvature(size_t nearest_i, double preview_distance) const
  {
    const auto & pts = latest_points_;
    if (pts.size() < 3 || nearest_i + 2 >= pts.size()) {
      return 0.0;
    }

    size_t end_i = nearest_i;
    double acc = 0.0;
    while (end_i + 1 < pts.size() && acc < preview_distance) {
      const auto & p0 = pts[end_i];
      const auto & p1 = pts[end_i + 1];
      acc += norm2d(p1.x - p0.x, p1.y - p0.y);
      ++end_i;
    }

    if (end_i < nearest_i + 2) {
      return 0.0;
    }

    double max_abs_kappa = 0.0;
    for (size_t i = nearest_i; i + 2 <= end_i; ++i) {
      const auto & a = pts[i];
      const auto & b = pts[i + 1];
      const auto & c = pts[i + 2];

      const double ab = norm2d(b.x - a.x, b.y - a.y);
      const double bc = norm2d(c.x - b.x, c.y - b.y);
      const double ac = norm2d(c.x - a.x, c.y - a.y);
      const double denom = ab * bc * ac;
      if (denom < 1e-9) {
        continue;
      }

      const double cross =
        (b.x - a.x) * (c.y - b.y) -
        (b.y - a.y) * (c.x - b.x);
      const double kappa = 2.0 * std::abs(cross) / denom;
      max_abs_kappa = std::max(max_abs_kappa, kappa);
    }

    return max_abs_kappa;
  }

  double compute_speed_target(double abs_kappa) const
  {
    if (v_max_ <= 0.0) {
      return 0.0;
    }
    if (abs_kappa <= 1e-6) {
      return v_max_;
    }

    const double curvature_limited_speed = std::sqrt(lateral_accel_limit_ / abs_kappa);
    return std::clamp(curvature_limited_speed, v_min_, v_max_);
  }

  double rate_limit_speed(double target_speed, double dt) const
  {
    if (dt <= 0.0) {
      return target_speed;
    }

    if (target_speed >= last_cmd_speed_) {
      return std::min(target_speed, last_cmd_speed_ + accel_rate_ * dt);
    }
    return std::max(target_speed, last_cmd_speed_ - decel_rate_ * dt);
  }

  // ===========================================================================
  // compute_target_relative: 상대좌표 경로에서 lookahead 목표점 계산
  // ===========================================================================
  //
  // [입력]
  //   latest_path_: base_link 기준 상대좌표 경로
  //     - path[i].pose.position.x : 전방 거리 (앞이 +)
  //     - path[i].pose.position.y : 횡방향 거리 (좌가 +)
  //
  // [출력 파라미터]
  //   tx, ty  : 선택된 목표점의 좌표 (base_link 기준)
  //   Ld_used : 차량 원점(0,0)에서 목표점까지의 직선 거리
  //
  // [반환값]
  //   true  = 유효한 목표점을 찾음
  //   false = 경로가 없거나 점이 부족함
  //
  // [알고리즘 상세]
  //
  //   Step 1: 최근접점(nearest point) 탐색
  //     - 경로의 모든 점에 대해 원점(0,0)과의 거리를 계산
  //     - 가장 가까운 점의 인덱스를 nearest_i로 저장
  //     - 이론적으로 path[0]이 가장 가까워야 하지만, planning 모듈의
  //       출력 타이밍에 따라 그렇지 않을 수 있으므로 전체 검색
  //
  //   Step 2: 누적 arc length로 lookahead 지점 탐색
  //     - nearest_i부터 path를 따라가며 연속된 점 사이의 거리를 누적
  //     - 누적 거리가 lookahead_dist 이상이 되는 첫 번째 점을 목표로 선택
  //     - 이렇게 하면 경로의 "곡률"을 반영한 lookahead가 가능
  //       (직선거리가 아닌 경로를 따른 거리 기준)
  //
  //   Step 3: 폴백 (fallback)
  //     - 경로 끝까지 가도 lookahead를 충족하지 못하면 마지막 점을 사용
  //     - 이는 경로가 짧을 때 (예: 장애물이 가까울 때) 발생
  //
  //   [시각적 설명]
  //
  //        목표점 (tx, ty)
  //           *
  //          /|
  //    Ld  /  |  ty (횡방향 오프셋)
  //       /   |
  //      /    |
  //     *-----+
  //   (0,0)  tx
  //   차량    (전방 거리)
  //
  // ===========================================================================
  bool compute_target_relative(
    size_t nearest_i,
    double lookahead_dist,
    double &tx,
    double &ty,
    double &Ld_used)
  {
    // 경로 유효성 확인
    if (!path_fresh()) {
      return false;
    }

    const auto &pts = latest_points_;

    // 최소 2개의 점이 필요 (1개로는 방향을 결정할 수 없음)
    if (pts.size() < 2) {
      return false;
    }

    // -----------------------------------------------------------------
    // Step 2: nearest_i부터 경로를 따라가며 누적 거리로 lookahead 찾기
    // -----------------------------------------------------------------
    // acc: 누적 arc length (경로를 따른 거리)
    //
    // 예시: path = [P0, P1, P2, P3, P4], nearest_i = 1, Ld = 2.0
    //   acc += |P1→P2| = 0.8  → 아직 부족
    //   acc += |P2→P3| = 0.9  → acc=1.7, 아직 부족
    //   acc += |P3→P4| = 0.5  → acc=2.2 ≥ 2.0 → P4를 목표점으로!
    // -----------------------------------------------------------------
    double acc = 0.0;

    for (size_t i = nearest_i; i + 1 < pts.size(); ++i) {
      const double x0 = pts[i].x;
      const double y0 = pts[i].y;
      const double x1 = pts[i + 1].x;
      const double y1 = pts[i + 1].y;

      acc += norm2d(x1 - x0, y1 - y0);  // 두 점 사이의 유클리드 거리 누적

      if (acc >= lookahead_dist) {
        tx = x1;
        ty = y1;
        // Ld_used는 경로를 따른 거리(acc)가 아닌,
        // 차량 원점에서 목표점까지의 "직선 거리"를 사용한다.
        // Pure Pursuit 공식에서 Ld는 직선 거리여야 하기 때문.
        Ld_used = norm2d(tx, ty);
        return true;
      }
    }

    // -----------------------------------------------------------------
    // Step 3: 경로 끝까지 가도 lookahead를 충족 못하면 마지막 점 사용
    // -----------------------------------------------------------------
    // 경로가 짧은 경우의 폴백 처리.
  // 이 경우 Ld_used가 목표 lookahead보다 작아질 수 있어 조향이 예민해질 수 있다.
    // -----------------------------------------------------------------
    tx = pts.back().x;
    ty = pts.back().y;
    Ld_used = norm2d(tx, ty);
    return true;
  }

  // ===========================================================================
  // on_timer: 메인 제어 루프 (20Hz)
  // ===========================================================================
  // 50ms마다 호출되어 다음 순서로 제어를 수행한다:
  //
  //   1) 경로 유효성 확인 → 없으면 정지
  //   2) lookahead 목표점 계산 → 실패하면 정지
  //   3) 안전 조건 확인 → 위반 시 정지
  //   4) Pure Pursuit 조향각 계산
  //   5) 제어 명령 발행
  //
  // [정지 조건 정리]
  //   - 경로 미수신 또는 타임아웃 (path_timeout_sec_ 초과)
  //   - planning 상태 FAIL (not enough seeds / no valid path / too short valid path)
  //   - 경로 점이 2개 미만
  //   - 목표점까지 거리가 0에 가까움 (Ld < 1e-3, 0으로 나누기 방지)
  //   - 목표점이 차량 뒤쪽 (tx ≤ min_x_target_)
  // ===========================================================================
  void on_timer()
  {
    const auto now = this->now();
    double control_dt = 0.05;
    if (last_control_time_.nanoseconds() > 0) {
      control_dt = std::clamp((now - last_control_time_).seconds(), 1e-3, 0.2);
    }
    last_control_time_ = now;

    // ----- 경로 유효성 확인 -----
    // path가 없거나 오래됐으면 정지
    if (!path_fresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,  // 1초에 한 번만 경고 출력 (로그 폭주 방지)
        "[PP Relative] Path is missing or stale. Stop."
      );
      publish_stop();
      return;
    }

    // ----- planning 상태 확인 -----
    // planning이 FAIL 상태를 발행하면 즉시 정지
    //   - "FAIL - not enough seeds"     : 체인 생성 실패 (좌/우 경계 없음)
    //   - "FAIL - no valid path"        : A* 경로 탐색 실패
    //   - "FAIL - too short valid path" : 경로 길이 < min_path_length (2.5m)
    if (latest_status_ == "FAIL - not enough seeds" ||
        latest_status_ == "FAIL - no valid path" ||
        latest_status_ == "FAIL - too short valid path")
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Planning status: %s. Stop.",
        latest_status_.c_str()
      );
      publish_stop();
      return;
    }

    const size_t nearest_i = find_nearest_index();
    const double preview_kappa = compute_preview_curvature(nearest_i, preview_distance_);
    const double preview_speed_target = compute_speed_target(preview_kappa);
    const double lookahead_cmd = compute_dynamic_lookahead(preview_speed_target);

    // ----- 목표점 계산 -----
    double tx = 0.0;       // 목표점 x (전방 거리)
    double ty = 0.0;       // 목표점 y (횡방향 거리)
    double Ld_used = 0.0;  // 실제 사용된 lookahead 거리 (직선)

    if (!compute_target_relative(nearest_i, lookahead_cmd, tx, ty, Ld_used)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Failed to compute target. Stop."
      );
      publish_stop();
      return;
    }

    // -----------------------------------------------------------------
    // 안전 조건 1: 목표점까지 거리가 0에 너무 가까운 경우
    // -----------------------------------------------------------------
    // Ld가 0에 가까우면 kappa = 2*y / Ld^2 에서 0 나누기가 발생하여
    // 조향각이 무한대로 발산할 수 있다. 이를 방지하기 위한 가드.
    // -----------------------------------------------------------------
    if (Ld_used < 1e-3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Target distance too small. Stop."
      );
      publish_stop();
      return;
    }

    // -----------------------------------------------------------------
    // 안전 조건 2: 목표점이 차량 뒤쪽에 있는 경우
    // -----------------------------------------------------------------
    // base_link 좌표계에서 x > 0이 전방이다.
    // tx ≤ min_x_target_ 이면 목표점이 차량 뒤쪽 또는 바로 옆에 있다는 뜻.
    // 이 상태에서 Pure Pursuit을 적용하면 180도 회전 등 비정상 동작이 발생.
    // -----------------------------------------------------------------
    if (tx <= min_x_target_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Target is behind or too close: tx=%.3f. Stop.",
        tx
      );
      publish_stop();
      return;
    }

    // =================================================================
    // Pure Pursuit 조향각 계산
    // =================================================================
    //
    // [수학적 유도]
    //
    // 차량이 반지름 R인 원호를 따라 이동한다고 가정하면:
    //   - 목표점 (tx, ty)는 이 원호 위에 있어야 한다
    //   - 원의 중심은 차량 좌측 (y > 0) 또는 우측 (y < 0)에 있다
    //
    // 기하학적 관계로부터:
    //   Ld^2 = tx^2 + ty^2          ... (목표점까지의 직선거리)
    //   R = Ld^2 / (2 * ty)         ... (원의 반지름, 기하학적 유도)
    //
    // 따라서 곡률(curvature):
    //   kappa = 1/R = 2 * ty / Ld^2
    //
    // Ackermann 조향 기하학에서 조향각:
    //   delta = atan(L / R) = atan(L * kappa)
    //
    // 여기서:
    //   L  = wheelbase (축간거리)
    //   ty > 0 → 좌회전 (delta > 0)
    //   ty < 0 → 우회전 (delta < 0)
    //
    // [상대좌표의 장점]
    //   일반 Pure Pursuit에서는 목표점을 차량 좌표계로 변환하기 위해
    //   차량의 글로벌 좌표 (X, Y, yaw)가 필요하다.
    //   하지만 여기서는 path가 이미 base_link 기준이므로
    //   tx, ty를 그대로 사용할 수 있다 → yaw 변환 불필요!
    //
    // =================================================================
    const double y_v = ty;  // 목표점의 횡방향 거리 (Pure Pursuit 핵심 입력)

    // 곡률 계산: kappa = 2 * y / Ld^2
    const double kappa_pp = (2.0 * y_v) / (Ld_used * Ld_used);

    // 조향각 계산: delta = atan(L * kappa)
    double delta = std::atan(L_ * kappa_pp);

    // 최대 조향각 제한 [-delta_max_, +delta_max_]
    // T870 하드웨어의 물리적 한계를 초과하지 않도록 clamp
    delta = std::clamp(delta, -delta_max_, delta_max_);

    const double effective_abs_kappa = std::max(std::abs(kappa_pp), preview_kappa);
    const double v_target = compute_speed_target(effective_abs_kappa);
    const double v_cmd = rate_limit_speed(v_target, control_dt);
    last_cmd_speed_ = v_cmd;

    // =================================================================
    // T870 제어 명령 발행
    // =================================================================
    // ControlCommand 메시지 구성:
    //   - speed:    곡률 기반으로 계산된 목표 속도 [m/s]
    //   - steering: 조향각 [rad] (Pure Pursuit으로 계산된 값)
    // =================================================================
    t870_msgs::msg::ControlCommand cmd;
    cmd.speed = v_cmd;
    cmd.steering = delta;
    cmd_pub_->publish(cmd);

    // Gazebo 시뮬레이션용 ERP42 명령 (구독자가 있을 때만)
    if (cmd_erp42_pub_->get_subscription_count() > 0) {
      erp42_msgs::msg::ControlCommand erp_cmd;
      erp_cmd.speed = v_cmd;
      erp_cmd.steering = delta;
      erp_cmd.brake = 0;
      cmd_erp42_pub_->publish(erp_cmd);
    }

    // 디버깅 로그 (500ms마다 출력, 터미널 가독성을 위해 throttle)
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "[PP Relative] target=(%.2f, %.2f), Ld=%.2f, kappa_pp=%.3f, kappa_prev=%.3f, v_target=%.2f, v_cmd=%.2f, delta=%.3f",
      tx, ty, Ld_used, kappa_pp, preview_kappa, v_target, v_cmd, delta
    );

    // =================================================================
    // RViz2 디버그 시각화 발행
    // =================================================================

    // --- 1) Lookahead 목표점 (SPHERE) ---
    // PP가 선택한 lookahead target point를 초록색 구로 표시
    if (dbg_lookahead_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "base_link";
      m.header.stamp = now;
      m.ns = "pp_debug";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = tx;
      m.pose.position.y = ty;
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.15;
      m.scale.y = 0.15;
      m.scale.z = 0.15;
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      dbg_lookahead_pub_->publish(m);
    }

    // --- 2) PP 원호 궤적 (LINE_STRIP) ---
    // kappa_pp로부터 차량이 따라갈 예상 원호를 샘플링하여 노란색 선으로 표시
    // kappa = 2*y/Ld^2 → R = 1/kappa, 회전 중심 = (0, R)
    // kappa ≈ 0이면 직선 경로로 폴백
    if (dbg_arc_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::Marker arc;
      arc.header.frame_id = "base_link";
      arc.header.stamp = now;
      arc.ns = "pp_debug";
      arc.id = 1;
      arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
      arc.action = visualization_msgs::msg::Marker::ADD;
      arc.pose.orientation.w = 1.0;
      arc.scale.x = 0.03;  // 선 두께
      arc.color.r = 1.0f;
      arc.color.g = 1.0f;
      arc.color.b = 0.0f;
      arc.color.a = 0.8f;

      constexpr int N_ARC = 30;  // 원호 샘플 수

      if (std::abs(kappa_pp) < 1e-6) {
        // 직선: 차량 원점 → 목표점까지 직선 보간
        for (int i = 0; i <= N_ARC; ++i) {
          const double t = static_cast<double>(i) / N_ARC;
          geometry_msgs::msg::Point p;
          p.x = tx * t;
          p.y = ty * t;
          p.z = 0.0;
          arc.points.push_back(p);
        }
      } else {
        // 원호: 회전 중심 (cx, cy) = (0, R), 반지름 |R|
        // R = 1/kappa_pp (좌회전: R>0, 우회전: R<0)
        const double R = 1.0 / kappa_pp;
        const double cx = 0.0;
        const double cy = R;
        const double abs_R = std::abs(R);

        // 시작각: 차량 원점(0,0)에서의 각도 = atan2(0 - cy, 0 - cx)
        const double theta_start = std::atan2(-cy, -cx);
        // 종료각: 목표점(tx, ty)에서의 각도 = atan2(ty - cy, tx - cx)
        const double theta_end = std::atan2(ty - cy, tx - cx);

        // 각도 차이 계산 (회전 방향 고려)
        double dtheta = theta_end - theta_start;
        // kappa_pp > 0 (좌회전): 반시계 방향 → dtheta > 0이어야 함
        // kappa_pp < 0 (우회전): 시계 방향 → dtheta < 0이어야 함
        if (kappa_pp > 0.0) {
          while (dtheta < 0.0) dtheta += 2.0 * M_PI;
          while (dtheta > 2.0 * M_PI) dtheta -= 2.0 * M_PI;
        } else {
          while (dtheta > 0.0) dtheta -= 2.0 * M_PI;
          while (dtheta < -2.0 * M_PI) dtheta += 2.0 * M_PI;
        }

        for (int i = 0; i <= N_ARC; ++i) {
          const double t = static_cast<double>(i) / N_ARC;
          const double theta = theta_start + dtheta * t;
          geometry_msgs::msg::Point p;
          p.x = cx + abs_R * std::cos(theta);
          p.y = cy + abs_R * std::sin(theta);
          p.z = 0.0;
          arc.points.push_back(p);
        }
      }

      dbg_arc_pub_->publish(arc);
    }
  }

private:
  // ======================== 멤버 변수 ========================

  // --- 토픽 이름 ---
  std::string path_topic_;   // 경로 입력 토픽 (기본: /planning/path)
  std::string cmd_topic_;    // 제어 출력 토픽 (기본: /t870/control_command)

  // --- 차량/알고리즘 파라미터 ---
  double L_{0.87};               // wheelbase: 축간거리 [m]
  double lookahead_min_{0.8};    // 속도 연동 lookahead 최소값 [m]
  double lookahead_max_{1.6};    // 속도 연동 lookahead 최대값 [m]
  double lookahead_speed_gain_{0.6};  // 속도 1m/s 증가당 lookahead 증가량 [m]
  double v_min_{0.4};            // 최소 주행 속도 [m/s]
  double v_max_{1.2};            // 최대 주행 속도 [m/s]
  double lateral_accel_limit_{0.9};   // 곡률 기반 감속용 최대 횡가속 [m/s^2]
  double preview_distance_{2.5};      // 선감속용 전방 curvature preview 거리 [m]
  double accel_rate_{0.8};       // 가속 rate limit [m/s^2]
  double decel_rate_{1.8};       // 감속 rate limit [m/s^2]
  double delta_max_{0.314};      // 최대 조향각 [rad] (≈18도)
  double path_timeout_sec_{0.5}; // 경로 타임아웃 [초]
  double min_x_target_{0.05};    // 목표점 최소 전방 거리 [m]
  double last_cmd_speed_{0.0};   // 직전 제어 주기의 속도 명령 [m/s]

  // --- ROS2 통신 객체 ---
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr path_sub_;  // 경로 구독자 (POINTS 마커)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;          // planning 상태 구독자
  rclcpp::Publisher<t870_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;    // 제어 명령 발행자
  rclcpp::Publisher<erp42_msgs::msg::ControlCommand>::SharedPtr cmd_erp42_pub_;  // ERP42 Gazebo 시뮬레이션용 (lazy)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dbg_lookahead_pub_;  // 디버그: lookahead 목표점 (SPHERE)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dbg_arc_pub_;        // 디버그: PP 원호 궤적 (LINE_STRIP)
  rclcpp::TimerBase::SharedPtr timer_;                                      // 20Hz 제어 루프 타이머

  // --- 상태 저장 ---
  std::vector<geometry_msgs::msg::Point> latest_points_;    // 가장 최근 수신한 경로 점 배열
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};       // 경로 마지막 수신 시각
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};    // 직전 제어 루프 시각
  std::string latest_status_{""};                           // 최신 planning 상태 ("OK", "FAIL - ..." 등)
};

// =============================================================================
// main: ROS2 노드 실행 진입점
// =============================================================================
// 1) rclcpp::init()    - ROS2 미들웨어 초기화
// 2) rclcpp::spin()    - 노드 실행 (콜백 + 타이머 처리, Ctrl+C까지 블록)
// 3) rclcpp::shutdown() - 정리 및 종료
//
// single-threaded executor가 기본이므로 on_path와 on_timer는 동시에 실행되지 않는다.
// =============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitRelativeNode>());
  rclcpp::shutdown();
  return 0;
}
