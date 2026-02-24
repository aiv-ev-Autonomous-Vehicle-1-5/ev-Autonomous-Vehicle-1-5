// ============================================================
// LocalPlannerNode 구현 파일
//
// 이 파일의 구조:
//   1. 생성자: 파라미터 로드 → 구독 생성 → 퍼블리셔 생성 → 타이머 생성
//   2. 입력 파싱 헬퍼 (parse_lanes, parse_cones, parse_obstacles)
//   3. Stale 검사 (check_stale)
//   4. 메인 파이프라인 (on_timer): 10단계로 경로 생성
//   5. RCLCPP_COMPONENTS_REGISTER_NODE 매크로: 컴포넌트 등록
// ============================================================

#include "track_planning/nodes/local_planner_node.hpp"
#include "track_planning/common/geometry.hpp"       // dist(), to_path_msg() 등 기하 유틸리티
#include "track_planning/common/debug_publish.hpp"  // to_bool_msg(), to_string_msg() 등
#include "track_planning/planner/mode_selector.hpp" // mode_selector::select(): 경로 모드 결정
#include "track_planning/safety/safety_checker.hpp" // safety_checker::check(): 안전 검사

// rclcpp_components: RCLCPP_COMPONENTS_REGISTER_NODE 매크로 제공
// 이 매크로로 등록해야 component_container가 런타임에 이 노드를 동적 로드할 수 있다.
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>   // std::chrono::milliseconds
#include <cmath>    // std::atan2, std::cos, std::sin, std::sqrt
#include <string>

namespace track_planning
{

// ============================================================
// 생성자
//
// 호출 순서:
//   1) Node("local_planner_node", options) 부모 클래스 초기화
//      - "local_planner_node": ROS 2 노드 이름 (ros2 node list에 표시됨)
//      - options: NodeOptions 객체 (use_intra_process_comms 플래그 포함)
//   2) 타임스탬프 멤버 초기화 리스트
//      - (0, 0, RCL_ROS_TIME): 에포크 기준 0초, ROS 시간 클럭 사용
//      - 초기값이 0이면 처음 check_stale() 호출 시 항상 stale로 판정됨
//   3) 본문: 파라미터 로드 → 구독 → 퍼블리셔 → 타이머
// ============================================================
LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options),
  // 타임스탬프 초기화: RCL_ROS_TIME = ROS 클럭 소스 사용 명시
  stamp_lanes_(0, 0, RCL_ROS_TIME),
  stamp_cones_(0, 0, RCL_ROS_TIME),
  stamp_obstacles_(0, 0, RCL_ROS_TIME),
  stamp_odom_(0, 0, RCL_ROS_TIME)
{
  // ---- 파라미터 로드 ----
  // params_.load(this): ROS 2 파라미터 서버에서 YAML 설정값을 읽어
  // PlanningParams 구조체의 각 필드에 채워 넣는다.
  // planning.yaml에 정의된 타임아웃, 코리더 파라미터 등이 여기서 로드됨.
  params_.load(this);

  // ============================================================
  // Subscription 생성 (UniquePtr 콜백 → intra-process Zero-copy)
  //
  // 핵심 개념:
  //   - 일반 콜백: std::function<void(std::shared_ptr<MsgT>)>
  //     → 항상 복사 또는 shared_ptr 참조 카운트 증가 발생
  //   - UniquePtr 콜백: std::function<void(MsgT::UniquePtr)>
  //     → intra-process 모드에서 메모리 복사 없이 소유권만 이전(std::move)
  //     → use_intra_process_comms: True 조건에서만 Zero-copy 효과 발생
  //
  // 콜백 구조:
  //   [this](MsgType::UniquePtr msg) {
  //     stamp_xxx_ = now();          // 수신 시각 기록 (stale 검사용)
  //     last_xxx_ = std::move(msg);  // 소유권 이전 (기존 데이터 자동 해제)
  //   }
  //
  // rclcpp::QoS(1): 깊이(depth) 1의 기본 QoS (Reliable, Volatile)
  //   → 가장 최신 메시지 1개만 유지하면 되므로 depth=1로 충분
  // ============================================================

  // /perception/lane_boundaries → 카메라가 인식한 차선 경계 배열
  sub_lanes_ = create_subscription<track_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", rclcpp::QoS(1),
    [this](track_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();            // 수신 시각 갱신
      last_lanes_ = std::move(msg);    // UniquePtr 소유권 이전 (Zero-copy)
    });

  // /perception/cones → LiDAR 기반 콘(교통 콘) 위치 배열
  sub_cones_ = create_subscription<track_msgs::msg::ConeArray>(
    "/perception/cones", rclcpp::QoS(1),
    [this](track_msgs::msg::ConeArray::UniquePtr msg) {
      stamp_cones_ = now();
      last_cones_ = std::move(msg);
    });

  // /perception/obstacles → LiDAR 기반 장애물(드럼 등) 위치 배열
  sub_obstacles_ = create_subscription<track_msgs::msg::ObstacleArray>(
    "/perception/obstacles", rclcpp::QoS(1),
    [this](track_msgs::msg::ObstacleArray::UniquePtr msg) {
      stamp_obstacles_ = now();
      last_obstacles_ = std::move(msg);
    });

  // /odometry → 자차의 위치(x,y), 자세(quaternion), 속도(twist)
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", rclcpp::QoS(1),
    [this](nav_msgs::msg::Odometry::UniquePtr msg) {
      stamp_odom_ = now();
      last_odom_ = std::move(msg);
    });

  // /system/state → 시스템 FSM 상태 (DRIVING, ESTOP 등)
  // 타임스탬프를 별도 기록하지 않는 이유:
  //   시스템 상태는 stale 검사 대상이 아니며, 상태 변화는 이벤트 기반이므로
  //   마지막 값만 유지하면 충분하다.
  sub_state_ = create_subscription<track_msgs::msg::SystemState>(
    "/system/state", rclcpp::QoS(1),
    [this](track_msgs::msg::SystemState::UniquePtr msg) {
      last_state_ = std::move(msg);
    });

  // ============================================================
  // Publisher 생성 (핵심 3개 + 디버그 6개)
  //
  // rclcpp::QoS(1): 모든 퍼블리셔도 depth=1
  //   → 플래너가 10Hz로 생성하는 최신 결과만 필요하므로 큐 크기 1로 충분
  //   → 구독자가 처리보다 늦으면 오래된 데이터는 자동으로 버려짐
  // ============================================================

  // ---- 핵심 퍼블리셔 ----

  // /planning/path → 후처리·안전검사 완료된 최종 주행 경로
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/path", rclcpp::QoS(1));

  // /planning/status → 플래너 상태 코드 및 사유 문자열
  pub_status_ = create_publisher<track_msgs::msg::PlannerStatus>(
    "/planning/status", rclcpp::QoS(1));

  // /planning/target_speed → 안전 검사 후 결정된 목표 속도 (단위: m/s)
  pub_target_speed_ = create_publisher<std_msgs::msg::Float64>(
    "/planning/target_speed", rclcpp::QoS(1));

  // ---- 디버그 퍼블리셔 (RViz2 시각화 목적) ----

  // /planning/debug/corridor_left  → 좌측 코리더 경계선 시각화
  pub_dbg_corridor_left_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/corridor_left", rclcpp::QoS(1));

  // /planning/debug/corridor_right → 우측 코리더 경계선 시각화
  pub_dbg_corridor_right_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/corridor_right", rclcpp::QoS(1));

  // /planning/debug/centerline     → 센터라인 시각화
  pub_dbg_centerline_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/centerline", rclcpp::QoS(1));

  // /planning/debug/pair_valid     → 페어 검증 통과 여부 (true/false)
  pub_dbg_pair_valid_ = create_publisher<std_msgs::msg::Bool>(
    "/planning/debug/pair_valid", rclcpp::QoS(1));

  // /planning/debug/virtual_used   → 이번 프레임에서 가상 경계를 사용했는지 여부
  pub_dbg_virtual_used_ = create_publisher<std_msgs::msg::Bool>(
    "/planning/debug/virtual_used", rclcpp::QoS(1));

  // /planning/debug/path_mode      → 경로 생성 모드 ("DIRECT" 또는 "ASTAR")
  pub_dbg_path_mode_ = create_publisher<std_msgs::msg::String>(
    "/planning/debug/path_mode", rclcpp::QoS(1));

  // ============================================================
  // 타이머 생성: 10Hz (100ms 주기)
  //
  // create_wall_timer: 실제 벽시계(wall clock) 기준 주기 타이머
  //   - 시뮬레이션 시간이 아닌 실제 시간에 의존
  //   - std::chrono::milliseconds(100) = 100ms = 10Hz
  //   - std::bind(&LocalPlannerNode::on_timer, this):
  //     멤버 함수 포인터를 콜백으로 바인딩 (this 캡처)
  // ============================================================
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LocalPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "LocalPlannerNode initialized (10 Hz)");
}

// ============================================================
// 입력 파싱 헬퍼 함수들
// ============================================================

// ------------------------------------------------------------
// parse_lanes: LaneBoundaryArray → lane_left / lane_right
//
// 동작:
//   last_lanes_가 nullptr이면(아직 수신 없음) 즉시 반환.
//   각 LaneBoundary의 side 필드를 검사:
//     LEFT  → left 벡터에 포인트 추가
//     그 외(RIGHT) → right 벡터에 포인트 추가
//   b.points의 각 포인트를 Point2D{x, y}로 변환한다.
// ------------------------------------------------------------
void LocalPlannerNode::parse_lanes(
  std::vector<Point2D> & left, std::vector<Point2D> & right) const
{
  if (!last_lanes_) return;  // 데이터 없음: 빈 벡터 유지
  for (const auto & b : last_lanes_->boundaries) {
    // side 필드로 좌/우 경계를 구분하여 적절한 벡터에 넣음
    auto & target = (b.side == track_msgs::msg::LaneBoundary::LEFT) ? left : right;
    for (const auto & p : b.points) {
      target.push_back({p.x, p.y});
    }
  }
}

// ------------------------------------------------------------
// parse_cones: ConeArray → cone_left / cone_right / cone_all
//
// 콘의 좌우 구분 기준: y 좌표 부호 (차량 좌표계)
//   - 차량 좌표계에서 y > 0 = 차량 왼쪽 방향
//   - 차량 좌표계에서 y < 0 = 차량 오른쪽 방향
//
// cone_all: 좌우 무관 전체 콘 목록
//   → DrivableMaskScanline, CostmapValidationBuilder 등에서
//     방향 구분 없이 모든 콘을 장애물로 취급할 때 사용
// ------------------------------------------------------------
void LocalPlannerNode::parse_cones(
  std::vector<Point2D> & cone_left,
  std::vector<Point2D> & cone_right,
  std::vector<Point2D> & cone_all) const
{
  if (!last_cones_) return;  // 데이터 없음: 빈 벡터 유지
  for (const auto & c : last_cones_->cones) {
    Point2D pt{c.position.x, c.position.y};
    cone_all.push_back(pt);  // 전체 목록에는 항상 추가
    if (c.position.y >= 0.0) {
      cone_left.push_back(pt);   // y >= 0: 왼쪽 콘
    } else {
      cone_right.push_back(pt);  // y < 0: 오른쪽 콘
    }
  }
}

// ------------------------------------------------------------
// parse_obstacles: ObstacleArray → Point2D 리스트
//
// 각 장애물의 중심 위치(position.x, position.y)만 추출한다.
// 크기(반지름 등)는 코스트맵 모듈이 params에서 읽어서 처리한다.
// ------------------------------------------------------------
void LocalPlannerNode::parse_obstacles(std::vector<Point2D> & out) const
{
  if (!last_obstacles_) return;  // 데이터 없음: 빈 벡터 유지
  for (const auto & o : last_obstacles_->obstacles) {
    out.push_back({o.position.x, o.position.y});
  }
}

// ============================================================
// Stale 검사 (check_stale)
//
// 반환:
//   true  → 입력 데이터가 만료됨 → on_timer에서 파이프라인 스킵
//   false → 데이터 신선 → 파이프라인 정상 실행
//
// 타임아웃 계산:
//   dt_ms = (now() - stamp_xxx_).nanoseconds() * 1e-6  [ms 단위]
//   nanoseconds()는 rclcpp::Duration → int64_t 반환
//   * 1e-6 으로 ms로 변환 후 params_.timeouts.*_ms 와 비교
//
// 두 가지 조건:
//   1) Odom: last_odom_이 nullptr이거나 dt > odom_ms → stale
//      → 위치/속도 없이는 경로를 생성할 수 없으므로 필수
//   2) Perception: lanes 또는 cones 중 최소 하나가 dt <= perception_ms
//      → 둘 다 만료됐거나 없으면 stale
//      → OR 조건: 카메라 또는 LiDAR 둘 중 하나만 살아있어도 진행 가능
// ============================================================
bool LocalPlannerNode::check_stale() const
{
  const auto t = now();

  // ---- 조건 1: Odom 신선도 확인 ----
  if (last_odom_) {
    // nanoseconds() * 1e-6 → 밀리초(ms) 단위로 변환
    const double dt_odom =
      (t - stamp_odom_).nanoseconds() * 1e-6;  // ms
    if (dt_odom > params_.timeouts.odom_ms) return true;  // 타임아웃 초과 → stale
  } else {
    return true;  // 아직 odom을 한 번도 받지 못한 경우 → stale
  }

  // ---- 조건 2: 인식 데이터 신선도 확인 (lanes OR cones 중 하나 이상 신선) ----
  bool have_perception = false;

  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_cones_) {
    const double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  // have_perception이 false이면 stale (true 반환)
  return !have_perception;
}

// ============================================================
// 메인 파이프라인: on_timer (10Hz 타이머 콜백)
//
// 전체 11단계 처리 흐름:
//   (0)  Stale 게이트:     입력 만료 시 STALE 상태 퍼블리시 후 조기 종료
//   (1)  입력 추출:        odom에서 ego_pos, ego_heading, ego_speed 계산
//   (2)  코리더 빌드:      차선+콘 → 좌우 경계 구성
//   (3)  페어 검증:        좌우 경계의 유효성(폭 등) 검사
//   (4)  가상 경계:        한쪽만 있을 때 w_hat으로 반대쪽 경계 복원
//   (5)  센터라인 빌드:    좌우 경계 중점 연결
//   (6)  Validation 코스트맵: 센터라인 충돌 가능성 판단
//   (7)  모드 선택:        DIRECT(센터라인 직접 사용) vs ASTAR(격자 탐색)
//   (8)  경로 생성:        선택된 모드로 raw_path 생성
//   (9)  후처리:           이상점 제거, 평활화, 리샘플링
//   (10) 안전 검사:        속도 제한, 비상 정지 조건 확인
//   (11) 퍼블리시 + 상태 갱신
// ============================================================
void LocalPlannerNode::on_timer()
{
  const auto stamp = now();               // 이번 프레임의 타임스탬프 (메시지 헤더에 사용)
  const std::string frame_id = "base_link"; // 모든 메시지의 기준 좌표계

  // ======== (0) Stale 게이트 ========
  // 입력 데이터가 만료된 경우: STALE 상태와 속도 0을 퍼블리시하고 파이프라인 중단
  const bool input_stale = check_stale();

  if (input_stale) {
    // STALE 상태 메시지 퍼블리시
    // std::make_unique: 힙에 객체 생성 후 UniquePtr 반환 → 퍼블리시 후 자동 해제
    auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
    status_msg->header.stamp = stamp;
    status_msg->header.frame_id = frame_id;
    status_msg->status = track_msgs::msg::PlannerStatus::STALE;
    status_msg->reason = "input_stale";
    pub_status_->publish(std::move(status_msg));  // 소유권 이전 후 퍼블리시

    // 목표 속도 0 퍼블리시 (안전을 위해 stale 시 항상 정지 명령)
    auto speed_msg = std::make_unique<std_msgs::msg::Float64>();
    speed_msg->data = 0.0;
    pub_target_speed_->publish(std::move(speed_msg));
    return;  // 파이프라인 나머지 단계 모두 스킵
  }

  // ======== (1) 입력 추출 ========
  // 자차 위치 (x, y) 추출 — odom의 pose.pose.position
  Point2D ego_pos{last_odom_->pose.pose.position.x,
                   last_odom_->pose.pose.position.y};

  // 쿼터니언 → yaw 변환 (2D 평면 주행이므로 yaw만 필요)
  //
  // 쿼터니언(q.x, q.y, q.z, q.w)에서 yaw(heading) 추출 공식:
  //   yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2))
  //
  // siny_cosp = 2*(w*z + x*y):  yaw의 sin 성분에 비례하는 값
  // cosy_cosp = 1 - 2*(y^2 + z^2): yaw의 cos 성분에 비례하는 값
  // atan2(sin, cos) = yaw 각도 (라디안, 범위: -π ~ +π)
  const auto & q = last_odom_->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  // yaw → 단위벡터로 변환: (cos(yaw), sin(yaw)) = 차량이 바라보는 방향
  Point2D ego_heading{std::cos(yaw), std::sin(yaw)};

  // 자차 속도 크기 계산: sqrt(vx^2 + vy^2)
  // twist.twist.linear은 차체 좌표계의 속도이므로 2D 크기를 구한다
  const double ego_speed = std::sqrt(
    last_odom_->twist.twist.linear.x * last_odom_->twist.twist.linear.x +
    last_odom_->twist.twist.linear.y * last_odom_->twist.twist.linear.y);

  // 파싱된 인식 데이터를 담을 벡터 선언
  std::vector<Point2D> lane_left, lane_right;         // 차선 경계
  std::vector<Point2D> cone_left, cone_right, cone_all; // 콘
  std::vector<Point2D> obstacles;                       // 장애물

  // 헬퍼 함수로 각 입력 메시지를 Point2D 벡터로 변환
  parse_lanes(lane_left, lane_right);
  parse_cones(cone_left, cone_right, cone_all);
  parse_obstacles(obstacles);

  // ======== (2) 코리더 빌드 ========
  // CorridorBuilder::Input: 모든 입력을 하나의 구조체로 묶어서 전달
  // corridor_input.centerline_prev: 이전 프레임 센터라인 → 경계선 정렬에 사용
  CorridorBuilder::Input corridor_input;
  corridor_input.lane_left       = lane_left;
  corridor_input.lane_right      = lane_right;
  corridor_input.cone_left       = cone_left;
  corridor_input.cone_right      = cone_right;
  corridor_input.ego_pos         = ego_pos;
  corridor_input.ego_heading     = ego_heading;
  corridor_input.centerline_prev = centerline_prev_;  // 이전 프레임 센터라인

  // build() 반환값: corridor.left, corridor.right, corridor.left_ok, corridor.right_ok
  auto corridor = corridor_builder_.build(corridor_input, params_);

  // ======== (3) 페어 검증 ========
  // PairValidator: 좌우 경계가 모두 유효할 때만 폭 검사 수행
  // pair_result.valid: 두 경계가 올바른 트랙 폭(min~max)을 형성하는지
  // pair_result.width_median: 측정된 트랙 폭의 중앙값 (w_hat 갱신에 사용)
  PairResult pair_result;
  if (corridor.left_ok && corridor.right_ok) {
    pair_result = pair_validator_.validate(corridor.left, corridor.right, params_);
  }

  // ======== (4) 가상 경계 생성 ========
  // 시나리오: 페어 검증 실패 (한쪽 경계만 인식됨)
  //   → 보이는 경계(visible)와 추정 트랙 폭(w_hat_)으로 반대쪽 경계를 생성
  //
  // w_hat_ (추정 트랙 폭) 관리:
  //   - 초기화 전 (w_hat_initialized_ == false):
  //       VirtualBoundary::init_w_hat()으로 초기값 설정
  //       pair_result.width_median 또는 params의 default_track_width 사용
  //   - 이후 페어 검증 성공 시:
  //       VirtualBoundary::update_w_hat()으로 EMA 업데이트
  //       EMA(지수이동평균): w_hat = alpha * width_median + (1-alpha) * w_hat_prev
  //       → 노이즈에 강건한 트랙 폭 추정 유지
  bool virtual_used   = false;  // 이번 프레임에서 가상 경계 사용 여부
  bool visible_is_left = false;  // 보이는 쪽이 왼쪽인지 오른쪽인지

  if (!pair_result.valid) {
    // 페어 검증 실패 → 가상 경계 시도
    bool one_side_ok = corridor.left_ok || corridor.right_ok;
    if (one_side_ok) {
      visible_is_left = corridor.left_ok;  // 왼쪽이 있으면 왼쪽이 visible
      const auto & visible = visible_is_left ? corridor.left : corridor.right;

      // w_hat_ 초기화 (처음 호출 시에만)
      if (!w_hat_initialized_) {
        // init_w_hat: pair_result.width_median이 유효하면 그 값 사용,
        //             아니면 params_.virt.default_track_width 사용
        w_hat_ = VirtualBoundary::init_w_hat(
          pair_result.width_median, false, params_.virt.default_track_width);
        w_hat_initialized_ = true;
      }

      // 가상 경계 생성: visible 경계에서 w_hat_ 거리만큼 옆에 가상 경계 생성
      auto virt_result = virtual_boundary_.generate(
        visible, visible_is_left, w_hat_, params_);

      if (virt_result.success) {
        virtual_used = true;
        // 생성된 가상 경계를 corridor에 반영하여 이후 단계에서 정상 사용
        if (visible_is_left) {
          corridor.right    = virt_result.boundary;  // 오른쪽 가상 경계 주입
          corridor.right_ok = true;
        } else {
          corridor.left     = virt_result.boundary;  // 왼쪽 가상 경계 주입
          corridor.left_ok  = true;
        }
      }
    }
  } else {
    // 페어 검증 성공 → 실측 폭으로 w_hat_ EMA 업데이트
    if (w_hat_initialized_) {
      // EMA 업데이트: alpha * new_value + (1 - alpha) * old_value
      w_hat_ = VirtualBoundary::update_w_hat(
        w_hat_, pair_result.width_median, params_.virt.ema_alpha);
    } else {
      // 처음 페어 검증이 성공한 경우: 바로 중앙값으로 초기화
      w_hat_           = pair_result.width_median;
      w_hat_initialized_ = true;
    }
  }

  // ======== (5) 센터라인 빌드 ========
  // 좌우 경계의 대응 점들의 중점을 연결하여 센터라인 생성
  // centerline_result.valid: 유효한 센터라인 생성 성공 여부
  // centerline_result.center: 센터라인 Point2D 벡터
  // pair_result.valid, virtual_used, visible_is_left: 어떤 방식으로 경계가 구성됐는지 전달
  auto centerline_result = centerline_builder_.build(
    corridor.left, corridor.right,
    pair_result.valid, virtual_used, visible_is_left,
    w_hat_, params_.pair.resample_ds);

  // ======== (6) Validation 코스트맵 빌드 ========
  // 센터라인 위의 각 점이 콘/장애물과 충돌하는지 검사하기 위한 비용 지도 생성
  // 코스트맵의 각 셀에는 장애물로부터의 거리에 반비례한 비용값이 저장됨
  auto validation_costmap = costmap_builder_.build(
    corridor.left, corridor.right, cone_all, obstacles, params_);

  // ======== (7) 모드 선택 ========
  // 센터라인이 장애물과 충돌하는지 확인
  bool centerline_collision = false;
  if (centerline_result.valid && validation_costmap.valid) {
    // 센터라인을 ds_check 간격으로 샘플링하여 각 점의 코스트맵 값 확인
    // 어느 점이라도 cost_th 초과이면 collision 판정
    centerline_collision = CostmapValidationBuilder::check_centerline_collision(
      validation_costmap, centerline_result.center,
      params_.costmap_valid.ds_check, params_.costmap_valid.cost_th);
  }

  // 센터라인 점프(급격한 위치 변화) 감지
  // 이전 프레임 대비 센터라인 앞점이 centerline_jump_th 이상 이동했으면 jump 판정
  // → 불안정한 센터라인 사용 방지
  bool centerline_jump = false;
  if (centerline_result.valid && !centerline_prev_.empty()) {
    centerline_jump = dist(centerline_result.center.front(), centerline_prev_.front())
                      > params_.mode_selector.centerline_jump_th;
  }

  // mode_selector::select(): 위 조건들을 종합해 PathMode 결정
  //   DIRECT: 센터라인을 그대로 경로로 사용 (빠르고 단순)
  //   ASTAR:  격자 위에서 A* 탐색 (장애물 회피 필요 시 사용)
  // enable_astar: false이면 항상 DIRECT (성능 우선 모드)
  PathMode mode = mode_selector::select(
    pair_result.valid,
    virtual_used,
    centerline_result.valid,
    centerline_collision,
    centerline_jump,
    params_.mode_selector.enable_astar);

  // ======== (8) 경로 생성 ========
  std::vector<Point2D> raw_path;  // 후처리 전의 원시 경로
  bool astar_failed = false;      // A* 실패 플래그 (안전 검사에서 사용)

  if (mode == PathMode::DIRECT) {
    // ---- DIRECT 모드: 센터라인을 경로로 직접 사용 ----
    // 추가 계산 없이 centerline_result.center를 raw_path로 사용
    if (centerline_result.valid) {
      raw_path = centerline_result.center;
    }
  } else {
    // ---- ASTAR 모드: 격자 기반 경로 탐색 ----
    // 단계별 처리:
    //   (8-A) DrivableMaskScanline: 주행 가능 영역을 2D 격자로 래스터화
    //   (8-B) ConnectedComponent::filter_ego_component: ego 위치와 연결된 셀만 유지
    //   (8-C) GoalSelector::select: 격자 내에서 목표 지점 선택
    //   (8-D) AstarPlanner::plan: ego → goal A* 경로 탐색

    // (8-A) Drivable 마스크 생성
    // 코리더 경계, 콘, 장애물을 고려해 주행 가능한 셀(free=1)과 불가능한 셀(obstacle=0) 구분
    auto drivable = drivable_builder_.build(
      corridor.left, corridor.right, cone_all, obstacles, params_);

    if (drivable.valid) {
      // (8-B) 연결 요소 필터링
      // 격자에서 ego 위치와 연결(adjacent)된 셀 그룹만 남기고 나머지 제거
      // 목적: 실제로 ego가 접근 가능한 영역만 고려하여 A*가 불필요한 영역을 탐색하지 않도록 함
      ConnectedComponent::filter_ego_component(
        drivable.grid, drivable.width, drivable.height,
        drivable.resolution, drivable.origin_x, drivable.origin_y,
        ego_pos);

      // (8-C) 목표 지점 선택
      // 센터라인, 이전 경로, 격자 내 주행 가능 영역, 자차 위치/방향/속도를 종합해 goal 결정
      auto goal_result = goal_selector_.select(
        centerline_result.center, path_prev_,
        drivable.grid, drivable.width, drivable.height,
        drivable.resolution, drivable.origin_x, drivable.origin_y,
        ego_pos, ego_heading, ego_speed, params_);

      if (goal_result.valid) {
        // (8-D) A* 경로 탐색
        // drivable.grid: 주행 가능 격자 (1=free, 0=obstacle)
        // ego_pos → goal_result.goal: 시작점 → 목표점
        // max iterations: 5000 (무한 루프 방지)
        // cell cost weight: 0.01 (이동 비용 가중치)
        auto astar_result = astar_planner_.plan(
          drivable.grid, drivable.width, drivable.height,
          drivable.resolution, drivable.origin_x, drivable.origin_y,
          ego_pos, goal_result.goal,
          5000,   // 최대 반복 횟수: 실시간 성능 보장을 위한 상한선
          0.01);  // 셀 비용 가중치: 이동 경로 선호도 조정

        if (astar_result.success) {
          raw_path = astar_result.path;
        } else {
          astar_failed = true;  // A* 탐색 실패 (목표 도달 불가 등)
        }
      } else {
        astar_failed = true;  // 유효한 목표 지점을 찾지 못함
      }
    } else {
      astar_failed = true;  // Drivable 마스크 생성 실패
    }
  }

  // ======== (9) 경로 후처리 ========
  // raw_path에 세 가지 처리를 순서대로 적용:
  //   1) prune(이상점 제거): 경로 점들 중 prune_max_dev 이상 벗어난 이상치 제거
  //   2) smooth(평활화):    smooth_window 크기 이동평균으로 경로를 부드럽게
  //   3) resample(리샘플링): resample_ds 간격으로 경로 점 간격을 균일하게 재배치
  // pp_result.valid: 후처리 성공 여부
  // pp_result.path: 후처리 완료된 최종 경로
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,    // 이상점 제거 최대 편차 (m)
    params_.postprocess.smooth_window,    // 평활화 윈도우 크기 (점 수)
    params_.postprocess.resample_ds);     // 리샘플링 간격 (m)

  // ======== (10) 안전 검사 ========
  // safety_checker::check(): 경로와 상황에 따라 안전 상태 및 목표 속도 결정
  // 인자:
  //   pp_result:   후처리된 경로 결과
  //   false:       비상 정지 플래그 (현재 미사용, 향후 확장 가능)
  //   astar_failed: A* 실패 시 속도 감소 등 보수적 처리
  //   params_:     속도 제한, 안전 거리 등의 파라미터
  // 반환:
  //   safety.state:        PlannerStatus enum (OK, ESTOP 등)
  //   safety.reason:       상태 설명 문자열
  //   safety.target_speed: 최종 목표 속도 (m/s)
  auto safety = safety_checker::check(pp_result, false, astar_failed, params_);

  // ======== (11) 퍼블리시 ========

  // ---- 핵심: 최종 경로 퍼블리시 ----
  // to_path_msg(): Point2D 벡터 → nav_msgs::msg::Path 변환 유틸리티
  // std::make_unique + std::move: Zero-copy 퍼블리시 (intra-process 모드 시 효과적)
  auto path_msg = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(pp_result.path, frame_id, stamp));
  pub_path_->publish(std::move(path_msg));

  // ---- 핵심: 플래너 상태 퍼블리시 ----
  // static_cast<uint8_t>: safety.state(enum) → 메시지 필드(uint8_t) 타입 변환
  auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
  status_msg->header.stamp    = stamp;
  status_msg->header.frame_id = frame_id;
  status_msg->status          = static_cast<uint8_t>(safety.state);
  status_msg->reason          = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // ---- 핵심: 목표 속도 퍼블리시 ----
  auto speed_msg = std::make_unique<std_msgs::msg::Float64>();
  speed_msg->data = safety.target_speed;
  pub_target_speed_->publish(std::move(speed_msg));

  // ---- 디버그: 코리더 경계 퍼블리시 (RViz2 시각화용) ----
  auto dbg_left = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(corridor.left, frame_id, stamp));
  pub_dbg_corridor_left_->publish(std::move(dbg_left));

  auto dbg_right = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(corridor.right, frame_id, stamp));
  pub_dbg_corridor_right_->publish(std::move(dbg_right));

  // ---- 디버그: 센터라인 퍼블리시 ----
  auto dbg_cl = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(centerline_result.center, frame_id, stamp));
  pub_dbg_centerline_->publish(std::move(dbg_cl));

  // ---- 디버그: 플래그들 퍼블리시 ----
  // to_bool_msg(): bool → std_msgs::msg::Bool 변환 유틸리티
  auto dbg_pair = std::make_unique<std_msgs::msg::Bool>(to_bool_msg(pair_result.valid));
  pub_dbg_pair_valid_->publish(std::move(dbg_pair));

  auto dbg_virt = std::make_unique<std_msgs::msg::Bool>(to_bool_msg(virtual_used));
  pub_dbg_virtual_used_->publish(std::move(dbg_virt));

  // to_string_msg(): string → std_msgs::msg::String 변환 유틸리티
  // 경로 모드를 문자열로 퍼블리시하여 현재 어떤 모드로 경로를 생성하고 있는지 확인 가능
  auto dbg_mode = std::make_unique<std_msgs::msg::String>(
    to_string_msg(mode == PathMode::DIRECT ? "DIRECT" : "ASTAR"));
  pub_dbg_path_mode_->publish(std::move(dbg_mode));

  // ======== 영속 상태 갱신 ========
  // 다음 프레임을 위해 이번 프레임 결과를 저장

  // 후처리가 유효한 경우에만 path_prev_ 갱신
  // 실패 시 이전 값을 유지하여 goal 선택 연속성 보장
  if (pp_result.valid) {
    path_prev_ = pp_result.path;
  }

  // 센터라인이 유효한 경우에만 centerline_prev_ 갱신
  // 다음 프레임의 점프 감지와 코리더 정렬에 사용됨
  if (centerline_result.valid) {
    centerline_prev_ = centerline_result.center;
  }
}

}  // namespace track_planning

// ============================================================
// RCLCPP_COMPONENTS_REGISTER_NODE 매크로
//
// 이 매크로는 track_planning::LocalPlannerNode 클래스를
// rclcpp_components 시스템에 등록한다.
//
// 동작 원리:
//   - 공유 라이브러리(.so) 안에 factory 함수를 심어놓음
//   - component_container 실행파일이 런타임에 이 .so를 dlopen()으로 로드
//   - factory 함수를 호출하여 NodeOptions를 전달하고 노드 인스턴스 생성
//   - 여러 노드를 같은 프로세스에서 실행 가능 (Composition)
//   - use_intra_process_comms: True 설정 시 같은 컨테이너 안의 노드끼리
//     메모리 복사 없이 UniquePtr 소유권 이전으로 통신 (Zero-copy)
//
// 이 매크로가 없으면 rclcpp_components가 이 노드를 찾지 못해 로드 실패.
// ============================================================
RCLCPP_COMPONENTS_REGISTER_NODE(track_planning::LocalPlannerNode)
