// ============================================================
// LocalPlannerNode 구현 파일
//
// 이 파일의 구조:
//   1. 생성자: 파라미터 로드 → 구독 생성 → 퍼블리셔 생성 → 타이머 생성
//   2. 입력 파싱 헬퍼 (parse_lanes, parse_cones)
//   3. Stale 검사 (check_stale)
//   4. 메인 파이프라인 (on_timer): 9단계 (0)-(8)로 경로 생성
//   5. RCLCPP_COMPONENTS_REGISTER_NODE 매크로: 컴포넌트 등록
// ============================================================

#include "track_planning/nodes/local_planner_node.hpp"
#include "track_planning/common/geometry.hpp"       // dist(), to_path_msg() 등 기하 유틸리티
#include "track_planning/common/debug_publish.hpp"  // to_bool_msg() 등
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
  stamp_cones_(0, 0, RCL_ROS_TIME)
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

  // ============================================================
  // Publisher 생성 (핵심 2개 + 디버그 4개)
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

  // /planning/debug/virtual_used   → 이번 프레임에서 가상 경계를 사용했는지 여부
  pub_dbg_virtual_used_ = create_publisher<std_msgs::msg::Bool>(
    "/planning/debug/virtual_used", rclcpp::QoS(1));

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
// parse_cones: ConeArray → cone_left / cone_right
//
// 콘의 좌우 구분 기준: y 좌표 부호 (차량 좌표계)
//   - 차량 좌표계에서 y > 0 = 차량 왼쪽 방향
//   - 차량 좌표계에서 y < 0 = 차량 오른쪽 방향
// ------------------------------------------------------------
void LocalPlannerNode::parse_cones(
  std::vector<Point2D> & cone_left,
  std::vector<Point2D> & cone_right) const
{
  if (!last_cones_) return;  // 데이터 없음: 빈 벡터 유지
  for (const auto & c : last_cones_->cones) {
    Point2D pt{c.position.x, c.position.y};
    if (c.position.y >= 0.0) {
      cone_left.push_back(pt);   // y >= 0: 왼쪽 콘
    } else {
      cone_right.push_back(pt);  // y < 0: 오른쪽 콘
    }
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
//   * 1e-6 으로 ms로 변환 후 params_.timeouts.perception_ms 와 비교
//
// 조건:
//   Perception: lanes 또는 cones 중 최소 하나가 dt <= perception_ms
//   → 둘 다 만료됐거나 없으면 stale
//   → OR 조건: 카메라 또는 LiDAR 둘 중 하나만 살아있어도 진행 가능
// ============================================================
bool LocalPlannerNode::check_stale() const
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

// ============================================================
// 메인 파이프라인: on_timer (10Hz 타이머 콜백)
//
// 전체 처리 흐름:
//   (0)  Stale 게이트:     입력 만료 시 STALE 상태 퍼블리시 후 조기 종료
//   (1)  입력 추출:        ego 고정(0,0), 인식 데이터 파싱
//   (2)  코리더 빌드:      차선+콘 → 좌우 경계 구성
//   (3)  w_hat + 가상 경계: both_ok → median width, 한쪽만 → 가상 경계 생성
//   (4)  DTR 센터라인 빌드: 외심 기반 또는 한쪽 오프셋
//   (5)  경로 생성:        센터라인을 직접 경로로 사용
//   (6)  후처리:           이상점 제거, 평활화, 리샘플링
//   (7)  안전 검사:        속도 제한 조건 확인
//   (8)  퍼블리시
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
    return;  // 파이프라인 나머지 단계 모두 스킵
  }

  // ======== (1) 입력 추출 ========
  // ego-centric 좌표계: 센서 데이터가 이미 base_link 기준이므로
  // ego_pos = (0,0), ego_heading = (1,0) 고정
  const Point2D ego_pos{0.0, 0.0};
  const Point2D ego_heading{1.0, 0.0};

  // 파싱된 인식 데이터를 담을 벡터 선언
  std::vector<Point2D> lane_left, lane_right;         // 차선 경계
  std::vector<Point2D> cone_left, cone_right;           // 콘

  // 헬퍼 함수로 각 입력 메시지를 Point2D 벡터로 변환
  parse_lanes(lane_left, lane_right);
  parse_cones(cone_left, cone_right);

  // ======== (2) 코리더 빌드 ========
  // CorridorBuilder::Input: 모든 입력을 하나의 구조체로 묶어서 전달
  CorridorBuilder::Input corridor_input;
  corridor_input.lane_left   = std::move(lane_left);
  corridor_input.lane_right  = std::move(lane_right);
  corridor_input.cone_left   = std::move(cone_left);
  corridor_input.cone_right  = std::move(cone_right);
  corridor_input.ego_pos     = ego_pos;
  corridor_input.ego_heading = ego_heading;

  // build() 반환값: corridor.left, corridor.right, corridor.left_ok, corridor.right_ok
  auto corridor = corridor_builder_.build(corridor_input, params_);

  // ======== (3) w_hat 갱신 + 가상 경계 생성 ========
  // both_ok: 양쪽 corridor 모두 유효한지 여부
  const bool both_ok = corridor.left_ok && corridor.right_ok;

  // both_ok일 때 median width 직접 계산 → w_hat_ 갱신
  if (both_ok) {
    w_hat_ = compute_median_width(corridor.left, corridor.right);
  }
  // 한쪽만 ok일 때는 w_hat_ 그대로 유지 (이전 both_ok에서 저장한 값 또는 초기값 1.5m)

  bool virtual_used    = false;
  bool visible_is_left = false;

  if (!both_ok) {
    // 한쪽만 보일 때 → 가상 경계 생성 시도
    const bool one_side_ok = corridor.left_ok || corridor.right_ok;
    if (one_side_ok) {
      visible_is_left = corridor.left_ok;
      const auto & visible = visible_is_left ? corridor.left : corridor.right;

      auto virt_result = virtual_boundary_.generate(
        visible, visible_is_left, w_hat_, params_);

      if (virt_result.success) {
        virtual_used = true;
        if (visible_is_left) {
          corridor.right    = virt_result.boundary;
          corridor.right_ok = true;
        } else {
          corridor.left     = virt_result.boundary;
          corridor.left_ok  = true;
        }
      }
    }
  }

  // ======== (4) 센터라인 빌드 ========
  auto centerline_result = centerline_builder_.build(
    corridor.left, corridor.right,
    both_ok, virtual_used, visible_is_left,
    w_hat_, params_);

  // 센터라인 실패 + 양쪽 경계 유효 → 경계 교차 가능성 로그
  if (!centerline_result.valid && corridor.left_ok && corridor.right_ok) {
    RCLCPP_ERROR(get_logger(), "Centerline failed: boundaries may cross");
  }

  // ======== (5) 센터라인을 경로로 직접 사용 ========
  // DTR 센터라인이 유효하면 그대로 raw_path로 사용
  std::vector<Point2D> raw_path;
  if (centerline_result.valid) {
    raw_path = centerline_result.center;
  }

  // ======== (6) 경로 후처리 ========
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

  // ======== (7) 안전 검사 ========
  // safety_checker::check(): 경로와 상황에 따라 안전 상태 및 목표 속도 결정
  // 반환:
  //   safety.state:        PlannerStatus enum (OK, STOP, INFEASIBLE, STALE)
  //   safety.reason:       상태 설명 문자열
  //   safety.reason:       상태 설명 문자열 (디버깅용)
  auto safety = safety_checker::check(pp_result, params_);

  // ======== (8) 퍼블리시 ========

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

  // ---- 디버그: 구독자가 있을 때만 메시지 생성 (Lazy Publishing) ----
  if (pub_dbg_corridor_left_->get_subscription_count() > 0) {
    pub_dbg_corridor_left_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(corridor.left, frame_id, stamp)));
  }
  if (pub_dbg_corridor_right_->get_subscription_count() > 0) {
    pub_dbg_corridor_right_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(corridor.right, frame_id, stamp)));
  }
  if (pub_dbg_centerline_->get_subscription_count() > 0) {
    pub_dbg_centerline_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(centerline_result.center, frame_id, stamp)));
  }
  if (pub_dbg_virtual_used_->get_subscription_count() > 0) {
    pub_dbg_virtual_used_->publish(std::make_unique<std_msgs::msg::Bool>(
      to_bool_msg(virtual_used)));
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
