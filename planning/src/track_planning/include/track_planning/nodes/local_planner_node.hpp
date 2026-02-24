// ============================================================
// LocalPlannerNode 헤더 파일
//
// 역할:
//   - 트랙 주행용 로컬 플래너 노드의 클래스 선언부
//   - 인식(perception) 결과를 받아 10Hz로 경로를 생성하고 퍼블리시한다
//
// 설계 패턴:
//   - ComposableNode 패턴: rclcpp::Node를 상속받고, NodeOptions를 생성자에서 받음
//     → rclcpp_components를 통해 컨테이너 프로세스에 동적 로드 가능
//     → use_intra_process_comms: True 설정 시 노드 간 메모리 복사 없이 Zero-copy 통신
//
// 파이프라인 개요 (on_timer에서 매 100ms 실행):
//   Stale 검사 → 입력 파싱 → 코리더 빌드 → 페어 검증 → 가상 경계 →
//   센터라인 → 코스트맵 → 모드 선택 → 경로 생성 → 후처리 → 안전 검사 → 퍼블리시
// ============================================================

#ifndef TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_
#define TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_

// 프로젝트 공통 타입 (Point2D 등)과 파라미터 구조체 포함
#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

// ---- 파이프라인 모듈 헤더 ----
// 각 모듈은 단일 책임 원칙(SRP)에 따라 분리된 클래스로 구현됨

// 코리더(주행 가능 영역) 좌우 경계를 구성하는 모듈
#include "track_planning/corridor/corridor_builder.hpp"
// 좌우 경계가 올바른 쌍(pair)을 이루는지 검증하는 모듈
#include "track_planning/corridor/pair_validator.hpp"
// 한쪽 경계만 보일 때 반대쪽 가상 경계를 생성하는 모듈
#include "track_planning/corridor/virtual_boundary.hpp"
// 좌우 코리더 경계의 중간선(센터라인)을 계산하는 모듈
#include "track_planning/corridor/centerline_builder.hpp"
// 충돌 검사용 validation costmap을 생성하는 모듈
#include "track_planning/costmap/costmap_validation_builder.hpp"
// 스캔라인 알고리즘으로 주행 가능 영역(drivable mask)을 생성하는 모듈
#include "track_planning/costmap/drivable_mask_scanline.hpp"
// 연결 요소(connected component) 분석 모듈 (A* 경로 탐색 전처리)
#include "track_planning/costmap/connected_component.hpp"
// A* 탐색용 목표 지점을 선택하는 모듈
#include "track_planning/goal/goal_selector.hpp"
// 격자 기반 A* 경로 탐색 알고리즘 모듈
#include "track_planning/planner/astar_planner.hpp"
// 생성된 경로를 평활화·리샘플링하는 후처리 모듈
#include "track_planning/postprocess/path_postprocessor.hpp"

// ---- ROS 2 헤더 ----
#include <rclcpp/rclcpp.hpp>               // rclcpp::Node, Timer, Publisher, Subscription
#include <nav_msgs/msg/odometry.hpp>        // 자차 위치·속도·자세 (Odometry)
#include <nav_msgs/msg/path.hpp>            // 경로 메시지 (Path)
#include <std_msgs/msg/bool.hpp>            // 디버그용 불리언 플래그
#include <std_msgs/msg/float64.hpp>         // 목표 속도 (Float64)
#include <std_msgs/msg/string.hpp>          // 디버그용 문자열 (경로 모드 이름)
#include <track_msgs/msg/lane_boundary_array.hpp>  // 차선 경계 배열 (카메라 인식 결과)
#include <track_msgs/msg/cone_array.hpp>           // 콘 배열 (LiDAR 인식 결과)
#include <track_msgs/msg/obstacle_array.hpp>       // 장애물 배열 (LiDAR 인식 결과)
#include <track_msgs/msg/planner_status.hpp>       // 플래너 상태 (STALE / OK / ESTOP 등)
#include <track_msgs/msg/system_state.hpp>         // 시스템 전체 상태 (FSM)

#include <vector>

namespace track_planning
{

// ============================================================
// LocalPlannerNode 클래스
//
// ComposableNode 패턴 설명:
//   - rclcpp::Node를 public 상속 → ROS 2의 기본 노드 기능 모두 사용 가능
//   - 생성자가 const rclcpp::NodeOptions & options를 받음
//     → rclcpp_components가 동적 로드 시 이 생성자를 호출
//     → NodeOptions 안에 use_intra_process_comms 플래그가 담겨 있음
//   - RCLCPP_COMPONENTS_REGISTER_NODE 매크로로 클래스를 등록하면
//     component_container 실행파일이 런타임에 이 노드를 로드할 수 있음
// ============================================================
class LocalPlannerNode : public rclcpp::Node
{
public:
  // NodeOptions 생성자: rclcpp_components가 동적 로드 시 반드시 이 서명이 있어야 함
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options);

private:
  // ============================================================
  // 타이머 콜백: 10Hz(100ms마다)로 전체 파이프라인을 실행한다
  // ============================================================
  void on_timer();

  // ============================================================
  // 입력 파싱 헬퍼 함수들
  //
  // 각 함수는 last_* UniquePtr에 저장된 최신 메시지를 읽어
  // 파이프라인에서 사용하기 쉬운 Point2D 벡터로 변환한다.
  // ============================================================

  // LaneBoundaryArray → lane_left / lane_right 분리
  // - b.side == LEFT 이면 left 벡터에, 나머지는 right 벡터에 추가
  void parse_lanes(
    std::vector<Point2D> & left, std::vector<Point2D> & right) const;

  // ConeArray → cone_left / cone_right / cone_all 분리
  // - y >= 0 이면 왼쪽 콘, y < 0 이면 오른쪽 콘 (차량 좌표계 기준)
  // - cone_all은 방향 무관 전체 콘 목록 (코스트맵 계산에 사용)
  void parse_cones(
    std::vector<Point2D> & cone_left,
    std::vector<Point2D> & cone_right,
    std::vector<Point2D> & cone_all) const;

  // ObstacleArray → Point2D 리스트 변환
  // - 장애물 위치만 추출 (크기 정보는 코스트맵 모듈 내부에서 처리)
  void parse_obstacles(std::vector<Point2D> & out) const;

  // ============================================================
  // Stale 검사: 입력 데이터가 너무 오래되었는지 확인
  //
  // 반환값:
  //   true  → stale (타임아웃 초과), 파이프라인 실행 중단
  //   false → 데이터 신선, 파이프라인 계속 진행
  //
  // 조건:
  //   1) odom이 params_.timeouts.odom_ms 이내여야 함
  //   2) lanes 또는 cones 중 하나 이상이 params_.timeouts.perception_ms 이내여야 함
  // ============================================================
  bool check_stale() const;

  // ============================================================
  // 파라미터 구조체
  //
  // PlanningParams에는 타임아웃, 코리더, 코스트맵, A*, 후처리 등
  // 모든 파이프라인 모듈의 설정값이 담겨 있다.
  // 생성자에서 params_.load(this)로 ROS 2 파라미터 서버에서 읽어온다.
  // ============================================================
  PlanningParams params_;

  // ============================================================
  // 파이프라인 모듈 인스턴스
  //
  // 각 모듈은 on_timer() 안에서 순서대로 호출된다.
  // 노드 멤버로 선언함으로써 매 프레임마다 재생성하지 않고
  // 내부 상태(캐시, 필터 등)를 프레임 간에 유지할 수 있다.
  // ============================================================

  // (2) 코리더 경계(좌/우) 구성: 차선+콘 데이터를 합쳐 주행 가능 구간 경계 생성
  CorridorBuilder corridor_builder_;

  // (3) 페어 검증: 좌우 경계가 실제로 유효한 트랙 폭을 형성하는지 확인
  PairValidator pair_validator_;

  // (4) 가상 경계 생성: 한쪽 경계만 존재할 때 트랙 폭 추정으로 반대쪽 경계 복원
  //     w_hat_: EMA(지수이동평균)으로 추정된 트랙 폭
  VirtualBoundary virtual_boundary_;

  // (5) 센터라인 빌더: 좌우 경계의 중점을 연결해 센터라인 생성
  CenterlineBuilder centerline_builder_;

  // (6) Validation 코스트맵: 센터라인 위 충돌 여부 검사용 비용 지도
  CostmapValidationBuilder costmap_builder_;

  // (8-A) Drivable 마스크: 주행 가능 영역을 스캔라인으로 래스터화한 격자
  DrivableMaskScanline drivable_builder_;

  // (8-B) 목표 지점 선택: 센터라인 위에서 A*가 향할 goal 후보 선정
  GoalSelector goal_selector_;

  // (8-C) A* 플래너: drivable 격자 위에서 ego → goal 최단 경로 탐색
  AstarPlanner astar_planner_;

  // (9) 경로 후처리: 이상점 제거(prune), 평활화(smooth), 등간격 리샘플링
  PathPostprocessor postprocessor_;

  // ============================================================
  // 프레임 간 영속 상태 (Persistent State)
  //
  // 이 값들은 on_timer() 호출이 끝난 후에도 살아남아
  // 다음 프레임에서 연속성 유지에 사용된다.
  // ============================================================

  // 추정된 트랙 폭 (EMA 필터링된 값, 단위: m)
  // 초기값 0.0, w_hat_initialized_가 false이면 아직 유효하지 않음
  double w_hat_{0.0};
  bool w_hat_initialized_{false};

  // 직전 프레임에서 퍼블리시된 최종 경로 (goal 선택 및 연속성 판단에 사용)
  std::vector<Point2D> path_prev_;

  // 직전 프레임의 센터라인 (센터라인 점프(jump) 감지에 사용)
  std::vector<Point2D> centerline_prev_;

  // ============================================================
  // 최신 입력 데이터 저장소 (UniquePtr)
  //
  // UniquePtr 사용 이유:
  //   - ROS 2 intra-process 통신에서 Zero-copy를 달성하려면
  //     Subscription 콜백의 인자도 UniquePtr이어야 한다.
  //   - std::move()로 소유권을 이전하므로 메모리 복사가 발생하지 않는다.
  //   - nullptr 상태 = 아직 해당 토픽 데이터를 받지 못했음을 의미한다.
  // ============================================================
  track_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;     // 최신 차선 경계
  track_msgs::msg::ConeArray::UniquePtr         last_cones_;     // 최신 콘 배열
  track_msgs::msg::ObstacleArray::UniquePtr     last_obstacles_; // 최신 장애물 배열
  nav_msgs::msg::Odometry::UniquePtr            last_odom_;      // 최신 오도메트리
  track_msgs::msg::SystemState::UniquePtr       last_state_;     // 최신 시스템 상태

  // ============================================================
  // 입력 타임스탬프 (Stale 검사용)
  //
  // 각 토픽의 메시지가 수신된 시각을 now()로 기록한다.
  // check_stale()에서 현재 시각과의 차이(dt)를 계산해 타임아웃 초과 여부를 판단.
  // 초기값: (0, 0, RCL_ROS_TIME) → 수신 전이므로 항상 stale로 판정됨
  // ============================================================
  rclcpp::Time stamp_lanes_;      // 마지막 차선 데이터 수신 시각
  rclcpp::Time stamp_cones_;      // 마지막 콘 데이터 수신 시각
  rclcpp::Time stamp_obstacles_;  // 마지막 장애물 데이터 수신 시각
  rclcpp::Time stamp_odom_;       // 마지막 오도메트리 수신 시각

  // ============================================================
  // Subscription 핸들 (5개)
  //
  // SharedPtr으로 구독 핸들을 멤버로 유지해야
  // 노드가 살아있는 동안 구독이 해제되지 않는다.
  // ============================================================

  // /perception/lane_boundaries  → 카메라 기반 차선 경계
  rclcpp::Subscription<track_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;

  // /perception/cones             → LiDAR 기반 콘 위치
  rclcpp::Subscription<track_msgs::msg::ConeArray>::SharedPtr sub_cones_;

  // /perception/obstacles         → LiDAR 기반 장애물 위치
  rclcpp::Subscription<track_msgs::msg::ObstacleArray>::SharedPtr sub_obstacles_;

  // /odometry                     → 자차 위치·자세·속도
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // /system/state                 → FSM 시스템 상태 (DRIVING, ESTOP 등)
  rclcpp::Subscription<track_msgs::msg::SystemState>::SharedPtr sub_state_;

  // ============================================================
  // Publisher 핸들 (9개 = 핵심 3개 + 디버그 6개)
  // ============================================================

  // ---- 핵심 퍼블리셔 (컨트롤러가 실제로 소비하는 토픽) ----

  // /planning/path          → 후처리 완료된 최종 경로
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  // /planning/status        → 플래너 상태 (STALE / OK / ESTOP 등)
  rclcpp::Publisher<track_msgs::msg::PlannerStatus>::SharedPtr pub_status_;

  // /planning/target_speed  → 안전 검사 후 결정된 목표 속도 (m/s)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_speed_;

  // ---- 디버그 퍼블리셔 (RViz2 시각화 및 개발 목적) ----

  // /planning/debug/corridor_left   → 좌측 코리더 경계 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_left_;

  // /planning/debug/corridor_right  → 우측 코리더 경계 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_right_;

  // /planning/debug/centerline      → 계산된 센터라인 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_centerline_;

  // /planning/debug/pair_valid      → 페어 검증 통과 여부 (Bool)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dbg_pair_valid_;

  // /planning/debug/virtual_used    → 가상 경계 사용 여부 (Bool)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dbg_virtual_used_;

  // /planning/debug/path_mode       → 경로 생성 모드 이름: "DIRECT" 또는 "ASTAR" (String)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dbg_path_mode_;

  // ============================================================
  // 타이머 핸들
  //
  // create_wall_timer(100ms, callback) → 실제 벽시계(wall clock) 기준 10Hz 타이머
  // std::bind로 on_timer 멤버 함수를 콜백으로 등록한다.
  // ============================================================
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_
