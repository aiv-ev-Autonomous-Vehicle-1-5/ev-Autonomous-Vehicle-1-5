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
//   Stale 검사 → 입력 파싱 → 코리더 빌드 → 가상 경계 →
//   DTR 센터라인 → 후처리 → 안전 검사 → 퍼블리시
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
// 한쪽 경계만 보일 때 반대쪽 가상 경계를 생성하는 모듈
#include "track_planning/corridor/virtual_boundary.hpp"
// 좌우 코리더 경계의 중간선(센터라인)을 계산하는 모듈 (DTR 외심 방식)
#include "track_planning/corridor/centerline_builder.hpp"
// 생성된 경로를 평활화·리샘플링하는 후처리 모듈
#include "track_planning/postprocess/path_postprocessor.hpp"

// ---- ROS 2 헤더 ----
#include <rclcpp/rclcpp.hpp>               // rclcpp::Node, Timer, Publisher, Subscription
#include <nav_msgs/msg/path.hpp>            // 경로 메시지 (Path)
#include <std_msgs/msg/bool.hpp>            // 디버그용 불리언 플래그
#include <track_msgs/msg/lane_boundary_array.hpp>  // 차선 경계 배열 (카메라 인식 결과)
#include <track_msgs/msg/cone_array.hpp>           // 콘 배열 (LiDAR 인식 결과)
#include <track_msgs/msg/planner_status.hpp>       // 플래너 상태 (STALE / OK / ESTOP 등)

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

  // ConeArray → cone_left / cone_right 분리
  // - y >= 0 이면 왼쪽 콘, y < 0 이면 오른쪽 콘 (차량 좌표계 기준)
  void parse_cones(
    std::vector<Point2D> & cone_left,
    std::vector<Point2D> & cone_right) const;

  // ============================================================
  // Stale 검사: 입력 데이터가 너무 오래되었는지 확인
  //
  // 반환값:
  //   true  → stale (타임아웃 초과), 파이프라인 실행 중단
  //   false → 데이터 신선, 파이프라인 계속 진행
  //
  // 조건:
  //   lanes 또는 cones 중 하나 이상이 params_.timeouts.perception_ms 이내여야 함
  // ============================================================
  bool check_stale() const;

  // ============================================================
  // 파라미터 구조체
  //
  // PlanningParams에는 타임아웃, 코리더, 후처리, 속도 제한 등
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

  // (3) 가상 경계 생성: 한쪽 경계만 존재할 때 트랙 폭 추정으로 반대쪽 경계 복원
  VirtualBoundary virtual_boundary_;

  // (4) 센터라인 빌더: DTR 외심 방식으로 센터라인 생성
  CenterlineBuilder centerline_builder_;

  // (6) 경로 후처리: 이상점 제거(prune), 평활화(smooth), 등간격 리샘플링
  PathPostprocessor postprocessor_;

  // ============================================================
  // 프레임 간 영속 상태 (최소한의 멤버만 유지)
  // ============================================================

  // 추정된 트랙 폭 [m]
  // both_ok일 때 compute_median_width()로 갱신, 한쪽만 ok일 때 직전 값 유지
  // 초기값: default_track_width (1.5m) — cold start 시 자동 적용
  double w_hat_{1.5};

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

  // ============================================================
  // 입력 타임스탬프 (Stale 검사용)
  //
  // 각 토픽의 메시지가 수신된 시각을 now()로 기록한다.
  // check_stale()에서 현재 시각과의 차이(dt)를 계산해 타임아웃 초과 여부를 판단.
  // 초기값: (0, 0, RCL_ROS_TIME) → 수신 전이므로 항상 stale로 판정됨
  // ============================================================
  rclcpp::Time stamp_lanes_;      // 마지막 차선 데이터 수신 시각
  rclcpp::Time stamp_cones_;      // 마지막 콘 데이터 수신 시각

  // ============================================================
  // Subscription 핸들 (2개)
  //
  // SharedPtr으로 구독 핸들을 멤버로 유지해야
  // 노드가 살아있는 동안 구독이 해제되지 않는다.
  // ============================================================

  // /perception/lane_boundaries  → 카메라 기반 차선 경계
  rclcpp::Subscription<track_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;

  // /perception/cones             → LiDAR 기반 콘 위치
  rclcpp::Subscription<track_msgs::msg::ConeArray>::SharedPtr sub_cones_;

  // ============================================================
  // Publisher 핸들 (핵심 2개 + 디버그 4개)
  // ============================================================

  // ---- 핵심 퍼블리셔 (컨트롤러가 실제로 소비하는 토픽) ----

  // /planning/path          → 후처리 완료된 최종 경로
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  // /planning/status        → 플래너 상태 (STALE / OK / ESTOP 등)
  rclcpp::Publisher<track_msgs::msg::PlannerStatus>::SharedPtr pub_status_;

  // ---- 디버그 퍼블리셔 (RViz2 시각화용) ----

  // /planning/debug/corridor_left   → 좌측 코리더 경계 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_left_;

  // /planning/debug/corridor_right  → 우측 코리더 경계 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_right_;

  // /planning/debug/centerline      → 계산된 센터라인 (Path 형식)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_centerline_;

  // /planning/debug/virtual_used    → 가상 경계 사용 여부 (Bool)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dbg_virtual_used_;

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
