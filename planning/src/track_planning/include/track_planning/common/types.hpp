/**
 * @file types.hpp
 * @brief 플래닝 시스템 전체에서 사용하는 공통 자료구조 정의
 *
 * 파이프라인의 각 모듈이 데이터를 주고받을 때 사용하는 구조체와 열거형을 모아놓은 파일.
 * 모든 좌표는 ego 차량 기준 base_link 프레임 (전방 +x, 좌측 +y).
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                    파이프라인 데이터 흐름도                      │
 * ├─────────────────────────────────────────────────────────────────┤
 * │                                                                 │
 * │  Perception Input                                               │
 * │    ├─ LaneBoundaryArray ─┐                                      │
 * │    └─ ConeArray ─────────┤                                      │
 * │                          ▼                                      │
 * │  (2) CorridorBuilder ──► CorridorPolylines {left, right}        │
 * │                          ▼                                      │
 * │  (3) VirtualBoundary ──► VirtualBoundaryResult {boundary}       │
 * │                          ▼                                      │
 * │  (5) CenterlineBuilder ► CenterlineResult {center}              │
 * │                          ▼                                      │
 * │  (6) PathPostprocessor ► PostprocessResult {path, yaw}          │
 * │                          ▼                                      │
 * │  (7) SafetyChecker ────► PlannerState {OK/STOP/INFEASIBLE}      │
 * │                          ▼                                      │
 * │  (8) Publish: /planning/path + /planning/status                 │
 * │                                                                 │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * 참조 파일:
 *   - params.hpp      : 전체 파라미터 정의 (PlanningParams)
 *   - geometry.hpp     : 2D 기하 유틸리티 (Point2D 연산, 폴리라인 처리)
 *   - corridor_builder : CorridorPolylines 생성
 *   - virtual_boundary : VirtualBoundaryResult 생성
 *   - centerline_builder: CenterlineResult 생성
 *   - path_postprocessor: PostprocessResult 생성
 *   - safety_checker   : PlannerState 판정
 */
#ifndef TRACK_PLANNING__COMMON__TYPES_HPP_
#define TRACK_PLANNING__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace track_planning
{

// ============================================================
// 기본 자료형 (Primitive)
// ============================================================

/**
 * @brief 2D 점/벡터 (x, y)
 *
 * 위치, 방향 벡터, 속도 등 다양한 용도로 사용.
 * 기본값은 원점 (0, 0).
 */
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

// ============================================================
// Corridor 관련 구조체
// ============================================================

/**
 * @brief CorridorBuilder의 출력: 좌/우 경계 폴리라인
 *
 * left  : 좌측 경계점 배열 (lane + cone 연결)
 * right : 우측 경계점 배열
 * left_ok / right_ok : 해당 쪽 경계가 유효하게 생성되었는지 여부
 *   - 한쪽만 ok이면 VirtualBoundary로 반대쪽을 생성한다.
 */
struct CorridorPolylines
{
  std::vector<Point2D> left;
  std::vector<Point2D> right;
  bool left_ok = false;
  bool right_ok = false;
};

/**
 * @brief VirtualBoundary의 출력: 보이지 않는 쪽 경계를 추정한 결과
 *
 * boundary : 생성된 가상 경계 폴리라인
 * success  : 가상 경계 생성 성공 여부
 */
struct VirtualBoundaryResult
{
  std::vector<Point2D> boundary;
  bool success = false;
};

/**
 * @brief CenterlineBuilder의 출력: 중앙선 경로
 *
 * center : 중앙선 점 배열 (리샘플링 완료)
 * valid  : 중앙선 생성 성공 여부
 */
struct CenterlineResult
{
  std::vector<Point2D> center;
  bool valid = false;
};

// ============================================================
// 후처리 (Postprocess)
// ============================================================

/**
 * @brief PathPostprocessor의 출력: 최종 경로 + heading 정보
 *
 * path  : 후처리된 경로점 배열 (prune → smooth → resample 완료)
 * yaw   : 각 점에서의 heading 각도 (rad), Control 노드에서 사용
 * valid : 후처리 성공 여부
 */
struct PostprocessResult
{
  std::vector<Point2D> path;
  std::vector<double> yaw;   // 각 점의 heading (rad)
  bool valid = false;
};

// ============================================================
// 플래너 상태 (Planner Status)
// ============================================================

/**
 * @brief 플래너의 현재 상태 (Safety 판정 결과)
 *
 * OK         : 정상 — 경로 + target_speed 출력
 * STOP       : 정지 — 경로 생성 실패, target_speed = 0
 * INFEASIBLE : 실현불가 — 곡률 초과 등, target_speed = 0
 * STALE      : 입력 데이터 지연/누락 — 감속 후 정지
 */
enum class PlannerState : uint8_t
{
  OK = 0,
  STOP = 1,
  INFEASIBLE = 2,
  STALE = 3
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__TYPES_HPP_
