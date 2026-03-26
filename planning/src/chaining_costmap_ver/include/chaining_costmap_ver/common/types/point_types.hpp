/**
 * @file point_types.hpp
 * @brief 기본 좌표 및 경계점 타입 정의
 *
 * 패키지 전반에서 사용하는 최소 단위 구조체:
 *   - Point2D:      순수 2D 좌표
 *   - PointType:    bbox/차선 구분 열거형
 *   - LaneSide:     차선 좌/우 소속 열거형 (yolo_lane_cluster boundary 순서 기반)
 *   - ChainedPoint: 체이닝된 경계점 (위치 + 타입 + is_backbone 플래그)
 *                   is_backbone=true이면 costmap에서 타입 무관하게 bbox_cost_max 적용
 *   - ChainPoint:   DirectionChainer 입력 포인트 (위치 + 메타정보 + is_backbone 플래그)
 *                   to_chained_point() 변환 시 is_backbone을 ChainedPoint로 전파
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_

#include <cstdint>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// 기본 타입 (Basic Types)
// ============================================================================

/**
 * @brief 2D 좌표를 나타내는 기본 점 구조체
 *
 * ego 차량 좌표계(base_link) 기준:
 *   x = 전방(+) / 후방(-)
 *   y = 좌측(+) / 우측(-)
 *
 * CostmapGenerator, AStarPlanner, PathPostprocessor 등
 * "센서 타입에 무관하게" 좌표만 다루는 모듈에서 사용됨.
 */
struct Point2D
{
  double x = 0.0;  ///< X 좌표 [m] — 양수가 차량 전방, 음수가 후방
  double y = 0.0;  ///< Y 좌표 [m] — 양수가 좌측, 음수가 우측 (ROS 좌표계 관례)
};

// ============================================================================
// 경계점 타입 (Boundary Point Types)
// ============================================================================

/**
 * @brief 경계점의 원본 타입 (bbox vs 차선)
 *
 * costmap에서 bbox는 bbox_cost_max + bbox_radius (flat zone),
 * 차선은 lane_cost_max + lane_radius (flat zone)로 차별 적용된다.
 *
 * bbox(PE 드럼, 직경 500mm)는 물리적 크기가 있어서 bbox_radius flat zone을 만들고,
 * 차선(10cm 흰색 테이프)은 lane_radius flat zone을 적용 (기본 0.0m).
 */
enum class PointType : uint8_t
{
  BBOX = 0,  ///< LiDAR DBSCAN 결과 (PE 드럼) — flat zone 적용
  LANE = 1   ///< 카메라 차선 인식 결과 — 거리 기반 감쇠만 적용
};

/**
 * @brief 차선 포인트의 좌/우 소속 — LaneBoundary.msg의 lane_side 필드 기반
 *
 * yolo_lane_cluster가 발행하는 LaneBoundary 메시지의 lane_side 필드에서
 * LEFT/RIGHT 라벨을 직접 읽어 ChainPoint.lane_side에 설정한다.
 * DirectionChainer가 좌/우 체이닝 시 반대편 차선 포인트를 제외할 수 있게 한다.
 *
 * BBOX 포인트는 NONE — 양쪽 chain 모두 사용 가능.
 */
enum class LaneSide : uint8_t
{
  NONE  = 0,  ///< BBOX 또는 소속 불명 — 좌/우 제약 없음
  LEFT  = 1,  ///< 왼쪽 차선 (boundaries[0])
  RIGHT = 2   ///< 오른쪽 차선 (boundaries[1])
};

/**
 * @brief 체이닝된 경계점 — 위치 + 원본 타입 정보
 *
 * DirectionChainer의 SideResult.component[]에 담긴 ChainPoint들이
 * to_chained_point()를 통해 변환된 후, CostmapGenerator에 전달된다.
 * ChainPoint보다 가벼운 구조체 — label, size 등 메타정보를 버리고
 * (x, y, type)만 남긴 것.
 */
struct ChainedPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< bbox/차선 구분 — costmap 비용 계산에 사용
  bool is_backbone = false;   ///< backbone 포인트 여부 — true이면 costmap에서 bbox_cost_max 적용

  /// Point2D로 변환 (costmap/planner 모듈과의 호환용)
  Point2D to_point2d() const { return {x, y}; }
};

/**
 * @brief DirectionChainer 입력 포인트 — 위치 + 메타정보
 *
 * BBox/LaneBoundary를 통합한 내부 표현.
 * ChainedPoint보다 label/size 정보가 추가됨.
 *
 * 데이터 흐름:
 *   LiDAR DBSCAN → BBox 메시지 → ChainPoint (type=BBOX, label=cluster_id)
 *   카메라 차선  → LaneBoundary → ChainPoint (type=LANE, label=lane_id)
 */
struct ChainPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< bbox/차선 구분
  int32_t label = -1;         ///< 원본 cluster_id (bbox: DBSCAN 번호, 차선: YOLO lane_id)
  LaneSide lane_side = LaneSide::NONE;  ///< 차선 좌/우 소속 (yolo_lane_cluster boundary 순서 기반, BBOX는 NONE)
  double size_x = 0.0;        ///< AABB X 크기 [m] (bbox만 유효)
  double size_y = 0.0;        ///< AABB Y 크기 [m] (bbox만 유효)
  bool is_backbone = false;   ///< backbone 포인트 여부 — true이면 costmap에서 bbox_cost_max 적용

  /// ChainedPoint로 변환 (CostmapGenerator 호환용, is_backbone 전파)
  ChainedPoint to_chained_point() const { return {x, y, type, is_backbone}; }
  /// Point2D로 변환 — 순수 좌표만 필요한 경우
  Point2D to_point2d() const { return {x, y}; }
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_
