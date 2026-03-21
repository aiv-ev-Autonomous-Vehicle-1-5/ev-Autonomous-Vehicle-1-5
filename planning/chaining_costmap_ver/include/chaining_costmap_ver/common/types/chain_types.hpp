/**
 * @file chain_types.hpp
 * @brief DirectionChainer v2 체이닝 타입 정의
 *
 * Component → Backbone 기반 좌/우 차선 경계 체이닝 시스템의 타입들:
 *   - ChainingGraph:         Undirected 그래프 (component 추출용)
 *   - StopReason:            Greedy chaining 종료 이유
 *   - NodeOwner:             노드 소유권 라벨
 *   - SideResult:            한쪽 side의 chaining 결과
 *   - DirectionChainResult:  좌/우 전체 결과 + 교차 판정 플래그
 *                            (left_crossed_right / right_crossed_left)
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES__CHAIN_TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES__CHAIN_TYPES_HPP_

#include "chaining_costmap_ver/common/types/point_types.hpp"

#include <cstdint>
#include <vector>

namespace chaining_costmap_ver
{

/**
 * @brief Undirected 그래프 — component 추출 전용
 *
 * undirected 그래프에는 "진행 방향" 개념이 없다.
 * BFS/DFS로 연결 컴포넌트를 찾을 때 방향이 있으면
 * 한쪽에서 도달 못하는 노드가 생기므로 무방향이 맞다.
 */
struct ChainingGraph
{
  std::vector<std::vector<int>> undirected;  ///< [node_index] → [이웃 node_index 리스트]
};

/**
 * @brief Greedy chaining 종료 이유
 */
enum class StopReason : uint8_t
{
  NO_CANDIDATE,   ///< 전방에 후보 점이 없음
  ALL_GATED,      ///< 후보는 있으나 3-게이트 모두 탈락
  CYCLE,          ///< 이미 방문한 노드를 재방문 (루프 감지)
  MAX_LEN         ///< max_chain_len 도달
};

/**
 * @brief 노드 소유권 라벨 — chain() 파이프라인에서 각 노드의 역할 구분
 *
 * 라벨 부여 순서:
 *   1. left backbone  확정 → LEFT_BACKBONE
 *   2. right backbone 확정 → RIGHT_BACKBONE
 */
enum class NodeOwner : uint8_t
{
  NONE = 0,        ///< 미할당
  LEFT_BACKBONE,   ///< 좌측 backbone
  RIGHT_BACKBONE   ///< 우측 backbone
};

/**
 * @brief 한쪽 side(좌 또는 우)의 chaining 결과
 *
 *   component (전체) → CostmapGenerator에 전달
 *   └── backbone (주 경계선) → RViz2 시각화
 */
struct SideResult
{
  std::vector<ChainPoint> component;       ///< side 전체 리샘플 포인트 (costmap 전달용)
  std::vector<ChainPoint> backbone;        ///< 주 경계선 (디버깅/시각화용)

  int seed_idx = -1;                       ///< seed 인덱스 (원본 points 배열 기준)
  int goal_idx = -1;                       ///< goal 인덱스
  int seed_backbone_pos = 0;               ///< backbone 내 seed_start 위치 (= backward chain 길이)
  StopReason stop_reason_forward = StopReason::NO_CANDIDATE;
};

/**
 * @brief DirectionChainer의 출력 결과 — 좌/우 side 전체
 *
 * 최소 한쪽 backbone이 성공적으로 생성되면 valid=true.
 * 양쪽 모두 실패하면 valid=false → "FAIL - not enough seeds".
 */
struct DirectionChainResult
{
  SideResult left;            ///< 좌측 경계 체이닝 결과
  SideResult right;           ///< 우측 경계 체이닝 결과
  std::vector<ChainPoint> unchained;  ///< 어떤 체인에도 속하지 못한 포인트들
  bool valid = false;         ///< 최소 한쪽 backbone 생성 성공 여부

  /// 교차 판정: 한쪽 backbone이 반대쪽 seed를 체이닝한 경우
  bool left_crossed_right = false;  ///< left backbone이 right seed를 포함
  bool right_crossed_left = false;  ///< right backbone이 left seed를 포함
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES__CHAIN_TYPES_HPP_
