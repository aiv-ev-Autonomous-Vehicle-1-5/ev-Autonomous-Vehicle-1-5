/**
 * @file types.hpp
 * @brief chaining_costmap_ver 패키지 전체에서 사용하는 공통 자료구조 — 통합 헤더
 *
 * 이 헤더는 4개의 세부 타입 헤더를 모두 포함하는 편의(umbrella) 헤더이다.
 * 기존 코드에서 #include "common/types.hpp" 하나로 모든 타입에 접근 가능.
 *
 * 세부 헤더:
 *   - types/point_types.hpp   : Point2D, PointType, ChainedPoint, ChainPoint
 *   - types/costmap_types.hpp : CostmapResult
 *   - types/planner_types.hpp : PostprocessResult, PlannerState
 *   - types/chain_types.hpp   : ChainingGraph, StopReason, NodeOwner,
 *                               SideResult, DirectionChainResult
 *
 * [파이프라인 전체 흐름과 타입 매핑]
 *  1) 인지 입력 → ChainPoint
 *  2) DirectionChainer → DirectionChainResult (SideResult)
 *  3) CostmapGenerator → CostmapResult
 *  4) AStarPlanner → Point2D[]
 *  5) PathPostprocessor → PostprocessResult
 *  6) SafetyChecker → PlannerState
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES_HPP_

#include "chaining_costmap_ver/common/types/point_types.hpp"
#include "chaining_costmap_ver/common/types/costmap_types.hpp"
#include "chaining_costmap_ver/common/types/planner_types.hpp"
#include "chaining_costmap_ver/common/types/chain_types.hpp"

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES_HPP_
