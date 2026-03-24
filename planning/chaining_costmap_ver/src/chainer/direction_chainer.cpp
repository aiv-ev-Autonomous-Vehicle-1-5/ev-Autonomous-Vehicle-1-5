/**
 * @file direction_chainer.cpp
 * @brief DirectionChainer — 메인 chain() 오케스트레이터
 *
 * Owner-Label 기반 독립 체이닝의 메인 진입점만 포함.
 * 각 단계의 구현은 별도 .cpp 파일로 분리되어 있다:
 *   - seed_selector.cpp:      find_seed(), knn()
 *   - graph_builder.cpp:      build_graph()
 *   - backbone_extractor.cpp: extract_backbone(), chain_one_direction(),
 *                             compute_cost(), compute_cost_prime(),
 *                             resolve_overlaps(), compute_backtrack_cost(),
 *                             rechain_from()
 *   - chain_resampler.cpp:    resample_component()
 *
 * [파이프라인 요약]
 *   준비: find_seed() → build_graph() → owner[] 초기화
 *   1단계: extract_backbone(left)   — 독립 owner 사용
 *   2단계: extract_backbone(right)  — 독립 owner 사용 (left와 무관)
 *   2.5단계: resolve_overlaps() — 중복 노드 backtracking 해소
 *   2.6단계: 교차 판정 — left_bb에 right_seed 포함? / right_bb에 left_seed 포함?
 *           교차 플래그는 DirectionChainResult에 저장되어 GoalCalculator에서
 *           backbone 중간점 폴백에 사용된다.
 *   2.75단계: trim_crossing_backbones() — 좌/우 backbone 선분 교차 검증
 *           CCW 기반으로 선분 교차 판정, 교차 시 양쪽 tail trim
 *           resample 전에 수행하여 trimming 결과가 리샘플링에 반영됨
 *   3단계: resample_component() × 2
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <algorithm>
#include <cstdio>
#include <vector>

namespace chaining_costmap_ver
{

DirectionChainResult DirectionChainer::chain(
  const std::vector<ChainPoint> & points,
  const PlanningParams & params) const
{
  DirectionChainResult result;
  const auto & cp = params.chainer;

  // 준비: 입력 검사
  const auto & filtered = points;
  if (filtered.size() < 2) return result;

  // 준비: Seed 선택
  int left_seed = find_seed(filtered, true, cp);
  int right_seed = find_seed(filtered, false, cp);
  if (left_seed < 0 && right_seed < 0) return result;

  // 준비: Undirected Graph 구성
  auto graph = build_graph(filtered, cp);

  // 1단계: Left Backbone 독립 추출
  // owner_left는 left 전용 — right에 영향 없음
  std::vector<int> left_backbone_ids;
  StopReason left_stop_fwd = StopReason::NO_CANDIDATE;
  int left_seed_bb_pos = 0;
  if (left_seed >= 0) {
    std::vector<NodeOwner> owner_left(filtered.size(), NodeOwner::NONE);
    left_backbone_ids = extract_backbone(
      filtered, owner_left, left_seed, true, left_stop_fwd, left_seed_bb_pos, cp);
  }

  // 2단계: Right Backbone 독립 추출
  // owner_right는 right 전용 — left에 영향 없음
  std::vector<int> right_backbone_ids;
  StopReason right_stop_fwd = StopReason::NO_CANDIDATE;
  int right_seed_bb_pos = 0;
  if (right_seed >= 0) {
    std::vector<NodeOwner> owner_right(filtered.size(), NodeOwner::NONE);
    right_backbone_ids = extract_backbone(
      filtered, owner_right, right_seed, false, right_stop_fwd, right_seed_bb_pos, cp);
  }

  // 2.5단계: 독립 체이닝 중복 해소 — Backtracking
  if (!left_backbone_ids.empty() && !right_backbone_ids.empty() &&
      cp.max_backtrack_count > 0) {
    resolve_overlaps(
      filtered, left_backbone_ids, right_backbone_ids, cp);
  }

  // 2.6단계: 교차 판정 — backbone 인덱스 리스트에서 직접 검색
  if (right_seed >= 0 && !left_backbone_ids.empty()) {
    if (std::find(left_backbone_ids.begin(), left_backbone_ids.end(), right_seed)
        != left_backbone_ids.end()) {
      result.left_crossed_right = true;
    }
  }
  if (left_seed >= 0 && !right_backbone_ids.empty()) {
    if (std::find(right_backbone_ids.begin(), right_backbone_ids.end(), left_seed)
        != right_backbone_ids.end()) {
      result.right_crossed_left = true;
    }
  }

  // 최종 owner 배열 병합 (trim_crossing + unchained 수집용)
  std::vector<NodeOwner> owner(filtered.size(), NodeOwner::NONE);
  for (int idx : left_backbone_ids) {
    owner[idx] = NodeOwner::LEFT_BACKBONE;
  }
  for (int idx : right_backbone_ids) {
    owner[idx] = NodeOwner::RIGHT_BACKBONE;
  }

  // 2.75단계: 교차 검증 — 좌/우 backbone 선분 교차 시 양쪽 tail trim
  // resample 전에 수행하여 trimming 결과가 리샘플링에 반영되도록 함
  trim_crossing_backbones(filtered, left_backbone_ids, right_backbone_ids, owner);

  // 3단계: 좌/우 각각 Resample + SideResult 구성
  if (!left_backbone_ids.empty()) {
    result.left.seed_idx = left_seed;
    result.left.goal_idx = left_backbone_ids.back();
    result.left.seed_backbone_pos = left_seed_bb_pos;
    result.left.stop_reason_forward = left_stop_fwd;
    result.left.backbone.reserve(left_backbone_ids.size());
    for (int idx : left_backbone_ids) {
      result.left.backbone.push_back(filtered[idx]);
    }
    result.left.component = resample_component(
      filtered, left_backbone_ids, cp.resample_ds);
  }

  if (!right_backbone_ids.empty()) {
    result.right.seed_idx = right_seed;
    result.right.goal_idx = right_backbone_ids.back();
    result.right.seed_backbone_pos = right_seed_bb_pos;
    result.right.stop_reason_forward = right_stop_fwd;
    result.right.backbone.reserve(right_backbone_ids.size());
    for (int idx : right_backbone_ids) {
      result.right.backbone.push_back(filtered[idx]);
    }
    result.right.component = resample_component(
      filtered, right_backbone_ids, cp.resample_ds);
  }

  // unchained 포인트 수집
  for (size_t i = 0; i < filtered.size(); ++i) {
    if (owner[i] == NodeOwner::NONE) {
      result.unchained.push_back(filtered[i]);
    }
  }

  result.valid = (!result.left.backbone.empty() ||
                  !result.right.backbone.empty());
  return result;
}

// ============================================================================
// trim_crossing_backbones — 좌/우 backbone 선분 교차 검증 + 양쪽 tail trim
// ============================================================================
//
// [동작]
//   1) left/right backbone의 모든 연속 선분 쌍에 대해 CCW 기반 교차 판정
//   2) 교차 발견 시 양쪽 backbone을 가장 이른 교차 지점에서 tail trim
//   3) 제거된 노드의 owner를 NONE으로 복원 → unchained로 수집됨
//
// [CCW 교차 판정]
//   선분 AB와 CD가 교차 ⟺
//     CCW(A,B,C)·CCW(A,B,D) < 0  AND  CCW(C,D,A)·CCW(C,D,B) < 0
//   CCW(P,Q,R) = (Q-P)×(R-P) 의 부호 (외적 z성분)
//
// ============================================================================

void DirectionChainer::trim_crossing_backbones(
  const std::vector<ChainPoint> & points,
  std::vector<int> & left_bb,
  std::vector<int> & right_bb,
  std::vector<NodeOwner> & owner) const
{
  if (left_bb.size() < 2 || right_bb.size() < 2) return;

  // 외적 z성분: (B-A) × (C-A)
  auto cross = [](const Point2D & A, const Point2D & B, const Point2D & C) -> double {
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
  };

  // CCW 기반 선분 교차 판정
  auto segments_intersect = [&cross](
    const Point2D & A, const Point2D & B,
    const Point2D & C, const Point2D & D) -> bool
  {
    double d1 = cross(A, B, C);
    double d2 = cross(A, B, D);
    double d3 = cross(C, D, A);
    double d4 = cross(C, D, B);
    return (d1 * d2 < 0.0) && (d3 * d4 < 0.0);
  };

  // 양쪽 backbone에서 가장 이른 교차 선분 인덱스 탐색
  int left_n = static_cast<int>(left_bb.size());
  int right_n = static_cast<int>(right_bb.size());
  int min_left_cut = left_n;   // 초기값 = 자르지 않음
  int min_right_cut = right_n;

  for (int i = 0; i + 1 < left_n; ++i) {
    Point2D la = points[left_bb[i]].to_point2d();
    Point2D lb = points[left_bb[i + 1]].to_point2d();
    for (int j = 0; j + 1 < right_n; ++j) {
      Point2D ra = points[right_bb[j]].to_point2d();
      Point2D rb = points[right_bb[j + 1]].to_point2d();
      if (segments_intersect(la, lb, ra, rb)) {
        min_left_cut  = std::min(min_left_cut,  i + 1);
        min_right_cut = std::min(min_right_cut, j + 1);
      }
    }
  }

  // 교차가 없으면 아무 작업 없이 반환
  if (min_left_cut >= left_n && min_right_cut >= right_n) return;

  // 제거 대상 노드의 owner를 NONE으로 복원
  for (int k = min_left_cut; k < left_n; ++k) {
    owner[left_bb[k]] = NodeOwner::NONE;
  }
  for (int k = min_right_cut; k < right_n; ++k) {
    owner[right_bb[k]] = NodeOwner::NONE;
  }

  std::fprintf(stderr,
    "[trim_crossing] left %d→%d pts, right %d→%d pts\n",
    left_n, min_left_cut, right_n, min_right_cut);

  left_bb.resize(min_left_cut);
  right_bb.resize(min_right_cut);
}

}  // namespace chaining_costmap_ver
