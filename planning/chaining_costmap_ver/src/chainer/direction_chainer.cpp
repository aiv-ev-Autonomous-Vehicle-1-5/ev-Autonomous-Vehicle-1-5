/**
 * @file direction_chainer.cpp
 * @brief DirectionChainer — 메인 chain() 오케스트레이터
 *
 * Owner-Label 기반 순차 체이닝의 메인 진입점만 포함.
 * 각 단계의 구현은 별도 .cpp 파일로 분리되어 있다:
 *   - seed_selector.cpp:      find_seed(), knn()
 *   - graph_builder.cpp:      build_graph()
 *   - backbone_extractor.cpp: extract_backbone(), chain_one_direction(),
 *                             compute_cost(), compute_cost_prime()
 *   - branch_extractor.cpp:   extract_branches()
 *   - chain_resampler.cpp:    resample_component()
 *
 * [파이프라인 요약]
 *   준비: find_seed() → build_graph() → owner[] 초기화
 *   1단계: extract_backbone(left, backward+forward)  → LEFT_BACKBONE 라벨
 *   1.5단계: 교차 판정 — owner[right_seed] == LEFT_BACKBONE → left_crossed_right
 *   2단계: extract_backbone(right, backward+forward) → RIGHT_BACKBONE 라벨
 *   2.5단계: 교차 판정 — owner[left_seed] == RIGHT_BACKBONE → right_crossed_left
 *           교차 플래그는 DirectionChainResult에 저장되어 GoalCalculator에서
 *           backbone 중간점 폴백에 사용된다.
 *   3단계: extract_branches(left)  → LEFT_BRANCH 라벨
 *   4단계: extract_branches(right) → RIGHT_BRANCH 라벨
 *   5단계: resample_component() × 2
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

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

  // 준비: Owner 배열 초기화 (all NONE)
  std::vector<NodeOwner> owner(filtered.size(), NodeOwner::NONE);

  // 1단계: Left Backbone 확정 (양방향: backward + forward)
  std::vector<int> left_backbone_ids;
  StopReason left_stop_fwd = StopReason::NO_CANDIDATE;
  int left_seed_bb_pos = 0;
  if (left_seed >= 0) {
    left_backbone_ids = extract_backbone(
      filtered, owner, left_seed, true, left_stop_fwd, left_seed_bb_pos, cp);
    for (int idx : left_backbone_ids) {
      owner[idx] = NodeOwner::LEFT_BACKBONE;
    }
  }

  // 교차 판정 (1단계 직후): left backbone이 right seed를 체이닝했는지
  // right backbone 빌드 전에 검사해야 owner가 덮어써지지 않음
  if (right_seed >= 0 && !left_backbone_ids.empty() &&
      owner[right_seed] == NodeOwner::LEFT_BACKBONE) {
    result.left_crossed_right = true;
  }

  // 2단계: Right Backbone 확정 (양방향: backward + forward)
  std::vector<int> right_backbone_ids;
  StopReason right_stop_fwd = StopReason::NO_CANDIDATE;
  int right_seed_bb_pos = 0;
  if (right_seed >= 0) {
    right_backbone_ids = extract_backbone(
      filtered, owner, right_seed, false, right_stop_fwd, right_seed_bb_pos, cp);
    for (int idx : right_backbone_ids) {
      owner[idx] = NodeOwner::RIGHT_BACKBONE;
    }
  }

  // 교차 판정 (2단계 직후): right backbone이 left seed를 체이닝했는지
  if (left_seed >= 0 && !right_backbone_ids.empty() &&
      owner[left_seed] == NodeOwner::RIGHT_BACKBONE) {
    result.right_crossed_left = true;
  }

  // 3단계: Left Branch 확정
  std::vector<BranchInfo> left_branches;
  if (!left_backbone_ids.empty()) {
    left_branches = extract_branches(
      filtered, left_backbone_ids, owner, NodeOwner::LEFT_BRANCH, cp);
  }

  // 4단계: Right Branch 확정
  std::vector<BranchInfo> right_branches;
  if (!right_backbone_ids.empty()) {
    right_branches = extract_branches(
      filtered, right_backbone_ids, owner, NodeOwner::RIGHT_BRANCH, cp);
  }

  // 5단계: 좌/우 각각 Resample + SideResult 구성
  if (!left_backbone_ids.empty()) {
    result.left.seed_idx = left_seed;
    result.left.goal_idx = left_backbone_ids.back();
    result.left.seed_backbone_pos = left_seed_bb_pos;
    result.left.stop_reason_forward = left_stop_fwd;
    result.left.backbone.reserve(left_backbone_ids.size());
    for (int idx : left_backbone_ids) {
      result.left.backbone.push_back(filtered[idx]);
    }
    result.left.branches = std::move(left_branches);
    result.left.component = resample_component(
      filtered, left_backbone_ids, result.left.branches, cp.resample_ds);
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
    result.right.branches = std::move(right_branches);
    result.right.component = resample_component(
      filtered, right_backbone_ids, result.right.branches, cp.resample_ds);
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

}  // namespace chaining_costmap_ver
