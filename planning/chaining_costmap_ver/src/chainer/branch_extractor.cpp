/**
 * @file branch_extractor.cpp
 * @brief DirectionChainer — Branch 추출 구현
 *
 * [포함 함수]
 *   - extract_branches(): Backbone 순회 기반 Greedy Chaining으로 branch 수집
 *
 * [의존 관계]
 *   - types.hpp: ChainPoint, BranchInfo, NodeOwner
 *   - params.hpp: PlanningParams::Chainer (d_max, max_branch_len)
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <cmath>
#include <limits>
#include <unordered_set>

namespace chaining_costmap_ver
{

// ============================================================================
// Branch 추출 — Backbone 순회 기반 Greedy Chaining
// ============================================================================
//
// backbone 노드를 B0→B1→B2→... 순서로 순회하며:
//   1) Bi에서 d_max 내 허용 노드 중 최근접을 chain 시작점으로 선택
//   2) chain 끝점에서 d_max 내 최근접 미방문 허용 노드로 greedy 이동
//   3) max_branch_len 도달 또는 후보 소진 시 종료
//
// 허용 조건: owner[node] == NONE || owner[node] == branch_label
//   → 같은 side의 branch 노드는 중복 소속 가능
//   → 반대 side의 backbone/branch는 차단
//
std::vector<BranchInfo> DirectionChainer::extract_branches(
  const std::vector<ChainPoint> & points,
  const std::vector<int> & backbone_ids,
  std::vector<NodeOwner> & owner,
  NodeOwner branch_label,
  const PlanningParams::Chainer & cp) const
{
  const int n = static_cast<int>(points.size());
  const double d_max = cp.d_max;
  const double d_max2 = d_max * d_max;

  std::unordered_set<int> backbone_set(backbone_ids.begin(), backbone_ids.end());
  std::vector<BranchInfo> branches;

  for (int b_idx = 0; b_idx < static_cast<int>(backbone_ids.size()); ++b_idx) {
    const int b_node = backbone_ids[b_idx];
    std::unordered_set<int> visited;

    // 1단계: backbone 노드에서 d_max 내 최근접 허용 노드 탐색
    int first = -1;
    double first_dist2 = std::numeric_limits<double>::infinity();

    for (int j = 0; j < n; ++j) {
      if (backbone_set.count(j)) continue;
      if (owner[j] != NodeOwner::NONE && owner[j] != branch_label) continue;

      const double dx = points[j].x - points[b_node].x;
      const double dy = points[j].y - points[b_node].y;
      const double dist2 = dx * dx + dy * dy;
      if (dist2 > d_max2) continue;

      if (dist2 < first_dist2) {
        first_dist2 = dist2;
        first = j;
      }
    }

    if (first < 0) continue;

    // 2단계: Greedy Chaining
    std::vector<int> chain_nodes;
    chain_nodes.push_back(first);
    visited.insert(first);
    owner[first] = branch_label;

    int current = first;
    while (static_cast<int>(chain_nodes.size()) < cp.max_branch_len) {
      int best = -1;
      double best_dist2 = std::numeric_limits<double>::infinity();

      for (int j = 0; j < n; ++j) {
        if (backbone_set.count(j)) continue;
        if (owner[j] != NodeOwner::NONE && owner[j] != branch_label) continue;
        if (visited.count(j)) continue;

        const double dx = points[j].x - points[current].x;
        const double dy = points[j].y - points[current].y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 > d_max2) continue;

        if (dist2 < best_dist2) {
          best_dist2 = dist2;
          best = j;
        }
      }

      if (best < 0) break;

      chain_nodes.push_back(best);
      visited.insert(best);
      owner[best] = branch_label;
      current = best;
    }

    // BranchInfo 구성
    BranchInfo bi;
    bi.parent_backbone_idx = b_idx;
    bi.points.reserve(chain_nodes.size());
    for (int idx : chain_nodes) {
      bi.points.push_back(points[idx]);
    }
    bi.score = 1.0 / static_cast<double>(chain_nodes.size());
    branches.push_back(std::move(bi));
  }

  return branches;
}

}  // namespace chaining_costmap_ver
