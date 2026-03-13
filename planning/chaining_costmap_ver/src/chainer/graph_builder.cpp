/**
 * @file graph_builder.cpp
 * @brief DirectionChainer — Undirected Graph 구성 구현
 *
 * [포함 함수]
 *   - build_graph(): kNN + G1(거리), G3(횡오차) 게이트로 undirected 그래프 구성
 *
 * [의존 관계]
 *   - types.hpp: ChainPoint, ChainingGraph
 *   - params.hpp: PlanningParams::Chainer (k, d_max, lateral_gate)
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <algorithm>
#include <cmath>

namespace chaining_costmap_ver
{

// ============================================================================
// Undirected Graph 구성 — kNN + G1(거리) + G3(횡오차) 게이트
// ============================================================================
//
// G1: d(i,j) ≤ d_max → 먼 점 연결 차단
// G3: |Δy| ≤ lateral_gate → 좌/우 경계 직접 연결 방지
// G2(전방 콘 게이트)는 backbone에서 동적으로 적용됨
//
ChainingGraph DirectionChainer::build_graph(
  const std::vector<ChainPoint> & points,
  const PlanningParams::Chainer & cp) const
{
  const int n = static_cast<int>(points.size());
  ChainingGraph graph;
  graph.undirected.resize(n);

  for (int i = 0; i < n; ++i) {
    auto neighbors = knn(points, i, cp.k);

    for (int j : neighbors) {
      if (j == i) continue;

      // G1: 거리 게이트
      const double dx = points[j].x - points[i].x;
      const double dy = points[j].y - points[i].y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d > cp.d_max) continue;

      // G3: 횡오차 게이트 (undirected 버전 — 단순 Δy)
      if (std::abs(dy) > cp.lateral_gate) continue;

      // 양방향 edge 추가 (중복 체크)
      auto & adj_i = graph.undirected[i];
      if (std::find(adj_i.begin(), adj_i.end(), j) == adj_i.end()) {
        adj_i.push_back(j);
        graph.undirected[j].push_back(i);
      }
    }
  }

  return graph;
}

}  // namespace chaining_costmap_ver
