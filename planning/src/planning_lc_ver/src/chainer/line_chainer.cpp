/**
 * @file line_chainer.cpp
 * @brief LineChainer — 구현부
 *
 * Flood fill 기반으로 경계점들을 좌/우 corridor 점 집합으로 분류한다.
 *
 * 핵심 아이디어:
 *   1. y>0 최근접점을 left seed, y<0 최근접점을 right seed로 선택
 *   2. 각 seed에서 flood fill — 탐색 반경 내 모든 점을 감염
 *   3. visited 배열 공유로 좌/우 중복 방지
 *   4. Cone priority: 반경 내 콘+차선 혼합 시 차선 스킵
 *   5. 감염 트리에서 seed→leaf 경로 추출 → 경로별 polyline 리샘플링
 */
#include "planning_lc_ver/chainer/line_chainer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace planning_lc_ver
{

/**
 * @brief 한쪽 점 집합을 flood fill로 수집
 *
 * 알고리즘:
 *   1. seed를 큐에 push, visited 마킹
 *   2. 큐에서 pop → 탐색 반경 내 미방문 이웃 수집
 *   3. Cone priority: 콘+차선 혼합 시 차선 스킵
 *   4. 이웃이 없으면 radius를 step씩 증가 (max까지)
 *   5. 찾은 이웃 전부 visited + chain에 추가 + 큐에 push
 *   6. 큐가 빌 때까지 반복
 *   7. 감염 트리에서 모든 seed→leaf 경로 추출 → 각 경로를 polyline 리샘플링
 */
std::vector<ChainedPoint> LineChainer::chain_one_side(
  const std::vector<ChainedPoint> & candidates,
  int seed_idx,
  std::vector<bool> & visited,
  const PlanningParams & params)
{
  std::vector<ChainedPoint> chain;
  const int n = static_cast<int>(candidates.size());
  if (seed_idx < 0 || seed_idx >= n) return chain;

  const auto & cp = params.chainer;

  // candidate idx → chain idx 매핑
  std::vector<int> cand_to_chain(n, -1);

  // seed 추가
  visited[seed_idx] = true;
  chain.push_back(candidates[seed_idx]);
  cand_to_chain[seed_idx] = 0;

  // edge 기록: (parent chain idx, child chain idx)
  std::vector<std::pair<int, int>> edges;

  std::queue<int> q;
  q.push(seed_idx);

  while (!q.empty()) {
    const int cur = q.front();
    q.pop();

    const double cx = candidates[cur].x;
    const double cy = candidates[cur].y;
    const int cur_chain_idx = cand_to_chain[cur];

    // ── 탐색: radius 확장 루프 ──
    for (double r = cp.search_radius; r <= cp.search_radius_max + 1e-6;
         r += cp.search_radius_step)
    {
      const double r_sq = r * r;

      // 반경 내 미방문 이웃 수집
      struct Neighbor { int idx; bool is_cone; };
      std::vector<Neighbor> neighbors;
      bool has_cone = false;
      bool has_lane = false;

      for (int i = 0; i < n; ++i) {
        if (visited[i]) continue;

        const double dx = candidates[i].x - cx;
        const double dy = candidates[i].y - cy;
        const double d_sq = dx * dx + dy * dy;

        if (d_sq < 1e-12 || d_sq > r_sq) continue;

        const bool cone = (candidates[i].type == PointType::CONE);
        neighbors.push_back({i, cone});
        if (cone) has_cone = true;
        else      has_lane = true;
      }

      if (neighbors.empty()) continue;  // 이 반경에서 없으면 확장

      // ── 감염: 전부 chain에 추가 + 큐에 push + edge 기록 ──
      for (const auto & nb : neighbors) {
        // Cone priority: 혼합 시 차선 스킵
        if (has_cone && has_lane && !nb.is_cone) continue;

        visited[nb.idx] = true;
        const int child_chain_idx = static_cast<int>(chain.size());
        chain.push_back(candidates[nb.idx]);
        cand_to_chain[nb.idx] = child_chain_idx;
        edges.push_back({cur_chain_idx, child_chain_idx});
        q.push(nb.idx);
      }

      break;  // 이 반경에서 이웃을 찾았으면 확장 중단
    }
  }

  // ── 트리에서 polyline 추출 + 경로별 리샘플링 ──
  const double ds = cp.resample_ds;
  const int cn = static_cast<int>(chain.size());

  // parent 배열 구축 + children 카운트 (leaf 판별용)
  std::vector<int> parent(cn, -1);
  std::vector<int> child_count(cn, 0);
  for (const auto & edge : edges) {
    parent[edge.second] = edge.first;
    child_count[edge.first]++;
  }

  // leaf 노드 수집 (children이 없는 노드)
  std::vector<int> leaves;
  for (int i = 0; i < cn; ++i) {
    if (child_count[i] == 0) leaves.push_back(i);
  }

  // 각 leaf → seed 역추적 → polyline 추출 + 리샘플링
  std::vector<ChainedPoint> result;
  result.reserve(chain.size() * 2);

  for (const int leaf : leaves) {
    // leaf → seed 역추적
    std::vector<int> path_idx;
    for (int cur = leaf; cur >= 0; cur = parent[cur]) {
      path_idx.push_back(cur);
    }
    std::reverse(path_idx.begin(), path_idx.end());
    // path_idx = [seed(0), ..., leaf]

    // polyline 리샘플링: 각 구간마다 보간점 삽입
    for (size_t s = 0; s < path_idx.size(); ++s) {
      result.push_back(chain[path_idx[s]]);

      if (s + 1 < path_idx.size()) {
        const auto & a = chain[path_idx[s]];
        const auto & b = chain[path_idx[s + 1]];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len = std::sqrt(dx * dx + dy * dy);

        if (len >= ds) {
          const PointType seg_type =
            (a.type == PointType::CONE && b.type == PointType::CONE)
              ? PointType::CONE : PointType::LANE;
          for (double d = ds; d < len; d += ds) {
            const double t = d / len;
            result.push_back({a.x + t * dx, a.y + t * dy, seg_type});
          }
        }
      }
    }
  }

  return result;
}

/**
 * @brief 메인 체이닝 함수
 *
 * 1. Left seed (y>0 최근접) / Right seed (y<0 최근접) 선택
 * 2. Flood fill (visited 공유) + edge별 리샘플링
 */
ChainResult LineChainer::chain(
  const std::vector<ChainedPoint> & candidates,
  const PlanningParams & params)
{
  ChainResult result;
  const int n = static_cast<int>(candidates.size());
  if (n < 2) return result;

  // ── Seed 선택 ──
  // Left seed: y > 0인 점 중 ego(0,0)에서 가장 가까운 점
  // Right seed: y < 0인 점 중 ego(0,0)에서 가장 가까운 점
  int left_seed = -1, right_seed = -1;
  double left_dist_sq = std::numeric_limits<double>::max();
  double right_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i < n; ++i) {
    const double d_sq = candidates[i].x * candidates[i].x +
                        candidates[i].y * candidates[i].y;
    if (candidates[i].y > 0.0) {
      if (d_sq < left_dist_sq) {
        left_dist_sq = d_sq;
        left_seed = i;
      }
    } else if (candidates[i].y < 0.0) {
      if (d_sq < right_dist_sq) {
        right_dist_sq = d_sq;
        right_seed = i;
      }
    }
    // y == 0.0인 점은 seed 선택에서 제외
  }

  if (left_seed < 0 && right_seed < 0) return result;

  // ── Flood fill (visited 공유) ──
  std::vector<bool> visited(n, false);

  std::vector<ChainedPoint> left_raw, right_raw;

  if (left_seed >= 0) {
    left_raw = chain_one_side(candidates, left_seed, visited, params);
  }
  if (right_seed >= 0) {
    right_raw = chain_one_side(candidates, right_seed, visited, params);
  }

  // chain_one_side 내부에서 edge별 리샘플링 완료 → 그대로 전달
  result.left_chain = std::move(left_raw);
  result.right_chain = std::move(right_raw);

  result.valid = (!result.left_chain.empty() || !result.right_chain.empty());
  return result;
}

}  // namespace planning_lc_ver
