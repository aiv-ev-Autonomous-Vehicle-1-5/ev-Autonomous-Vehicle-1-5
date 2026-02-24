/**
 * @file astar_planner.cpp
 * @brief AstarPlanner::plan() 구현부 — 8-방향 A* 그리드 경로 탐색
 *
 * ## A* 핵심 수식
 *  f(n) = g(n) + h(n)
 *  g(n) = 이전까지 실제 비용 + 이동 비용 + 셀 비용 가중치
 *  h(n) = Octile 거리 휴리스틱 (admissible)
 *
 * ## 이동 비용 (edge cost)
 *  직선(상하좌우): move_cost = 1.0 * resolution
 *  대각선:        move_cost = √2 * resolution ≈ 1.414 * resolution
 *  셀 비용:       cell_cost = cell_cost_weight * cell_value * resolution
 *  total:         edge_cost = move_cost + cell_cost
 *
 * ## Octile 거리 휴리스틱
 *  h = (max(dr,dc) + 0.41421356 * min(dr,dc)) * resolution
 *  0.41421356 = √2 - 1 ≈ 0.4142
 *  8-방향 이동에 admissible (결코 실제 비용보다 크지 않음)
 *
 * ## 자료구조
 *  g_score[]: 각 셀까지의 최소 g값 (초기값=무한대)
 *  parent[]:  각 셀의 부모 셀 인덱스 (경로 역추적용)
 *  closed[]:  이미 확장된 셀 표시 (중복 처리 방지)
 *  open:      우선순위 큐 (f값 기준 최소힙)
 *
 * ## goal snapping
 *  목표 셀이 occupied이면 5셀 반경 내 최근접 free 셀로 자동 이동
 */

#include "track_planning/planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace track_planning
{

// ============================================================
// 8-neighbor A*
// ============================================================
/**
 * @brief 8-방향 A* 경로 탐색
 *
 * ## 초기화 단계
 *  1. start, goal을 world → grid 변환
 *  2. start 또는 goal이 그리드 범위 밖이면 즉시 실패
 *  3. goal 셀이 occupied이면 5셀 반경에서 가장 가까운 free 셀로 스냅
 *     (actual_goal_idx에 저장)
 *  4. g_score[], parent[], closed[] 초기화
 *  5. start 셀의 g=0, f=h(start)로 open에 추가
 *
 * ## 탐색 루프
 *  while (open이 비지 않음):
 *    1. open에서 f값이 가장 작은 셀(cur) 추출
 *    2. cur == actual_goal_idx이면 경로 역추적 후 반환
 *    3. closed[cur]이면 skip (late duplicate detection)
 *    4. closed[cur] = true
 *    5. 8방향 이웃(nr, nc) 계산:
 *       - 범위 밖이면 skip
 *       - closed이면 skip
 *       - cell_value >= 100이면 skip (hard obstacle)
 *       - edge_cost = move_cost[d]*res + cell_cost_weight*cell_val*res
 *       - tentative_g = g_score[cur] + edge_cost
 *       - tentative_g < g_score[ni]이면 g_score, parent 갱신 후 open 추가
 *    6. max_iterations 초과 시 실패 반환
 *
 * ## 경로 역추적
 *  actual_goal_idx에서 parent[]를 역추적하여 path_indices 수집
 *  reverse()로 start→goal 순서로 변환
 *  각 인덱스를 to_world()로 월드 좌표(셀 중심)로 변환
 *
 * ## 좌표 변환 람다
 *  to_grid(pt, col, row): 월드 → 그리드 셀 (floor 기반)
 *  to_world(col, row):    그리드 셀 중심 → 월드
 *    world_x = origin_x + (col + 0.5) * resolution
 *    world_y = origin_y + (row + 0.5) * resolution
 */
AstarPlanner::Result AstarPlanner::plan(
  const std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  const Point2D & start,
  const Point2D & goal,
  int max_iterations,
  double cell_cost_weight)
{
  Result result;

  // ---- 좌표 변환 람다 함수 ----
  // 월드 좌표 → 그리드 셀 인덱스 (floor 기반, 범위 체크 포함)
  auto to_grid = [&](const Point2D & pt, int & col, int & row) -> bool {
    col = static_cast<int>(std::floor((pt.x - origin_x) / resolution));
    row = static_cast<int>(std::floor((pt.y - origin_y) / resolution));
    return (col >= 0 && col < width && row >= 0 && row < height);
  };

  // 그리드 셀 → 월드 좌표 (셀 중심점 반환)
  auto to_world = [&](int col, int row) -> Point2D {
    return {origin_x + (col + 0.5) * resolution,   // 셀 중심 X
            origin_y + (row + 0.5) * resolution};   // 셀 중심 Y
  };

  // ---- start, goal을 그리드 셀로 변환 ----
  int start_col, start_row, goal_col, goal_row;
  if (!to_grid(start, start_col, start_row) ||
      !to_grid(goal, goal_col, goal_row))
  {
    return result;  // start 또는 goal이 그리드 범위 밖 → 탐색 불가
  }

  const int total = width * height;
  const int start_idx = start_row * width + start_col;  // start의 1D 인덱스
  const int goal_idx = goal_row * width + goal_col;      // goal의 1D 인덱스

  // ---- goal snapping: 목표 셀이 occupied이면 근처 free 셀 탐색 ----
  int actual_goal_idx = goal_idx;
  if (grid[goal_idx] != 0) {
    // 목표 셀이 occupied(팽창 영역 포함) → 5셀 반경 탐색
    int best_d2 = std::numeric_limits<int>::max();
    const int sr = 5;  // 탐색 반경: 5셀
    for (int dr = -sr; dr <= sr; ++dr) {
      for (int dc = -sr; dc <= sr; ++dc) {
        int nr = goal_row + dr;
        int nc = goal_col + dc;
        if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;
        int ni = nr * width + nc;
        if (grid[ni] == 0) {  // free 셀 발견
          int d2 = dr * dr + dc * dc;  // 유클리드 거리² (정수 비교)
          if (d2 < best_d2) {
            best_d2 = d2;
            actual_goal_idx = ni;  // 더 가까운 free 셀로 업데이트
          }
        }
      }
    }
    if (grid[actual_goal_idx] != 0) return result;  // 근처 free 셀 없음 → 탐색 불가
  }

  // 실제 목표 셀의 행/열 인덱스 역산
  const int actual_goal_row = actual_goal_idx / width;
  const int actual_goal_col = actual_goal_idx % width;

  // ---- Octile 거리 휴리스틱 람다 ----
  // h(n) = (max(dr,dc) + (√2-1)*min(dr,dc)) * resolution
  // 8-방향 이동에 admissible (실제 비용을 절대 초과하지 않음)
  auto heuristic = [&](int idx) -> double {
    const int r = idx / width;
    const int c = idx % width;
    const int dr = std::abs(r - actual_goal_row);  // 행 방향 거리
    const int dc = std::abs(c - actual_goal_col);  // 열 방향 거리
    // Octile 거리: max(dr,dc) + (√2-1)*min(dr,dc)
    // 0.41421356 = √2 - 1
    return (std::max(dr, dc) + 0.41421356 * std::min(dr, dc)) * resolution;
  };

  // ---- A* 자료구조 초기화 ----
  // g_score: 각 셀까지의 최소 실제 비용 (초기값=무한대)
  std::vector<double> g_score(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
  // parent: 각 셀의 부모 셀 인덱스 (경로 역추적용, -1=부모 없음)
  std::vector<int> parent(static_cast<size_t>(total), -1);
  // closed: 이미 확장된(최적 경로 확정된) 셀 표시
  std::vector<bool> closed(static_cast<size_t>(total), false);

  // 우선순위 큐 (최소힙): (f값, 셀 인덱스) 쌍을 f값 기준으로 정렬
  using PQEntry = std::pair<double, int>;
  std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> open;

  // start 셀 초기화: g=0, f=h(start)
  g_score[start_idx] = 0.0;
  open.push({heuristic(start_idx), start_idx});

  // ---- 8-방향 이동 오프셋 ----
  // dx: 열 방향 이동 (양수=오른쪽, 음수=왼쪽)
  // dy: 행 방향 이동 (양수=위, 음수=아래)
  const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, -1, 1};
  // 직선 이동(인덱스 0~3): 비용=1.0, 대각선(4~7): 비용=√2
  const double move_cost[8] = {1.0, 1.0, 1.0, 1.0,
                                1.41421356, 1.41421356, 1.41421356, 1.41421356};

  int iterations = 0;

  // ---- A* 메인 탐색 루프 ----
  while (!open.empty()) {
    ++iterations;
    // 반복 예산 초과 시 탐색 중단 (0=무제한)
    if (max_iterations > 0 && iterations > max_iterations) {
      result.iterations = iterations;
      return result;  // success=false
    }

    // f값이 가장 작은 셀 추출 (C++17 구조적 바인딩)
    auto [f, cur] = open.top();
    open.pop();

    // ---- 목표 도달 확인 ----
    if (cur == actual_goal_idx) {
      // 경로 역추적: goal → start 방향으로 parent[] 따라감
      std::vector<int> path_indices;
      int node = cur;
      while (node != -1) {
        path_indices.push_back(node);
        node = parent[node];
      }
      // 역순 → start → goal 순서로 변환
      std::reverse(path_indices.begin(), path_indices.end());

      // 1D 인덱스 → 월드 좌표(셀 중심) 변환
      result.path.reserve(path_indices.size());
      for (int idx : path_indices) {
        result.path.push_back(to_world(idx % width, idx / width));
      }
      result.success = true;
      result.iterations = iterations;
      return result;
    }

    // Late duplicate detection: 이미 확장된 셀은 skip
    // (같은 셀이 더 낮은 f값으로 이미 처리됨)
    if (closed[cur]) continue;
    closed[cur] = true;

    // cur의 행/열 인덱스 역산
    const int cur_row = cur / width;
    const int cur_col = cur % width;

    // ---- 8-방향 이웃 탐색 ----
    for (int d = 0; d < 8; ++d) {
      const int nr = cur_row + dy[d];  // 이웃 셀의 행
      const int nc = cur_col + dx[d];  // 이웃 셀의 열

      // 그리드 범위 체크
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;

      const int ni = nr * width + nc;
      if (closed[ni]) continue;  // 이미 확장된 셀 skip

      // 셀 비용 확인
      const int8_t cell_val = grid[ni];
      if (cell_val >= 100) continue;  // hard obstacle (완전 점유) — 통과 불가

      // ---- g값 계산 ----
      // 이동 비용: 직선=1*res, 대각선=√2*res
      // 셀 비용: cell_cost_weight * cell_value * resolution
      //   - free(0): 추가 비용 없음
      //   - inflated(1~99): 장애물 근접에 따른 페널티
      const double edge_cost = move_cost[d] * resolution +
                               cell_cost_weight * static_cast<double>(cell_val) * resolution;
      const double tentative_g = g_score[cur] + edge_cost;

      // 더 낮은 비용 경로 발견 시 업데이트
      if (tentative_g < g_score[ni]) {
        g_score[ni] = tentative_g;
        parent[ni] = cur;  // 부모 셀 기록 (역추적용)
        // f = g + h, open에 추가 (중복 추가 허용 — late duplicate detection으로 처리)
        open.push({tentative_g + heuristic(ni), ni});
      }
    }
  }

  // open이 빈 채로 루프 종료 → 경로 없음
  result.iterations = iterations;
  return result;  // success=false
}

}  // namespace track_planning
