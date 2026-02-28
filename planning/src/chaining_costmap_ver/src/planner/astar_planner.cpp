/**
 * @file astar_planner.cpp
 * @brief A* 격자 경로 탐색기 — 구현부
 *
 * 8방향 A* (open set = min-heap by f = g + h).
 * costmap 비용이 obstacle_cost 이상인 셀은 벽으로 처리.
 * goal_tolerance 이내에 도달하면 성공.
 */
#include "chaining_costmap_ver/planner/astar_planner.hpp"

#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>

namespace chaining_costmap_ver
{

namespace
{

// 8방향 이웃 (dr, dc, 이동비용)
struct Neighbor { int dr; int dc; double move_cost; };
constexpr Neighbor kNeighbors[8] = {
  { -1,  0, 1.0 },   // 상
  {  1,  0, 1.0 },   // 하
  {  0, -1, 1.0 },   // 좌
  {  0,  1, 1.0 },   // 우
  { -1, -1, 1.414213562 }, // 좌상
  { -1,  1, 1.414213562 }, // 우상
  {  1, -1, 1.414213562 }, // 좌하
  {  1,  1, 1.414213562 }, // 우하
};

// open set 노드
struct Node
{
  double f;   // f = g + h
  double g;   // 누적 비용
  int idx;    // row * cols + col

  // min-heap: f가 작은 것이 우선
  bool operator>(const Node & o) const { return f > o.f; }
};

// 월드 좌표 → 그리드 인덱스
inline int world_to_cell(double w, double origin, double resolution)
{
  return static_cast<int>(std::round((w - origin) / resolution));
}

// 그리드 인덱스 → 월드 좌표 (셀 중심)
inline double cell_to_world(int c, double origin, double resolution)
{
  return origin + (c + 0.5) * resolution;
}

}  // anonymous namespace

std::vector<Point2D> AStarPlanner::plan(
  const CostmapResult & costmap,
  const Point2D & start,
  const Point2D & goal,
  const PlanningParams & params) const
{
  const auto & ap = params.astar;
  const int rows = costmap.rows;
  const int cols = costmap.cols;
  const double res = costmap.resolution;
  const double ox = costmap.origin_x;
  const double oy = costmap.origin_y;
  const int total = rows * cols;

  if (total <= 0) return {};

  // start/goal → 그리드 좌표
  int sc = world_to_cell(start.x, ox, res);
  int sr = world_to_cell(start.y, oy, res);
  int gc = world_to_cell(goal.x, ox, res);
  int gr = world_to_cell(goal.y, oy, res);

  // 범위 체크
  auto in_bounds = [&](int r, int c) {
    return r >= 0 && r < rows && c >= 0 && c < cols;
  };

  if (!in_bounds(sr, sc) || !in_bounds(gr, gc)) return {};

  int start_idx = sr * cols + sc;

  // goal tolerance를 셀 단위로 변환
  double goal_tol_cells = ap.goal_tolerance / res;
  double goal_tol_cells_sq = goal_tol_cells * goal_tol_cells;

  // g 값 배열 + parent 배열
  std::vector<double> g_score(total, 1e18);
  std::vector<int> parent(total, -1);

  // heuristic: 유클리드 거리 (셀 단위)
  auto heuristic = [&](int idx) -> double {
    int r = idx / cols;
    int c = idx % cols;
    double dr = r - gr;
    double dc = c - gc;
    return std::sqrt(dr * dr + dc * dc);
  };

  // min-heap (priority queue)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  g_score[start_idx] = 0.0;
  open.push({heuristic(start_idx), 0.0, start_idx});

  int best_idx = start_idx;
  double best_h = heuristic(start_idx);
  int iterations = 0;

  while (!open.empty() && iterations < ap.max_iterations) {
    ++iterations;

    Node cur = open.top();
    open.pop();

    // 이미 더 좋은 경로를 찾은 셀이면 건너뜀
    if (cur.g > g_score[cur.idx] + 1e-9) continue;

    int cr = cur.idx / cols;
    int cc = cur.idx % cols;

    // goal 도달 체크 (tolerance)
    double dr_goal = cr - gr;
    double dc_goal = cc - gc;
    if (dr_goal * dr_goal + dc_goal * dc_goal <= goal_tol_cells_sq) {
      best_idx = cur.idx;
      break;
    }

    // best_h 갱신 (탐색 실패 시 가장 가까운 곳까지의 경로 반환)
    double h = heuristic(cur.idx);
    if (h < best_h) {
      best_h = h;
      best_idx = cur.idx;
    }

    // 8방향 확장
    for (const auto & nb : kNeighbors) {
      int nr = cr + nb.dr;
      int nc = cc + nb.dc;
      if (!in_bounds(nr, nc)) continue;

      int nidx = nr * cols + nc;
      double cell_cost = costmap.data[nidx];

      // 장애물 셀은 통과 불가
      if (cell_cost >= ap.obstacle_cost) continue;

      // g(n) = 이전 g + 이동비용(1.0 or √2) × resolution + costmap비용 × weight
      double tentative_g = cur.g
        + nb.move_cost * res
        + cell_cost * ap.cost_weight;

      if (tentative_g < g_score[nidx] - 1e-9) {
        g_score[nidx] = tentative_g;
        parent[nidx] = cur.idx;
        double f = tentative_g + heuristic(nidx) * res;
        open.push({f, tentative_g, nidx});
      }
    }
  }

  // 경로 역추적
  std::vector<Point2D> path;
  int idx = best_idx;
  while (idx != -1) {
    int r = idx / cols;
    int c = idx % cols;
    path.push_back({
      cell_to_world(c, ox, res),
      cell_to_world(r, oy, res)
    });
    if (idx == start_idx) break;
    idx = parent[idx];
  }

  // start→goal 순서로 뒤집기
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace chaining_costmap_ver
