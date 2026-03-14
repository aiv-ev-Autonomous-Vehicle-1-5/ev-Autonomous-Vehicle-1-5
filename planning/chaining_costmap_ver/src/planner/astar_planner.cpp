/**
 * @file astar_planner.cpp
 * @brief A* 격자 경로 탐색기 — 구현부
 *
 * 8방향 A* (open set = min-heap by f = g + h).
 * costmap 비용이 obstacle_cost 이상인 셀은 벽으로 처리.
 * goal_tolerance 이내에 도달하면 성공.
 *
 * ── 알고리즘 전체 흐름 ──
 *
 *   1) 좌표 변환: 월드 좌표(start, goal)를 격자 좌표(row, col)로 변환
 *   2) 초기화: g_score 배열(∞), parent 배열(-1), open set에 start 삽입
 *   3) 메인 루프:
 *      a) open set에서 f 최소인 노드를 꺼낸다
 *      b) goal tolerance 이내이면 성공 → 역추적으로 경로 복원
 *      c) 8방향 이웃을 확장:
 *         - 범위 밖 / obstacle_cost 이상 → 건너뜀
 *         - tentative_g = g(cur) + 이동비용×해상도 + 셀비용×가중치
 *         - tentative_g < g_score[이웃] → 갱신 + open set에 삽입
 *      d) max_iterations 초과 시 탐색 중단
 *   4) 역추적: parent 배열을 따라 goal→start 순서로 경로 수집 후 reverse
 *
 * ── best_idx 폴백 ──
 *   goal에 정확히 도달하지 못해도, 탐색 중 goal에 가장 가까웠던 셀까지의
 *   경로를 반환한다 (partial path). 완전 실패(start만 있는 경우)보다 나은
 *   결과를 제공하여, 차량이 최대한 goal 방향으로 이동할 수 있게 한다.
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

// ============================================================================
// 8방향 이웃 정의
// ============================================================================
// dr, dc: 행/열 변위, move_cost: 이동 비용 (직선=1.0, 대각선=√2)
//
//   좌상(-1,-1)  상(-1,0)  우상(-1,+1)
//         좌(0,-1)    ●    우(0,+1)
//   좌하(+1,-1)  하(+1,0)  우하(+1,+1)
//
// 대각선 이동은 직선보다 √2 ≈ 1.414배 더 비싸다.
// 이렇게 해야 "대각선 2번 = 직선 2번" 이 아닌 실제 거리를 반영한다.
struct Neighbor { int dr; int dc; double move_cost; };
constexpr Neighbor kNeighbors[8] = {
  { -1,  0, 1.0 },          // 상 (row 감소)
  {  1,  0, 1.0 },          // 하 (row 증가)
  {  0, -1, 1.0 },          // 좌 (col 감소)
  {  0,  1, 1.0 },          // 우 (col 증가)
  { -1, -1, 1.414213562 },  // 좌상 (대각선 √2)
  { -1,  1, 1.414213562 },  // 우상 (대각선 √2)
  {  1, -1, 1.414213562 },  // 좌하 (대각선 √2)
  {  1,  1, 1.414213562 },  // 우하 (대각선 √2)
};

// ============================================================================
// open set 노드 구조체
// ============================================================================
// A*의 priority queue에 들어가는 항목.
// f = g + h 값이 작을수록 우선 탐색한다 (min-heap).
struct Node
{
  double f;   // f(n) = g(n) + h(n) — 총 추정 비용
  double g;   // g(n) — start에서 이 노드까지의 실제 누적 비용
  int idx;    // 1D 인덱스 = row * cols + col (2D 격자를 1D 배열로 매핑)

  // min-heap용 비교: f가 작은 노드가 우선 (std::greater 사용)
  bool operator>(const Node & o) const { return f > o.f; }
};

// ============================================================================
// 좌표 변환 유틸
// ============================================================================

/**
 * @brief 월드 좌표 → 그리드 셀 인덱스 변환
 *
 * 수식: cell = round((world - origin) / resolution)
 *
 * origin은 격자의 (0,0) 셀이 대응하는 월드 좌표이다.
 * round를 사용하므로 셀 중심에 가장 가까운 셀이 선택된다.
 *
 * @param w          월드 좌표 [m]
 * @param origin     격자 원점의 월드 좌표 [m]
 * @param resolution 셀 크기 [m/cell]
 * @return 셀 인덱스 (0-based)
 */
inline int world_to_cell(double w, double origin, double resolution)
{
  return static_cast<int>(std::round((w - origin) / resolution));
}

/**
 * @brief 그리드 셀 인덱스 → 월드 좌표 변환 (셀 중심 반환)
 *
 * 수식: world = origin + (cell + 0.5) * resolution
 *
 * +0.5를 하는 이유: 셀의 "왼쪽 모서리"가 아닌 "중심"의 좌표를 반환하기 위함.
 * 예: cell=0, origin=0, res=0.15 → world = 0.075m (셀 중심)
 *
 * @param c          셀 인덱스
 * @param origin     격자 원점의 월드 좌표 [m]
 * @param resolution 셀 크기 [m/cell]
 * @return 셀 중심의 월드 좌표 [m]
 */
inline double cell_to_world(int c, double origin, double resolution)
{
  return origin + (c + 0.5) * resolution;
}

}  // anonymous namespace

// ============================================================================
// plan() — A* 경로 탐색 메인 함수
// ============================================================================
//
// [입력]
//   costmap : CostmapGenerator가 생성한 2D 비용 격자 (rows × cols, double[])
//   start   : 시작점 (보통 ego 위치 {0, 0})
//   goal    : 목표점 (좌/우 backbone 끝점의 중점)
//   params  : A* 하이퍼파라미터 (max_iterations, goal_tolerance, cost_weight, obstacle_cost)
//
// [출력]
//   Point2D 벡터 (start→goal 순서의 월드 좌표 경로)
//   탐색 실패 시: goal에 가장 가까웠던 셀까지의 partial path 반환
//   costmap이 비어있거나 start/goal이 범위 밖이면 빈 벡터 반환
//
// [비용 계산 상세]
//   g(n→m) = g(n) + move_cost × resolution + costmap[m] × cost_weight
//
//   ┌─────────────────────────────────────────────────────────────────┐
//   │  move_cost × resolution                                        │
//   │    = 직선(1.0) or 대각선(√2) × 셀 크기                         │
//   │    → 실제 이동 거리 [m] (짧은 경로 선호)                        │
//   │                                                                 │
//   │  costmap[m] × cost_weight                                       │
//   │    = 해당 셀의 가우시안 비용 × 가중치                            │
//   │    → bbox/차선 근처를 지나면 추가 비용 (장애물 회피 유도)          │
//   │    예: cost=100(bbox), weight=0.05 → 5.0 추가비용                 │
//   │        cost=50(차선), weight=0.05 → 2.5 추가비용                │
//   └─────────────────────────────────────────────────────────────────┘
//
//   f(n) = g(n) + h(n) × resolution
//   h(n) = 유클리드 거리 (셀 단위) → ×resolution으로 [m] 단위로 변환
//   → h는 항상 실제 거리 이하 (admissible) → A*가 최적 경로를 보장한다.
//
// ============================================================================

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
  const double ox = costmap.origin_x;    // 격자 원점 X (월드 좌표)
  const double oy = costmap.origin_y;    // 격자 원점 Y (월드 좌표)
  const int total = rows * cols;         // 전체 셀 수

  if (total <= 0) return {};

  // ── 1) 월드 좌표 → 격자 좌표 변환 ──
  // start(ego) → (sr, sc), goal → (gr, gc)
  // 주의: col=X축, row=Y축 (격자의 행은 Y 방향)
  int sc = world_to_cell(start.x, ox, res);  // start col (X)
  int sr = world_to_cell(start.y, oy, res);  // start row (Y)
  int gc = world_to_cell(goal.x, ox, res);   // goal col (X)
  int gr = world_to_cell(goal.y, oy, res);   // goal row (Y)

  // 범위 체크 람다: (row, col)이 격자 안에 있는지 확인
  auto in_bounds = [&](int r, int c) {
    return r >= 0 && r < rows && c >= 0 && c < cols;
  };

  // start 또는 goal이 격자 밖이면 탐색 불가
  if (!in_bounds(sr, sc) || !in_bounds(gr, gc)) return {};

  // 1D 인덱스로 변환 (2D 격자를 1D 배열로 매핑)
  int start_idx = sr * cols + sc;

  // ── 2) goal tolerance 변환 ──
  // 월드 좌표의 tolerance [m]를 셀 단위로 변환
  // goal_tol_cells_sq: 제곱 형태로 저장하여 매 비교마다 sqrt 생략
  double goal_tol_cells = ap.goal_tolerance / res;
  double goal_tol_cells_sq = goal_tol_cells * goal_tol_cells;

  // ── 3) 배열 초기화 ──
  // g_score: 각 셀까지의 최소 누적 비용 (초기값 ∞ = 1e18)
  // parent : 경로 역추적용 부모 셀 인덱스 (초기값 -1 = 부모 없음)
  std::vector<double> g_score(total, 1e18);
  std::vector<int> parent(total, -1);

  // ── 4) 휴리스틱 함수 ──
  // 유클리드 거리 (셀 단위): h(n) = √((nr-gr)² + (nc-gc)²)
  // admissible: 항상 실제 거리 이하 → A*의 최적성 보장
  // consistent: 삼각 부등식 만족 → closed set 재방문 불필요
  auto heuristic = [&](int idx) -> double {
    int r = idx / cols;
    int c = idx % cols;
    double dr = r - gr;
    double dc = c - gc;
    return std::sqrt(dr * dr + dc * dc);
  };

  // ── 5) open set 초기화 ──
  // min-heap: f값이 가장 작은 노드를 먼저 꺼낸다
  // std::greater<Node>: Node::operator>를 사용하여 f 오름차순 정렬
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  g_score[start_idx] = 0.0;
  open.push({heuristic(start_idx), 0.0, start_idx});

  // best_idx: goal에 가장 가까이 도달한 셀 (탐색 실패 시 폴백용)
  // best_h : best_idx의 휴리스틱 값 (작을수록 goal에 가까움)
  int best_idx = start_idx;
  double best_h = heuristic(start_idx);
  int iterations = 0;

  // ============================================================
  // 6) A* 메인 루프
  // ============================================================
  while (!open.empty() && iterations < ap.max_iterations) {
    ++iterations;

    // open set에서 f 최소인 노드를 꺼낸다
    Node cur = open.top();
    open.pop();

    // lazy deletion: 이미 더 좋은 경로를 찾은 셀이면 건너뜀
    // (같은 셀이 open set에 여러 번 들어갈 수 있으므로)
    if (cur.g > g_score[cur.idx] + 1e-9) continue;

    int cr = cur.idx / cols;  // 현재 노드의 row
    int cc = cur.idx % cols;  // 현재 노드의 col

    // ── goal 도달 체크 ──
    // 현재 셀과 goal 셀 사이의 거리²가 tolerance² 이내이면 성공
    double dr_goal = cr - gr;
    double dc_goal = cc - gc;
    if (dr_goal * dr_goal + dc_goal * dc_goal <= goal_tol_cells_sq) {
      best_idx = cur.idx;
      break;  // 탐색 성공 → 역추적으로 이동
    }

    // ── best_idx 갱신 ──
    // goal에 도달 못 하더라도, 가장 가까웠던 셀을 기록해 둔다.
    // 탐색 실패(max_iterations 초과) 시 이 셀까지의 partial path를 반환한다.
    double h = heuristic(cur.idx);
    if (h < best_h) {
      best_h = h;
      best_idx = cur.idx;
    }

    // ── 8방향 이웃 확장 ──
    for (const auto & nb : kNeighbors) {
      int nr = cr + nb.dr;  // 이웃 row
      int nc = cc + nb.dc;  // 이웃 col
      if (!in_bounds(nr, nc)) continue;  // 격자 범위 밖 → 건너뜀

      int nidx = nr * cols + nc;
      double cell_cost = costmap.data[nidx];

      // 장애물 셀 판정: costmap 비용이 obstacle_cost 이상이면 "벽"
      // → A*가 이 셀로 절대 이동하지 않는다
      // 예: obstacle_cost=100, bbox_cost_max=100 → bbox 중심은 벽
      //     lane_cost_max=50 < 100 → 차선은 통과 가능 (비용만 추가)
      if (cell_cost >= ap.obstacle_cost) continue;

      // ── tentative g 계산 ──
      // g(이웃) = g(현재) + 이동 거리[m] + costmap 비용 × 가중치
      //
      // move_cost × res: 실제 이동 거리 (직선=1×0.15=0.15m, 대각선=√2×0.15≈0.21m)
      // cell_cost × cost_weight: 비용 패널티 (bbox 근처일수록 큼)
      double tentative_g = cur.g
        + nb.move_cost * res         // 유클리드 이동 거리 [m]
        + cell_cost * ap.cost_weight; // costmap 비용 패널티

      // 기존 g_score보다 작으면 갱신 (더 좋은 경로 발견)
      if (tentative_g < g_score[nidx] - 1e-9) {
        g_score[nidx] = tentative_g;
        parent[nidx] = cur.idx;  // 역추적용 부모 기록

        // f(n) = g(n) + h(n) × resolution
        // h는 셀 단위이므로 resolution을 곱해서 [m] 단위로 맞춤
        double f = tentative_g + heuristic(nidx) * res;
        open.push({f, tentative_g, nidx});
      }
    }
  }

  // ============================================================
  // 7) 경로 역추적 (backtracking)
  // ============================================================
  // parent 배열을 따라 best_idx(또는 goal) → start까지 셀을 수집한다.
  // 각 셀의 1D 인덱스를 (row, col)로 분해한 뒤 월드 좌표로 변환.
  // 수집 순서는 goal→start이므로, 마지막에 reverse하여 start→goal 순서로 만든다.
  std::vector<Point2D> path;
  int idx = best_idx;
  while (idx != -1) {
    int r = idx / cols;
    int c = idx % cols;
    path.push_back({
      cell_to_world(c, ox, res),   // col → X 월드 좌표 (셀 중심)
      cell_to_world(r, oy, res)    // row → Y 월드 좌표 (셀 중심)
    });
    if (idx == start_idx) break;   // start에 도달하면 종료
    idx = parent[idx];             // 부모 셀로 이동
  }

  // goal→start 순서를 start→goal 순서로 뒤집기
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace chaining_costmap_ver
