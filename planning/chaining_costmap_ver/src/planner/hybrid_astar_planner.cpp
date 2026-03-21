/**
 * @file hybrid_astar_planner.cpp
 * @brief Hybrid A* 경로 탐색기 — 구현부
 *
 * ── 알고리즘 전체 흐름 ──
 *
 *   1) 조향각 후보 생성: -delta_max ~ +delta_max 사이 N_STEER(7)개 등간격
 *
 *   2) 시작 노드 삽입: (start.x, start.y, yaw=0.0), g=0
 *
 *   3) 메인 루프:
 *      a) open set에서 f 최소 노드 꺼냄
 *      b) lazy deletion: 이미 더 좋은 g가 기록된 상태면 건너뜀
 *      c) goal tolerance 이내이면 성공 → 역추적
 *      d) N_STEER개 모션 프리미티브 확장:
 *         - 자전거 모델 (midpoint rule)로 다음 상태 계산
 *         - 격자 범위 / 장애물 체크
 *         - (col, row, yaw_bin) 키로 중복 방문 체크
 *         - g 갱신 후 open set에 삽입
 *      e) max_iterations 초과 시 best_node(goal에 가장 가까운 노드)로 폴백
 *
 *   4) 역추적: nodes[] 배열의 parent 인덱스를 따라 경로 복원 후 reverse
 *
 * ── 자전거 모델 (midpoint rule) ──
 *
 *   kappa   = tan(δ) / L          (곡률)
 *   mid_yaw = yaw + kappa * s/2   (중간 헤딩 — Euler 적분보다 정확)
 *   x'      = x + s * cos(mid_yaw)
 *   y'      = y + s * sin(mid_yaw)
 *   yaw'    = yaw + kappa * s
 *
 *   L = wheelbase, s = arc_length, δ = 조향각
 *
 * ── 방문 체크 ──
 *
 *   상태 (x, y, yaw)를 (col, row, yaw_bin) 삼중 키로 이산화.
 *   key = (row * cols + col) * N_YAW + yaw_bin
 *   unordered_map<int, double>에 best_g[key]를 저장.
 *   새 g < best_g[key] 일 때만 확장.
 *
 * ── 비용 함수 ──
 *
 *   g(n→m) = g(n) + arc_length + costmap[m] * cost_weight
 *     arc_length : 실제 이동 거리 [m]
 *     costmap[m] * cost_weight : 장애물 근접 패널티
 *
 *   f(n) = g(n) + h(n)
 *   h(n) = 유클리드 거리(goal까지) [m]
 */
#include "chaining_costmap_ver/planner/hybrid_astar_planner.hpp"

#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <limits>

namespace chaining_costmap_ver
{

namespace
{

// ============================================================================
// 상수
// ============================================================================

// 조향각 후보 수: -delta_max, ..., 0, ..., +delta_max
constexpr int N_STEER = 7;

// 방향각 이산화 bin 수: 360° / 5° = 72
constexpr int N_YAW = 72;

// bin당 각도 [rad]: 2π / 72 ≈ 0.0873 rad ≈ 5°
constexpr double YAW_RES = 2.0 * M_PI / N_YAW;

// ============================================================================
// 노드 구조체
// ============================================================================
//
// nodes[] 벡터에 탐색된 모든 노드를 저장.
// parent: 역추적용 부모 노드 인덱스 (-1 = 시작 노드)
//
struct HNode
{
  double x;       // 연속 X 좌표 [m]
  double y;       // 연속 Y 좌표 [m]
  double yaw;     // 방향각 [rad]
  double g;       // start까지 실제 누적 비용
  int    parent;  // 부모 노드 인덱스 (-1: 없음)
};

// ============================================================================
// 유틸 함수
// ============================================================================

/**
 * @brief 방향각 → yaw bin 인덱스 [0, N_YAW)
 *
 * 임의의 yaw를 [0, 2π) 범위로 정규화한 후 YAW_RES로 나눠 bin 인덱스를 반환.
 */
inline int yaw_to_bin(double yaw)
{
  double y = std::fmod(yaw, 2.0 * M_PI);
  if (y < 0.0) y += 2.0 * M_PI;
  return static_cast<int>(y / YAW_RES) % N_YAW;
}

/**
 * @brief costmap에서 월드 좌표의 비용 조회
 *
 * 격자 범위 밖이면 통과 불가 비용(999)을 반환.
 */
inline double get_costmap_cost(
  const CostmapResult & cm, double wx, double wy)
{
  int col = static_cast<int>((wx - cm.origin_x) / cm.resolution);
  int row = static_cast<int>((wy - cm.origin_y) / cm.resolution);
  if (col < 0 || col >= cm.cols || row < 0 || row >= cm.rows) {
    return 999.0;
  }
  return cm.data[row * cm.cols + col];
}

/**
 * @brief (col, row, yaw_bin) → 1D 방문 체크 키
 *
 * key = (row * cols + col) * N_YAW + yaw_bin
 * 세 값이 유효 범위 내일 때 고유한 정수를 반환.
 */
inline int state_key(int col, int row, int yaw_bin, int cols)
{
  return (row * cols + col) * N_YAW + yaw_bin;
}

}  // anonymous namespace

// ============================================================================
// plan() — Hybrid A* 메인 함수
// ============================================================================

std::vector<Point2D> HybridAStarPlanner::plan(
  const CostmapResult & cm,
  const Point2D & start,
  const Point2D & goal,
  const PlanningParams & params) const
{
  if (!cm.valid || cm.rows <= 0 || cm.cols <= 0) return {};

  const auto & ap  = params.astar;
  const auto & vp  = params.vehicle;
  const auto & hap = params.hybrid_astar;

  const double L         = vp.wheelbase;       // 축거 [m]
  const double delta_max = vp.delta_max;        // 최대 조향각 [rad]
  const double arc_len   = hap.arc_length;      // 한 스텝 호 길이 [m]
  const double cost_w    = ap.cost_weight;      // costmap 비용 가중치
  const double obs_cost  = ap.obstacle_cost;    // 통과 불가 비용 임계값

  // ── 1) 조향각 후보 생성 ──
  // -delta_max ~ +delta_max 사이를 N_STEER 등간격으로 나눔
  double steers[N_STEER];
  for (int i = 0; i < N_STEER; ++i) {
    steers[i] = -delta_max + (2.0 * delta_max / (N_STEER - 1)) * i;
  }

  // ── 2) 방문 체크용 해시맵 ──
  // key → 해당 (col, row, yaw_bin) 상태에서 발견된 최소 g 비용
  // unordered_map을 사용하여 희소한 상태만 저장 (메모리 절약)
  std::unordered_map<int, double> best_g;
  best_g.reserve(ap.max_iterations * 4);

  // ── 3) 탐색된 모든 노드 저장 (역추적용) ──
  // open set에서 꺼낼 때마다 nodes에 추가.
  // parent 인덱스로 역추적하여 경로를 복원한다.
  std::vector<HNode> nodes;
  nodes.reserve(ap.max_iterations);

  // ── 4) 휴리스틱 — 유클리드 거리 ──
  auto heuristic = [&](double x, double y) -> double {
    return std::hypot(goal.x - x, goal.y - y);
  };

  // ── 5) open set (min-heap) ──
  // (f값, nodes 인덱스) 쌍으로 관리.
  // f가 작은 노드를 먼저 꺼낸다.
  using PQItem = std::pair<double, int>;
  std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;

  // ── 6) 시작 노드 삽입 ──
  // 시작 시 yaw = 0.0 (base_link 전방 = x축 방향)
  // goal_h: 시작점에서 goal까지 유클리드 거리
  {
    double h0 = heuristic(start.x, start.y);
    nodes.push_back({start.x, start.y, 0.0, 0.0, -1});
    open.push({h0, 0});

    int sc  = static_cast<int>((start.x - cm.origin_x) / cm.resolution);
    int sr  = static_cast<int>((start.y - cm.origin_y) / cm.resolution);
    int syi = yaw_to_bin(0.0);
    best_g[state_key(sc, sr, syi, cm.cols)] = 0.0;
  }

  // best_node: goal에 가장 가까이 도달한 노드 인덱스 (폴백용)
  int    best_node_idx = 0;
  double best_h        = heuristic(start.x, start.y);

  int goal_node_idx = -1;  // 탐색 성공 시 goal 노드 인덱스
  int iterations    = 0;

  // ============================================================
  // 7) Hybrid A* 메인 루프
  // ============================================================
  while (!open.empty() && iterations < ap.max_iterations) {
    ++iterations;

    auto [f_cur, cur_idx] = open.top();
    open.pop();

    const HNode & cur = nodes[cur_idx];

    // ── lazy deletion ──
    // open set에 같은 상태가 여러 번 들어갈 수 있음.
    // 현재 g보다 더 좋은 g가 이미 기록됐으면 건너뜀.
    {
      int cc  = static_cast<int>((cur.x - cm.origin_x) / cm.resolution);
      int cr  = static_cast<int>((cur.y - cm.origin_y) / cm.resolution);
      int cyi = yaw_to_bin(cur.yaw);
      if (cc < 0 || cc >= cm.cols || cr < 0 || cr >= cm.rows) continue;

      auto it = best_g.find(state_key(cc, cr, cyi, cm.cols));
      if (it != best_g.end() && cur.g > it->second + 1e-9) continue;
    }

    // ── goal 도달 체크 ──
    double dist_to_goal = std::hypot(goal.x - cur.x, goal.y - cur.y);
    if (dist_to_goal <= ap.goal_tolerance) {
      goal_node_idx = cur_idx;
      break;
    }

    // best_node 갱신 (폴백용)
    double h_cur = heuristic(cur.x, cur.y);
    if (h_cur < best_h) {
      best_h        = h_cur;
      best_node_idx = cur_idx;
    }

    // ── 8) 모션 프리미티브 확장 ──
    for (int si = 0; si < N_STEER; ++si) {
      const double delta = steers[si];

      // 자전거 모델 — midpoint rule
      // kappa: 조향각에 의한 곡률 [1/m]
      // mid_yaw: 호의 중간 지점 헤딩 (Euler보다 정확한 적분)
      const double kappa   = std::tan(delta) / L;
      const double mid_yaw = cur.yaw + kappa * arc_len * 0.5;
      const double nx      = cur.x + arc_len * std::cos(mid_yaw);
      const double ny      = cur.y + arc_len * std::sin(mid_yaw);
      const double nyaw    = cur.yaw + kappa * arc_len;

      // 格자 좌표 변환
      int nc = static_cast<int>((nx - cm.origin_x) / cm.resolution);
      int nr = static_cast<int>((ny - cm.origin_y) / cm.resolution);

      // 格자 범위 체크
      if (nc < 0 || nc >= cm.cols || nr < 0 || nr >= cm.rows) continue;

      // 장애물 체크: costmap 비용이 obstacle_cost 이상인 셀은 통과 불가
      double cell_cost = get_costmap_cost(cm, nx, ny);
      if (cell_cost >= obs_cost) continue;

      // 비용 계산: 이동 거리 + costmap 패널티
      double ng = cur.g + arc_len + cell_cost * cost_w;

      // 방문 체크 + 갱신
      int nyi  = yaw_to_bin(nyaw);
      int nkey = state_key(nc, nr, nyi, cm.cols);

      auto it = best_g.find(nkey);
      if (it != best_g.end() && ng >= it->second - 1e-9) continue;
      best_g[nkey] = ng;

      // 새 노드 삽입
      double nh      = heuristic(nx, ny);
      int    new_idx = static_cast<int>(nodes.size());
      nodes.push_back({nx, ny, nyaw, ng, cur_idx});
      open.push({ng + nh, new_idx});
    }
  }

  // ============================================================
  // 9) 역추적 (backtracking)
  // ============================================================
  // goal에 도달했으면 goal_node_idx, 아니면 best_node_idx에서 역추적.
  // parent 인덱스를 따라 start(parent=-1)까지 거슬러 올라간다.
  // 수집 순서가 goal→start이므로 마지막에 reverse.
  int trace_idx = (goal_node_idx >= 0) ? goal_node_idx : best_node_idx;

  std::vector<Point2D> path;
  while (trace_idx >= 0) {
    const HNode & n = nodes[trace_idx];
    path.push_back({n.x, n.y});
    if (trace_idx == 0) break;
    trace_idx = n.parent;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace chaining_costmap_ver
