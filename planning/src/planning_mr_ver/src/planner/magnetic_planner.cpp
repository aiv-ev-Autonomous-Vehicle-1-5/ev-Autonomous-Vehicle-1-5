/**
 * @file magnetic_planner.cpp
 * @brief Magnetic Resistance Planner — 구현부
 *
 * costmap 위에서 ego(0,0) → heading 방향으로 Greedy 전진 탐색하여
 * cost가 가장 낮은 경로(raw_path)를 생성한다.
 *
 * 물리적 비유:
 *   N극(차량)이 S극(장애물) costmap 위에서 반발력이 가장 약한 곳으로 이동
 */
#include "planning_mr_ver/planner/magnetic_planner.hpp"
#include "planning_mr_ver/common/geometry.hpp"

#include <cmath>
#include <limits>

namespace planning_mr_ver
{

/**
 * @brief 월드 좌표 → 그리드 인덱스 변환
 *
 * 변환 공식:
 *   col = (wx - origin_x) / resolution   (X축 → 열)
 *   row = (wy - origin_y) / resolution   (Y축 → 행)
 *
 * @return true: 변환된 인덱스가 그리드 범위 내, false: 범위 밖
 */
bool MagneticPlanner::world_to_grid(
  double wx, double wy,
  double origin_x, double origin_y,
  double resolution,
  int rows, int cols,
  int & row, int & col)
{
  col = static_cast<int>((wx - origin_x) / resolution);
  row = static_cast<int>((wy - origin_y) / resolution);
  return (row >= 0 && row < rows && col >= 0 && col < cols);
}

/**
 * @brief 그리드 인덱스 → 월드 좌표 변환
 *
 * +0.5 보정으로 셀의 "중심" 좌표를 반환한다.
 * (셀 좌하단이 아닌 중심을 사용해야 경로가 자연스러움)
 */
Point2D MagneticPlanner::grid_to_world(
  int row, int col,
  double origin_x, double origin_y,
  double resolution)
{
  return {
    origin_x + (col + 0.5) * resolution,  // 셀 중심 X
    origin_y + (row + 0.5) * resolution   // 셀 중심 Y
  };
}

/**
 * @brief 전방 180° 내에서 가장 낮은 cost의 셀을 탐색
 *
 * 탐색 과정:
 *   1. 현재 위치를 그리드 좌표로 변환
 *   2. search_radius에 해당하는 셀 범위(bounding box) 계산
 *   3. bounding box 내 모든 셀을 순회하며:
 *      a. 거리 검사: search_radius 이내 + 자기 자신 제외
 *      b. 전방 검사: dot(heading, direction) > 0 (전방 180°)
 *      c. cost 비교: 최소 cost 업데이트
 *      d. 동률 처리: alignment(heading 정렬도) 높은 셀 우선
 */
Point2D MagneticPlanner::find_best_forward_cell(
  const CostmapResult & costmap,
  const Point2D & current_pos,
  const Point2D & hdg,
  const PlanningParams & params,
  bool & found)
{
  found = false;
  double best_cost = std::numeric_limits<double>::max();  // 현재까지 최소 cost
  double best_dot = -1.0;  // tie-breaker: heading 방향과의 정렬도 (cos angle)
  Point2D best_pos{0.0, 0.0};

  const double r = params.planner.search_radius;
  const double r_sq = r * r;  // 거리 비교용 제곱값 (sqrt 절약)
  int r_cells = static_cast<int>(std::ceil(r / costmap.resolution));

  // 현재 위치의 그리드 인덱스
  int cur_row, cur_col;
  if (!world_to_grid(current_pos.x, current_pos.y,
      costmap.origin_x, costmap.origin_y,
      costmap.resolution, costmap.rows, costmap.cols,
      cur_row, cur_col)) {
    return best_pos;  // 그리드 밖이면 탐색 불가
  }

  // 탐색 bounding box (search_radius에 해당하는 셀 범위)
  int row_min = std::max(0, cur_row - r_cells);
  int row_max = std::min(costmap.rows - 1, cur_row + r_cells);
  int col_min = std::max(0, cur_col - r_cells);
  int col_max = std::min(costmap.cols - 1, cur_col + r_cells);

  for (int row = row_min; row <= row_max; ++row) {
    for (int col = col_min; col <= col_max; ++col) {
      // 이 셀의 월드 좌표
      Point2D cell_pos = grid_to_world(
        row, col, costmap.origin_x, costmap.origin_y, costmap.resolution);

      // 현재 위치 → 후보 셀까지의 변위 벡터
      double dx = cell_pos.x - current_pos.x;
      double dy = cell_pos.y - current_pos.y;
      double d_sq = dx * dx + dy * dy;

      // 자기 자신(거리≈0) 또는 search_radius 밖 → 제외
      if (d_sq < 1e-12 || d_sq > r_sq) continue;

      // ── 전방 180° 체크 ──
      // heading과 변위 벡터의 내적 > 0 이면 전방(같은 반구)
      // dot ≤ 0 이면 후방 → 제외 (뒤로 가지 않음)
      double dot_val = hdg.x * dx + hdg.y * dy;
      if (dot_val <= 0.0) continue;

      // 이 셀의 cost 값
      double cost = costmap.data[row * costmap.cols + col];

      // heading 방향 정렬도 계산: cos(heading과 이동 방향의 각도 차이)
      // alignment = 1.0이면 완전히 heading과 같은 방향 (직진)
      // alignment = 0.0이면 heading과 90° 차이 (최대 꺾임)
      double d = std::sqrt(d_sq);
      double alignment = dot_val / d;  // cos(angle) = dot / (||hdg|| * ||dir||), ||hdg||=1

      // ── 최적 셀 선택 ──
      // 1순위: cost가 더 낮은 셀
      // 2순위: cost가 같으면 alignment가 높은 셀 (직진 방향 선호)
      //        → 좌우 대칭인 cost에서 oscillation(좌우 흔들림) 방지
      if (cost < best_cost || (cost == best_cost && alignment > best_dot)) {
        best_cost = cost;
        best_dot = alignment;
        best_pos = cell_pos;
        found = true;
      }
    }
  }

  return best_pos;
}

/**
 * @brief costmap 위에서 Greedy 전진 탐색으로 raw_path 생성
 *
 * 전체 흐름:
 *   1. ego(0,0)을 시작점으로, 초기 heading을 params에서 읽음
 *   2. 매 스텝마다 find_best_forward_cell()로 다음 셀 선택
 *   3. heading을 (직전→현재) 방향으로 업데이트
 *   4. 종료 조건: max_steps 도달, 셀 찾기 실패, 그리드 경계 도달
 *   5. 이동한 점들을 raw_path로 반환
 */
std::vector<Point2D> MagneticPlanner::plan(
  const CostmapResult & costmap,
  const PlanningParams & params)
{
  std::vector<Point2D> raw_path;

  if (!costmap.valid) return raw_path;  // costmap이 유효하지 않으면 빈 경로

  // 시작점: ego(0,0), 초기 heading: params에서 설정 (기본: +x = 전방)
  Point2D current_pos{0.0, 0.0};
  Point2D hdg = normalize(
    Point2D{params.planner.heading_init_x, params.planner.heading_init_y});

  raw_path.push_back(current_pos);  // 시작점 추가

  // ── Greedy 전진 탐색 루프 ──
  for (int step = 0; step < params.planner.max_steps; ++step) {
    bool found = false;
    Point2D next_pos = find_best_forward_cell(
      costmap, current_pos, hdg, params, found);

    if (!found) break;  // 전방에 갈 수 있는 셀이 없음 → 종료

    // heading 업데이트: (직전 위치 → 새 위치) 방향벡터로 갱신
    Point2D new_hdg = normalize(next_pos - current_pos);
    if (norm(new_hdg) < 1e-6) break;  // 이동 거리가 거의 0 → 종료

    hdg = new_hdg;
    current_pos = next_pos;
    raw_path.push_back(current_pos);

    // ── 그리드 경계 도달 검사 ──
    // 가장자리 2셀 이내에 도달하면 조기 종료
    // (경계 밖 탐색 시 find_best_forward_cell의 후보가 부족해질 수 있음)
    int r, c;
    if (!world_to_grid(current_pos.x, current_pos.y,
        costmap.origin_x, costmap.origin_y,
        costmap.resolution, costmap.rows, costmap.cols,
        r, c)) {
      break;  // 그리드 완전 밖
    }
    if (r <= 1 || r >= costmap.rows - 2 ||
        c <= 1 || c >= costmap.cols - 2) {
      break;  // 경계 근처 → 종료
    }
  }

  return raw_path;
}

}  // namespace planning_mr_ver
