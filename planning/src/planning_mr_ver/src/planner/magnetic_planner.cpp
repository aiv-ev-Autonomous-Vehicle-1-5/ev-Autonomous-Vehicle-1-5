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
 * @brief 전방 콘(forward cone) 내에서 가장 낮은 cost의 셀을 탐색
 *
 * APF Local Minima / 역주행 방지를 위해 탐색 콘 각도를 제한한다.
 * 기존 180° → forward_cone_deg (기본 120°, ±60°)
 *
 * 참고 문헌:
 *   - Borenstein & Koren (1991) VFH: sector 기반 탐색 영역 제한
 *   - 본 구현은 VFH의 sector 제한을 cos(half_angle) threshold로 단순화
 *
 * 탐색 과정:
 *   1. cos_half_cone = cos(forward_cone_deg / 2) 계산
 *   2. bounding box 내 셀 순회:
 *      a. 거리 검사: search_radius 이내 + 자기 자신 제외
 *      b. 전방 콘 검사: alignment > cos_half_cone (120°→cos60°=0.5)
 *      c. cost 비교 → alignment tie-break
 */
Point2D MagneticPlanner::find_best_forward_cell(
  const CostmapResult & costmap,
  const Point2D & current_pos,
  const Point2D & hdg,
  const PlanningParams & params,
  bool & found)
{
  found = false;
  double best_cost = std::numeric_limits<double>::max();
  double best_dot = -1.0;
  Point2D best_pos{0.0, 0.0};

  const double r = params.planner.search_radius;
  const double r_sq = r * r;
  int r_cells = static_cast<int>(std::ceil(r / costmap.resolution));

  // ── Forward Cone 임계값 계산 ──
  // forward_cone_deg=120° → half=60° → cos(60°)=0.5
  // alignment(= cos θ) > 0.5 인 셀만 후보 (±60° 이내)
  const double half_cone_rad = params.planner.forward_cone_deg * 0.5 * M_PI / 180.0;
  const double cos_half_cone = std::cos(half_cone_rad);

  int cur_row, cur_col;
  if (!world_to_grid(current_pos.x, current_pos.y,
      costmap.origin_x, costmap.origin_y,
      costmap.resolution, costmap.rows, costmap.cols,
      cur_row, cur_col)) {
    return best_pos;
  }

  int row_min = std::max(0, cur_row - r_cells);
  int row_max = std::min(costmap.rows - 1, cur_row + r_cells);
  int col_min = std::max(0, cur_col - r_cells);
  int col_max = std::min(costmap.cols - 1, cur_col + r_cells);

  for (int row = row_min; row <= row_max; ++row) {
    for (int col = col_min; col <= col_max; ++col) {
      Point2D cell_pos = grid_to_world(
        row, col, costmap.origin_x, costmap.origin_y, costmap.resolution);

      double dx = cell_pos.x - current_pos.x;
      double dy = cell_pos.y - current_pos.y;
      double d_sq = dx * dx + dy * dy;

      if (d_sq < 1e-12 || d_sq > r_sq) continue;

      double d = std::sqrt(d_sq);
      double dot_val = hdg.x * dx + hdg.y * dy;
      double alignment = dot_val / d;  // cos(heading과 이동방향의 각도차)

      // ── Forward Cone 체크 ──
      // 기존: alignment > 0 (180°)
      // 변경: alignment > cos_half_cone (120° → ±60° 이내만 허용)
      // 이로써 heading과 60° 이상 차이나는 방향으로는 이동 불가
      if (alignment <= cos_half_cone) continue;

      double cost = costmap.data[row * costmap.cols + col];

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

    // ── Heading 업데이트: Damping + Max Turn Rate Clamp ──
    //
    // 2단계로 역주행을 방지한다:
    //
    // [1단계] Heading Damping (soft limit)
    //   new_hdg = normalize(damping * old_hdg + (1-damping) * move_dir)
    //   이전 heading을 일부 유지하여 급격한 방향 전환을 완화
    //
    // [2단계] Max Turn Rate Clamp (hard limit)
    //   블렌딩 후에도 heading 변화가 max_steer_per_step_deg를 초과하면
    //   해당 각도로 강제 클램프 → 물리적 조향 한계와 유사한 역할
    //
    // 참고: forward cone(120°)이 후보 셀을 제한하고,
    //       damping + clamp가 heading 갱신을 제한하므로
    //       3중 안전장치로 역주행을 차단한다.

    Point2D move_dir = normalize(next_pos - current_pos);
    if (norm(move_dir) < 1e-6) break;

    // [1단계] Heading Damping
    const double damp = params.planner.heading_damping;
    Point2D blended{
      damp * hdg.x + (1.0 - damp) * move_dir.x,
      damp * hdg.y + (1.0 - damp) * move_dir.y
    };
    Point2D new_hdg = normalize(blended);

    // [2단계] Max Turn Rate Clamp
    // heading 변화각 = atan2(cross, dot) 으로 계산
    const double max_steer_rad = params.planner.max_steer_per_step_deg * M_PI / 180.0;
    double delta_angle = std::atan2(
      cross2(hdg, new_hdg),   // sin(θ): 회전 방향 (+CCW, -CW)
      dot2(hdg, new_hdg));    // cos(θ): 각도 크기

    if (std::fabs(delta_angle) > max_steer_rad) {
      // 최대 회전각으로 클램프: hdg를 ±max_steer_rad만큼만 회전
      double clamped = (delta_angle > 0.0) ? max_steer_rad : -max_steer_rad;
      double cos_a = std::cos(clamped);
      double sin_a = std::sin(clamped);
      new_hdg = {hdg.x * cos_a - hdg.y * sin_a,
                 hdg.x * sin_a + hdg.y * cos_a};
    }

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
