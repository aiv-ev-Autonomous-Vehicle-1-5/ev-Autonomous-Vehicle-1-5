/**
 * @file magnetic_planner.hpp
 * @brief Magnetic Resistance Planner — costmap 위 Greedy 전진 탐색
 *
 * ──────────────────────────────────────────────────────────────────
 * [MagneticPlanner 알고리즘 개요]
 *
 * Costmap(비용 지도) 위에서 탐욕적(greedy) 전진 탐색을 수행하는 플래너.
 * "자석이 최소 저항 경로를 따라가듯" 매 스텝마다 비용이 가장 낮은 셀을
 * 선택하여 경로를 생성한다.
 *
 * ── 핵심 동작 원리 ──
 *
 *   1. 시작점: ego 차량 위치 (0, 0), 초기 heading 방향은 파라미터로 지정
 *   2. 매 스텝:
 *      a) 현재 위치에서 search_radius 반경 내의 "전방 원뿔(forward cone)" 안에 있는
 *         모든 그리드 셀을 스캔
 *      b) 가장 낮은 cost를 가진 셀을 선택 (동일 cost면 heading 정렬도가 높은 것 우선)
 *      c) heading damping: 이전 heading과 새 이동 방향을 가중 혼합하여 부드러운 회전 생성
 *      d) max steer clamp: 한 스텝당 최대 회전각을 제한 (차량 조향 한계 반영)
 *   3. 종료 조건:
 *      - 그리드 경계에 도달
 *      - 전방 원뿔 내에 유효한 셀이 없음
 *      - max_steps 도달
 *
 * ── 좌표계 규칙 ──
 *
 *   [월드 좌표계 (meters)]
 *   - x: 차량 전방(+), 후방(-)
 *   - y: 좌측(+), 우측(-)
 *   - 원점(origin): costmap의 좌측 하단 (origin_x, origin_y)
 *
 *   [그리드 좌표계 (인덱스)]
 *   - row: y축 방향 (행 번호). row=0이 origin_y에 해당 (하단)
 *   - col: x축 방향 (열 번호). col=0이 origin_x에 해당 (좌측)
 *   - data[row * cols + col]: row-major 1D 배열 접근
 *
 *   즉, 그리드 원점은 좌측 하단이며, row가 증가하면 y(위쪽)으로,
 *   col이 증가하면 x(오른쪽/전방)으로 이동한다.
 *
 * ── 파이프라인 위치 ──
 *
 *   DirectionChainer → CostmapGenerator → [MagneticPlanner] → PathPostprocessor
 *   costmap에서 greedy 탐색으로 원시 경로(raw_path)를 생성하는 단계.
 * ──────────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
#define CHAINING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_

#include "chaining_mr_ver/common/types.hpp"
#include "chaining_mr_ver/common/params.hpp"
#include <vector>

namespace chaining_mr_ver
{

/**
 * @class MagneticPlanner
 * @brief Costmap 위 Greedy 전진 탐색 플래너
 *
 * [알고리즘 특성]
 * - 탐욕적(greedy): 매 스텝 국소적으로 최적인 셀만 선택 (A*처럼 전역 탐색 안 함)
 * - 전진 전용(forward-only): 전방 원뿔 안에서만 다음 셀을 탐색 (후진 불가)
 * - 경량 & 빠름: 실시간 제어 루프(~10Hz)에 적합한 단순한 구조
 *
 * [단점]
 * - 국소 최적에 빠질 수 있음 (좁은 통로에서 막힐 가능성)
 * - 대신 costmap이 잘 설계되면 실용적으로 충분히 좋은 경로 생성
 */
class MagneticPlanner
{
public:
  /**
   * @brief 메인 경로 생성 함수
   *
   * ego 위치 (0,0)에서 출발하여 costmap 위에서 greedy 전진 탐색을 수행.
   * heading damping + max steer clamp를 적용한 부드러운 경로를 반환한다.
   *
   * @param costmap   CostmapGenerator가 생성한 2D 비용 지도
   * @param params    플래너 파라미터 (search_radius, max_steps, damping 등)
   * @return          원시 경로 좌표열 (Point2D 벡터). 월드 좌표계 [m] 단위.
   *                  경로가 생성 불가하면 빈 벡터 또는 시작점만 포함.
   */
  std::vector<Point2D> plan(
    const CostmapResult & costmap,
    const PlanningParams & params);

private:
  /**
   * @brief 월드 좌표 → 그리드 인덱스 변환
   *
   * 연속적인 미터 단위 좌표(wx, wy)를 이산적인 그리드 셀 인덱스(row, col)로 변환.
   *
   * 변환 공식:
   *   col = (int)((wx - origin_x) / resolution)   ← x축 → 열 방향
   *   row = (int)((wy - origin_y) / resolution)   ← y축 → 행 방향
   *
   * @param wx, wy         월드 좌표 [m]
   * @param origin_x, origin_y  그리드 원점의 월드 좌표 [m] (좌측 하단)
   * @param resolution     그리드 셀 한 변의 크기 [m/cell]
   * @param rows, cols     그리드 행/열 수
   * @param row, col       [출력] 변환된 그리드 인덱스
   * @return               true: 그리드 범위 안에 있음, false: 범위 밖
   */
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution,
    int rows, int cols,
    int & row, int & col);

  /**
   * @brief 그리드 인덱스 → 월드 좌표 변환 (셀 중심점 반환)
   *
   * 셀의 중심 좌표를 반환하기 위해 (col + 0.5), (row + 0.5)를 사용.
   * 셀 (0,0)의 중심은 origin + 0.5*resolution 위치가 된다.
   *
   * 변환 공식:
   *   wx = origin_x + (col + 0.5) * resolution
   *   wy = origin_y + (row + 0.5) * resolution
   *
   * @param row, col       그리드 인덱스
   * @param origin_x, origin_y  그리드 원점의 월드 좌표 [m]
   * @param resolution     그리드 셀 크기 [m/cell]
   * @return               셀 중심의 월드 좌표 (Point2D)
   */
  static Point2D grid_to_world(
    int row, int col,
    double origin_x, double origin_y,
    double resolution);

  /**
   * @brief 전방 원뿔 내 최소 비용 셀 탐색 (greedy 선택의 핵심)
   *
   * 현재 위치에서 search_radius 반경 내의 전방 원뿔(forward cone)에 속하는
   * 모든 셀을 순회하며, 가장 비용이 낮은 셀을 찾는다.
   *
   * [셀 선택 기준 (우선순위)]
   *   1차: cost가 가장 낮은 셀 (장애물에서 먼 셀)
   *   2차: cost가 동일하면, heading과의 정렬도(alignment)가 높은 셀
   *        → alignment = dot(heading, direction_to_cell) / distance
   *        → 1.0에 가까울수록 heading과 일직선, 즉 직진 방향에 가까움
   *
   * [전방 원뿔 판정]
   *   heading 벡터와 셀까지의 방향벡터 사이의 cos(각도)가
   *   cos(forward_cone_deg / 2) 보다 클 때만 전방 원뿔 안에 있다고 판정.
   *   예: forward_cone_deg=60° → cos(30°)=0.866 이상이면 전방
   *
   * @param costmap      2D 비용 지도
   * @param current_pos  현재 위치 (월드 좌표)
   * @param heading      현재 heading 단위 벡터 (진행 방향)
   * @param params       탐색 파라미터
   * @param found        [출력] true: 유효한 셀을 찾음, false: 못 찾음
   * @return             최적 셀의 월드 좌표 (found==false이면 의미 없음)
   */
  static Point2D find_best_forward_cell(
    const CostmapResult & costmap,
    const Point2D & current_pos,
    const Point2D & heading,
    const PlanningParams & params,
    bool & found);
};

}  // namespace chaining_mr_ver

#endif  // CHAINING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
