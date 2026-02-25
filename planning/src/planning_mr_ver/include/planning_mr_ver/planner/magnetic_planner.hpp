/**
 * @file magnetic_planner.hpp
 * @brief Magnetic Resistance Planner — 핵심 알고리즘 2
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: costmap 위에서 Greedy 전진 탐색으로 경로(raw_path)를 생성
 *
 * 알고리즘 개요 (N극 입장에서 S극 costmap 탐색):
 *   1. ego(0,0)에서 출발, 초기 heading = (+x, 0) = 전방
 *   2. 현재 위치에서 heading 기준 전방 180° 범위 탐색
 *   3. search_radius 내에서 cost가 가장 낮은 셀 선택
 *   4. 해당 셀로 이동, heading 업데이트 (직전→현재 방향)
 *   5. 2~4 반복 (max_steps 또는 그리드 경계까지)
 *   6. 이동한 점들의 궤적이 raw_path가 됨
 *
 * 특수 처리:
 *   - cost 동률 시 heading 방향에 더 정렬된 셀 우선 선택
 *     → oscillation(좌우 흔들림) 방지
 *   - 그리드 경계 근처(edge ±1 cell) 도달 시 조기 종료
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
#define PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_

#include "planning_mr_ver/common/types.hpp"
#include "planning_mr_ver/common/params.hpp"
#include <vector>

namespace planning_mr_ver
{

class MagneticPlanner
{
public:
  /**
   * @brief costmap 위에서 Greedy 전진 탐색으로 경로 생성
   *
   * @param costmap CostmapGenerator가 생성한 costmap
   * @param params  탐색 파라미터 (search_radius, max_steps, heading 초기값)
   * @return 탐색된 raw 경로 점 목록 (후처리 전)
   */
  std::vector<Point2D> plan(
    const CostmapResult & costmap,
    const PlanningParams & params);

private:
  /**
   * @brief 월드 좌표 → 그리드 인덱스 변환
   *
   * @param wx, wy         월드 좌표 [m]
   * @param origin_x, origin_y  그리드 원점의 월드 좌표
   * @param resolution     셀 크기 [m/cell]
   * @param rows, cols     그리드 크기
   * @param row, col       [출력] 변환된 행/열 인덱스
   * @return true: 그리드 범위 내, false: 범위 밖
   */
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution,
    int rows, int cols,
    int & row, int & col);

  /**
   * @brief 그리드 인덱스 → 월드 좌표 변환 (셀 중심점)
   *
   * +0.5 보정: 셀의 좌하단이 아닌 중심 좌표를 반환
   *
   * @return 해당 셀 중심의 월드 좌표
   */
  static Point2D grid_to_world(
    int row, int col,
    double origin_x, double origin_y,
    double resolution);

  /**
   * @brief 전방 180° 내에서 가장 낮은 cost의 셀을 찾는다
   *
   * 탐색 조건:
   *   1. search_radius 이내
   *   2. heading과의 내적(dot) > 0 (전방 180°)
   *   3. 자기 자신(d≈0) 제외
   *
   * 선택 기준:
   *   - 1순위: cost가 가장 낮은 셀
   *   - 2순위 (동률): heading과의 정렬도(cos angle)가 높은 셀
   *     → 직진 방향 선호, oscillation 방지
   *
   * @param costmap     현재 costmap
   * @param current_pos 현재 위치 (월드 좌표)
   * @param heading     현재 heading 방향 단위벡터
   * @param params      탐색 파라미터
   * @param found       [출력] 유효한 셀을 찾았는지 여부
   * @return 찾은 셀의 월드 좌표 (found=false면 무의미)
   */
  static Point2D find_best_forward_cell(
    const CostmapResult & costmap,
    const Point2D & current_pos,
    const Point2D & heading,
    const PlanningParams & params,
    bool & found);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
