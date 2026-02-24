/**
 * @file astar_planner.hpp
 * @brief 8-방향 A* 그리드 플래너 헤더
 *
 * ## 역할
 * - 주행 가능 마스크(DrivableMaskScanline) 위에서 A* 알고리즘으로
 *   start → goal 경로를 탐색한다.
 * - ASTAR 모드에서 센터라인 대신 장애물을 회피하는 경로를 생성한다.
 *
 * ## A* 알고리즘 개요
 *  f(n) = g(n) + h(n)
 *  - g(n): start에서 n까지의 실제 비용
 *          = 이동 거리 + cell_cost_weight * 셀 비용
 *  - h(n): n에서 goal까지의 휴리스틱 (Octile 거리)
 *
 * ## 8-방향 이동
 *  직선 4방향: 비용 = 1.0 * resolution
 *  대각선 4방향: 비용 = √2 * resolution ≈ 1.414 * resolution
 *
 * ## Octile 거리 휴리스틱
 *  h = (max(dr,dc) + (√2-1) * min(dr,dc)) * resolution
 *  - 8-방향 이동에 admissible한 휴리스틱 (절대 실제 비용 초과 안 함)
 *  - Manhattan보다 정확하고 Euclidean보다 계산이 빠름
 *
 * ## 셀 비용 처리
 *  - free(0): 추가 비용 없음
 *  - inflated(1~99): cell_cost_weight * cell_value * resolution 추가
 *  - occupied(>=100): 통과 불가 (hard obstacle)
 *
 * ## goal snapping (목표 스냅)
 *  - 목표 셀이 occupied이면 5셀 반경 내에서 가장 가까운 free 셀로 자동 이동
 *  - 근처 free 셀이 없으면 탐색 실패
 */

#ifndef TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_
#define TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_

#include "track_planning/common/types.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/**
 * @class AstarPlanner
 * @brief 8-방향 A* 그리드 기반 경로 플래너
 *
 * plan()을 호출하면 start에서 goal까지 최소 비용 경로를 찾아 반환한다.
 */
class AstarPlanner
{
public:
  /**
   * @struct Result
   * @brief plan()의 출력 구조체
   *
   * @param path        경로 포인트 목록 (월드 좌표, start → goal 순서)
   * @param success     경로 탐색 성공 여부
   * @param iterations  A* 반복 횟수 (성능 분석용)
   */
  struct Result
  {
    std::vector<Point2D> path;   // 세계 좌표 경로 (월드 좌표)
    bool success = false;
    int iterations = 0;
  };

  /**
   * @brief A* 경로 탐색
   *
   * @param grid              row-major 그리드 (0=free, 1~99=비용, >=100=occupied)
   * @param width             그리드 열 수
   * @param height            그리드 행 수
   * @param resolution        셀 크기(m/cell)
   * @param origin_x          그리드 원점 X(m)
   * @param origin_y          그리드 원점 Y(m)
   * @param start             시작 포인트 (월드 좌표, 보통 ego 위치)
   * @param goal              목표 포인트 (월드 좌표, GoalSelector가 선택)
   * @param max_iterations    최대 반복 횟수 (0=무제한)
   *                          예산 초과 시 success=false 반환
   * @param cell_cost_weight  셀 비용 가중치
   *                          g += cell_cost_weight * cell_value * resolution
   *                          높을수록 팽창 영역을 더 강하게 회피
   * @return                  탐색 결과 (success=false이면 경로 없음)
   */
  Result plan(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & start,
    const Point2D & goal,
    int max_iterations,
    double cell_cost_weight);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_
