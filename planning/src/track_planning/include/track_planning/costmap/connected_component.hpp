/**
 * @file connected_component.hpp
 * @brief BFS 기반 연결 성분 필터 — ego(자차)에서 도달 불가능한 free 셀을 occupied로 변환
 *
 * ## 역할
 * - 주행 가능 마스크(DrivableMaskScanline) 빌드 후에 적용된다.
 * - 코리도가 여러 조각으로 나뉘거나, 폴리라인 보간 오류로 고립된 free 영역이
 *   생겼을 때 이를 제거한다.
 * - ego 위치에서 BFS로 도달 가능한 free 셀만 유지하고,
 *   나머지 고립된 free 셀은 100(occupied)으로 차단한다.
 *
 * ## BFS 연결 성분 알고리즘
 *  1. ego 위치 → 그리드 셀 변환
 *  2. ego 셀이 occupied이면 주변 5×5 내에서 가장 가까운 free 셀을 시드로 선택
 *  3. BFS로 시드에서 4-방향 인접 free 셀을 탐색 (visited 표시)
 *  4. 탐색 후 free(0)이지만 visited가 false인 셀 → 100(occupied)으로 변환
 *
 * ## 4-방향 이웃 (4-neighbor)
 *  상(↑), 하(↓), 좌(←), 우(→) — 대각선 불포함
 *  대각선을 포함하면 코너를 통해 의도치 않게 연결될 수 있음
 */

#ifndef TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_
#define TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_

#include "track_planning/common/types.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/**
 * @class ConnectedComponent
 * @brief BFS 기반 연결 성분 필터
 *
 * ego 위치에서 도달 불가능한 free 셀을 occupied(100)로 변환하여
 * 플래너가 고립된 영역으로 경로를 계획하지 않도록 한다.
 */
class ConnectedComponent
{
public:
  /**
   * @brief 그리드를 in-place로 필터링 — ego에서 도달 불가 free 셀 → occupied
   *
   * ## 동작 상세
   *  1. ego_pos를 그리드 셀로 변환 (floor 기반)
   *  2. ego 셀이 occupied이면 5셀 반경 내에서 가장 가까운 free 셀 탐색
   *     - 주차/정지 상황에서 ego가 팽창 영역(99)에 있을 수 있음
   *     - 근처 free 셀이 없으면 아무 작업도 하지 않고 반환
   *  3. BFS 탐색:
   *     - std::queue<int>에 시드 셀 인덱스 추가
   *     - 큐에서 꺼낸 셀의 4-방향 이웃 중 free(0)이고 미방문인 셀을 추가
   *     - visited 배열로 재방문 방지
   *  4. 탐색 완료 후 grid[i]==0 && !visited[i] 인 셀을 100으로 변환
   *
   * @param grid       row-major 그리드 버퍼 (in-place 수정)
   *                   0=free, 100=occupied
   * @param width      그리드 열(column) 수
   * @param height     그리드 행(row) 수
   * @param resolution 셀 크기(m/cell)
   * @param origin_x   그리드 좌하단 월드 X 좌표(m)
   * @param origin_y   그리드 좌하단 월드 Y 좌표(m)
   * @param ego_pos    자차 위치 (월드 좌표)
   */
  static void filter_ego_component(
    std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_
