/**
 * @file costmap_types.hpp
 * @brief 코스트맵 결과 타입 — CostmapGenerator → AStarPlanner 인터페이스
 *
 * CostmapGenerator가 생성한 2D 그리드 비용 지도 결과를 담는 구조체.
 * AStarPlanner가 이 코스트맵 위에서 8방향 A* 탐색을 수행한다.
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES__COSTMAP_TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES__COSTMAP_TYPES_HPP_

#include <vector>

namespace chaining_costmap_ver
{

/**
 * @brief CostmapGenerator의 출력 결과를 담는 구조체
 *
 * 좌/우 경계 체인(ChainedPoint[])으로부터 2D 그리드 비용 지도를 생성한 결과.
 *
 * [비용 지도의 원리 — 가우시안 비용장 모델]
 * - 각 경계점(bbox/차선)에서 가우시안 비용을 방사하여, 가까울수록 비용이 높다.
 * - bbox는 flat zone(반지름 내 최대 비용), 차선은 거리 기반 감쇠만 적용.
 * - A*는 비용이 낮은 셀을 따라가므로 경계 사이 중앙으로 경로가 유도된다.
 *
 * [데이터 레이아웃]
 * data[]는 1차원 배열, rows x cols 크기의 2D 그리드 (row-major).
 * grid[row][col] = data[row * cols + col]
 *
 * [좌표 변환: 그리드 인덱스 ↔ 실제 좌표]
 *   world_x = origin_x + col * resolution
 *   world_y = origin_y + row * resolution
 */
struct CostmapResult
{
  std::vector<double> data;   ///< row-major flat grid: data[row * cols + col]
  int rows = 0;               ///< 그리드 행 수 — y축 방향 셀 개수
  int cols = 0;               ///< 그리드 열 수 — x축 방향 셀 개수
  double resolution = 0.05;   ///< 셀 하나의 실제 크기 [m/cell]
  double origin_x = -5.0;     ///< 그리드 좌하단의 x 좌표 [m] (base_link 기준)
  double origin_y = -5.0;     ///< 그리드 좌하단의 y 좌표 [m] (base_link 기준)
  bool valid = false;          ///< true: 코스트맵 정상 생성됨

  /**
   * @brief 실제 좌표(x,y)에 해당하는 costmap 셀 비용을 반환
   * @return 셀 비용. 범위 밖이면 0.0 (free space 취급)
   */
  double cost_at(double x, double y) const
  {
    int c = static_cast<int>((x - origin_x) / resolution);
    int r = static_cast<int>((y - origin_y) / resolution);
    if (r < 0 || r >= rows || c < 0 || c >= cols) return 0.0;
    return data[r * cols + c];
  }
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES__COSTMAP_TYPES_HPP_
