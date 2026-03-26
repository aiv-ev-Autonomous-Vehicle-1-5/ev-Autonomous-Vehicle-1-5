/**
 * @file smooth.cpp
 * @brief PathPostprocessor — ③ Smooth 단계 구현
 *
 * 이동 평균(Moving Average) 필터로 경로의 잔여 꺾임을 완화.
 * 시작점/끝점은 보존하고, 중간점만 윈도우 내 이웃의 산술 평균으로 대체.
 *
 * [costmap-aware 확장]
 *   costmap이 주어지면, 이동평균 결과가 obstacle_cost 이상인 셀에
 *   위치할 경우 원래 좌표를 유지하여 장애물 침범을 방지한다.
 *
 * [의존 관계]
 *   - path_postprocessor.hpp: PathPostprocessor 클래스 선언
 */
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"

#include <algorithm>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// ③ Smooth 단계 — 이동 평균(Moving Average) 필터 (costmap-aware)
// ============================================================================
//
// 각 중간점 i에 대해, 윈도우 [i - half, i + half] 범위의
// 이웃 점들의 좌표를 산술 평균하여 새 좌표로 대체한다.
// costmap이 주어지면, 새 좌표가 obstacle 셀이면 원래 좌표를 유지.
//
std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window,
  const CostmapResult * costmap,
  double obstacle_cost)
{
  if (pts.size() <= 2 || window <= 1) return pts;

  const int half = window / 2;
  const int n = static_cast<int>(pts.size());

  std::vector<Point2D> result(pts.size());
  result.front() = pts.front();
  result.back() = pts.back();

  for (int i = 1; i < n - 1; ++i) {
    double sx = 0.0, sy = 0.0;
    int count = 0;

    const int lo = std::max(0, i - half);
    const int hi = std::min(n - 1, i + half);

    for (int j = lo; j <= hi; ++j) {
      sx += pts[j].x;
      sy += pts[j].y;
      ++count;
    }

    const double new_x = sx / count;
    const double new_y = sy / count;

    // costmap-aware: 이동평균 결과가 obstacle 셀이면 원래 좌표 유지
    if (costmap && costmap->valid &&
        costmap->cost_at(new_x, new_y) >= obstacle_cost)
    {
      result[i] = pts[i];
    } else {
      result[i] = {new_x, new_y};
    }
  }

  return result;
}

}  // namespace chaining_costmap_ver
