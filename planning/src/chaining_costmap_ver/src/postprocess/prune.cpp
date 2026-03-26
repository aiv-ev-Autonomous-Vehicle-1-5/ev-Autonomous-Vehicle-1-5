/**
 * @file prune.cpp
 * @brief PathPostprocessor — ① Prune 단계 구현
 *
 * Douglas-Peucker 유사 탐욕적 경로 단순화.
 * 직선 구간의 불필요한 중간점을 max_dev 허용 편차 기준으로 제거.
 *
 * [costmap-aware 확장]
 *   costmap이 주어지면 shortcut 직선(i→j)을 resolution 간격으로 샘플링하여,
 *   obstacle_cost 이상인 셀을 통과하는 shortcut은 거부한다.
 *   이를 통해 prune → resample 시 선형 보간이 obstacle을 관통하는 문제를 방지.
 *
 * [의존 관계]
 *   - path_postprocessor.hpp: PathPostprocessor 클래스 선언
 *   - geometry.hpp: dist() — 두 점 사이 거리
 */
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"

#include <cmath>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// ① Prune 단계 — Douglas-Peucker 유사 탐욕적 경로 단순화 (costmap-aware)
// ============================================================================
//
// [알고리즘 상세]
//   현재 앵커(anchor) 점 i에서 출발하여:
//   1) 가장 먼 점 j = (끝점)부터 역순으로 시도
//   2) i→j 직선을 긋고, 사이의 모든 중간점 k에 대해
//      수직 편차(perpendicular distance)를 계산
//   3) 모든 중간점의 편차 ≤ max_dev 이면 → shortcut 후보
//   4) costmap이 있으면 i→j 직선을 resolution 간격으로 걸으며
//      obstacle_cost 이상인 셀이 있으면 shortcut 거부
//   5) 편차 초과 점이 있으면 → j를 하나 줄여서 재시도
//   6) 최악의 경우 j = i+1 (바로 다음 점)이 채택됨
//
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev,
  const CostmapResult * costmap,
  double obstacle_cost)
{
  if (pts.size() <= 2) return pts;

  std::vector<Point2D> result;
  result.push_back(pts.front());

  size_t i = 0;
  while (i < pts.size() - 1) {
    size_t best_j = i + 1;

    for (size_t j = pts.size() - 1; j > i + 1; --j) {
      const Point2D & a = pts[i];
      const Point2D & b = pts[j];
      const double ab_len = dist(a, b);

      bool can_shortcut = true;

      if (ab_len < 1e-12) {
        can_shortcut = false;
      } else {
        const Point2D dir = {(b.x - a.x) / ab_len, (b.y - a.y) / ab_len};

        // 기존: 수직 편차 검사
        for (size_t k = i + 1; k < j; ++k) {
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;
          const double perp = std::abs(dx * dir.y - dy * dir.x);
          if (perp > max_dev) {
            can_shortcut = false;
            break;
          }
        }

        // costmap-aware: shortcut 직선 위에 obstacle 셀이 있는지 검사
        if (can_shortcut && costmap && costmap->valid) {
          const double step = costmap->resolution;
          const int num_samples = static_cast<int>(ab_len / step) + 1;
          for (int s = 0; s <= num_samples; ++s) {
            const double t = static_cast<double>(s) / num_samples;
            const double sx = a.x + t * (b.x - a.x);
            const double sy = a.y + t * (b.y - a.y);
            if (costmap->cost_at(sx, sy) >= obstacle_cost) {
              can_shortcut = false;
              break;
            }
          }
        }
      }

      if (can_shortcut) {
        best_j = j;
        break;
      }
    }

    result.push_back(pts[best_j]);
    i = best_j;
  }

  return result;
}

}  // namespace chaining_costmap_ver
