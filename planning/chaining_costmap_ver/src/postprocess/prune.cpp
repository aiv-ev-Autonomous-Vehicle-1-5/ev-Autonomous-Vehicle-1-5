/**
 * @file prune.cpp
 * @brief PathPostprocessor — ① Prune 단계 구현
 *
 * Douglas-Peucker 유사 탐욕적 경로 단순화.
 * 직선 구간의 불필요한 중간점을 max_dev 허용 편차 기준으로 제거.
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
// ① Prune 단계 — Douglas-Peucker 유사 탐욕적 경로 단순화
// ============================================================================
//
// [알고리즘 상세]
//   현재 앵커(anchor) 점 i에서 출발하여:
//   1) 가장 먼 점 j = (끝점)부터 역순으로 시도
//   2) i→j 직선을 긋고, 사이의 모든 중간점 k에 대해
//      수직 편차(perpendicular distance)를 계산
//   3) 모든 중간점의 편차 ≤ max_dev 이면 → shortcut 성공
//   4) 편차 초과 점이 있으면 → j를 하나 줄여서 재시도
//   5) 최악의 경우 j = i+1 (바로 다음 점)이 채택됨
//
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev)
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

        for (size_t k = i + 1; k < j; ++k) {
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;
          const double perp = std::abs(dx * dir.y - dy * dir.x);
          if (perp > max_dev) {
            can_shortcut = false;
            break;
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
