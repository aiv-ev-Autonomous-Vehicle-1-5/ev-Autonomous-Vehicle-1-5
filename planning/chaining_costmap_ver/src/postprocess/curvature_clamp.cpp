/**
 * @file curvature_clamp.cpp
 * @brief PathPostprocessor — ④ Curvature Clamp 단계 구현
 *
 * Menger 곡률이 kappa_max를 초과하는 지점의 중간점을
 * P0-P2 중점 방향으로 밀어 곡률을 낮춘다.
 * 차량의 최소 회전 반경(R_min)을 보장하기 위한 후처리.
 *
 * [의존 관계]
 *   - path_postprocessor.hpp: PathPostprocessor 클래스 선언
 */
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// ④ Curvature Clamp — 최대 곡률 제한
// ============================================================================
//
// [Menger 곡률]
//   세 점 (P_{i-1}, P_i, P_{i+1})이 이루는 삼각형의 외접원 반경 R에서:
//     kappa = 1/R = 4·Area / (|a|·|b|·|c|)
//
// [곡률 감소 방법]
//   kappa > kappa_max인 triplet에서, 중간점 P_i를
//   P0-P2 중점 방향으로 이동시킨다. 반복(max_iter회)으로 수렴.
//
std::vector<Point2D> PathPostprocessor::curvature_clamp(
  const std::vector<Point2D> & pts, double kappa_max, int max_iter)
{
  if (pts.size() < 3 || kappa_max <= 0.0) return pts;

  std::vector<Point2D> result = pts;

  for (int iter = 0; iter < max_iter; ++iter) {
    int violations = 0;

    for (size_t i = 1; i + 1 < result.size(); ++i) {
      const auto & p0 = result[i - 1];
      const auto & p1 = result[i];
      const auto & p2 = result[i + 1];

      double ax = p1.x - p0.x, ay = p1.y - p0.y;
      double bx = p2.x - p0.x, by = p2.y - p0.y;

      double cross = ax * by - ay * bx;
      double area2 = std::abs(cross);
      if (area2 < 1e-12) continue;

      double la = std::sqrt(ax * ax + ay * ay);
      double lb = std::sqrt(bx * bx + by * by);
      double cx_v = p2.x - p1.x, cy_v = p2.y - p1.y;
      double lc = std::sqrt(cx_v * cx_v + cy_v * cy_v);
      if (la < 1e-12 || lb < 1e-12 || lc < 1e-12) continue;

      double kappa = 2.0 * area2 / (la * lb * lc);
      // 5% 마진을 두어 safety_checker 경계에서 FAIL 방지
      if (kappa <= kappa_max * 0.95) continue;

      ++violations;

      // P1을 P0-P2 중점 방향으로 이동하여 곡률을 낮춤
      double mx = (p0.x + p2.x) * 0.5;
      double my = (p0.y + p2.y) * 0.5;

      // 이동 비율: 초과량에 비례, 한 번에 최대 70%
      double ratio = 1.0 - 0.95 * (kappa_max / kappa);
      ratio = std::min(ratio, 0.7);

      result[i].x = p1.x + ratio * (mx - p1.x);
      result[i].y = p1.y + ratio * (my - p1.y);
    }

    if (violations == 0) break;  // 모든 곡률이 한계 이내 → 수렴 완료
  }

  return result;
}

}  // namespace chaining_costmap_ver
