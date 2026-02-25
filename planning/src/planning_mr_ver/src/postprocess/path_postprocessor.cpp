#include "planning_mr_ver/postprocess/path_postprocessor.hpp"
#include "planning_mr_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace planning_mr_ver
{

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

std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window)
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
    result[i] = {sx / count, sy / count};
  }

  return result;
}

PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds)
{
  PostprocessResult result;

  if (raw_path.size() < 2) return result;

  auto pruned = prune(raw_path, prune_max_dev);
  auto smoothed = smooth(pruned, smooth_window);
  result.path = resample_polyline(smoothed, resample_ds);

  if (result.path.size() < 2) return result;

  auto tangents = polyline_tangents(result.path);
  result.yaw.resize(result.path.size());
  for (size_t i = 0; i < tangents.size(); ++i) {
    result.yaw[i] = heading(tangents[i]);
  }

  result.valid = true;
  return result;
}

}  // namespace planning_mr_ver
