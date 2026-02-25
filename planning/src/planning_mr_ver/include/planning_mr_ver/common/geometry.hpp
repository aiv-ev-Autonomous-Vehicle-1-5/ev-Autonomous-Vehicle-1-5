#ifndef PLANNING_MR_VER__COMMON__GEOMETRY_HPP_
#define PLANNING_MR_VER__COMMON__GEOMETRY_HPP_

#include "planning_mr_ver/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace planning_mr_ver
{

inline double dot2(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
}

inline double cross2(const Point2D & a, const Point2D & b)
{
  return a.x * b.y - a.y * b.x;
}

inline double norm(const Point2D & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

inline double dist(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double dist_sq(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline Point2D operator+(const Point2D & a, const Point2D & b)
{
  return {a.x + b.x, a.y + b.y};
}

inline Point2D operator-(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

inline Point2D operator*(double s, const Point2D & v)
{
  return {s * v.x, s * v.y};
}

inline Point2D operator*(const Point2D & v, double s)
{
  return {v.x * s, v.y * s};
}

inline Point2D normalize(const Point2D & v)
{
  const double n = norm(v);
  if (n < 1e-12) {
    return {0.0, 0.0};
  }
  return {v.x / n, v.y / n};
}

inline Point2D rotate90(const Point2D & v)
{
  return {-v.y, v.x};
}

inline Point2D rotate90_cw(const Point2D & v)
{
  return {v.y, -v.x};
}

inline double wrap_pi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

inline double angle_diff(double a, double b)
{
  return wrap_pi(b - a);
}

inline double heading(const Point2D & v)
{
  return std::atan2(v.y, v.x);
}

inline Point2D lerp(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

inline double polyline_length(const std::vector<Point2D> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += dist(pts[i - 1], pts[i]);
  }
  return len;
}

inline std::vector<Point2D> resample_polyline(
  const std::vector<Point2D> & pts, double ds)
{
  if (pts.size() < 2 || ds <= 0.0) {
    return pts;
  }

  std::vector<Point2D> out;
  out.push_back(pts.front());

  double accum = 0.0;
  size_t seg = 0;

  while (seg < pts.size() - 1) {
    const double seg_len = dist(pts[seg], pts[seg + 1]);
    if (seg_len < 1e-12) {
      ++seg;
      continue;
    }

    double remaining = ds - accum;
    if (remaining <= seg_len) {
      const double t = remaining / seg_len;
      Point2D p = lerp(pts[seg], pts[seg + 1], t);
      out.push_back(p);

      accum = 0.0;
      double pos_in_seg = remaining;
      while (pos_in_seg + ds <= seg_len) {
        pos_in_seg += ds;
        const double t2 = pos_in_seg / seg_len;
        out.push_back(lerp(pts[seg], pts[seg + 1], t2));
      }
      accum = seg_len - pos_in_seg;
      ++seg;
    } else {
      accum += seg_len;
      ++seg;
    }
  }

  if (dist(out.back(), pts.back()) > ds * 0.1) {
    out.push_back(pts.back());
  }

  return out;
}

inline std::vector<Point2D> polyline_tangents(const std::vector<Point2D> & pts)
{
  std::vector<Point2D> tangents(pts.size(), {1.0, 0.0});
  if (pts.size() < 2) {
    return tangents;
  }
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    tangents[i] = normalize(pts[i + 1] - pts[i]);
  }
  tangents.back() = tangents[tangents.size() - 2];
  return tangents;
}

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__GEOMETRY_HPP_
