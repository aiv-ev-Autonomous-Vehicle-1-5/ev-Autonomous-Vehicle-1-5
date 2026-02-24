#ifndef TRACK_PLANNING__COMMON__GEOMETRY_HPP_
#define TRACK_PLANNING__COMMON__GEOMETRY_HPP_

#include "track_planning/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace track_planning
{

// ---- Scalar ops ----

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

// ---- Vector ops ----

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

/// Rotate 90 degrees counter-clockwise (left normal)
inline Point2D rotate90(const Point2D & v)
{
  return {-v.y, v.x};
}

/// Rotate 90 degrees clockwise (right normal)
inline Point2D rotate90_cw(const Point2D & v)
{
  return {v.y, -v.x};
}

// ---- Angle ops ----

/// Wrap angle to [-pi, pi]
inline double wrap_pi(double angle)
{
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

/// Signed angle difference (b - a), result in [-pi, pi]
inline double angle_diff(double a, double b)
{
  return wrap_pi(b - a);
}

/// Heading angle from vector (atan2)
inline double heading(const Point2D & v)
{
  return std::atan2(v.y, v.x);
}

// ---- Interpolation ----

inline Point2D lerp(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

// ---- Polyline ops ----

/// Compute total arc length of a polyline
inline double polyline_length(const std::vector<Point2D> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += dist(pts[i - 1], pts[i]);
  }
  return len;
}

/// Resample a polyline to uniform spacing ds
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
      // Interpolate within this segment
      const double t = remaining / seg_len;
      Point2D p = lerp(pts[seg], pts[seg + 1], t);
      out.push_back(p);

      // Continue from this interpolated point within the same segment
      // Update pts[seg] conceptually by adjusting accum
      accum = 0.0;
      // Move forward by 'remaining' distance along the segment
      // We need to handle multiple samples within one long segment
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

  // Always include the last point if it's not too close
  if (dist(out.back(), pts.back()) > ds * 0.1) {
    out.push_back(pts.back());
  }

  return out;
}

/// Compute tangent at each point of a polyline (forward difference, last = prev)
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

/// Regress tangent direction from last N points of a polyline
inline Point2D regress_tangent(const std::vector<Point2D> & pts, size_t n_reg)
{
  if (pts.size() < 2) {
    return {1.0, 0.0};
  }
  const size_t n = std::min(n_reg, pts.size());
  const size_t start = pts.size() - n;
  Point2D sum{0.0, 0.0};
  for (size_t i = start; i + 1 < pts.size(); ++i) {
    Point2D d = pts[i + 1] - pts[i];
    double dn = norm(d);
    if (dn > 1e-12) {
      sum = sum + (1.0 / dn) * d;
    }
  }
  const double sn = norm(sum);
  if (sn < 1e-12) {
    return normalize(pts.back() - pts[start]);
  }
  return {sum.x / sn, sum.y / sn};
}

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__GEOMETRY_HPP_
