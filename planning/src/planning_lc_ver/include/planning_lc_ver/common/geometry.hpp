/**
 * @file geometry.hpp
 * @brief 2D 기하학 유틸리티 함수 모음 (inline 헤더 전용)
 *
 * planning_mr_ver 기반 + regress_direction() 추가.
 * LineChainer에서 chain 진행 방향을 PCA로 추정할 때 사용한다.
 */
#ifndef PLANNING_LC_VER__COMMON__GEOMETRY_HPP_
#define PLANNING_LC_VER__COMMON__GEOMETRY_HPP_

#include "planning_lc_ver/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace planning_lc_ver
{

// ============================================================================
// 벡터 기본 연산
// ============================================================================

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

// ============================================================================
// 벡터 산술 연산자 오버로딩
// ============================================================================

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

// ============================================================================
// 벡터 변환 함수
// ============================================================================

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

// ============================================================================
// 각도 처리 함수
// ============================================================================

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

// ============================================================================
// 보간 및 폴리라인 처리 함수
// ============================================================================

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

// ============================================================================
// PCA 기반 방향벡터 추정 (LineChainer 전용, 신규)
// ============================================================================

/**
 * @brief 최근 n개 점에 대한 PCA 주성분 방향벡터 계산
 *
 * 2×2 공분산 행렬의 최대 고유벡터를 구하여 chain 진행 방향을 추정한다.
 * hint_dir과 내적이 음수이면 부호를 반전하여 진행 방향과 일치시킨다.
 *
 * 공분산 행렬:
 *   C = [ Σ(xi-μx)²      Σ(xi-μx)(yi-μy) ]
 *       [ Σ(xi-μx)(yi-μy)  Σ(yi-μy)²      ]
 *
 * 고유값: λ = (a+d)/2 ± sqrt(((a-d)/2)² + b²)
 * 최대 고유값에 대응하는 고유벡터가 주성분 방향.
 *
 * @param pts       chain에 추가된 점들 (전체)
 * @param n         사용할 최근 점 수 (min_regress_pts ~ max_regress_pts)
 * @param hint_dir  chain 진행 방향 힌트 (부호 보정용)
 * @return 정규화된 방향 단위벡터
 */
inline Point2D regress_direction(
  const std::vector<Point2D> & pts,
  int n,
  const Point2D & hint_dir)
{
  const int total = static_cast<int>(pts.size());
  if (total < 2 || n < 2) {
    return normalize(hint_dir);
  }

  // 최근 n개 점 범위: [start, total)
  const int use_n = std::min(n, total);
  const int start = total - use_n;

  // 평균 계산
  double mx = 0.0, my = 0.0;
  for (int i = start; i < total; ++i) {
    mx += pts[i].x;
    my += pts[i].y;
  }
  mx /= use_n;
  my /= use_n;

  // 공분산 행렬 요소
  double cxx = 0.0, cxy = 0.0, cyy = 0.0;
  for (int i = start; i < total; ++i) {
    const double dx = pts[i].x - mx;
    const double dy = pts[i].y - my;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }

  // 2×2 대칭 행렬의 최대 고유벡터
  // 고유값: λ = (cxx+cyy)/2 ± sqrt(((cxx-cyy)/2)² + cxy²)
  const double trace_half = (cxx + cyy) * 0.5;
  const double diff_half = (cxx - cyy) * 0.5;
  const double disc = std::sqrt(diff_half * diff_half + cxy * cxy);

  Point2D dir;
  if (disc < 1e-12) {
    // 모든 점이 한 곳에 모여 있음 → hint 사용
    return normalize(hint_dir);
  }

  // 최대 고유값 λ1 = trace_half + disc
  // 대응 고유벡터: (λ1 - cyy, cxy) 또는 (cxy, λ1 - cxx)
  const double lambda1 = trace_half + disc;
  double ex = lambda1 - cyy;
  double ey = cxy;
  const double en = std::sqrt(ex * ex + ey * ey);
  if (en < 1e-12) {
    ex = cxy;
    ey = lambda1 - cxx;
    const double en2 = std::sqrt(ex * ex + ey * ey);
    if (en2 < 1e-12) return normalize(hint_dir);
    ex /= en2;
    ey /= en2;
  } else {
    ex /= en;
    ey /= en;
  }

  dir = {ex, ey};

  // 부호 보정: hint_dir과 같은 방향이 되도록
  if (dot2(dir, hint_dir) < 0.0) {
    dir = {-dir.x, -dir.y};
  }

  return dir;
}

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COMMON__GEOMETRY_HPP_
