/**
 * @file geometry.hpp
 * @brief 2D 기하학 유틸리티 (header-only)
 *
 * 플래닝 파이프라인 전체에서 사용하는 2D 벡터 연산, 각도 연산, 폴리라인 처리 함수.
 * 모든 함수가 inline으로 정의되어 있어 별도 .cpp 없이 헤더만 include하면 사용 가능.
 *
 * ┌────────────────────────────────────────────────────────────────┐
 * │                  base_link 좌표계 기준                         │
 * ├────────────────────────────────────────────────────────────────┤
 * │                                                                │
 * │           +y (좌측)                                            │
 * │            ▲                                                   │
 * │            │                                                   │
 * │            │    · · ·   ← 좌측 경계 (corridor.left)           │
 * │            │                                                   │
 * │  ──────────●────────────► +x (전방)                           │
 * │       ego (0,0)                                                │
 * │            │                                                   │
 * │            │    · · ·   ← 우측 경계 (corridor.right)          │
 * │            │                                                   │
 * │           -y (우측)                                            │
 * │                                                                │
 * │  heading = atan2(y, x): +x = 0°, +y = 90° (반시계 양수)      │
 * │  rotate90(v): (x,y) → (-y,x) = CCW 90° (좌측 법선)           │
 * └────────────────────────────────────────────────────────────────┘
 *
 * 주요 기능:
 *   - 스칼라 연산: dot2, cross2, norm, dist, dist_sq
 *   - 벡터 연산: +, -, *, normalize, rotate90, rotate90_cw
 *   - 각도 연산: wrap_pi, angle_diff, heading
 *   - 보간: lerp (선형 보간)
 *   - 폴리라인: polyline_length, resample_polyline, polyline_tangents, regress_tangent
 *   - 기하 판정: circumcenter, segments_intersect, polylines_cross
 *   - 통계: compute_median_width (좌/우 경계 간 중앙값 폭)
 *
 * 사용 위치:
 *   - corridor_builder.cpp  : dot2, cross2, norm, dist, heading, angle_diff, regress_tangent
 *   - virtual_boundary.cpp  : polyline_tangents, rotate90
 *   - centerline_builder.cpp: resample_polyline, polyline_tangents, circumcenter, polylines_cross
 *   - path_postprocessor.cpp: resample_polyline, polyline_tangents, heading
 *   - safety_checker.hpp    : dist, cross2 (Menger 곡률)
 *   - local_planner_node.cpp: compute_median_width
 */
#ifndef TRACK_PLANNING__COMMON__GEOMETRY_HPP_
#define TRACK_PLANNING__COMMON__GEOMETRY_HPP_

#include "track_planning/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace track_planning
{

// ============================================================
// 스칼라 연산 (Scalar Operations)
// ============================================================

/// 2D 내적 (dot product): a · b = ax*bx + ay*by
inline double dot2(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
}

/// 2D 외적 (cross product): a × b = ax*by - ay*bx
/// 양수 = b가 a 기준 반시계 방향, 음수 = 시계 방향
inline double cross2(const Point2D & a, const Point2D & b)
{
  return a.x * b.y - a.y * b.x;
}

/// 벡터의 크기 (magnitude): |v| = sqrt(x² + y²)
inline double norm(const Point2D & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

/// 두 점 사이의 유클리드 거리
inline double dist(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/// 두 점 사이의 거리 제곱 (sqrt 없이 비교용으로 빠름)
inline double dist_sq(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// ============================================================
// 벡터 연산 (Vector Operations)
// ============================================================

/// 벡터 덧셈: a + b
inline Point2D operator+(const Point2D & a, const Point2D & b)
{
  return {a.x + b.x, a.y + b.y};
}

/// 벡터 뺄셈: a - b
inline Point2D operator-(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

/// 스칼라 곱: s * v (왼쪽 스칼라)
inline Point2D operator*(double s, const Point2D & v)
{
  return {s * v.x, s * v.y};
}

/// 스칼라 곱: v * s (오른쪽 스칼라)
inline Point2D operator*(const Point2D & v, double s)
{
  return {v.x * s, v.y * s};
}

/// 단위 벡터로 정규화: v / |v|
/// 크기가 거의 0이면 영벡터 반환 (division by zero 방지)
inline Point2D normalize(const Point2D & v)
{
  const double n = norm(v);
  if (n < 1e-12) {
    return {0.0, 0.0};
  }
  return {v.x / n, v.y / n};
}

/// 반시계 방향 90도 회전 (좌측 법선벡터 생성)
/// (x, y) → (-y, x)
inline Point2D rotate90(const Point2D & v)
{
  return {-v.y, v.x};
}

/// 시계 방향 90도 회전 (우측 법선벡터 생성)
/// (x, y) → (y, -x)
inline Point2D rotate90_cw(const Point2D & v)
{
  return {v.y, -v.x};
}

// ============================================================
// 각도 연산 (Angle Operations)
// ============================================================

/// 각도를 [-π, π] 범위로 정규화 (NaN/Inf 안전: std::fmod 기반)
inline double wrap_pi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

/// 부호 있는 각도 차이: (b - a), 결과는 [-π, π]
inline double angle_diff(double a, double b)
{
  return wrap_pi(b - a);
}

/// 벡터에서 heading 각도 추출: atan2(y, x)
/// 결과 범위: [-π, π], +x 방향이 0, 반시계가 양수
inline double heading(const Point2D & v)
{
  return std::atan2(v.y, v.x);
}

// ============================================================
// 보간 (Interpolation)
// ============================================================

/// 선형 보간 (Linear Interpolation): a와 b 사이를 t 비율로 보간
/// t=0이면 a, t=1이면 b, t=0.5이면 중간점
inline Point2D lerp(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

// ============================================================
// 폴리라인 연산 (Polyline Operations)
// ============================================================

/// 폴리라인의 총 호(arc) 길이 계산
/// 연속된 점 사이 거리의 합
inline double polyline_length(const std::vector<Point2D> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += dist(pts[i - 1], pts[i]);
  }
  return len;
}

/**
 * @brief 폴리라인을 균일 간격 ds로 리샘플링
 *
 * 알고리즘 (Segment Walking):
 *   1. 첫 점을 출력에 추가
 *   2. 각 세그먼트를 따라 이동하면서 ds 간격마다 보간점 생성
 *   3. 세그먼트 경계를 넘을 때 누적 거리(accum) 유지
 *   4. 마지막 점이 너무 가까우면(< ds*0.1) 생략
 *
 * @param pts  입력 폴리라인 (최소 2점 필요)
 * @param ds   출력 점 간격 (m)
 * @return 균일 간격으로 리샘플링된 폴리라인
 */
inline std::vector<Point2D> resample_polyline(
  const std::vector<Point2D> & pts, double ds)
{
  if (pts.size() < 2 || ds <= 0.0) {
    return pts;
  }

  std::vector<Point2D> out;
  out.push_back(pts.front());  // 항상 첫 점 포함

  double accum = 0.0;  // 현재 세그먼트에서 이전 출력점 이후 누적 거리
  size_t seg = 0;       // 현재 세그먼트 인덱스

  while (seg < pts.size() - 1) {
    const double seg_len = dist(pts[seg], pts[seg + 1]);
    if (seg_len < 1e-12) {  // 길이 0인 세그먼트 건너뛰기
      ++seg;
      continue;
    }

    double remaining = ds - accum;  // 다음 출력점까지 남은 거리
    if (remaining <= seg_len) {
      // 이 세그먼트 내에서 보간점 생성
      const double t = remaining / seg_len;
      Point2D p = lerp(pts[seg], pts[seg + 1], t);
      out.push_back(p);

      // 하나의 긴 세그먼트에서 여러 출력점이 나올 수 있음
      accum = 0.0;
      double pos_in_seg = remaining;
      while (pos_in_seg + ds <= seg_len) {
        pos_in_seg += ds;
        const double t2 = pos_in_seg / seg_len;
        out.push_back(lerp(pts[seg], pts[seg + 1], t2));
      }
      accum = seg_len - pos_in_seg;  // 남은 거리를 다음 세그먼트로 이월
      ++seg;
    } else {
      // 이 세그먼트에서는 출력점 없음 — 누적만 증가
      accum += seg_len;
      ++seg;
    }
  }

  // 마지막 점이 너무 가까우면 중복 방지
  if (dist(out.back(), pts.back()) > ds * 0.1) {
    out.push_back(pts.back());
  }

  return out;
}

/**
 * @brief 폴리라인의 각 점에서 접선 벡터 계산 (단위벡터)
 *
 * 전방 차분(forward difference) 사용:
 *   tangent[i] = normalize(pts[i+1] - pts[i])
 * 마지막 점은 이전 점의 접선을 복사.
 */
inline std::vector<Point2D> polyline_tangents(const std::vector<Point2D> & pts)
{
  std::vector<Point2D> tangents(pts.size(), {1.0, 0.0});  // 기본값: +x 방향
  if (pts.size() < 2) {
    return tangents;
  }
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    tangents[i] = normalize(pts[i + 1] - pts[i]);
  }
  tangents.back() = tangents[tangents.size() - 2];  // 마지막 = 이전 복사
  return tangents;
}

/**
 * @brief 폴리라인의 마지막 N개 점에서 접선 방향을 회귀 추정
 *
 * 각 세그먼트의 단위 방향벡터를 합산한 후 정규화하여
 * 노이즈에 강건한 평균 접선 방향을 얻는다.
 * Cold start에서 이전 centerline의 끝부분 방향을 참조할 때 사용.
 *
 * @param pts    입력 폴리라인
 * @param n_reg  회귀에 사용할 마지막 점 개수
 * @return 추정된 접선 단위벡터
 */
inline Point2D regress_tangent(const std::vector<Point2D> & pts, size_t n_reg)
{
  if (pts.size() < 2) {
    return {1.0, 0.0};  // 기본값: +x 방향
  }
  const size_t n = std::min(n_reg, pts.size());
  const size_t start = pts.size() - n;

  // 각 세그먼트의 단위 방향벡터를 합산
  Point2D sum{0.0, 0.0};
  for (size_t i = start; i + 1 < pts.size(); ++i) {
    Point2D d = pts[i + 1] - pts[i];
    double dn = norm(d);
    if (dn > 1e-12) {
      sum = sum + (1.0 / dn) * d;  // 단위벡터로 변환 후 합산
    }
  }

  // 합산 벡터를 정규화
  const double sn = norm(sum);
  if (sn < 1e-12) {
    return normalize(pts.back() - pts[start]);  // fallback: 처음→끝 방향
  }
  return {sum.x / sn, sum.y / sn};
}

/// 삼각형 외심(circumcenter) 계산
/// 세 꼭짓점에서 등거리인 점 (Delaunay 삼각분할에서 Voronoi 정점에 해당)
/// 퇴화 삼각형(D≈0)이면 무게중심 반환
inline Point2D circumcenter(const Point2D & a, const Point2D & b, const Point2D & c)
{
  const double ax = a.x, ay = a.y;
  const double bx = b.x, by = b.y;
  const double cx = c.x, cy = c.y;
  const double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(D) < 1e-12) {
    return {(ax + bx + cx) / 3.0, (ay + by + cy) / 3.0};
  }
  const double a2 = ax * ax + ay * ay;
  const double b2 = bx * bx + by * by;
  const double c2 = cx * cx + cy * cy;
  return {
    (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / D,
    (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / D
  };
}

/// 두 선분 (p1-p2)와 (p3-p4)가 교차하는지 판정 (CCW 기반)
/// 끝점 공유(T-접촉)는 교차로 판정하지 않음
inline bool segments_intersect(
  const Point2D & p1, const Point2D & p2,
  const Point2D & p3, const Point2D & p4)
{
  auto ccw = [](const Point2D & a, const Point2D & b, const Point2D & c) -> double {
    return cross2(b - a, c - a);
  };
  const double d1 = ccw(p3, p4, p1);
  const double d2 = ccw(p3, p4, p2);
  const double d3 = ccw(p1, p2, p3);
  const double d4 = ccw(p1, p2, p4);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return true;
  }
  return false;
}

/// 두 폴리라인이 교차하는지 판정 (O(N*M))
inline bool polylines_cross(
  const std::vector<Point2D> & a,
  const std::vector<Point2D> & b)
{
  for (size_t i = 0; i + 1 < a.size(); ++i) {
    for (size_t j = 0; j + 1 < b.size(); ++j) {
      if (segments_intersect(a[i], a[i + 1], b[j], b[j + 1])) {
        return true;
      }
    }
  }
  return false;
}

/// 좌/우 corridor에서 각 점의 반대편 경계까지 최근접점 거리의 중앙값 계산
/// 인덱스 매칭 대신 nearest-point 방식으로 커브에서도 정확한 폭 추정
inline double compute_median_width(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  double ds = 0.2)
{
  auto left_rs  = resample_polyline(left, ds);
  auto right_rs = resample_polyline(right, ds);
  if (left_rs.empty() || right_rs.empty()) return 0.0;

  std::vector<double> widths;
  widths.reserve(left_rs.size());

  // 각 좌측 점에서 우측 경계까지 최근접 거리
  for (const auto & lp : left_rs) {
    double min_d = std::numeric_limits<double>::max();
    for (const auto & rp : right_rs) {
      const double d = dist(lp, rp);
      if (d < min_d) min_d = d;
    }
    widths.push_back(min_d);
  }

  std::sort(widths.begin(), widths.end());
  return widths[widths.size() / 2];
}

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__GEOMETRY_HPP_
