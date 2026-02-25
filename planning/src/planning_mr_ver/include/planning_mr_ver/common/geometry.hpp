/**
 * @file geometry.hpp
 * @brief 2D 기하학 유틸리티 함수 모음 (inline 헤더 전용)
 *
 * planning_mr_ver 파이프라인 전체에서 사용하는 벡터 연산, 각도 처리,
 * 폴리라인 리샘플링 등의 기하학 함수를 제공한다.
 * 모든 함수가 inline으로 정의되어 있어 별도 .cpp 없이 헤더만 포함하면 사용 가능하다.
 */
#ifndef PLANNING_MR_VER__COMMON__GEOMETRY_HPP_
#define PLANNING_MR_VER__COMMON__GEOMETRY_HPP_

#include "planning_mr_ver/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace planning_mr_ver
{

// ============================================================================
// 벡터 기본 연산
// ============================================================================

/**
 * @brief 2D 내적(dot product): a·b = ax*bx + ay*by
 *
 * 용도: 두 벡터의 방향 유사도 판단, 전방 180° 필터링 등
 * 결과 > 0: 같은 방향, = 0: 직교, < 0: 반대 방향
 */
inline double dot2(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
}

/**
 * @brief 2D 외적(cross product): a×b = ax*by - ay*bx
 *
 * 용도: 곡률 계산, 회전 방향 판단
 * 결과 > 0: b가 a의 반시계 방향, < 0: 시계 방향
 */
inline double cross2(const Point2D & a, const Point2D & b)
{
  return a.x * b.y - a.y * b.x;
}

/**
 * @brief 벡터의 크기(노름): ||v|| = √(vx² + vy²)
 */
inline double norm(const Point2D & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief 두 점 사이의 유클리드 거리: ||a - b||
 */
inline double dist(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 두 점 사이의 거리 제곱 (sqrt 생략 → 비교 연산에 효율적)
 *
 * dist()보다 빠르므로 "거리 비교"만 필요할 때 사용한다.
 * 예: if (dist_sq(a,b) < r*r) → 반경 r 이내 판정
 */
inline double dist_sq(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// ============================================================================
// 벡터 산술 연산자 오버로딩
// ============================================================================

/// 벡터 덧셈: a + b
inline Point2D operator+(const Point2D & a, const Point2D & b)
{
  return {a.x + b.x, a.y + b.y};
}

/// 벡터 뺄셈: a - b (a에서 b로의 변위 벡터)
inline Point2D operator-(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

/// 스칼라 × 벡터: s * v
inline Point2D operator*(double s, const Point2D & v)
{
  return {s * v.x, s * v.y};
}

/// 벡터 × 스칼라: v * s
inline Point2D operator*(const Point2D & v, double s)
{
  return {v.x * s, v.y * s};
}

// ============================================================================
// 벡터 변환 함수
// ============================================================================

/**
 * @brief 단위벡터로 정규화: v / ||v||
 *
 * 크기가 거의 0인 벡터(< 1e-12)는 영벡터(0,0)를 반환한다 (0으로 나누기 방지).
 */
inline Point2D normalize(const Point2D & v)
{
  const double n = norm(v);
  if (n < 1e-12) {
    return {0.0, 0.0};
  }
  return {v.x / n, v.y / n};
}

/**
 * @brief 반시계 방향(CCW) 90° 회전: (x, y) → (-y, x)
 *
 * 용도: 경로의 법선 벡터 계산 (왼쪽 방향)
 */
inline Point2D rotate90(const Point2D & v)
{
  return {-v.y, v.x};
}

/**
 * @brief 시계 방향(CW) 90° 회전: (x, y) → (y, -x)
 *
 * 용도: 경로의 법선 벡터 계산 (오른쪽 방향)
 */
inline Point2D rotate90_cw(const Point2D & v)
{
  return {v.y, -v.x};
}

// ============================================================================
// 각도 처리 함수
// ============================================================================

/**
 * @brief 각도를 [-π, +π) 범위로 정규화 (wrap)
 *
 * 예: 4.0 rad → 4.0 - 2π ≈ -2.28 rad
 * 차량의 heading 차이 계산 시 반드시 wrap 해야 올바른 방향을 알 수 있다.
 */
inline double wrap_pi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

/**
 * @brief 두 각도의 차이 (b - a)를 [-π, +π) 범위로 반환
 *
 * 예: angle_diff(170°, -170°) = 20° (360°를 넘지 않고 최단 회전)
 */
inline double angle_diff(double a, double b)
{
  return wrap_pi(b - a);
}

/**
 * @brief 벡터 v의 heading 각도를 반환 [rad]
 *
 * atan2(y, x): +x축 기준 반시계 방향 각도
 *   (1,0)=0°, (0,1)=90°, (-1,0)=180°, (0,-1)=-90°
 */
inline double heading(const Point2D & v)
{
  return std::atan2(v.y, v.x);
}

// ============================================================================
// 보간 및 폴리라인 처리 함수
// ============================================================================

/**
 * @brief 선형 보간(Linear Interpolation): a에서 b로 t만큼 이동한 점
 *
 * @param t 보간 비율 (0.0=a 위치, 0.5=중간, 1.0=b 위치)
 * @return a + t * (b - a)
 */
inline Point2D lerp(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

/**
 * @brief 폴리라인의 총 길이를 계산
 *
 * 연속된 점들 사이의 거리를 모두 합산한다.
 * @param pts 폴리라인을 구성하는 점 목록
 * @return 폴리라인 총 길이 [m]
 */
inline double polyline_length(const std::vector<Point2D> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += dist(pts[i - 1], pts[i]);
  }
  return len;
}

/**
 * @brief 폴리라인을 균일한 간격(ds)으로 리샘플링
 *
 * 입력 폴리라인의 점 간격이 불균일할 때, 일정 간격 ds마다 새 점을 생성한다.
 * 경로의 후처리 단계에서 사용된다.
 *
 * 알고리즘:
 *   1. 첫 번째 점을 출력에 추가
 *   2. 각 세그먼트(pts[seg] → pts[seg+1])를 따라 이동하면서
 *      ds 간격마다 lerp로 새 점을 생성
 *   3. 누적 거리(accum)로 세그먼트 경계를 넘는 경우 처리
 *   4. 마지막 점이 너무 멀면(ds의 10% 초과) 추가
 *
 * @param pts 원본 폴리라인 점 목록
 * @param ds  리샘플링 간격 [m] (기본: 0.1m)
 * @return 균일 간격으로 리샘플링된 새 폴리라인
 */
inline std::vector<Point2D> resample_polyline(
  const std::vector<Point2D> & pts, double ds)
{
  if (pts.size() < 2 || ds <= 0.0) {
    return pts;
  }

  std::vector<Point2D> out;
  out.push_back(pts.front());  // 시작점은 항상 포함

  double accum = 0.0;  // 현재 세그먼트에서 이미 진행한 거리
  size_t seg = 0;      // 현재 처리 중인 세그먼트 인덱스

  while (seg < pts.size() - 1) {
    const double seg_len = dist(pts[seg], pts[seg + 1]);
    if (seg_len < 1e-12) {
      // 두 점이 거의 같은 위치 → 건너뜀
      ++seg;
      continue;
    }

    double remaining = ds - accum;  // ds 간격을 채우기 위해 남은 거리
    if (remaining <= seg_len) {
      // 이 세그먼트 안에서 최소 1개 이상의 리샘플 점 생성 가능
      const double t = remaining / seg_len;
      Point2D p = lerp(pts[seg], pts[seg + 1], t);
      out.push_back(p);

      accum = 0.0;
      double pos_in_seg = remaining;
      // 같은 세그먼트 안에서 추가 리샘플 점 생성
      while (pos_in_seg + ds <= seg_len) {
        pos_in_seg += ds;
        const double t2 = pos_in_seg / seg_len;
        out.push_back(lerp(pts[seg], pts[seg + 1], t2));
      }
      accum = seg_len - pos_in_seg;  // 다음 세그먼트로 넘길 나머지 거리
      ++seg;
    } else {
      // 이 세그먼트만으로 ds를 못 채움 → 다음 세그먼트에서 이어서
      accum += seg_len;
      ++seg;
    }
  }

  // 마지막 점이 너무 멀면 추가 (ds의 10% 이상 차이 시)
  if (dist(out.back(), pts.back()) > ds * 0.1) {
    out.push_back(pts.back());
  }

  return out;
}

/**
 * @brief 폴리라인 각 점의 접선(tangent) 단위벡터를 계산
 *
 * 각 점에서 다음 점으로의 방향벡터를 정규화하여 접선으로 사용한다.
 * 마지막 점의 접선은 직전 점의 접선을 복사한다.
 *
 * @param pts 폴리라인 점 목록
 * @return 각 점의 접선 단위벡터 (pts와 같은 크기)
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
  tangents.back() = tangents[tangents.size() - 2];  // 마지막 = 직전 복사
  return tangents;
}

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__GEOMETRY_HPP_
