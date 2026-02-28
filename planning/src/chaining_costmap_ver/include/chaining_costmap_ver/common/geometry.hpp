/**
 * @file geometry.hpp
 * @brief 2D 기하학 유틸리티 함수 모음 (inline 헤더 전용)
 *
 * ──────────────────────────────────────────────────────────────
 * [파이프라인 내 역할]
 *
 *   이 헤더는 chaining_costmap_ver 패키지의 **모든 모듈**에서 공통으로
 *   사용하는 2D 기하학 유틸리티를 모아놓은 파일이다.
 *   inline 함수로만 구성되어 있어 별도의 .cpp 컴파일 없이
 *   헤더만 include하면 바로 사용할 수 있다.
 *
 *   주요 사용처:
 *   - PathPostprocessor: dist, resample_polyline, polyline_tangents, heading
 *   - SafetyChecker   : dist, cross2 (곡률 계산)
 *   - DirectionChainer: dot2 (그래프 엣지 가중치)
 * ──────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__GEOMETRY_HPP_
#define CHAINING_COSTMAP_VER__COMMON__GEOMETRY_HPP_

#include "chaining_costmap_ver/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// 벡터 기본 연산
// ============================================================================

/**
 * @brief 2D 내적 (dot product)
 *
 * 수학적 정의:
 *   dot2(a, b) = a.x * b.x + a.y * b.y = |a||b|cos(θ)
 *
 * 의미:
 *   - 결과 > 0 → 두 벡터가 같은 방향 (θ < 90°)
 *   - 결과 = 0 → 두 벡터가 수직 (θ = 90°)
 *   - 결과 < 0 → 두 벡터가 반대 방향 (θ > 90°)
 *
 * 사용처:
 *   - MagneticPlanner: 현재 heading과 새 heading 사이의 cos(θ) 계산
 *     → atan2(cross2, dot2)로 정확한 회전 각도 δ를 구할 때 사용
 *   - DirectionChainer: 방향 정렬도(cos_angle) 계산 시 사용
 *   - regress_direction: PCA 결과의 부호 보정 (hint_dir과의 방향 비교)
 */
inline double dot2(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
}

/**
 * @brief 2D 외적 (cross product, z-성분만)
 *
 * 수학적 정의:
 *   cross2(a, b) = a.x * b.y - a.y * b.x = |a||b|sin(θ)
 *
 * 의미:
 *   - 결과 > 0 → b가 a의 반시계 방향(CCW)에 위치
 *   - 결과 = 0 → 두 벡터가 평행 (같은 방향 또는 반대 방향)
 *   - 결과 < 0 → b가 a의 시계 방향(CW)에 위치
 *   - |cross2|는 두 벡터로 만든 평행사변형의 넓이와 같다
 *
 * 사용처:
 *   - MagneticPlanner: heading 변화의 부호 있는 각도 계산
 *     → atan2(cross2(hdg, new_hdg), dot2(hdg, new_hdg))으로 δ 계산
 *   - SafetyChecker: 연속 3점(a→b→c)의 곡률 계산
 *     → κ = 2|cross(b-a, c-b)| / (|ab| * |bc| * |ac|)
 */
inline double cross2(const Point2D & a, const Point2D & b)
{
  return a.x * b.y - a.y * b.x;
}

/**
 * @brief 2D 벡터의 크기 (L2 노름)
 *
 * 수학적 정의:
 *   norm(v) = √(v.x² + v.y²) = |v|
 *
 * 사용처:
 *   - normalize() 내부에서 단위벡터 변환 전 크기 계산
 *   - MagneticPlanner: 이동 방향 벡터의 크기가 0인지 확인
 *     → norm(move_dir) < 1e-6이면 이동 불가로 판단하여 루프 종료
 */
inline double norm(const Point2D & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief 두 점 사이의 유클리드 거리
 *
 * 수학적 정의:
 *   dist(a, b) = √((a.x - b.x)² + (a.y - b.y)²)
 *
 * sqrt 연산이 포함되므로 단순 비교 목적이면 dist_sq() 사용을 권장한다.
 *
 * 사용처:
 *   - PathPostprocessor::prune(): 두 점 사이 직선 거리로 shortcut 가능 여부 판단
 *   - SafetyChecker: 곡률 계산 시 삼각형 세 변의 길이(ab, bc, ac) 계산
 *   - resample_polyline(): 세그먼트 길이 계산
 *   - polyline_length(): 폴리라인 총 길이 계산
 */
inline double dist(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/** @brief 벡터 뺄셈: a에서 b로 향하는 변위 벡터 = b - a */
inline Point2D operator-(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

// ============================================================================
// 벡터 변환 함수
// ============================================================================

/**
 * @brief 단위벡터 변환 (정규화)
 *
 * 수학적 정의:
 *   normalize(v) = v / |v| = (v.x/|v|, v.y/|v|)
 *   결과 벡터의 크기는 항상 1.0 (방향만 보존)
 *
 * |v|가 1e-12 미만이면 영벡터를 반환한다 (0으로 나누기 방지).
 *
 * 사용처:
 *   - MagneticPlanner: heading 벡터를 단위벡터로 유지
 *     → heading_init, move_dir, blended heading 모두 normalize 적용
 *   - polyline_tangents(): 폴리라인의 각 점에서 접선 단위벡터 계산
 *   - regress_direction(): PCA 결과 및 hint_dir을 단위벡터로 변환
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
 * @brief 벡터의 방위각 (heading angle)
 *
 * 수학적 정의:
 *   heading(v) = atan2(v.y, v.x)
 *   결과 범위: [-π, +π]
 *
 * 좌표계 규약 (ROS 2 표준):
 *   - 0 rad    → +x 방향 (전방)
 *   - +π/2 rad → +y 방향 (왼쪽)
 *   - ±π rad   → -x 방향 (후방)
 *   - -π/2 rad → -y 방향 (오른쪽)
 *
 * 사용처:
 *   - PathPostprocessor::process(): 경로의 각 점에서 yaw 값 계산
 *     → polyline_tangents()로 접선벡터를 구한 뒤, heading()으로 라디안 변환
 *     → 이 yaw 값이 차량의 목표 heading이 된다
 */
inline double heading(const Point2D & v)
{
  return std::atan2(v.y, v.x);
}

// ============================================================================
// 보간 및 폴리라인 처리 함수
// ============================================================================

/**
 * @brief 선형 보간 (Linear Interpolation, LERP)
 *
 * 수학적 정의:
 *   lerp(a, b, t) = a + t * (b - a) = (1 - t) * a + t * b
 *
 * 매개변수:
 *   - t = 0.0 → 점 a를 반환
 *   - t = 1.0 → 점 b를 반환
 *   - t = 0.5 → a와 b의 중점
 *   - 0 < t < 1 → a와 b 사이의 점 (t에 비례하여 b에 가까워짐)
 *
 * 사용처:
 *   - resample_polyline() 내부에서 등간격 리샘플링 점을 생성할 때
 *     → 세그먼트 위의 정확한 위치에 새 점을 삽입
 */
inline Point2D lerp(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

/**
 * @brief 폴리라인을 등간격(ds)으로 리샘플링
 *
 * 원래 폴리라인의 점 간격이 불균일할 때, 정확히 ds 간격으로
 * 새 점들을 보간하여 균일한 폴리라인을 생성한다.
 *
 * ── 알고리즘 상세 (step by step) ──
 *
 *   입력: pts = [P0, P1, P2, ...], ds = 등간격 거리
 *   출력: out = [Q0, Q1, Q2, ...], 각 Qi 사이의 거리 ≈ ds
 *
 *   1) 시작점 P0를 출력에 추가
 *
 *   2) 세그먼트 순회 (seg = 0, 1, 2, ...):
 *      각 세그먼트는 pts[seg] → pts[seg+1] 구간
 *
 *      2a) 세그먼트 길이 seg_len = dist(pts[seg], pts[seg+1])
 *          seg_len ≈ 0이면 스킵 (중복점)
 *
 *      2b) remaining = ds - accum
 *          accum: 이전 세그먼트에서 누적된 "남은 거리"
 *          remaining: 다음 리샘플 점까지 남은 거리
 *
 *      2c) 만약 remaining ≤ seg_len (이 세그먼트 안에서 점을 찍을 수 있음):
 *          - t = remaining / seg_len 으로 보간 비율 계산
 *          - lerp(pts[seg], pts[seg+1], t) → 새 점 추가
 *          - 같은 세그먼트 안에서 ds 간격으로 추가 점을 계속 생성
 *          - 세그먼트 끝을 넘으면, 남은 거리를 accum에 저장
 *
 *      2d) 만약 remaining > seg_len (이 세그먼트가 너무 짧음):
 *          - accum += seg_len으로 누적하고 다음 세그먼트로 이동
 *
 *   3) 마지막 점 처리:
 *      출력의 마지막 점과 원본의 끝점 사이 거리가 ds*0.1보다 크면
 *      끝점을 추가하여 경로 끝을 보존
 *
 * 예시 (ds = 0.5m):
 *   입력: [P0]---0.3m---[P1]---0.8m---[P2]---0.2m---[P3]
 *   과정: P0에서 시작, 0.3m 지나면 accum=0.3, remaining=0.2
 *         P1~P2 세그먼트에서 0.2m 위치에 Q1 생성 (0.3+0.2=0.5)
 *         Q1에서 0.5m 더 가면 Q2, ...
 *
 * 사용처:
 *   - PathPostprocessor::process(): 후처리 파이프라인의 마지막 단계
 *     → prune → smooth → resample_polyline(smoothed, resample_ds)
 *     → 균일한 간격의 경로를 만들어 제어기에 전달
 */
inline std::vector<Point2D> resample_polyline(
  const std::vector<Point2D> & pts, double ds)
{
  // 점이 2개 미만이거나 간격이 0 이하면 원본 반환
  if (pts.size() < 2 || ds <= 0.0) {
    return pts;
  }

  std::vector<Point2D> out;
  out.push_back(pts.front());  // Step 1: 시작점 추가

  double accum = 0.0;  // 이전 세그먼트에서 누적된 잔여 거리
  size_t seg = 0;      // 현재 처리 중인 세그먼트 인덱스

  // Step 2: 모든 세그먼트 순회
  while (seg < pts.size() - 1) {
    // Step 2a: 현재 세그먼트 길이 계산
    const double seg_len = dist(pts[seg], pts[seg + 1]);
    if (seg_len < 1e-12) {
      ++seg;  // 중복점(길이 0) 스킵
      continue;
    }

    // Step 2b: 다음 리샘플 점까지 남은 거리
    double remaining = ds - accum;

    if (remaining <= seg_len) {
      // Step 2c: 이 세그먼트 안에서 리샘플 점 생성 가능
      const double t = remaining / seg_len;
      Point2D p = lerp(pts[seg], pts[seg + 1], t);
      out.push_back(p);

      // 같은 세그먼트 안에서 ds 간격으로 추가 점 생성
      accum = 0.0;
      double pos_in_seg = remaining;
      while (pos_in_seg + ds <= seg_len) {
        pos_in_seg += ds;
        const double t2 = pos_in_seg / seg_len;
        out.push_back(lerp(pts[seg], pts[seg + 1], t2));
      }
      // 세그먼트 끝까지의 잔여 거리를 다음 세그먼트로 이월
      accum = seg_len - pos_in_seg;
      ++seg;
    } else {
      // Step 2d: 세그먼트가 짧아서 점을 찍지 못함 → 누적
      accum += seg_len;
      ++seg;
    }
  }

  // Step 3: 끝점 보존 (마지막 점과 원본 끝점이 멀면 추가)
  if (dist(out.back(), pts.back()) > ds * 0.1) {
    out.push_back(pts.back());
  }

  return out;
}

/**
 * @brief 폴리라인의 각 점에서 접선 단위벡터(tangent) 계산
 *
 * 알고리즘:
 *   - i번째 점의 접선 = normalize(pts[i+1] - pts[i])
 *     → 현재 점에서 다음 점으로 향하는 방향의 단위벡터
 *   - 마지막 점은 다음 점이 없으므로 직전 점의 접선을 복사
 *   - 점이 1개뿐이면 기본값 (1,0) = +x 방향 반환
 *
 * 수학적 의미:
 *   접선벡터는 경로의 "순간 진행 방향"을 나타낸다.
 *   곡선 미분 dr/ds ≈ (pts[i+1] - pts[i]) / |pts[i+1] - pts[i]|
 *
 * 사용처:
 *   - PathPostprocessor::process(): 리샘플링된 경로의 각 점에서
 *     접선벡터를 구한 뒤, heading()으로 yaw(방위각) 변환
 *     → result.yaw[i] = heading(tangents[i])
 *     → 이 yaw 배열이 차량 제어기의 목표 heading이 된다
 */
inline std::vector<Point2D> polyline_tangents(const std::vector<Point2D> & pts)
{
  // 기본값: +x 방향 (전방). 점이 1개뿐이면 이 값이 그대로 반환됨
  std::vector<Point2D> tangents(pts.size(), {1.0, 0.0});
  if (pts.size() < 2) {
    return tangents;
  }
  // 각 점에서 다음 점으로의 방향 단위벡터 계산
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    tangents[i] = normalize(pts[i + 1] - pts[i]);
  }
  // 마지막 점: 다음 점이 없으므로 직전 접선을 복사
  tangents.back() = tangents[tangents.size() - 2];
  return tangents;
}

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__GEOMETRY_HPP_
