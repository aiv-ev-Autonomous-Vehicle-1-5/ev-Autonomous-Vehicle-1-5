/**
 * @file geometry.hpp
 * @brief 2D 기하학 유틸리티 함수 모음 (inline 헤더 전용)
 *
 * planning_mr_ver 기반 + regress_direction() 추가.
 *
 * ──────────────────────────────────────────────────────────────
 * [파이프라인 내 역할]
 *
 *   이 헤더는 chaining_mr_ver 패키지의 **모든 모듈**에서 공통으로
 *   사용하는 2D 기하학 유틸리티를 모아놓은 파일이다.
 *   inline 함수로만 구성되어 있어 별도의 .cpp 컴파일 없이
 *   헤더만 include하면 바로 사용할 수 있다.
 *
 *   주요 사용처:
 *   - MagneticPlanner : dot2, cross2, norm, normalize (헤딩 계산/회전)
 *   - PathPostprocessor: dist, resample_polyline, polyline_tangents, heading
 *   - SafetyChecker   : dist, cross2 (곡률 계산)
 *   - DirectionChainer: dot2, dist_sq (그래프 엣지 가중치, 시드 탐색)
 * ──────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_MR_VER__COMMON__GEOMETRY_HPP_
#define CHAINING_MR_VER__COMMON__GEOMETRY_HPP_

#include "chaining_mr_ver/common/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chaining_mr_ver
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

/**
 * @brief 두 점 사이의 거리 제곱 (sqrt 생략 → 성능 최적화)
 *
 * 수학적 정의:
 *   dist_sq(a, b) = (a.x - b.x)² + (a.y - b.y)²
 *
 * dist()와 달리 sqrt를 하지 않으므로 비교 연산에 유리하다.
 * 예: "dist(a,b) < threshold" 대신 "dist_sq(a,b) < threshold²"
 *
 * 사용처:
 *   - DirectionChainer: 시드 탐색 시 가장 가까운 좌/우 점을 찾을 때
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
// Point2D를 수학적 벡터처럼 사용할 수 있도록 사칙연산을 정의한다.
// 예: Point2D result = a + s * (b - a);  → 선형 보간과 동일

/** @brief 벡터 덧셈: (a.x+b.x, a.y+b.y) */
inline Point2D operator+(const Point2D & a, const Point2D & b)
{
  return {a.x + b.x, a.y + b.y};
}

/** @brief 벡터 뺄셈: a에서 b로 향하는 변위 벡터 = b - a */
inline Point2D operator-(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

/** @brief 스칼라 * 벡터 (왼쪽 곱): 벡터를 s배 스케일링 */
inline Point2D operator*(double s, const Point2D & v)
{
  return {s * v.x, s * v.y};
}

/** @brief 벡터 * 스칼라 (오른쪽 곱): 위와 동일, 교환법칙 지원 */
inline Point2D operator*(const Point2D & v, double s)
{
  return {v.x * s, v.y * s};
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
 * @brief 90도 반시계 방향(CCW) 회전
 *
 * 수학적 정의 (2D 회전 행렬, θ = +90°):
 *   | cos90  -sin90 | | x |   | -y |
 *   | sin90   cos90 | | y | = |  x |
 *
 * 즉, (x, y) → (-y, x)
 *
 * 용도:
 *   - 법선벡터(normal) 계산: 접선을 90° 회전하면 왼쪽 법선이 된다
 *   - 차선 오프셋 계산 시 진행 방향의 수직 방향을 구할 때 사용
 */
inline Point2D rotate90(const Point2D & v)
{
  return {-v.y, v.x};
}

/**
 * @brief 90도 시계 방향(CW) 회전
 *
 * 수학적 정의 (2D 회전 행렬, θ = -90°):
 *   | cos(-90)  -sin(-90) | | x |   |  y |
 *   | sin(-90)   cos(-90) | | y | = | -x |
 *
 * 즉, (x, y) → (y, -x)
 *
 * 용도:
 *   - rotate90()과 반대 방향의 법선벡터(오른쪽 법선) 계산
 */
inline Point2D rotate90_cw(const Point2D & v)
{
  return {v.y, -v.x};
}

// ============================================================================
// 각도 처리 함수
// ============================================================================

/**
 * @brief 각도를 [-π, +π) 범위로 정규화 (wrapping)
 *
 * 알고리즘:
 *   1) angle + π 를 2π로 나머지 연산 → [0, 2π) 범위로 변환
 *   2) 음수일 경우 2π 더하기 (fmod가 음수 반환할 수 있으므로)
 *   3) π를 빼서 [-π, +π) 범위로 최종 변환
 *
 * 예시:
 *   wrap_pi(3π)   → π        (3π - 2π = π)
 *   wrap_pi(-3π)  → -π       (-3π + 2π = -π → +2π = π → -π)
 *   wrap_pi(0.5)  → 0.5      (변화 없음)
 *
 * 용도:
 *   - 각도 차이 계산 시 결과를 한 바퀴 이내로 제한
 *   - 자율주행에서 heading 오차를 ±180° 이내로 표현
 */
inline double wrap_pi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

/**
 * @brief 두 각도 사이의 최단 차이 (부호 있음, [-π, +π))
 *
 * 수학적 정의:
 *   angle_diff(a, b) = wrap_pi(b - a)
 *
 * 의미:
 *   - 결과 > 0 → a에서 b로 반시계 방향이 최단
 *   - 결과 < 0 → a에서 b로 시계 방향이 최단
 *
 * 주의: "a에서 b까지"의 차이이므로 인자 순서가 중요하다.
 */
inline double angle_diff(double a, double b)
{
  return wrap_pi(b - a);
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
 * @brief 폴리라인의 총 길이 계산
 *
 * 수학적 정의:
 *   L = Σ dist(pts[i-1], pts[i])  (i = 1, ..., N-1)
 *
 * 연속된 점들 사이의 유클리드 거리를 모두 더한 값.
 * 직선이 아닌 꺾인 경로의 "도로 위 거리"(arc length)에 해당한다.
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

// ============================================================================
// PCA 기반 방향벡터 추정
// ============================================================================

/**
 * @brief 최근 n개 점에 대한 PCA 주성분 방향벡터 계산
 *
 * ── PCA(Principal Component Analysis)란? ──
 *
 *   데이터 점들의 "가장 큰 분산 방향"을 찾는 통계적 기법이다.
 *   2D에서 점들이 길쭉하게 늘어서 있다면, 그 늘어선 방향이
 *   주성분(제1주성분, PC1)이 된다.
 *
 *   이 함수에서는 chain에 추가된 최근 점들의 배열 방향을
 *   PCA로 분석하여 "차선이 향하고 있는 방향"을 추정한다.
 *
 * ── 알고리즘 상세 ──
 *
 *   Step 1. 최근 n개 점 선택 (pts[total-n] ~ pts[total-1])
 *
 *   Step 2. 평균(중심) 계산
 *     μx = (1/n) Σ xi,  μy = (1/n) Σ yi
 *
 *   Step 3. 2x2 공분산 행렬 C 구성 (중심 이동 후)
 *     C = | cxx  cxy |   =  | Σ(xi-μx)²        Σ(xi-μx)(yi-μy) |
 *         | cxy  cyy |      | Σ(xi-μx)(yi-μy)   Σ(yi-μy)²       |
 *
 *     공분산 행렬은 "점들이 어느 방향으로 얼마나 퍼져 있는지"를 나타낸다.
 *     - cxx가 크면 → x 방향으로 많이 퍼져 있음
 *     - cyy가 크면 → y 방향으로 많이 퍼져 있음
 *     - cxy가 크면 → x,y 방향이 함께 증가하는 경향 (대각선 방향)
 *
 *   Step 4. 2x2 대칭 행렬의 고유값(eigenvalue) 해석적 계산
 *     고유값 공식 (특성방정식 det(C - λI) = 0 에서 유도):
 *       λ₁,₂ = (cxx + cyy)/2 ± √(((cxx - cyy)/2)² + cxy²)
 *
 *     여기서:
 *       trace_half = (cxx + cyy) / 2  ← 대각합의 절반
 *       diff_half  = (cxx - cyy) / 2  ← 대각 차이의 절반
 *       disc = √(diff_half² + cxy²)   ← 판별식
 *
 *       λ₁ = trace_half + disc  (최대 고유값 → 주성분 방향)
 *       λ₂ = trace_half - disc  (최소 고유값 → 수직 방향)
 *
 *   Step 5. 최대 고유값 λ₁에 대응하는 고유벡터 계산
 *     고유벡터 공식 (C * v = λ₁ * v 에서 유도):
 *       v₁ = (λ₁ - cyy, cxy) 방향, 정규화하여 단위벡터로 만듦
 *       → 이 벡터가 점들이 가장 많이 퍼진 방향 = chain 진행 방향
 *
 *     만약 (λ₁ - cyy, cxy)가 영벡터이면:
 *       대안으로 (cxy, λ₁ - cxx) 사용
 *       → 이 경우도 영벡터면 hint_dir 사용 (퇴화 케이스)
 *
 *   Step 6. 부호 보정
 *     PCA는 방향만 알려주고 "앞/뒤" 구분은 못한다.
 *     (고유벡터는 ±v 모두 유효하므로)
 *     → hint_dir과 내적하여 같은 방향이 되도록 부호 반전
 *
 * ── 시각적 예시 ──
 *
 *        . .  .  .              ← 최근 n개의 chain 점들
 *      .               .       ← 이 점들이 늘어선 방향이
 *   .                     .    ← PCA 주성분(PC1) = 반환값
 *   ←─────────────────────→
 *        PC1 (주성분 방향)
 *
 * @param pts       chain에 추가된 점들 (전체 벡터)
 * @param n         사용할 최근 점 수 (min_regress_pts ~ max_regress_pts)
 * @param hint_dir  chain 진행 방향 힌트 (부호 보정용, 이전 프레임의 방향 등)
 * @return 정규화된 방향 단위벡터 (|v| = 1)
 */
inline Point2D regress_direction(
  const std::vector<Point2D> & pts,
  int n,
  const Point2D & hint_dir)
{
  const int total = static_cast<int>(pts.size());
  // 점이 2개 미만이면 PCA 불가 → hint_dir을 정규화해서 반환
  if (total < 2 || n < 2) {
    return normalize(hint_dir);
  }

  // ── Step 1: 최근 n개 점 범위 결정 ──
  // use_n = min(n, total): 요청된 n이 전체 점 수보다 크면 전체 사용
  // start = total - use_n: 배열에서 사용할 시작 인덱스
  // 예: pts = [P0, P1, P2, P3, P4], n=3 → start=2, 범위 [P2, P3, P4]
  const int use_n = std::min(n, total);
  const int start = total - use_n;

  // ── Step 2: 평균(중심점) 계산 ──
  // μ = (μx, μy) = 점들의 무게중심
  double mx = 0.0, my = 0.0;
  for (int i = start; i < total; ++i) {
    mx += pts[i].x;
    my += pts[i].y;
  }
  mx /= use_n;
  my /= use_n;

  // ── Step 3: 공분산 행렬 요소 계산 ──
  // 각 점을 중심(μ)으로부터의 편차(dx, dy)로 변환한 뒤
  // cxx = Σ(dx²), cxy = Σ(dx*dy), cyy = Σ(dy²)
  double cxx = 0.0, cxy = 0.0, cyy = 0.0;
  for (int i = start; i < total; ++i) {
    const double dx = pts[i].x - mx;
    const double dy = pts[i].y - my;
    cxx += dx * dx;  // x 방향 분산
    cxy += dx * dy;  // x-y 공분산 (대각선 방향 상관관계)
    cyy += dy * dy;  // y 방향 분산
  }

  // ── Step 4: 2x2 대칭 행렬의 고유값 계산 ──
  // 특성방정식: λ² - (cxx+cyy)λ + (cxx*cyy - cxy²) = 0
  // 근의 공식 → λ = trace_half ± disc
  const double trace_half = (cxx + cyy) * 0.5;  // 대각합의 절반
  const double diff_half = (cxx - cyy) * 0.5;   // 대각 차이의 절반
  const double disc = std::sqrt(diff_half * diff_half + cxy * cxy);  // 판별식

  Point2D dir;
  if (disc < 1e-12) {
    // 판별식 ≈ 0 → 점들이 한 곳에 모여 있거나 완전히 등방성(원형 분포)
    // → 방향을 결정할 수 없으므로 hint_dir 사용
    return normalize(hint_dir);
  }

  // ── Step 5: 최대 고유값에 대응하는 고유벡터 계산 ──
  // λ₁ = trace_half + disc (최대 고유값 = 가장 큰 분산 방향)
  // 고유벡터: (C - λ₁I) * v = 0 에서
  //   (cxx - λ₁) * ex + cxy * ey = 0
  //   → ex/ey = -cxy / (cxx - λ₁) = (λ₁ - cxx) / cxy (비율만 필요)
  // 또는 동치로: v = (λ₁ - cyy, cxy)
  const double lambda1 = trace_half + disc;
  double ex = lambda1 - cyy;  // 고유벡터의 x 성분
  double ey = cxy;            // 고유벡터의 y 성분
  const double en = std::sqrt(ex * ex + ey * ey);
  if (en < 1e-12) {
    // 첫 번째 형태가 영벡터인 경우 (cyy가 매우 클 때)
    // 대안 형태 사용: v = (cxy, λ₁ - cxx)
    ex = cxy;
    ey = lambda1 - cxx;
    const double en2 = std::sqrt(ex * ex + ey * ey);
    if (en2 < 1e-12) return normalize(hint_dir);  // 퇴화 케이스
    ex /= en2;
    ey /= en2;
  } else {
    // 정규화 (단위벡터로 변환)
    ex /= en;
    ey /= en;
  }

  dir = {ex, ey};

  // ── Step 6: 부호 보정 ──
  // PCA 고유벡터는 ±v 모두 유효 (방향선만 결정, 방향 미결정)
  // hint_dir과 내적이 음수이면 → 반대 방향이므로 부호 반전
  // 이렇게 하면 chain이 진행하는 방향과 일치하게 된다
  if (dot2(dir, hint_dir) < 0.0) {
    dir = {-dir.x, -dir.y};
  }

  return dir;
}

}  // namespace chaining_mr_ver

#endif  // CHAINING_MR_VER__COMMON__GEOMETRY_HPP_
