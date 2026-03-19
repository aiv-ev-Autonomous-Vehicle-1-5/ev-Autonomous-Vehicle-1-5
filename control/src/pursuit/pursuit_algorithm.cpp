// ============================================================================
// pursuit_algorithm.cpp — Pure Pursuit 핵심 알고리즘 구현
// ============================================================================

#include "pp_controller_cpp/pursuit/pursuit_algorithm.hpp"
#include "pp_controller_cpp/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pp_controller_cpp
{
namespace pursuit
{

// ---------------------------------------------------------------------------
// find_nearest_index: 원점(0,0)에 가장 가까운 경로점 탐색
// ---------------------------------------------------------------------------
// 이론적으로 path[0]이 가장 가까워야 하지만, planning 모듈의
// 출력 타이밍에 따라 그렇지 않을 수 있으므로 전체 검색한다.
// ---------------------------------------------------------------------------
size_t find_nearest_index(const std::vector<geometry_msgs::msg::Point> & pts)
{
  size_t nearest_i = 0;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < pts.size(); ++i) {
    const double d2 = pts[i].x * pts[i].x + pts[i].y * pts[i].y;
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest_i = i;
    }
  }

  return nearest_i;
}

// ---------------------------------------------------------------------------
// compute_dynamic_lookahead: 속도 적응형 lookahead 거리
// ---------------------------------------------------------------------------
// 저속(커브) → 짧은 lookahead (정밀 추적)
// 고속(직선) → 긴 lookahead (안정성)
// ---------------------------------------------------------------------------
double compute_dynamic_lookahead(
  double speed,
  double lookahead_min,
  double lookahead_max,
  double speed_gain)
{
  const double unclamped = lookahead_min + speed_gain * std::max(0.0, speed);
  return std::clamp(unclamped, lookahead_min, lookahead_max);
}

// ---------------------------------------------------------------------------
// compute_preview_curvature: 전방 곡률 분석
// ---------------------------------------------------------------------------
// nearest_i부터 preview_distance만큼 앞의 경로를 분석하여 최대 곡률을 반환.
// 3점 외적 기반 곡률 추정:
//   kappa = 2 * |cross(AB, BC)| / (|AB| * |BC| * |AC|)
// 이 값이 크면 급커브 → 미리 감속하는 용도로 사용.
// ---------------------------------------------------------------------------
double compute_preview_curvature(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double preview_distance)
{
  if (pts.size() < 3 || nearest_i + 2 >= pts.size()) {
    return 0.0;
  }

  // preview_distance 만큼의 구간 끝 인덱스 결정
  size_t end_i = nearest_i;
  double acc = 0.0;
  while (end_i + 1 < pts.size() && acc < preview_distance) {
    const auto & p0 = pts[end_i];
    const auto & p1 = pts[end_i + 1];
    acc += norm2d(p1.x - p0.x, p1.y - p0.y);
    ++end_i;
  }

  if (end_i < nearest_i + 2) {
    return 0.0;
  }

  // 구간 내 3점 세트에서 최대 곡률 탐색
  double max_abs_kappa = 0.0;
  for (size_t i = nearest_i; i + 2 <= end_i; ++i) {
    const auto & a = pts[i];
    const auto & b = pts[i + 1];
    const auto & c = pts[i + 2];

    const double ab = norm2d(b.x - a.x, b.y - a.y);
    const double bc = norm2d(c.x - b.x, c.y - b.y);
    const double ac = norm2d(c.x - a.x, c.y - a.y);
    const double denom = ab * bc * ac;
    if (denom < 1e-9) {
      continue;
    }

    const double cross =
      (b.x - a.x) * (c.y - b.y) -
      (b.y - a.y) * (c.x - b.x);
    const double kappa = 2.0 * std::abs(cross) / denom;
    max_abs_kappa = std::max(max_abs_kappa, kappa);
  }

  return max_abs_kappa;
}

// ---------------------------------------------------------------------------
// compute_speed_target: 곡률 → 목표 속도 변환
// ---------------------------------------------------------------------------
// v = sqrt(lateral_accel_limit / kappa)
// 곡률이 클수록 속도를 낮춰 횡가속도를 lateral_accel_limit 이내로 유지.
// ---------------------------------------------------------------------------
double compute_speed_target(
  double abs_kappa,
  double v_min,
  double v_max,
  double lateral_accel_limit)
{
  if (v_max <= 0.0) {
    return 0.0;
  }
  if (abs_kappa <= 1e-6) {
    return v_max;
  }

  const double curvature_limited_speed = std::sqrt(lateral_accel_limit / abs_kappa);
  return std::clamp(curvature_limited_speed, v_min, v_max);
}

// ---------------------------------------------------------------------------
// compute_target_relative: arc-length 기반 lookahead 목표점 계산
// ---------------------------------------------------------------------------
//
// [알고리즘]
//   nearest_i부터 경로를 따라가며 연속된 점 사이의 거리를 누적.
//   누적 거리가 lookahead_dist 이상이 되는 첫 번째 점을 목표로 선택.
//   경로 끝까지 가도 충족 못하면 마지막 점을 사용 (폴백).
//
//   [시각적 설명]
//        목표점 (tx, ty)
//           *
//          /|
//    Ld  /  |  ty (횡방향 오프셋)
//       /   |
//      /    |
//     *-----+
//   (0,0)  tx
//   차량    (전방 거리)
//
// ---------------------------------------------------------------------------
bool compute_target_relative(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double lookahead_dist,
  double & tx,
  double & ty,
  double & Ld_used)
{
  // 최소 2개의 점이 필요 (1개로는 방향을 결정할 수 없음)
  if (pts.size() < 2) {
    return false;
  }

  // nearest_i부터 경로를 따라가며 누적 거리로 lookahead 찾기
  double acc = 0.0;
  for (size_t i = nearest_i; i + 1 < pts.size(); ++i) {
    const double x0 = pts[i].x;
    const double y0 = pts[i].y;
    const double x1 = pts[i + 1].x;
    const double y1 = pts[i + 1].y;

    acc += norm2d(x1 - x0, y1 - y0);

    if (acc >= lookahead_dist) {
      tx = x1;
      ty = y1;
      // Ld_used는 경로를 따른 거리(acc)가 아닌,
      // 차량 원점에서 목표점까지의 "직선 거리"를 사용한다.
      // Pure Pursuit 공식에서 Ld는 직선 거리여야 하기 때문.
      Ld_used = norm2d(tx, ty);
      return true;
    }
  }

  // 경로 끝까지 가도 lookahead를 충족 못하면 마지막 점 사용 (폴백)
  // 이 경우 Ld_used가 목표 lookahead보다 작아질 수 있어 조향이 예민해질 수 있다.
  tx = pts.back().x;
  ty = pts.back().y;
  Ld_used = norm2d(tx, ty);
  return true;
}

// ---------------------------------------------------------------------------
// compute_remaining_length: nearest_i부터 경로 끝까지 남은 arc length [m]
// ---------------------------------------------------------------------------
double compute_remaining_length(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i)
{
  double len = 0.0;
  for (size_t i = nearest_i + 1; i < pts.size(); ++i) {
    len += norm2d(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y);
  }
  return len;
}

// ---------------------------------------------------------------------------
// compute_path_end_speed: 남은 거리 안에 멈출 수 있는 최대 속도
// ---------------------------------------------------------------------------
// 제동거리 공식 d = v²/(2a) 를 역으로: v = √(2a × d)
// ---------------------------------------------------------------------------
double compute_path_end_speed(
  double remaining_length,
  double decel_rate,
  double v_min,
  double v_max)
{
  if (remaining_length <= 0.0 || decel_rate <= 0.0) return v_min;
  double v = std::sqrt(2.0 * decel_rate * remaining_length);
  return std::clamp(v, v_min, v_max);
}

// ---------------------------------------------------------------------------
// rate_limit_speed: 가감속 rate limit 적용
// ---------------------------------------------------------------------------
// 가속: v_cmd = min(target, prev + accel_rate * dt)
// 감속: v_cmd = max(target, prev - decel_rate * dt)
// 급격한 속도 변화를 방지하여 부드러운 주행을 보장한다.
// ---------------------------------------------------------------------------
double rate_limit_speed(
  double target_speed,
  double last_cmd_speed,
  double dt,
  double accel_rate,
  double decel_rate)
{
  if (dt <= 0.0) {
    return target_speed;
  }

  if (target_speed >= last_cmd_speed) {
    return std::min(target_speed, last_cmd_speed + accel_rate * dt);
  }
  return std::max(target_speed, last_cmd_speed - decel_rate * dt);
}

// ---------------------------------------------------------------------------
// compute_steering: Pure Pursuit 조향각 계산
// ---------------------------------------------------------------------------
// 핵심 공식:
//   kappa = 2 * ty / Ld^2        (곡률)
//   delta = atan(L * kappa)      (조향각)
//   delta = clamp(delta, ±delta_max)  (하드웨어 제한)
//
// ty > 0 → 좌회전 (delta > 0)
// ty < 0 → 우회전 (delta < 0)
// ---------------------------------------------------------------------------
double compute_steering(
  double ty,
  double Ld_used,
  double wheelbase,
  double delta_max)
{
  const double kappa = (2.0 * ty) / (Ld_used * Ld_used);
  double delta = std::atan(wheelbase * kappa);
  return std::clamp(delta, -delta_max, delta_max);
}

}  // namespace pursuit
}  // namespace pp_controller_cpp
