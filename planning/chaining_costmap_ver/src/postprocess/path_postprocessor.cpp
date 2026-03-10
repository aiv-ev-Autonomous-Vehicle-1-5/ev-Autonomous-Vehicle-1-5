/**
 * @file path_postprocessor.cpp
 * @brief 경로 후처리기 — 구현부
 *
 * MagneticPlanner가 출력한 원시(raw) greedy 경로를
 * 차량 제어기에 적합한 깨끗하고 부드러운 경로로 변환한다.
 *
 * [파이프라인 전체 흐름]
 *
 *   MagneticPlanner 출력 (격자 기반 지그재그 경로)
 *       │
 *       ├─① prune()    : 직선 구간의 불필요한 중간점 제거
 *       ├─② smooth()   : 이동 평균으로 잔여 꺾임 완화
 *       ├─③ resample() : 등간격 점 재배치 (geometry.hpp의 resample_polyline)
 *       └─④ yaw 계산   : 접선 벡터 → atan2 헤딩 각도
 *       │
 *       ▼
 *   PostprocessResult { path[], yaw[], valid }
 *       → SafetyChecker에서 곡률/속도 체크 후 차량 제어기로 전달
 */
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"    // dist, resample_polyline, polyline_tangents, heading

#include <algorithm>  // std::max, std::min
#include <cmath>      // std::abs, std::atan2
#include <iostream>
#include <numeric>
#include <limits>
#include <string>

namespace chaining_costmap_ver
{

namespace
{

int normalize_window_odd(int window)
{
  if (window < 1) {
    return 1;
  }
  if (window % 2 == 0) {
    return window + 1;
  }
  return window;
}


void warn_adjust(const std::string & name, const std::string & msg)
{
  std::cerr << "[Postprocess][WARN] " << name << " " << msg << std::endl;
}

void ensure_yaw_size_matches_path(
  std::vector<double> & yaw,
  size_t target_size)
{
  if (target_size == 0) {
    yaw.clear();
    return;
  }

  if (yaw.empty()) {
    yaw.assign(target_size, 0.0);
    return;
  }

  if (yaw.size() < target_size) {
    yaw.resize(target_size, yaw.back());
  } else if (yaw.size() > target_size) {
    yaw.resize(target_size);
  }
}

double path_length(const std::vector<Point2D> & pts)
{
  if (pts.size() < 2) {
    return 0.0;
  }

  double total = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    total += dist(pts[i - 1], pts[i]);
  }
  return total;
}

std::vector<Point2D> remove_near_duplicates(
  const std::vector<Point2D> & pts,
  double min_dist)
{
  if (pts.empty()) {
    return pts;
  }

  if (!std::isfinite(min_dist) || min_dist <= 0.0) {
    min_dist = 1e-4;
  }

  std::vector<Point2D> out;
  out.reserve(pts.size());
  out.push_back(pts.front());

  for (size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - out.back().x;
    const double dy = pts[i].y - out.back().y;
    const double ds = std::hypot(dx, dy);

    if (ds >= min_dist) {
      out.push_back(pts[i]);
    }
  }

  if (out.size() == 1 && pts.size() > 1) {
    const double dx = pts.back().x - out.back().x;
    const double dy = pts.back().y - out.back().y;
    if (std::hypot(dx, dy) > 0.0) {
      out.push_back(pts.back());
    }
  }

  return out;
}

}  // namespace

// ============================================================================
// ① Prune 단계 — Douglas-Peucker 유사 탐욕적 경로 단순화
// ============================================================================
//
// [목적]
//   Planner의 출력은 costmap grid cell 하나하나를 따라가므로
//   점 수가 매우 많고, 거의 직선인 구간에도 불필요한 중간점이 가득하다.
//   이 함수는 "직선으로 간주해도 될 구간"의 중간점을 모두 제거하여
//   점 수를 대폭 줄인다.
//
// [알고리즘 상세]
//   현재 앵커(anchor) 점 i에서 출발하여:
//   1) 가장 먼 점 j = (끝점)부터 역순으로 시도
//   2) i→j 직선을 긋고, 사이의 모든 중간점 k에 대해
//      수직 편차(perpendicular distance)를 계산
//   3) 모든 중간점의 편차 ≤ max_dev 이면 → shortcut 성공, j를 다음 앵커로 채택
//   4) 편차 초과 점이 있으면 → j를 하나 줄여서 재시도
//   5) 최악의 경우 j = i+1 (바로 다음 점)이 채택됨
//
//   이 방식은 Douglas-Peucker처럼 재귀적 분할이 아니라,
//   앞에서부터 탐욕적(greedy)으로 가장 먼 shortcut을 찾는 방식이다.
//   결과적으로 O(n²) 최악 복잡도이지만, 실제로는 shortcut이 빨리
//   성공하므로 매우 빠르다.
//
// [수직 편차(perpendicular distance) 계산 원리]
//
//   점 a(앵커)에서 점 b(후보 끝점)로의 방향 단위벡터를 dir이라 하면:
//     dir = (b - a) / |b - a|
//
//   중간점 k의 벡터 (k - a) = (dx, dy)에 대해:
//     수직 편차 = |dx * dir.y - dy * dir.x|
//
//   이것은 2D 외적(cross product)의 절댓값으로,
//   (k - a)를 dir에 수직인 성분으로 투영한 크기와 같다.
//
//            k
//            ·
//           /|
//          / | ← perp (수직 편차)
//         /  |
//   a ───────────── b
//        dir 방향
//
// ============================================================================
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev)
{
  // 점이 2개 이하면 단순화할 것이 없으므로 그대로 반환
  if (pts.size() <= 2) return pts;

  std::vector<Point2D> result;
  result.push_back(pts.front());  // 시작점은 항상 보존

  size_t i = 0;  // 현재 앵커(anchor) 인덱스
  while (i < pts.size() - 1) {
    // 기본값: 바로 다음 점 (shortcut 실패 시 최소 진행)
    size_t best_j = i + 1;

    // 가장 먼 점(끝점)부터 역순으로 shortcut 시도
    // → 가능한 한 많은 중간점을 한 번에 건너뛰기 위함
    for (size_t j = pts.size() - 1; j > i + 1; --j) {
      const Point2D & a = pts[i];  // 앵커 점
      const Point2D & b = pts[j];  // 후보 끝점
      const double ab_len = dist(a, b);  // 앵커↔후보 사이 거리

      bool can_shortcut = true;

      // 두 점이 사실상 같은 위치이면 shortcut 불가
      // (방향 벡터를 구할 수 없으므로)
      if (ab_len < 1e-12) {
        can_shortcut = false;
      } else {
        // a→b 방향의 단위벡터 계산
        const Point2D dir = {(b.x - a.x) / ab_len, (b.y - a.y) / ab_len};

        // i와 j 사이의 모든 중간점 k에 대해 수직 편차 검사
        for (size_t k = i + 1; k < j; ++k) {
          // 중간점 k에서 앵커 a까지의 벡터
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;

          // 수직 편차 = |2D 외적| = |(k-a) × dir|
          // 기하학적 의미: 점 k에서 직선 a→b까지의 수직 거리
          const double perp = std::abs(dx * dir.y - dy * dir.x);

          // 편차가 허용치를 초과하면 이 shortcut은 불가
          if (perp > max_dev) {
            can_shortcut = false;
            break;  // 나머지 중간점은 확인할 필요 없음
          }
        }
      }

      // 모든 중간점이 허용 편차 이내 → shortcut 성공!
      // 가장 먼 j부터 시도했으므로 이것이 최대 shortcut
      if (can_shortcut) {
        best_j = j;
        break;  // 더 가까운 j를 시도할 필요 없음
      }
    }

    // shortcut 끝점을 결과에 추가하고, 앵커를 이동
    result.push_back(pts[best_j]);
    i = best_j;
  }

  return result;
}

// ============================================================================
// ② Smooth 단계 — 이동 평균(Moving Average) 필터
// ============================================================================
//
// [목적]
//   prune 이후에도 남아있는 경미한 꺾임(각도 변화)을 완화한다.
//   prune는 "점을 제거"하는 반면, smooth는 "점을 이동"시켜
//   더 부드러운 곡선을 만든다.
//
// [이동 평균 원리]
//   각 중간점 i에 대해, 윈도우 [i - half, i + half] 범위의
//   이웃 점들의 좌표를 산술 평균하여 새 좌표로 대체한다.
//
//   예: window = 5, half = 2 일 때, 점 i의 새 좌표:
//     new_x[i] = (x[i-2] + x[i-1] + x[i] + x[i+1] + x[i+2]) / 5
//     new_y[i] = (y[i-2] + y[i-1] + y[i] + y[i+1] + y[i+2]) / 5
//
//   경계 근처에서는 윈도우가 잘리므로 실제 평균 대상 수가 줄어든다.
//   예: i = 1, half = 2 → 범위 [0, 3], count = 4 (5가 아님)
//
// [시작점/끝점 보존 이유]
//   - 시작점(front): 차량의 현재 위치 → 이동시키면 현재 위치와 불일치
//   - 끝점(back):    경로의 목표 지점 → 이동시키면 목표 도달 불가
//   - 따라서 이 두 점은 평균 처리에서 제외하고 원래 좌표를 그대로 유지
//
// [왜 이동 평균인가?]
//   이동 평균은 가장 단순한 로우패스 필터(low-pass filter)이다.
//   고주파 성분(격자 단위 지그재그)을 제거하면서도
//   저주파 성분(도로의 대략적 커브)은 보존한다.
//   계산이 O(n·window)로 매우 빠르고 구현이 간단하다.
//
// ============================================================================
std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window)
{
  // 점이 2개 이하이거나 윈도우가 1 이하이면 스무딩 불필요
  if (pts.size() <= 2 || window <= 1) return pts;

  const int half = window / 2;  // 양쪽으로 half개씩 참조 (예: window=5 → half=2)
  const int n = static_cast<int>(pts.size());

  std::vector<Point2D> result(pts.size());
  result.front() = pts.front();  // 시작점 보존 (차량 현재 위치)
  result.back() = pts.back();    // 끝점 보존 (목표 지점)

  // 중간점만 이동 평균 적용 (index 1 ~ n-2)
  for (int i = 1; i < n - 1; ++i) {
    double sx = 0.0, sy = 0.0;  // 좌표 합산 누적기
    int count = 0;

    // 윈도우 범위 계산 — 배열 범위를 벗어나지 않도록 클램핑
    const int lo = std::max(0, i - half);      // 윈도우 좌측 경계
    const int hi = std::min(n - 1, i + half);  // 윈도우 우측 경계

    // 윈도우 내 모든 점의 좌표를 합산
    for (int j = lo; j <= hi; ++j) {
      sx += pts[j].x;
      sy += pts[j].y;
      ++count;
    }

    // 산술 평균으로 새 좌표 결정
    result[i] = {sx / count, sy / count};
  }

  return result;
}

// ============================================================================
// process() — 4단계 파이프라인 메인 함수
// ============================================================================
//
// [호출 흐름]
//   lc_planner_node.cpp의 planning 콜백에서 호출된다:
//     MagneticPlanner::plan() → raw_path (Point2D 배열)
//     PathPostprocessor::process(raw_path, ...) → PostprocessResult
//     SafetyChecker::check(result, ...) → PlannerState
//
// [각 단계의 역할과 순서가 중요한 이유]
//
//   ① prune 먼저: 점 수를 줄여야 smooth 비용이 감소하고,
//      불필요한 점에 의한 노이즈 증폭을 방지
//
//   ② smooth 다음: prune 후 남은 꺾임을 부드럽게 만듦
//
//   ③ resample이 smooth 뒤에 오는 이유:
//      smooth가 점의 위치를 이동시키면 인접 점 간 거리가 불균등해진다.
//      (밀집 구간은 더 밀집, 희소 구간은 더 희소해질 수 있음)
//      제어기(Pure Pursuit 등)는 등간격 waypoint를 전제로 설계되므로,
//      반드시 smooth 후에 등간격 리샘플링을 수행해야 한다.
//
//   ④ yaw 마지막: 최종 경로 좌표가 확정된 후에야 정확한 헤딩을 계산할 수 있음
//
// ============================================================================
// ============================================================================
// ②½ Curvature Clamp — 최대 곡률 제한
// ============================================================================
//
// [목적]
//   차량의 최소 회전 반경(R_min)을 보장하기 위해,
//   Menger 곡률이 kappa_max(= 1/R_min)를 초과하는 지점의
//   중간점을 곡률 원 중심 방향으로 밀어 곡률을 낮춘다.
//
// [Menger 곡률]
//   세 점 (P_{i-1}, P_i, P_{i+1})이 이루는 삼각형의 외접원 반경 R에서:
//     kappa = 1/R = 4·Area / (|a|·|b|·|c|)
//   여기서 Area = 0.5 * |cross(a, b)|, a = P_i - P_{i-1}, b = P_{i+1} - P_{i-1}
//
// [곡률 감소 방법]
//   kappa > kappa_max인 triplet에서, 중간점 P_i를
//   현재 위치에서 곡률 원 중심 방향으로 약간 이동시킨다.
//   이동량 = (R_desired - R_current) 방향의 fraction.
//   반복 적용(3회)으로 수렴시킨다.
//
// ============================================================================
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
      double ratio = 1.0 - 0.95*(kappa_max / kappa);
      ratio = std::min(ratio, 0.7);

      result[i].x = p1.x + ratio * (mx - p1.x);
      result[i].y = p1.y + ratio * (my - p1.y);
    }

    if (violations == 0) break;  // 모든 곡률이 한계 이내 → 수렴 완료
  }

  return result;
}

double PathPostprocessor::wrap_angle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

std::vector<double> PathPostprocessor::compute_yaw_from_path(
  const std::vector<Point2D> & pts,
  double min_segment_len)
{
  std::vector<double> yaw;

  if (pts.empty()) {
    return yaw;
  }

  yaw.resize(pts.size(), 0.0);

  // 점이 1개면 진행 방향을 계산할 수 없으므로 0으로 둠
  if (pts.size() == 1) {
    return yaw;
  }

  if (!std::isfinite(min_segment_len) || min_segment_len <= 0.0) {
    min_segment_len = 0.05;
  }

  double last_valid_yaw = 0.0;
  bool has_valid_yaw = false;

  for (size_t i = 0; i < pts.size(); ++i) {
    bool found = false;

    // i에서 min_segment_len 이상 떨어진 "다음 유효 점" 찾기
    for (size_t j = i + 1; j < pts.size(); ++j) {
      const double dx = pts[j].x - pts[i].x;
      const double dy = pts[j].y - pts[i].y;
      const double ds = std::sqrt(dx * dx + dy * dy);

      if (ds >= min_segment_len) {
        yaw[i] = std::atan2(dy, dx);
        last_valid_yaw = yaw[i];
        has_valid_yaw = true;
        found = true;
        break;
      }
    }

    if (!found) {
      // 유효 segment를 못 찾은 경우 fallback
      if (has_valid_yaw) {
        yaw[i] = last_valid_yaw;
      } else {
        yaw[i] = 0.0;
      }
    }
  }

  // 마지막 yaw는 직전 yaw로 채움
  if (yaw.size() >= 2) {
    yaw.back() = yaw[yaw.size() - 2];
  }

  return yaw;
}

std::vector<double> PathPostprocessor::smooth_yaw(
  const std::vector<double> & yaw,
  int window)
{
  if (yaw.empty()) {
    return yaw;
  }

  window = normalize_window_odd(window);

  // yaw가 너무 짧으면 smoothing 이득이 거의 없음
  if (yaw.size() < 3 || window <= 1) {
    return yaw;
  }

  // window가 yaw size보다 크면 가능한 최대 홀수로 줄임
  if (window > static_cast<int>(yaw.size())) {
    window = static_cast<int>(yaw.size());
    if (window % 2 == 0) {
      window -= 1;
    }
    if (window < 1) {
      window = 1;
    }
  }

  if (window <= 1) {
    return yaw;
  }

  const int half = window / 2;
  std::vector<double> unwrapped = yaw;

  // unwrap
  for (size_t i = 1; i < unwrapped.size(); ++i) {
    double diff = unwrapped[i] - unwrapped[i - 1];

    while (diff > M_PI) {
      unwrapped[i] -= 2.0 * M_PI;
      diff = unwrapped[i] - unwrapped[i - 1];
    }

    while (diff < -M_PI) {
      unwrapped[i] += 2.0 * M_PI;
      diff = unwrapped[i] - unwrapped[i - 1];
    }
  }

  std::vector<double> smoothed(unwrapped.size(), 0.0);

  for (size_t i = 0; i < unwrapped.size(); ++i) {
    const int begin = std::max<int>(0, static_cast<int>(i) - half);
    const int end = std::min<int>(
      static_cast<int>(unwrapped.size()) - 1,
      static_cast<int>(i) + half);

    double sum = 0.0;
    int count = 0;

    for (int j = begin; j <= end; ++j) {
      sum += unwrapped[j];
      ++count;
    }

    smoothed[i] = sum / static_cast<double>(count);
  }

  // endpoint 처리: 시작/끝은 원본을 유지하는 방식이 더 안전함
  smoothed.front() = unwrapped.front();
  smoothed.back() = unwrapped.back();

  // wrap back
  for (auto & a : smoothed) {
    a = wrap_angle(a);
  }

  return smoothed;
}

PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds,
  double kappa_max,
  int curvature_clamp_max_iter,
  int yaw_smooth_window,
  double yaw_min_segment_len)
{
  PostprocessResult result;

  std::cout << "[Postprocess] raw_path size = " << raw_path.size() << std::endl;

  // ------------------------------------------------------------------
  // 0) 입력 path 방어
  // ------------------------------------------------------------------
  if (raw_path.empty()) {
    warn_adjust("raw_path", "is empty -> return invalid result");
    return result;
  }

  if (raw_path.size() == 1) {
    warn_adjust("raw_path", "has only 1 point -> return single-point path");
    result.path = raw_path;
    result.yaw = {0.0};
    result.valid = true;
    return result;
  }

  // ------------------------------------------------------------------
  // 1) 파라미터 자동 보정
  // ------------------------------------------------------------------
  if (!std::isfinite(prune_max_dev) || prune_max_dev < 0.0) {
    warn_adjust("prune_max_dev", "invalid -> fallback to 0.15");
    prune_max_dev = 0.15;
  }

  if (smooth_window <= 0) {
    warn_adjust("smooth_window", "invalid -> fallback to 1");
    smooth_window = 1;
  }

  const int original_smooth_window = smooth_window;
  smooth_window = normalize_window_odd(smooth_window);
  if (smooth_window != original_smooth_window) {
    warn_adjust(
      "smooth_window",
      "was even -> adjusted to odd value " + std::to_string(smooth_window));
}

  if (!std::isfinite(resample_ds) || resample_ds <= 0.0) {
    warn_adjust("resample_ds", "invalid -> fallback to 0.10");
    resample_ds = 0.10;
  }

  if (!std::isfinite(kappa_max) || kappa_max < 0.0) {
    warn_adjust("kappa_max", "invalid -> fallback to 0.0");
    kappa_max = 0.0;
  }

  if (curvature_clamp_max_iter <= 0) {
    warn_adjust("curvature_clamp_max_iter", "invalid -> fallback to 30");
    curvature_clamp_max_iter = 30;
  }

  if (yaw_smooth_window <= 0) {
    warn_adjust("yaw_smooth_window", "invalid -> fallback to 1");
    yaw_smooth_window = 1;
  }

  const int original_yaw_window = yaw_smooth_window;
  yaw_smooth_window = normalize_window_odd(yaw_smooth_window);
  if (yaw_smooth_window != original_yaw_window) {
    warn_adjust(
      "yaw_smooth_window",
      "was even -> adjusted to odd value " + std::to_string(yaw_smooth_window));
  }

  if (!std::isfinite(yaw_min_segment_len) || yaw_min_segment_len <= 0.0) {
    warn_adjust("yaw_min_segment_len", "invalid -> fallback to 0.05");
    yaw_min_segment_len = 0.05;
  }

  // ------------------------------------------------------------------
  // 2) prune
  // ------------------------------------------------------------------
  auto pruned = prune(raw_path, prune_max_dev);
  std::cout << "[Postprocess] after prune size = " << pruned.size() << std::endl;

  pruned = remove_near_duplicates(pruned, 1e-4);
  std::cout << "[Postprocess] after prune dedup size = " << pruned.size() << std::endl;
  
  if (pruned.empty()) {
    warn_adjust("prune", "result is empty -> fallback to raw_path");
    pruned = raw_path;
  }

  // ------------------------------------------------------------------
  // 3) smooth
  // ------------------------------------------------------------------
  
  std::vector<Point2D> smoothed;
  if (pruned.size() < 3 || smooth_window <= 1) {
    std::cout << "[Postprocess] skip path smoothing "
              << "(path_size=" << pruned.size()
              << ", smooth_window=" << smooth_window << ")"
              << std::endl;
    smoothed = pruned;
  } else {
    smoothed = smooth(pruned, smooth_window);
  }

  std::cout << "[Postprocess] after smooth size = " << smoothed.size() << std::endl;

  smoothed = remove_near_duplicates(smoothed, 1e-4);
  std::cout << "[Postprocess] after smooth dedup size = " << smoothed.size() << std::endl;

  if (smoothed.empty()) {
    warn_adjust("smooth", "result is empty -> fallback to pruned");
    smoothed = pruned;
  }

  // ------------------------------------------------------------------
  // 4) curvature clamp
  // ------------------------------------------------------------------
  if (kappa_max > 0.0 && smoothed.size() >= 3) {
    std::cout << "[Postprocess] before curvature clamp size = "
              << smoothed.size() << std::endl;

    smoothed = curvature_clamp(smoothed, kappa_max, curvature_clamp_max_iter);

    std::cout << "[Postprocess] after curvature clamp size = "
              << smoothed.size() << std::endl;
  }

  // ------------------------------------------------------------------
  // 5) resample
  // ------------------------------------------------------------------
  result.path = resample_polyline(smoothed, resample_ds);
  std::cout << "[Postprocess] after resample size = "
            << result.path.size() << std::endl;

  result.path = remove_near_duplicates(result.path, 1e-4);
  std::cout << "[Postprocess] after resample dedup size = "
            << result.path.size() << std::endl;

  if (result.path.empty()) {
    warn_adjust("resample", "result is empty -> fallback to smoothed");
    result.path = smoothed;
  }

  // resample 후 curvature clamp 재적용
  if (kappa_max > 0.0 && result.path.size() >= 3) {
    std::cout << "[Postprocess] before curvature clamp (resample) size = "
              << result.path.size() << std::endl;

    result.path = curvature_clamp(result.path, kappa_max, curvature_clamp_max_iter);

    std::cout << "[Postprocess] after curvature clamp (resample) size = "
              << result.path.size() << std::endl;
  }

  // ------------------------------------------------------------------
  // 6) path 길이/점 수 기반 yaw smoothing 생략 조건
  // ------------------------------------------------------------------
  if (result.path.empty()) {
    warn_adjust("result.path", "is empty after postprocess -> invalid");
    return result;
  }

  if (result.path.size() == 1) {
    warn_adjust("result.path", "has only 1 point -> yaw set to 0");
    result.yaw = {0.0};
    result.valid = true;
    return result;
  }

  // ------------------------------------------------------------------
  // 7) yaw 계산
  // ------------------------------------------------------------------
  result.yaw = compute_yaw_from_path(result.path, yaw_min_segment_len);

  // yaw/path size mismatch 방어
  if (result.yaw.size() != result.path.size()) {
    warn_adjust(
      "yaw_size",
      "mismatch with path size -> auto resize from " +
      std::to_string(result.yaw.size()) + " to " +
      std::to_string(result.path.size()));
    ensure_yaw_size_matches_path(result.yaw, result.path.size());
  }

  // path가 너무 짧으면 yaw smoothing 생략
  const double total_len = path_length(result.path);
  const bool skip_yaw_smoothing =
    (result.path.size() < 3) ||
    (total_len < std::max(resample_ds * 2.0, yaw_min_segment_len * 2.0)) ||
    (yaw_smooth_window <= 1);

  if (skip_yaw_smoothing) {
    std::cout << "[Postprocess] skip yaw smoothing "
              << "(path_size=" << result.path.size()
              << ", path_length=" << total_len
              << ", yaw_window=" << yaw_smooth_window << ")"
              << std::endl;
  } else {
    result.yaw = smooth_yaw(result.yaw, yaw_smooth_window);
  }

  // smoothing 후 size mismatch 재확인
  if (result.yaw.size() != result.path.size()) {
    warn_adjust(
      "yaw_size",
      "mismatch after smoothing -> auto resize from " +
      std::to_string(result.yaw.size()) + " to " +
      std::to_string(result.path.size()));
    ensure_yaw_size_matches_path(result.yaw, result.path.size());
  }

  // 마지막 yaw를 직전 yaw로 한 번 더 보장
  if (result.yaw.size() >= 2) {
    result.yaw.back() = result.yaw[result.yaw.size() - 2];
  }

  std::cout << "[Postprocess] yaw size = " << result.yaw.size() << std::endl;
  const double final_len = path_length(result.path);
  std::cout << "[Postprocess] final path length = " << final_len << std::endl;
  // ------------------------------------------------------------------
  // 8) 최종 validity check
  // ------------------------------------------------------------------
  if (result.path.empty()) {
    warn_adjust("result", "final path is empty -> invalid");
    result.valid = false;
    return result;
  }

  if (result.yaw.size() != result.path.size()) {
    warn_adjust(
      "result",
      "final yaw/path size mismatch -> invalid");
    result.valid = false;
    return result;
  }

  result.valid = true;
  return result;
}

}  // namespace chaining_costmap_ver
