/**
 * @file goal_selector.cpp
 * @brief GoalSelector 구현부 — A* 목표 포인트 선택
 *
 * ## 목표 선택 흐름
 *  select()가 호출되면:
 *  1. lookahead = l0 + kv * ego_speed  (속도 적응형 룩어헤드)
 *  2. path_prev가 있으면 Method1 시도 (시간 연속성)
 *  3. 실패하면 centerline으로 Method1 시도
 *  4. 모두 실패하면 Method2 (ring 샘플링) 시도
 *
 * ## 파일 의존성
 *  - geometry.hpp: dist(), lerp(), heading(), wrap_pi(), resample_polyline()
 *  - types.hpp: Point2D, GoalResult
 */

#include "track_planning/goal/goal_selector.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace track_planning
{

// ============================================================
// Check if a world point maps to a free grid cell
// ============================================================
/**
 * @brief 월드 포인트 pt가 그리드에서 free(0) 셀인지 확인
 *
 * 1. 월드 좌표 → 그리드 셀 변환 (floor 기반)
 * 2. 범위 밖이면 false (보수적 처리 — 범위 밖은 주행 불가로 간주)
 * 3. grid[셀] == 0이면 true
 *
 * @return free(0)이면 true, occupied/inflated/범위밖이면 false
 */
bool GoalSelector::is_free(
  const std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  const Point2D & pt)
{
  // 월드 좌표 → 그리드 셀 인덱스 변환
  const int col = static_cast<int>(std::floor((pt.x - origin_x) / resolution));
  const int row = static_cast<int>(std::floor((pt.y - origin_y) / resolution));

  // 그리드 범위 밖이면 주행 불가로 처리
  if (col < 0 || col >= width || row < 0 || row >= height) return false;
  // 셀 값이 정확히 0(free)인지 확인 (99=inflated, 100=occupied 모두 false)
  return grid[row * width + col] == 0;
}

// ============================================================
// Method 1: Lookahead along reference polyline
// ============================================================
/**
 * @brief 참조 폴리라인을 따라 lookahead 거리의 free 목표 포인트 탐색
 *
 * ## 알고리즘
 *
 * ### 1차 시도: 정확한 lookahead 거리에서 보간
 *   폴리라인을 처음부터 순회하며 각 선분의 길이를 누적(accum).
 *   accum + seg_len >= lookahead 인 선분 발견 시:
 *     t = (lookahead - accum) / seg_len  ... 선분 내 비율
 *     goal = lerp(pts[i], pts[i+1], t)   ... 선형 보간
 *   goal이 free이면 반환 (score=1.0), occupied이면 break
 *
 * ### 2차 시도: 폴리라인 끝점
 *   ref_line.back()이 free이면 반환 (score=0.5, 덜 정확한 lookahead)
 *
 * ### 3차 시도: 역방향 탐색
 *   끝점에서 시작 방향으로 역순 탐색하며 가장 먼 free 포인트 반환 (score=0.3)
 *
 * ### 최종 실패
 *   모두 실패하면 valid=false 반환 → 상위 로직이 폴백 처리
 *
 * @param ref_line   참조 폴리라인 (최소 2개 포인트 필요)
 * @param lookahead  목표 거리(m)
 * @param method_id  결과의 method 필드값 (0=path_prev, 0=centerline)
 */
GoalResult GoalSelector::method_lookahead(
  const std::vector<Point2D> & ref_line,
  const std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  double lookahead,
  uint8_t method_id)
{
  GoalResult result;
  if (ref_line.size() < 2) return result;  // 폴리라인 너무 짧음

  // 폴리라인을 따라 누적 거리 계산
  double accum = 0.0;
  for (size_t i = 0; i + 1 < ref_line.size(); ++i) {
    const double seg_len = dist(ref_line[i], ref_line[i + 1]);  // 선분 길이
    if (seg_len < 1e-12) continue;  // 중복 포인트 무시

    if (accum + seg_len >= lookahead) {
      // lookahead 거리가 이 선분 내에 있음
      const double t = (lookahead - accum) / seg_len;  // 보간 비율 [0, 1]
      // 선형 보간으로 lookahead 포인트 계산
      Point2D goal_pt = lerp(ref_line[i], ref_line[i + 1], t);

      if (is_free(grid, width, height, resolution, origin_x, origin_y, goal_pt)) {
        // 목표 포인트가 free → 성공
        result.goal = goal_pt;
        result.valid = true;
        result.method = method_id;
        result.score = 1.0;  // 최고 스코어: 정확한 lookahead 거리
        return result;
      }
      // 목표 포인트가 occupied → 더 짧은 lookahead 시도를 위해 break
      break;
    }
    accum += seg_len;
  }

  // 2차 시도: 폴리라인의 마지막 포인트 시도
  if (is_free(grid, width, height, resolution, origin_x, origin_y, ref_line.back())) {
    result.goal = ref_line.back();
    result.valid = true;
    result.method = method_id;
    result.score = 0.5;  // 중간 스코어: lookahead보다 짧은 거리
    return result;
  }

  // 3차 시도: 역방향으로 탐색하여 가장 먼(끝에 가까운) free 포인트 찾기
  accum = 0.0;
  for (size_t i = ref_line.size() - 1; i > 0; --i) {
    const double seg_len = dist(ref_line[i], ref_line[i - 1]);
    accum += seg_len;
    if (is_free(grid, width, height, resolution, origin_x, origin_y, ref_line[i - 1])) {
      result.goal = ref_line[i - 1];
      result.valid = true;
      result.method = method_id;
      result.score = 0.3;  // 낮은 스코어: 매우 짧은 목표 거리
      return result;
    }
  }

  return result;  // valid = false — 모든 시도 실패
}

// ============================================================
// Method 2: Ring sampling
// ============================================================
/**
 * @brief ego 주변 ring(원) 위 포인트를 샘플링하여 최고 스코어 목표 선택
 *
 * ## 샘플링
 *  theta = -π + 2π * i / n_samples  (i=0, ..., n_samples-1)
 *  모든 방향에 균등하게 n_samples 개 포인트를 배치
 *  candidate = (ego.x + lookahead*cos(θ), ego.y + lookahead*sin(θ))
 *
 * ## 스코어링 (free 셀인 candidate만 평가)
 *
 *  [heading_score]
 *   angle_to_candidate = atan2(cy-ey, cx-ex)
 *   heading_cost = |wrap_pi(angle - heading_angle)|  ∈ [0, π]
 *   heading_score = 1 - heading_cost / π             ∈ [0, 1]
 *   → 전방 방향(heading과 일치)일수록 1에 가까움
 *
 *  [progress_score]
 *   progress = dot(candidate - ego, ego_heading)    (전방 진행 거리)
 *   progress_score = progress / lookahead            ∈ [-1, 1]
 *   → 전방일수록 양수, 후방일수록 음수
 *
 *  [ref_score]
 *   min_dist = 참조 폴리라인의 가장 가까운 포인트까지 거리
 *   ref_score = max(0, 1 - min_dist / lookahead)    ∈ [0, 1]
 *   → 센터라인에 가까울수록 1에 가까움
 *
 *  [total_score]
 *   = 0.4 * heading_score + 0.3 * progress_score + 0.3 * ref_score
 *
 * @param n_samples  링 위 샘플 수 (많을수록 정확하지만 계산량 증가)
 * @param ref_line   ref_score 계산용 참조 폴리라인 (없으면 0.0)
 */
GoalResult GoalSelector::method_ring(
  const std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  const Point2D & ego_pos,
  const Point2D & ego_heading,
  double lookahead,
  int n_samples,
  const std::vector<Point2D> & ref_line)
{
  GoalResult result;
  if (n_samples <= 0) return result;

  // ego 진행 방향 각도 (라디안)
  const double heading_angle = heading(ego_heading);
  double best_score = -1e9;  // 가장 높은 스코어 추적

  for (int i = 0; i < n_samples; ++i) {
    // 원 위에 균등하게 분포한 각도 계산 (-π부터 시작)
    const double theta = -M_PI + 2.0 * M_PI * i / n_samples;

    // lookahead 반경 원 위의 후보 포인트 계산
    const Point2D candidate = {
      ego_pos.x + lookahead * std::cos(theta),
      ego_pos.y + lookahead * std::sin(theta)
    };

    // occupied 셀은 스코어링 건너뜀
    if (!is_free(grid, width, height, resolution, origin_x, origin_y, candidate)) {
      continue;
    }

    // ---- 스코어 계산 ----

    // [1] heading_score: ego 진행 방향과 얼마나 일치하는지
    const double angle_to_candidate = std::atan2(
      candidate.y - ego_pos.y, candidate.x - ego_pos.x);
    // 두 각도 차이를 [-π, π]로 정규화
    const double heading_cost = std::abs(wrap_pi(angle_to_candidate - heading_angle));
    // 0(일치) → 1점, π(반대) → 0점
    const double heading_score = 1.0 - heading_cost / M_PI;

    // [2] progress_score: ego 전방으로 얼마나 진행하는지 (내적)
    const double progress = (candidate.x - ego_pos.x) * ego_heading.x +
                            (candidate.y - ego_pos.y) * ego_heading.y;
    const double progress_score = progress / lookahead;  // [-1, 1]

    // [3] ref_score: 참조 폴리라인(센터라인)에 얼마나 가까운지
    double ref_score = 0.0;
    if (ref_line.size() >= 2) {
      double min_dist = 1e9;
      for (const auto & rp : ref_line) {
        double d = dist(candidate, rp);
        if (d < min_dist) min_dist = d;
      }
      // 거리가 0이면 1점, lookahead 이상이면 0점
      ref_score = std::max(0.0, 1.0 - min_dist / lookahead);
    }

    // 가중합으로 최종 스코어 계산
    const double score = 0.4 * heading_score + 0.3 * progress_score + 0.3 * ref_score;

    // 최고 스코어 갱신
    if (score > best_score) {
      best_score = score;
      result.goal = candidate;
      result.valid = true;
      result.method = 1;   // Method 2 (ring sampling)
      result.score = score;
    }
  }

  return result;
}

// ============================================================
// Main select()
// ============================================================
/**
 * @brief A* 목표 포인트 선택 (1a → 1b → 2 순서로 시도)
 *
 * ## lookahead 거리 계산
 *   lookahead = lookahead_l0 + lookahead_kv * ego_speed
 *   - lookahead_l0: 기본 룩어헤드 거리(m), 정지 시 최소 거리
 *   - lookahead_kv: 속도 계수 (m/s당 추가 거리)
 *   예) l0=5.0, kv=0.5, speed=10 → lookahead=10.0m
 *
 * ## 선택 순서
 *  1. path_prev (이전 경로) - 시간 연속성
 *  2. centerline (트랙 센터라인)
 *  3. ring sampling (최후 수단)
 *
 * @param ego_speed 현재 자차 속도(m/s)
 */
GoalResult GoalSelector::select(
  const std::vector<Point2D> & centerline,
  const std::vector<Point2D> & path_prev,
  const std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  const Point2D & ego_pos,
  const Point2D & ego_heading,
  double ego_speed,
  const PlanningParams & p)
{
  // 속도 적응형 룩어헤드 거리 계산
  // 빠를수록 더 멀리 앞을 바라봄 (Pure Pursuit 스타일)
  const double lookahead = p.goal.lookahead_l0 + p.goal.lookahead_kv * ego_speed;

  // ---- Method 1a: path_prev 룩어헤드 (시간 연속성 우선) ----
  // 이전 프레임 경로가 있으면 먼저 시도
  if (path_prev.size() >= 2) {
    auto result = method_lookahead(
      path_prev, grid, width, height,
      resolution, origin_x, origin_y, lookahead, 0);
    if (result.valid) return result;  // 성공 시 즉시 반환
  }

  // ---- Method 1b: centerline 룩어헤드 ----
  // path_prev 실패 시 트랙 센터라인으로 시도
  if (centerline.size() >= 2) {
    auto result = method_lookahead(
      centerline, grid, width, height,
      resolution, origin_x, origin_y, lookahead, 0);
    if (result.valid) return result;  // 성공 시 즉시 반환
  }

  // ---- Method 2: ring 샘플링 (최후 수단) ----
  // 1a, 1b 모두 실패했을 때만 사용 (장애물로 센터라인이 막혔을 때)
  return method_ring(
    grid, width, height,
    resolution, origin_x, origin_y,
    ego_pos, ego_heading, lookahead,
    p.goal.ring_samples,
    centerline);
}

}  // namespace track_planning
