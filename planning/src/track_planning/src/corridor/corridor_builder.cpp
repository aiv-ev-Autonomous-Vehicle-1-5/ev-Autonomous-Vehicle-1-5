/**
 * @file corridor_builder.cpp
 * @brief 코리도 빌더 구현 — greedy chaining 알고리즘
 *
 * 파이프라인 Step (2): Perception이 제공하는 차선점 + 콘 점을 좌/우 경계 폴리라인으로 변환.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                   s/d 좌표계 (Frenet-like)                      │
 * ├─────────────────────────────────────────────────────────────────┤
 * │                                                                 │
 * │            d (횡방향)                                           │
 * │            ▲                                                    │
 * │   d_max    │· · · · · · ·  ← 횡방향 상한                      │
 * │            │        ★      ← 후보점 (s, d)                    │
 * │            │                                                    │
 * │  ──────────●───────────────► s (종방향 = 접선 방향)            │
 * │          c_end   s_min  s_max                                   │
 * │   (참조점) │                                                    │
 * │  -d_max    │· · · · · · ·  ← 횡방향 하한                      │
 * │            │                                                    │
 * │                                                                 │
 * │  s = dot(pt - c_end, t_end)        전방 진행거리               │
 * │  d = |cross(t_end, pt - c_end)|    횡방향 거리                 │
 * │                                                                 │
 * │  필터 조건: s_min < s < s_max, d < d_max                       │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Greedy Chaining 흐름:
 *   1. determine_reference()로 참조점/접선 결정
 *      - 항상 ego_pos + ego_heading (이전 프레임 의존 제거)
 *   2. find_seed()로 체이닝 시작점 탐색 (콘 우선)
 *      - ego 중심 반지름 r_seed 이내 + 접선 방향 forward_range [rad] 각도 이내
 *      - 적응형 반경: r_step씩 확장하며 가장 가까운 적합 점 선택
 *   3. 반복 (Greedy Chaining Loop):
 *      a. 미사용 후보점 수집 (used[] 플래그로 재방문 방지)
 *      b. filter_candidates()로 s/d 좌표계 필터링
 *      c. 콘 후보가 있으면 콘만, 없으면 차선 후보 사용 (cone priority per step)
 *      d. score_and_select()로 가중 점수 최적 점 선택
 *         score = w_s·(진행) - w_d·(편차) - w_a·(각도) - w_p·(예측오차)
 *      e. chain에 추가, used 마킹
 *      f. x > roi.x_max 이면 종료
 *
 * 점수 산정 공식:
 *   score = w_s * norm_s           ← 전방 진행 보상 (멀수록 높은 점수)
 *         - w_d * norm_d           ← 횡방향 편차 패널티
 *         - w_a * norm_a           ← 방향 편차 패널티 (접선과의 각도 차이)
 *         - w_p * norm_p           ← 예측 오차 패널티 (예측 위치와의 거리)
 *   각 항은 [0, 1]로 정규화 (clamp)
 */
#include "track_planning/corridor/corridor_builder.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace track_planning
{

// ============================================================
// 참조점 결정 (Cold Start 지원)
// ============================================================
/**
 * @brief 체이닝 기준이 되는 참조점(c_end)과 참조 접선(t_end)을 결정한다.
 *
 * 항상 ego_pos + ego_heading 사용 (이전 프레임 의존 제거).
 * 센서 데이터가 이미 base_link 기준이므로 ego = (0,0), heading = (1,0).
 */
std::pair<Point2D, Point2D> CorridorBuilder::determine_reference(const Input & in)
{
  return {in.ego_pos, in.ego_heading};
}

// ============================================================
// Seed 선택: 참조 접선 방향 전방 범위 내에서 ego에 가장 가까운 점
// 콘 우선: 콘에서 seed가 있으면 차선 seed보다 우선 사용
// ============================================================
/**
 * @brief 그리디 체이닝의 시작점(seed)을 찾는다.
 *
 * 탐색 방법 (적응형 반경 + 점수 기반):
 *   1. 반경 r_step부터 시작, r_max까지 r_step씩 확장하며 반복
 *   2. 현재 반경 이내 + forward_range 각도 이내인 점을 후보로 수집
 *   3. 각 후보 점수 = w_dist*(거리/반경) + w_center*(1/(1+횡편차))
 *      - 이전 seed로부터 멀수록 + 예측 중앙선에 가까울수록 높은 점수
 *   4. 최고 점수 후보를 seed로 선택 (찾는 즉시 반환, 반경 더 넓히지 않음)
 *
 * 콘 우선(Cone Priority):
 *   - 콘과 차선 양쪽에서 seed 탐색 후, 콘 seed가 있으면 콘 우선 사용
 *   - 콘이 없을 때만 차선 seed 사용
 *   → 교통 콘이 신뢰도가 더 높다고 가정 (경진대회 환경)
 *
 * @param t_end     참조 접선 단위벡터 (이전 corridor 방향 또는 ego heading)
 * @param seed_out  찾은 seed 점 (출력 파라미터)
 * @return          seed 발견 시 true, 없으면 false
 */
bool CorridorBuilder::find_seed(
  const std::vector<Point2D> & lane_pts,
  const std::vector<Point2D> & cone_pts,
  const Point2D & ego,
  const Point2D & t_end,
  Point2D & seed_out,
  int & seed_source,
  size_t & seed_index,
  const PlanningParams & p)
{
  const double r_max   = p.corridor_seed.r_seed;
  const double r_step  = p.corridor_seed.r_seed_step;
  const double cos_fwd = std::cos(p.corridor_seed.forward_range);
  const double w_dist  = p.corridor_seed.w_dist;
  const double w_ctr   = p.corridor_seed.w_center;

  // 람다: 적응형 반경(r_step씩 확장, r_max까지) + 점수 기반 최적 seed 선택
  //   점수 = w_dist * (거리/반경) + w_ctr * (1 / (1 + 횡편차))
  //   → 이전 seed로부터 멀수록, 예측 중앙선에 가까울수록 높은 점수
  auto find_best = [&](const std::vector<Point2D> & pts,
                       Point2D & out, size_t & out_idx) -> bool {
    for (double r = r_step; r <= r_max + 1e-9; r += r_step) {
      double best_score = -std::numeric_limits<double>::max();
      bool found = false;
      for (size_t i = 0; i < pts.size(); ++i) {
        const double d = dist(pts[i], ego);
        if (d > r || d < 1e-9) continue;
        if (dot2(normalize(pts[i] - ego), t_end) < cos_fwd) continue;

        const double s     = dot2(pts[i] - ego, t_end);
        const double d_lat = std::sqrt(std::max(0.0, d * d - s * s));
        const double score = w_dist * (d / r) + w_ctr * (1.0 / (1.0 + d_lat));

        if (score > best_score) {
          best_score = score;
          out = pts[i];
          out_idx = i;
          found = true;
        }
      }
      if (found) return true;
    }
    return false;
  };

  // 콘 우선: 콘 seed를 먼저 탐색
  Point2D cone_seed, lane_seed;
  size_t cone_idx = 0, lane_idx = 0;
  bool cone_found = find_best(cone_pts, cone_seed, cone_idx);
  bool lane_found = find_best(lane_pts, lane_seed, lane_idx);

  if (cone_found) {
    seed_out = cone_seed;
    seed_source = 0;  // cone
    seed_index = cone_idx;
    return true;
  }
  if (lane_found) {
    seed_out = lane_seed;
    seed_source = 1;  // lane
    seed_index = lane_idx;
    return true;
  }
  return false;
}

// ============================================================
// 후보점 필터링: s/d 좌표계 기반
// ============================================================
/**
 * @brief 현재 chain 끝점에서 s/d 좌표계로 후보점을 필터링한다.
 *
 * s/d 좌표계 정의:
 *   diff = pt - c_end                    (현재 참조점에서 후보점까지의 벡터)
 *   s = dot(diff, t_end)                 (접선 방향 전방 진행거리, 양수 = 전방)
 *   d = |cross(t_end, diff)|             (접선에 수직인 횡방향 거리)
 *
 * 필터 조건 (모두 만족해야 통과):
 *   1. s_min < s < s_max : 너무 가깝지도 멀지도 않은 전방 범위
 *   2. d < d_max         : 횡방향으로 너무 멀지 않음 (경계 폭 제한)
 *   3. dist(pt, p_k) < r_search : 현재 끝점으로부터 원형 반경 내 (r_search > 0 시)
 *
 * @param c_end   현재 참조점 (s/d 계산 기준)
 * @param t_end   현재 참조 접선 단위벡터
 * @param p_k     현재 chain 끝점 (원형 탐색 중심)
 * @return        필터를 통과한 점의 인덱스 목록
 */
std::vector<size_t> CorridorBuilder::filter_candidates(
  const std::vector<Point2D> & pts,
  const Point2D & c_end,
  const Point2D & t_end,
  const Point2D & p_k,
  const PlanningParams & p)
{
  const auto & f = p.corridor_filter;  // 필터 파라미터 (s_min, s_max, d_max, r_search)
  std::vector<size_t> result;

  for (size_t i = 0; i < pts.size(); ++i) {
    const Point2D diff = pts[i] - c_end;
    const double s = dot2(diff, t_end);              // 전방 진행거리 (s 성분)
    const double d = std::abs(cross2(t_end, diff));  // 횡방향 거리 (d 성분)

    if (s < f.s_min || s > f.s_max) continue;  // 전방 범위 밖이면 제외
    if (d > f.d_max) continue;                  // 횡방향으로 너무 멀면 제외
    // 원형 탐색 반경 필터 (r_search <= 0 이면 비활성화)
    if (f.r_search > 0.0 && dist(pts[i], p_k) > f.r_search) continue;

    result.push_back(i);
  }
  return result;
}

// ============================================================
// 후보점 점수 산정 및 최적 선택
// ============================================================
/**
 * @brief 필터를 통과한 후보점에 가중 점수를 매겨 최적 점을 선택한다.
 *
 * 점수 공식:
 *   score = w_s * norm_s           (전방 진행 보상: 더 앞에 있을수록 높은 점수)
 *         - w_d * norm_d           (횡방향 편차 패널티: 경계선 위에 있을수록 낮은 패널티)
 *         - w_a * norm_a           (방향 편차 패널티: 현재 접선과 같은 방향일수록 낮은 패널티)
 *         - w_p * norm_p           (예측 오차 패널티: 예측 위치에 가까울수록 낮은 패널티)
 *
 * 각 항 정규화 [0, 1]:
 *   norm_s = clamp(s / s_max)                     (전방 진행거리 정규화)
 *   norm_d = clamp(d / d_max)                     (횡방향 거리 정규화)
 *   norm_a = clamp(delta_theta / theta_max)        (방향 편차 정규화)
 *   norm_p = clamp(pred_err / r_search)            (예측 오차 정규화, r_search > 0 시)
 *
 * 예측점(p_pred):
 *   p_pred = p_k + step_pred * t_k               (현재 접선 방향으로 step_pred 만큼 이동)
 *   → 다음 점이 있어야 할 예상 위치
 *
 * Top-K 최적화:
 *   후보가 많으면 d 기준 오름차순 정렬 후 상위 k_top 개만 평가
 *   → 경계에 가장 가까운 점들만 상세 평가하여 계산 효율화
 *
 * @param t_k  현재 chain 끝점의 로컬 접선 (방향 편차 및 예측 기준)
 * @return     최적 인덱스 (-1이면 유효한 후보 없음)
 */
int CorridorBuilder::score_and_select(
  const std::vector<Point2D> & pts,
  const std::vector<size_t> & candidates,
  const Point2D & c_end,
  const Point2D & t_end,
  const Point2D & p_k,
  const Point2D & t_k,
  const PlanningParams & p)
{
  if (candidates.empty()) return -1;

  const auto & sc = p.corridor_score;   // 점수 가중치 파라미터 (w_s, w_d, w_a, w_p)
  const auto & f = p.corridor_filter;   // 필터 파라미터 (정규화 분모로 사용)
  const auto & tk = p.corridor_topk;   // Top-K 파라미터 (enable, k_top)

  // Top-K 최적화를 위한 작업용 인덱스 복사
  std::vector<size_t> work = candidates;

  // Top-K 최적화: 후보가 k_top 초과 시 d 기준 오름차순 정렬 후 상위 K개만 유지
  if (tk.enable && static_cast<int>(work.size()) > tk.k_top) {
    std::sort(work.begin(), work.end(), [&](size_t a, size_t b) {
      double da = std::abs(cross2(t_end, pts[a] - c_end));  // a의 횡방향 거리
      double db = std::abs(cross2(t_end, pts[b] - c_end));  // b의 횡방향 거리
      return da < db;  // d 작은 순 (경계에 가까운 점 우선)
    });
    work.resize(static_cast<size_t>(tk.k_top));
  }

  // 예측점: 현재 끝점 p_k에서 로컬 접선 t_k 방향으로 step_pred 만큼 이동
  // → 다음 경계점이 있어야 할 예상 위치
  const Point2D p_pred = p_k + sc.step_pred * t_k;

  double best_score = -std::numeric_limits<double>::max();
  int best_idx = -1;

  for (size_t idx : work) {
    const Point2D & pt = pts[idx];
    const Point2D diff = pt - c_end;

    // s/d 성분 계산
    const double s = dot2(diff, t_end);              // 전방 진행거리
    const double d = std::abs(cross2(t_end, diff));  // 횡방향 거리

    // 방향 편차: (p_k → pt) 벡터 방향 vs 현재 로컬 접선 t_k 사이의 각도 차이
    const Point2D dir = pt - p_k;
    const double dir_heading = heading(dir);     // (p_k → pt) 방향각 [rad]
    const double tk_heading = heading(t_k);      // 로컬 접선 방향각 [rad]
    const double delta_theta = std::abs(angle_diff(tk_heading, dir_heading));

    // 예측 오차: 예측점 p_pred와 후보점 pt 사이의 거리
    const double pred_err = dist(pt, p_pred);

    // 각 항을 [0, 1]로 정규화 (clamp)
    auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };
    const double norm_s = clamp01(s / f.s_max);         // 전방 진행 정규화
    const double norm_d = clamp01(d / f.d_max);         // 횡방향 편차 정규화
    const double norm_a = clamp01(delta_theta / sc.theta_max);  // 방향 편차 정규화
    // 예측 오차 정규화 (r_search가 0 이하면 예측 패널티 비활성화)
    const double norm_p = (f.r_search > 0.0) ? clamp01(pred_err / f.r_search) : 0.0;

    // 가중 점수 계산: 전방 진행은 보상(+), 나머지는 패널티(-)
    const double score =
      sc.w_s * norm_s - sc.w_d * norm_d - sc.w_a * norm_a - sc.w_p * norm_p;

    if (score > best_score) {
      best_score = score;
      best_idx = static_cast<int>(idx);
    }
  }

  return best_idx;
}

// ============================================================
// 한 쪽 경계 구축 (Greedy Chaining 루프)
// ============================================================
/**
 * @brief 좌측 또는 우측 경계를 greedy chaining 방식으로 구축한다.
 *
 * 알고리즘:
 *   1. find_seed()로 시작점 탐색 (콘 우선)
 *      → seed를 chain[0]으로 추가
 *   2. Greedy 루프 (max_pts 제한):
 *      a. 현재 끝점 p_k와 로컬 접선 t_k 계산
 *         - chain 길이 >= 2: 마지막 두 점으로 접선 계산
 *         - chain 길이 1: 초기 참조 접선 t_end 사용
 *      b. 미사용 후보 수집 (used_cone[], used_lane[] 플래그 확인)
 *      c. filter_candidates()로 s/d 필터링
 *      d. 콘 우선(Cone Priority):
 *         - 콘 후보 있으면 → 콘 pool만 사용
 *         - 없으면 → 차선 pool 사용
 *      e. score_and_select()로 최적 점 선택
 *      f. chain에 추가, used[] 마킹
 *      g. 추가된 점의 x > x_max 이면 루프 종료
 *
 * @param lane_pts       차선 경계점 배열
 * @param cone_pts       콘 중심점 배열
 * @param c_end          초기 참조점 (warm/cold start에서 결정됨)
 * @param t_end          초기 참조 접선 단위벡터
 * @param ego_pos        ego 위치 (seed 탐색 기준)
 * @return               greedy chaining으로 연결된 경계 폴리라인 점 배열
 */
std::vector<Point2D> CorridorBuilder::build_one_side(
  const std::vector<Point2D> & lane_pts,
  const std::vector<Point2D> & cone_pts,
  const Point2D & c_end,
  const Point2D & t_end,
  const Point2D & ego_pos,
  const PlanningParams & p)
{
  std::vector<Point2D> chain;  // 체이닝 결과 경계 폴리라인
  const auto & ref = p.corridor_ref_tangent; // 접선 회귀 파라미터

  // seed 탐색 (콘 우선): 항상 ego 기준 (이전 프레임 의존 제거)
  Point2D seed;
  int seed_source = -1;    // 0 = cone, 1 = lane
  size_t seed_idx = 0;
  if (!find_seed(lane_pts, cone_pts, ego_pos, t_end, seed, seed_source, seed_idx, p)) {
    return chain;  // seed 실패 → 빈 벡터 반환
  }
  chain.push_back(seed);

  // 사용된 점 추적 (같은 점 재방문 방지)
  std::vector<bool> used_cone(cone_pts.size(), false);
  std::vector<bool> used_lane(lane_pts.size(), false);

  // seed를 인덱스 기반으로 정확히 마킹
  if (seed_source == 0) { used_cone[seed_idx] = true; }
  else                   { used_lane[seed_idx] = true; }
 
  const int max_pts = p.corridor_general.max_points_side;  // 한 쪽 경계 최대 점 수
  const double x_max = p.roi.x_max;  // ROI 전방 한계 (이 x 초과 시 체이닝 종료)

  // ---- Greedy Chaining 메인 루프 ----
  while (static_cast<int>(chain.size()) < max_pts) {
    const Point2D & p_k = chain.back();  // 현재 chain 끝점

    // 로컬 접선 계산: chain이 2개 이상이면 회귀 접선, 아니면 초기 참조 접선 사용
    Point2D t_k;
    if (chain.size() >= 2) {
      // 마지막 n_reg개 점으로 회귀 접선 계산 (단일 outlier에 강건)
      t_k = regress_tangent(chain, static_cast<size_t>(ref.n_reg));
    } else {
      t_k = t_end;  // 첫 번째 반복: 초기 참조 접선 사용
    }

    // 참조점 갱신:
    // - 첫 반복(chain.size()==1): 초기 c_end 사용 (외부에서 결정된 기준점)
    // - 이후: chain 끝점 p_k를 참조점으로 사용 (이동하며 탐색)
    const Point2D local_c_end = (chain.size() <= 1) ? c_end : p_k;
    const Point2D local_t_end = t_k;  // 로컬 접선을 filter/score 기준으로 사용

    // 미사용 후보 수집 (used[] = false 인 점만 수집)
    std::vector<Point2D> avail_cone, avail_lane;
    std::vector<size_t> avail_cone_orig, avail_lane_orig;  // 원본 인덱스 (used[] 마킹용)

    for (size_t i = 0; i < cone_pts.size(); ++i) {
      if (!used_cone[i]) {
        avail_cone.push_back(cone_pts[i]);
        avail_cone_orig.push_back(i);  // 원본 배열 인덱스 기억
      }
    }
    for (size_t i = 0; i < lane_pts.size(); ++i) {
      if (!used_lane[i]) {
        avail_lane.push_back(lane_pts[i]);
        avail_lane_orig.push_back(i);  // 원본 배열 인덱스 기억
      }
    }

    // 후보 필터링 (s/d 좌표계 기반)
    auto cone_filtered = filter_candidates(avail_cone, local_c_end, local_t_end, p_k, p);
    auto lane_filtered = filter_candidates(avail_lane, local_c_end, local_t_end, p_k, p);

    // 콘 우선(Cone Priority per step):
    // 이번 스텝에서 콘 후보가 있으면 콘 pool만 사용, 없으면 차선 pool 사용
    const std::vector<Point2D> * pool = nullptr;          // 선택된 pool 포인터
    const std::vector<size_t> * filtered = nullptr;       // 필터된 인덱스 목록 포인터
    const std::vector<size_t> * orig_indices = nullptr;   // 원본 인덱스 포인터
    std::vector<bool> * used_flags = nullptr;             // 사용 플래그 포인터

    if (!cone_filtered.empty()) {
      // 콘 후보 있음 → 콘 pool 사용
      pool = &avail_cone;
      filtered = &cone_filtered;
      orig_indices = &avail_cone_orig;
      used_flags = &used_cone;
    } else if (!lane_filtered.empty()) {
      // 콘 없음 → 차선 pool 사용
      pool = &avail_lane;
      filtered = &lane_filtered;
      orig_indices = &avail_lane_orig;
      used_flags = &used_lane;
    } else {
      break;  // 어떤 후보도 없음 → 체이닝 종료
    }

    // 점수 산정 및 최적 점 선택
    int best = score_and_select(*pool, *filtered, local_c_end, local_t_end, p_k, t_k, p);
    if (best < 0) break;  // 선택 실패 → 체이닝 종료

    // 최적 점을 chain에 추가
    chain.push_back((*pool)[static_cast<size_t>(best)]);

    // 선택된 점을 사용됨으로 마킹 (원본 인덱스 사용)
    size_t orig_idx = (*orig_indices)[static_cast<size_t>(best)];
    (*used_flags)[orig_idx] = true;

    // ROI 초과 시 체이닝 종료 (전방 한계 도달)
    if (chain.back().x > x_max) break;
  }

  return chain;
}

// ============================================================
// 메인 빌드 진입점
// ============================================================
/**
 * @brief 좌/우 경계 폴리라인을 독립적으로 구축하여 반환한다.
 *
 * 처리 순서:
 *   1. determine_reference()로 참조점/접선 결정 (항상 ego 기준)
 *   2. build_one_side()로 좌측 경계 구축
 *   3. build_one_side()로 우측 경계 구축
 *   4. 각 경계가 2점 이상이면 ok 플래그 설정
 *
 * 좌/우 경계는 동일한 c_end, t_end를 기준으로 독립적으로 구축됨.
 */
CorridorPolylines CorridorBuilder::build(const Input & in, const PlanningParams & p)
{
  CorridorPolylines result;

  // 참조점/접선 결정 (항상 ego_pos + ego_heading)
  auto [c_end, t_end] = determine_reference(in);

  // 좌측 경계 구축
  result.left = build_one_side(
    in.lane_left, in.cone_left, c_end, t_end, in.ego_pos, p);
  result.left_ok = (result.left.size() >= 2);

  // 우측 경계 구축
  result.right = build_one_side(
    in.lane_right, in.cone_right, c_end, t_end, in.ego_pos, p);
  result.right_ok = (result.right.size() >= 2);

  return result;
}

}  // namespace track_planning
