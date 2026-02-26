/**
 * @file line_chainer.cpp
 * @brief LineChainer — 구현부
 *
 * DFS 기반 nearest-neighbor 체이닝으로 경계점들을 좌/우 corridor로 분류한다.
 *
 * 핵심 아이디어:
 *   1. y>0 최근접점을 left seed, y<0 최근접점을 right seed로 선택
 *   2. 각 seed에서 출발하여 방향벡터(PCA regression) 기반 전진 탐색
 *   3. visited 배열 공유로 좌/우 중복 방지
 *   4. DFS 후 미방문 노이즈는 가장 가까운 chain 점에 병렬 할당
 *   5. 최종 리샘플링 (0.1m 간격, cone-cone 구간은 CONE 유지)
 */
#include "planning_lc_ver/chainer/line_chainer.hpp"
#include "planning_lc_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace planning_lc_ver
{

/**
 * @brief 한쪽 체인을 DFS로 구축
 *
 * 알고리즘:
 *   1. seed를 chain에 추가, visited 마킹
 *   2. 첫 스텝 (seed만 있을 때):
 *      - 방향벡터 없음 → 원형 범위 내 최근접 탐색 (forward_angle 없음)
 *      - Cone priority: 콘이 있으면 차선 제거, 그 중 최근접
 *   3. 이후 스텝 (2+점):
 *      - PCA regression으로 방향벡터 추정
 *      - forward_angle 내 후보 수집
 *      - Cone priority + cross2 scoring
 *   4. 후보 없으면 radius를 step씩 증가 (max까지)
 *   5. max radius까지 후보 없으면 chain 종료
 */
std::vector<ChainedPoint> LineChainer::chain_one_side(
  const std::vector<ChainedPoint> & candidates,
  int seed_idx,
  bool is_left,
  std::vector<bool> & visited,
  const PlanningParams & params)
{
  std::vector<ChainedPoint> chain;
  const int n = static_cast<int>(candidates.size());
  if (seed_idx < 0 || seed_idx >= n) return chain;

  const auto & cp = params.chainer;

  // seed 추가
  chain.push_back(candidates[seed_idx]);
  visited[seed_idx] = true;

  // 방향벡터용 Point2D 배열 (regress_direction에 전달)
  std::vector<Point2D> chain_pts;
  chain_pts.push_back(candidates[seed_idx].to_point2d());

  // 방향벡터가 유효한지 여부 (첫 스텝에서는 false)
  bool has_direction = false;

  while (true) {
    const Point2D tail = chain_pts.back();

    // ── 방향벡터 계산 (2+점부터만) ──
    Point2D direction{0.0, 0.0};
    if (chain_pts.size() >= 2) {
      has_direction = true;
      Point2D hint = normalize(
        chain_pts.back() - chain_pts[chain_pts.size() - 2]);
      if (norm(hint) < 1e-6) hint = {1.0, 0.0};

      int regress_n = std::min(
        cp.max_regress_pts,
        std::max(cp.min_regress_pts, static_cast<int>(chain_pts.size())));
      direction = regress_direction(chain_pts, regress_n, hint);
    }

    // ── 탐색: radius 확장 루프 ──
    int best_idx = -1;

    for (double r = cp.search_radius; r <= cp.search_radius_max + 1e-6;
         r += cp.search_radius_step)
    {
      const double r_sq = r * r;
      const double cos_fwd = std::cos(cp.forward_angle * 0.5);

      struct Candidate {
        int idx;
        double d_sq;      // 거리 제곱 (첫 스텝 최근접용)
        double score;      // cross2 점수 (2+ 스텝용)
        bool is_cone;
      };
      std::vector<Candidate> fwd_candidates;

      for (int i = 0; i < n; ++i) {
        if (visited[i]) continue;

        const double dx = candidates[i].x - tail.x;
        const double dy = candidates[i].y - tail.y;
        const double d_sq = dx * dx + dy * dy;

        if (d_sq < 1e-12 || d_sq > r_sq) continue;

        if (has_direction) {
          // 2+점: forward angle + cross2 scoring
          const double d = std::sqrt(d_sq);
          Point2D to_cand = {dx / d, dy / d};
          const double alignment = dot2(direction, to_cand);
          if (alignment < cos_fwd) continue;

          const double cross_val = cross2(direction, to_cand);
          const double score = is_left ? -cross_val : cross_val;

          fwd_candidates.push_back({i, d_sq, score,
            candidates[i].type == PointType::CONE});
        } else {
          // 첫 스텝: 원형 전체, score는 사용 안 함
          fwd_candidates.push_back({i, d_sq, 0.0,
            candidates[i].type == PointType::CONE});
        }
      }

      if (fwd_candidates.empty()) continue;

      // ── Cone priority: 콘 후보가 있으면 차선 제거 ──
      bool has_cone = false;
      for (const auto & fc : fwd_candidates) {
        if (fc.is_cone) { has_cone = true; break; }
      }
      if (has_cone) {
        fwd_candidates.erase(
          std::remove_if(fwd_candidates.begin(), fwd_candidates.end(),
            [](const Candidate & c) { return !c.is_cone; }),
          fwd_candidates.end());
      }

      if (has_direction) {
        // ── 2+점: 최고 cross2 점수 후보 선택 ──
        double best_score = -std::numeric_limits<double>::max();
        for (const auto & fc : fwd_candidates) {
          if (fc.score > best_score) {
            best_score = fc.score;
            best_idx = fc.idx;
          }
        }
      } else {
        // ── 첫 스텝: 최근접 후보 선택 ──
        double min_d_sq = std::numeric_limits<double>::max();
        for (const auto & fc : fwd_candidates) {
          if (fc.d_sq < min_d_sq) {
            min_d_sq = fc.d_sq;
            best_idx = fc.idx;
          }
        }
      }

      if (best_idx >= 0) break;  // 후보를 찾았으면 radius 확장 중단
    }

    if (best_idx < 0) break;  // max radius까지 후보 없음 → chain 종료

    // 선택된 점 추가
    chain.push_back(candidates[best_idx]);
    chain_pts.push_back(candidates[best_idx].to_point2d());
    visited[best_idx] = true;
  }

  return chain;
}

/**
 * @brief 체인을 ds 간격으로 리샘플링 (타입 보존)
 *
 * 각 세그먼트의 양 끝 점 타입에 따라 보간점의 타입을 결정:
 *   - cone-cone 구간 → CONE (콘 사이는 콘으로 채움)
 *   - 그 외 (lane-lane, lane-cone, cone-lane) → LANE
 */
std::vector<ChainedPoint> LineChainer::resample_chain(
  const std::vector<ChainedPoint> & chain_pts,
  double ds)
{
  if (chain_pts.size() < 2 || ds <= 0.0) {
    return chain_pts;
  }

  std::vector<ChainedPoint> out;
  out.push_back(chain_pts.front());

  double accum = 0.0;
  size_t seg = 0;

  while (seg < chain_pts.size() - 1) {
    const auto & a = chain_pts[seg];
    const auto & b = chain_pts[seg + 1];

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double seg_len = std::sqrt(dx * dx + dy * dy);

    if (seg_len < 1e-12) {
      ++seg;
      continue;
    }

    // 구간 타입: 양쪽 모두 CONE이면 CONE, 아니면 LANE
    const PointType seg_type =
      (a.type == PointType::CONE && b.type == PointType::CONE)
        ? PointType::CONE : PointType::LANE;

    double remaining = ds - accum;
    if (remaining <= seg_len) {
      double t = remaining / seg_len;
      out.push_back({a.x + t * dx, a.y + t * dy, seg_type});

      accum = 0.0;
      double pos_in_seg = remaining;
      while (pos_in_seg + ds <= seg_len) {
        pos_in_seg += ds;
        double t2 = pos_in_seg / seg_len;
        out.push_back({a.x + t2 * dx, a.y + t2 * dy, seg_type});
      }
      accum = seg_len - pos_in_seg;
      ++seg;
    } else {
      accum += seg_len;
      ++seg;
    }
  }

  // 마지막 점 추가 (너무 멀면)
  const auto & last_out = out.back();
  const auto & last_in = chain_pts.back();
  const double tail_dx = last_in.x - last_out.x;
  const double tail_dy = last_in.y - last_out.y;
  if (std::sqrt(tail_dx * tail_dx + tail_dy * tail_dy) > ds * 0.1) {
    out.push_back(chain_pts.back());
  }

  return out;
}

/**
 * @brief 메인 체이닝 함수
 *
 * 1. Left seed (y>0 최근접) / Right seed (y<0 최근접) 선택
 * 2. DFS 체이닝 (visited 공유)
 * 3. 노이즈 할당 (병렬 — 원본 chain 점만 참조)
 * 4. 리샘플링
 */
ChainResult LineChainer::chain(
  const std::vector<ChainedPoint> & candidates,
  const PlanningParams & params)
{
  ChainResult result;
  const int n = static_cast<int>(candidates.size());
  if (n < 2) return result;

  // ── Seed 선택 ──
  // Left seed: y > 0인 점 중 ego(0,0)에서 가장 가까운 점
  // Right seed: y < 0인 점 중 ego(0,0)에서 가장 가까운 점
  int left_seed = -1, right_seed = -1;
  double left_dist_sq = std::numeric_limits<double>::max();
  double right_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i < n; ++i) {
    const double d_sq = candidates[i].x * candidates[i].x +
                        candidates[i].y * candidates[i].y;
    if (candidates[i].y > 0.0) {
      if (d_sq < left_dist_sq) {
        left_dist_sq = d_sq;
        left_seed = i;
      }
    } else if (candidates[i].y < 0.0) {
      if (d_sq < right_dist_sq) {
        right_dist_sq = d_sq;
        right_seed = i;
      }
    }
    // y == 0.0인 점은 seed 선택에서 제외 (이후 노이즈 할당으로 처리)
  }

  if (left_seed < 0 && right_seed < 0) return result;

  // ── DFS 체이닝 (visited 공유) ──
  std::vector<bool> visited(n, false);

  std::vector<ChainedPoint> left_raw, right_raw;

  if (left_seed >= 0) {
    left_raw = chain_one_side(candidates, left_seed, true, visited, params);
  }
  if (right_seed >= 0) {
    right_raw = chain_one_side(candidates, right_seed, false, visited, params);
  }

  // ── 노이즈 할당 (병렬적 — 원본 chain 점만 참조) ──
  // DFS로 확정된 chain 포인트만으로 참조 집합 생성
  // 각 노이즈는 다른 노이즈를 참조하지 않음
  const size_t left_size = left_raw.size();
  const size_t right_size = right_raw.size();

  for (int i = 0; i < n; ++i) {
    if (visited[i]) continue;

    // 가장 가까운 chain 점 찾기 (left + right 원본만 참조)
    double min_dist_sq = std::numeric_limits<double>::max();
    bool assign_left = true;

    for (size_t j = 0; j < left_size; ++j) {
      const double dx = candidates[i].x - left_raw[j].x;
      const double dy = candidates[i].y - left_raw[j].y;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq < min_dist_sq) {
        min_dist_sq = d_sq;
        assign_left = true;
      }
    }
    for (size_t j = 0; j < right_size; ++j) {
      const double dx = candidates[i].x - right_raw[j].x;
      const double dy = candidates[i].y - right_raw[j].y;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq < min_dist_sq) {
        min_dist_sq = d_sq;
        assign_left = false;
      }
    }

    // 할당
    if (assign_left) {
      left_raw.push_back(candidates[i]);
    } else {
      right_raw.push_back(candidates[i]);
    }
    visited[i] = true;
  }

  // ── 리샘플링 ──
  const double ds = params.chainer.resample_ds;

  if (left_raw.size() >= 2) {
    result.left_chain = resample_chain(left_raw, ds);
  } else {
    result.left_chain = left_raw;
  }

  if (right_raw.size() >= 2) {
    result.right_chain = resample_chain(right_raw, ds);
  } else {
    result.right_chain = right_raw;
  }

  result.valid = (!result.left_chain.empty() || !result.right_chain.empty());
  return result;
}

}  // namespace planning_lc_ver
