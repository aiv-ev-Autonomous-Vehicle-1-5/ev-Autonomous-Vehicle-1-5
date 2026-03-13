/**
 * @file path_postprocessor.cpp
 * @brief PathPostprocessor — process() 메인 오케스트레이터
 *
 * 5단계 파이프라인의 진입점만 포함.
 * 각 단계의 구현은 별도 .cpp 파일로 분리되어 있다:
 *   - prune.cpp:           prune() — Douglas-Peucker 유사 경로 단순화
 *   - smooth.cpp:          smooth() — 이동 평균 필터
 *   - curvature_clamp.cpp: curvature_clamp() — 최대 곡률 제한
 *
 * [파이프라인 요약]
 *   ① prune → ② resample → ③ smooth → ④ curvature_clamp → ⑤ yaw
 */
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"    // resample_polyline, polyline_tangents, heading

#include <cmath>

namespace chaining_costmap_ver
{

PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds,
  double kappa_max,
  int curvature_clamp_max_iter)
{
  PostprocessResult result;

  // 경로가 2점 미만이면 의미 있는 후처리 불가
  if (raw_path.size() < 2) return result;

  // ① Prune: 직선 구간의 중간점 제거 (점 수 대폭 감소)
  auto pruned = prune(raw_path, prune_max_dev);
  result.pruned = pruned;

  // ② Resample: 등간격(ds) 리샘플링
  auto resampled = resample_polyline(pruned, resample_ds);

  // ③ Smooth: 이동 평균 필터로 잔여 꺾임 완화
  auto smoothed = smooth(resampled, smooth_window);

  // ④ Curvature Clamp: 최대 곡률 제한
  if (kappa_max > 0.0 && smoothed.size() >= 3) {
    smoothed = curvature_clamp(smoothed, kappa_max, curvature_clamp_max_iter);
  }

  result.path = smoothed;

  if (result.path.size() < 2) return result;

  // ⑤ Yaw 계산: 접선 벡터 → 헤딩 각도
  auto tangents = polyline_tangents(result.path);
  result.yaw.resize(result.path.size());
  for (size_t i = 0; i < tangents.size(); ++i) {
    result.yaw[i] = heading(tangents[i]);
  }

  result.valid = true;
  return result;
}

}  // namespace chaining_costmap_ver
