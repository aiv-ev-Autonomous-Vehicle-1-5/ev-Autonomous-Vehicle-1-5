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
  int curvature_clamp_max_iter,
  const CostmapResult * costmap,
  double obstacle_cost)
{
  PostprocessResult result;

  // 경로가 2점 미만이면 의미 있는 후처리 불가
  if (raw_path.size() < 2) return result;

  // ① Prune: 직선 구간의 중간점 제거 (costmap obstacle 관통 shortcut 거부)
  auto pruned = prune(raw_path, prune_max_dev, costmap, obstacle_cost);
  result.pruned = pruned;

  // ② Resample: 등간격(ds) 리샘플링
  auto resampled = resample_polyline(pruned, resample_ds);

  // ③ Smooth: 이동 평균 필터 (obstacle 셀 침범 시 원래 좌표 유지)
  auto smoothed = smooth(resampled, smooth_window, costmap, obstacle_cost);

  // ④ Curvature Clamp: 최대 곡률 제한 (obstacle 셀 침범 시 이동 거부)
  if (kappa_max > 0.0 && smoothed.size() >= 3) {
    smoothed = curvature_clamp(smoothed, kappa_max, curvature_clamp_max_iter,
                               costmap, obstacle_cost);
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
