/**
 * @file path_postprocessor.hpp
 * @brief 경로 후처리기 — prune, smooth, resample, yaw 4단계 파이프라인
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: MagneticPlanner가 생성한 raw_path를 control이 사용하기 좋게 다듬는다.
 *
 * 4단계 후처리:
 *   1. Prune (가지치기) — Greedy Shortcut으로 불필요한 중간점 제거
 *   2. Smooth (스무딩) — 이동 평균 필터로 경로 부드럽게
 *   3. Resample (리샘플) — 균일한 간격(resample_ds)으로 재배치
 *   4. Yaw (heading 계산) — 각 점의 접선 방향 계산
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
#define PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_

#include "planning_mr_ver/common/types.hpp"

#include <vector>

namespace planning_mr_ver
{

class PathPostprocessor
{
public:
  /**
   * @brief raw_path에 4단계 후처리를 적용하여 최종 경로 생성
   *
   * @param raw_path      MagneticPlanner가 생성한 원본 경로
   * @param prune_max_dev 가지치기 최대 횡편차 [m]
   * @param smooth_window 이동 평균 윈도우 크기
   * @param resample_ds   리샘플링 간격 [m]
   * @return PostprocessResult 후처리된 경로 + yaw + valid 플래그
   */
  PostprocessResult process(
    const std::vector<Point2D> & raw_path,
    double prune_max_dev,
    int smooth_window,
    double resample_ds);

private:
  /**
   * @brief Greedy Shortcut 가지치기
   *
   * 직선으로 건너뛸 수 있는 중간점을 제거한다.
   * 모든 중간점이 직선으로부터 max_dev 이내에 있으면 건너뛰기 가능.
   *
   * 예: A-B-C-D에서 A→D 직선으로부터 B,C가 모두 max_dev 이내이면
   *     A-D로 단축 (B,C 제거)
   *
   * @param pts     입력 경로
   * @param max_dev 최대 허용 수직거리 [m]
   * @return 가지치기된 경로 (점 수 ≤ 입력)
   */
  static std::vector<Point2D> prune(
    const std::vector<Point2D> & pts, double max_dev);

  /**
   * @brief 이동 평균(Moving Average) 스무딩
   *
   * 각 점을 주변 window 크기만큼의 점들의 평균으로 대체한다.
   * 시작점과 끝점은 보존한다 (시작/끝이 움직이면 경로가 이탈).
   *
   * @param pts    입력 경로
   * @param window 윈도우 크기 (홀수 권장, 예: 5 → 좌2+자신+우2)
   * @return 스무딩된 경로
   */
  static std::vector<Point2D> smooth(
    const std::vector<Point2D> & pts, int window);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
