/**
 * @file path_postprocessor.hpp
 * @brief 원시 경로(raw path)를 후처리하는 PathPostprocessor 클래스 헤더
 *
 * ## 역할
 * - DTR 센터라인에서 생성된 원시 경로를 부드럽고 균일한 경로로 변환한다.
 * - 제어기(컨트롤러)에 직접 전달할 수 있는 형태로 가공한다.
 *
 * ## 4단계 후처리 파이프라인 (process())
 *
 *  단계 1: Prune (가지치기 / 숏컷 제거)
 *   - 원시 경로의 불필요한 중간 포인트(노이즈, 이상점)를 제거
 *   - Greedy shortcut pruning으로 중간 포인트 제거
 *   - 두 끝점을 잇는 직선에서 중간 포인트들의 수직 거리(perpendicular distance)가
 *     prune_max_dev 이하이면 중간 포인트를 모두 건너뜀
 *
 *  단계 2: Smooth (이동 평균 스무딩)
 *   - smooth_window 크기의 이동 평균으로 경로를 부드럽게 만듦
 *   - 양쪽 끝점(start, goal)은 보존 (이동 제약)
 *
 *  단계 3: Resample (균일 간격 재샘플링)
 *   - geometry.hpp의 resample_polyline()으로 resample_ds 간격으로 재샘플링
 *   - 제어기가 일정한 간격의 waypoint를 기대할 때 필요
 *
 *  단계 4: Yaw 계산
 *   - polyline_tangents()로 각 포인트의 접선 방향 계산
 *   - heading()으로 접선을 yaw 각도(라디안)로 변환
 *
 * ## Greedy Shortcut Pruning 알고리즘
 *  i=0 (시작점)에서 출발하여 가능한 한 가장 먼 j를 탐색:
 *   - j = pts.size()-1부터 i+1까지 역순으로 시도
 *   - i→j 직선에서 모든 중간점(k=i+1..j-1)의 수직 거리 ≤ max_dev이면 숏컷 허용
 *   - 허용되면 best_j = j, break
 *   - result에 pts[best_j] 추가, i = best_j
 *  이를 통해 원시 경로의 불필요한 중간 포인트를 제거하고 직선에 가깝게 단순화
 */

#ifndef TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
#define TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_

#include "track_planning/common/types.hpp"

#include <vector>

namespace track_planning
{

/**
 * @class PathPostprocessor
 * @brief 원시 경로를 후처리하는 파이프라인 클래스
 *
 * process()로 prune → smooth → resample → yaw 4단계를 한 번에 실행.
 */
class PathPostprocessor
{
public:
  /**
   * @brief 전체 후처리 파이프라인 실행
   *
   * @param raw_path       입력 원시 경로 (DTR 센터라인)
   * @param prune_max_dev  숏컷 허용 최대 수직 편차(m)
   *                       크게 설정할수록 더 많이 잘라냄 (직선화)
   *                       작게 설정할수록 원래 경로에 가까움
   * @param smooth_window  이동 평균 윈도우 크기 (홀수 권장)
   *                       1이면 스무딩 없음, 클수록 더 부드러움
   * @param resample_ds    재샘플링 간격(m) — 출력 waypoint 간 거리
   * @return               후처리된 경로 결과 (PostprocessResult)
   */
  PostprocessResult process(
    const std::vector<Point2D> & raw_path,
    double prune_max_dev,
    int smooth_window,
    double resample_ds);

private:
  /**
   * @brief Greedy shortcut pruning — 불필요한 중간 포인트 제거
   *
   * ## 알고리즘
   *  result = [pts[0]]  // 시작점 고정
   *  i = 0
   *  while i < pts.size()-1:
   *    best_j = i+1  // 기본값: 다음 포인트
   *    for j = pts.size()-1 downto i+2:  // 가장 먼 점부터 시도
   *      ab_len = dist(pts[i], pts[j])
   *      dir = (pts[j] - pts[i]) / ab_len  // 단위 방향 벡터
   *      can_shortcut = true
   *      for k = i+1 to j-1:  // 중간 포인트 검사
   *        perp = |cross2(pts[k]-pts[i], dir)|  // 수직 거리
   *        if perp > max_dev: can_shortcut=false; break
   *      if can_shortcut: best_j=j; break
   *    result.append(pts[best_j])
   *    i = best_j
   *  return result
   *
   * 수직 거리 계산: perp = |dx * dir.y - dy * dir.x|
   *  여기서 dx = pts[k].x - pts[i].x, dy = pts[k].y - pts[i].y
   *  (2D 외적의 절댓값 = 수직 거리)
   *
   * @param pts     입력 포인트 목록
   * @param max_dev 허용 최대 수직 편차(m)
   * @return        가지치기된 포인트 목록
   */
  static std::vector<Point2D> prune(
    const std::vector<Point2D> & pts, double max_dev);

  /**
   * @brief 이동 평균(Moving Average) 스무딩
   *
   * ## 알고리즘
   *  for i = 1 to n-2:  // 끝점(0, n-1)은 보존
   *    lo = max(0, i - half)    half = window / 2
   *    hi = min(n-1, i + half)
   *    result[i] = mean(pts[lo..hi])  // X, Y 각각 평균
   *
   * ## 경계 처리
   *  - 양쪽 끝점은 항상 원본 값 유지 (start/goal 위치 고정)
   *  - 경계 근처는 가용한 포인트로만 평균 (비대칭 윈도우)
   *
   * @param pts    입력 포인트 목록
   * @param window 윈도우 크기 (1이면 스무딩 없음)
   * @return       스무딩된 포인트 목록
   */
  static std::vector<Point2D> smooth(
    const std::vector<Point2D> & pts, int window);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
