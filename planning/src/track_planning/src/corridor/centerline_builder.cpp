/**
 * @file centerline_builder.cpp
 * @brief 코리도 경계에서 중심선(centerline) 생성 구현
 *
 * 파이프라인 Step (4): 좌/우 경계 상태에 따라 3가지 방법 중 하나로 중심선 생성.
 *
 * Case 별 처리 방법:
 *
 *   Case 1 — build_from_pair() [pair_valid = true]:
 *     양쪽 경계가 모두 유효한 쌍을 이룰 때 (PairValidator 통과)
 *     - 균일 리샘플링으로 대응점 인덱스 일치
 *     - 중간값(midpoint): center[i] = (left[i] + right[i]) / 2
 *     - 가장 정확한 중심선 (두 경계 데이터를 모두 활용)
 *
 *   Case 2 — build_from_one_side() [virtual_used = true]:
 *     한쪽만 실측, 반대쪽은 VirtualBoundary에서 생성된 경우
 *     - 실측 경계(visible)의 각 점에서 법선 방향으로 w_hat/2 이동
 *     - rotate90(tangent): CCW 90도 법선 벡터 생성
 *     - sgn = (visible_is_left ? -1 : +1): 중심 방향 결정
 *     - w_hat 추정 오차가 중심선 오차에 직접 영향
 *
 *   Case 3 — 실패 [빈 CenterlineResult 반환]:
 *     두 경우 모두 해당 안 됨 (경계 데이터 부족 또는 불량)
 */
#include "track_planning/corridor/centerline_builder.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace track_planning
{

/**
 * @brief 코리도 경계에서 중심선을 생성한다. (메인 진입점)
 *
 * 판단 로직:
 *   - Case 1: pair_valid == true AND 양쪽 길이 >= 2
 *             → build_from_pair() 호출
 *   - Case 2: virtual_used == true AND visible 길이 >= 2
 *             → build_from_one_side() 호출
 *   - Case 3: 나머지 → 빈 결과 반환
 *
 * 우선순위: Case 1 > Case 2 > Case 3
 */
CenterlineResult CenterlineBuilder::build(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  bool pair_valid,
  bool virtual_used,
  bool visible_is_left,
  double w_hat,
  double resample_ds)
{
  // Case 1: 양쪽 경계가 모두 유효한 쌍을 이루는 경우 → midpoint 방법
  if (pair_valid && left.size() >= 2 && right.size() >= 2) {
    return build_from_pair(left, right, resample_ds);
  }

  // Case 2: 가상 경계 사용 → 한쪽에서 오프셋 방법
  if (virtual_used) {
    // visible 경계 선택: visible_is_left에 따라 left 또는 right 사용
    const auto & visible = visible_is_left ? left : right;
    if (visible.size() >= 2) {
      return build_from_one_side(visible, visible_is_left, w_hat, resample_ds);
    }
  }

  // Case 3: 어느 조건도 만족하지 않음 → 실패 (빈 결과)
  return CenterlineResult{};
}

/**
 * @brief Case 1: 양쪽 유효 경계에서 중간값 평균으로 중심선 생성
 *
 * 알고리즘:
 *   1. resample_polyline(): 양쪽을 resample_ds 간격으로 균일 리샘플링
 *      → 대응점(같은 인덱스의 점)이 비슷한 s 위치를 나타내도록 정렬
 *   2. n = min(left_rs.size(), right_rs.size()): 짧은 쪽 기준으로 비교
 *   3. center[i] = 0.5 * (left_rs[i] + right_rs[i]): 중간점 계산
 *      → 두 경계의 정중앙을 주행 목표선으로 설정
 *   4. 중심선 결과를 다시 resample_ds 로 리샘플링 (균일 출력 보장)
 */
CenterlineResult CenterlineBuilder::build_from_pair(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  double resample_ds)
{
  CenterlineResult result;

  // 양쪽을 균일 간격으로 리샘플링 (대응점 인덱스 맞추기 위함)
  auto left_rs = resample_polyline(left, resample_ds);
  auto right_rs = resample_polyline(right, resample_ds);

  // 짧은 쪽 길이를 기준으로 중간값 계산 (길이 불일치 대응)
  const size_t n = std::min(left_rs.size(), right_rs.size());
  if (n < 2) {
    return result;  // 리샘플링 후 점이 부족 → 실패
  }

  // 대응점 중간값 계산: center[i] = 0.5 * (left[i] + right[i])
  result.center.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    result.center.push_back({
      0.5 * (left_rs[i].x + right_rs[i].x),  // x 중간값
      0.5 * (left_rs[i].y + right_rs[i].y)   // y 중간값
    });
  }

  // 중심선 결과를 다시 균일 간격으로 리샘플링 (출력 균일성 보장)
  if (result.center.size() >= 2) {
    result.center = resample_polyline(result.center, resample_ds);
  }

  result.valid = (result.center.size() >= 2);  // 2점 이상이면 유효
  return result;
}

/**
 * @brief Case 2: 한쪽(visible) 경계에서 법선 방향 w_hat/2 오프셋으로 중심선 생성
 *
 * 알고리즘:
 *   1. resample_polyline(): visible을 균일 리샘플링
 *   2. polyline_tangents(): 각 점의 접선 단위벡터 계산
 *   3. 부호(sgn) 결정:
 *      - visible이 좌측(LEFT): 중심은 좌측 경계에서 오른쪽으로 w_hat/2 이동
 *        → rotate90(t)는 좌측 법선 → 오른쪽 = 법선 반대 → sgn = -1
 *      - visible이 우측(RIGHT): 중심은 우측 경계에서 왼쪽으로 w_hat/2 이동
 *        → 왼쪽 = 법선 방향 → sgn = +1
 *   4. offset = sgn * (w_hat / 2): 오프셋 거리 (음수: 오른쪽, 양수: 왼쪽)
 *   5. center[i] = vis_rs[i] + offset * rotate90(tangent[i])
 *   6. 결과를 다시 resample_ds 로 리샘플링
 *
 * 참고: rotate90(t) = (-t.y, t.x) — CCW 90도 회전 = 좌측 법선
 *   예: t = (1,0) → n = (0,1) (위쪽)
 *   예: t = (0,1) → n = (-1,0) (왼쪽)
 */
CenterlineResult CenterlineBuilder::build_from_one_side(
  const std::vector<Point2D> & visible,
  bool visible_is_left,
  double w_hat,
  double resample_ds)
{
  CenterlineResult result;

  // visible 경계를 균일 간격으로 리샘플링
  auto vis_rs = resample_polyline(visible, resample_ds);
  if (vis_rs.size() < 2) {
    return result;  // 리샘플링 후 점이 부족 → 실패
  }

  // 각 점의 접선 단위벡터 계산
  auto tangents = polyline_tangents(vis_rs);

  // 부호 결정:
  //   rotate90(t): CCW 회전 → 좌측 법선(left normal)
  //   visible이 LEFT: 중심은 오른쪽 방향 → 좌측 법선의 반대 → sgn = -1
  //   visible이 RIGHT: 중심은 왼쪽 방향 → 좌측 법선 방향 → sgn = +1
  const double sgn = visible_is_left ? -1.0 : 1.0;
  // 실제 오프셋 거리: 차로 폭의 절반을 법선 방향으로 이동
  const double offset = sgn * (w_hat / 2.0);

  result.center.reserve(vis_rs.size());
  for (size_t i = 0; i < vis_rs.size(); ++i) {
    const Point2D n = rotate90(tangents[i]);     // 좌측 법선 단위벡터 생성
    result.center.push_back(vis_rs[i] + offset * n);  // 오프셋 적용
  }

  // 결과를 다시 균일 간격으로 리샘플링 (출력 균일성 보장)
  if (result.center.size() >= 2) {
    result.center = resample_polyline(result.center, resample_ds);
  }

  result.valid = (result.center.size() >= 2);  // 2점 이상이면 유효
  return result;
}

}  // namespace track_planning
