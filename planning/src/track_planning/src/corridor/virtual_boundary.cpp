/**
 * @file virtual_boundary.cpp
 * @brief 가상 경계(Virtual Boundary) 생성 구현
 *
 * 파이프라인 Step (3): 한쪽 경계가 없을 때 추정 차로 폭(w_hat)으로 반대쪽 가상 경계 생성.
 *
 * 핵심 기하 연산:
 *   - rotate90(t): 접선 t를 CCW 90도 회전 → 좌측 법선(left normal) 생성
 *     예: t = (1, 0) → rotate90(t) = (0, 1) (위쪽 = 좌측)
 *   - sgn 결정:
 *     visible이 좌측(left) → 가상은 우측(right) → 우측 방향 = 법선 반대 → sgn = -1
 *     visible이 우측(right) → 가상은 좌측(left) → 좌측 방향 = 법선 방향 → sgn = +1
 *   - 가상점: p_virtual = visible[i] + (sgn * w_hat) * rotate90(tangent[i])
 *
 * w_hat(추정 차로 폭) 관리:
 *   - 양쪽 corridor 모두 보일 때 median_width 직접 계산하여 w_hat_ 갱신
 *   - 한쪽만 보일 때 직전 w_hat_ 사용, 초기값: default_track_width
 */
#include "track_planning/corridor/virtual_boundary.hpp"
#include "track_planning/common/geometry.hpp"

#include <cmath>

namespace track_planning
{

/**
 * @brief 보이는 경계에서 반대쪽 가상 경계를 생성한다.
 *
 * [Step 1] 최소 요건 확인
 *   - visible 길이 < 2: 폴리라인 없음 → 실패
 *   - w_hat < vehicle.width + safety.margin: 가상 경계가 차량보다 좁음 → 실패
 *
 * [Step 2] 접선 계산
 *   - polyline_tangents(visible): 각 점의 접선 단위벡터 배열 생성
 *
 * [Step 3] 법선 방향 부호 결정
 *   - rotate90(t): CCW 회전으로 좌측 법선(n) 생성
 *   - visible이 좌측이면 → 가상은 우측 → 우측으로 이동 → sgn = -1
 *   - visible이 우측이면 → 가상은 좌측 → 좌측으로 이동 → sgn = +1
 *
 * [Step 4] 가상점 생성 및 ROI 필터링
 *   - p_virtual = visible[i] + sgn * w_hat * n
 *   - ROI 범위 [x_min, x_max] x [y_min, y_max] 밖이면 해당 점 제외
 *   - 일부 점이 ROI 밖이어도 계속 진행 (부분 가상 경계도 유용)
 *
 * [Step 5] 결과 확인
 *   - boundary 크기 >= 2이면 success = true
 */
VirtualBoundaryResult VirtualBoundary::generate(
  const std::vector<Point2D> & visible,
  bool visible_is_left,
  double w_hat,
  const PlanningParams & p)
{
  VirtualBoundaryResult result;  // 기본값: success = false, boundary 비어있음

  // [Step 1-a] visible 폴리라인 최소 길이 확인
  if (visible.size() < 2) {
    return result;  // 폴리라인이 없으면 가상 경계 생성 불가
  }

  // [Step 1-b] 최소 차로 폭 확인
  // 가상 경계가 차량 폭 + 안전 여유보다 좁으면 주행 불가능하므로 실패 처리
  const double min_width = p.vehicle.width + p.safety.margin;
  if (w_hat < min_width) {
    return result;  // 추정 차로 폭이 너무 좁음 → 가상 경계 생성 안전하지 않음
  }

  // [Step 2] visible 폴리라인의 각 점에서 접선 벡터 계산
  auto tangents = polyline_tangents(visible);  // 각 점의 접선 단위벡터 배열

  result.boundary.reserve(visible.size());  // 결과 배열 메모리 미리 할당

  // [Step 3] 법선 방향 부호 결정
  // rotate90(t) = (-t.y, t.x): CCW 90도 회전 → 좌측 법선(n) 생성
  // 부호(sgn):
  //   visible이 좌측(LEFT): 가상은 우측 → 좌측 법선 반대 방향 → sgn = -1
  //   visible이 우측(RIGHT): 가상은 좌측 → 좌측 법선 방향 → sgn = +1
  const double sgn = visible_is_left ? -1.0 : 1.0;

  // [Step 4] 각 visible 점에서 가상점 생성
  for (size_t i = 0; i < visible.size(); ++i) {
    const Point2D n = rotate90(tangents[i]);  // 좌측 법선 단위벡터
    // 가상점 = visible 점에서 법선 방향으로 w_hat 만큼 이동
    Point2D p_virtual = visible[i] + (sgn * w_hat) * n;

    // ROI 범위 확인: 범위 밖이면 해당 점 스킵 (전체 실패하지 않음)
    if (p_virtual.x < p.roi.x_min || p_virtual.x > p.roi.x_max ||
        p_virtual.y < p.roi.y_min || p_virtual.y > p.roi.y_max)
    {
      // ROI 밖 점은 제외하지만 나머지 점은 계속 처리
      // (부분적 가상 경계도 centerline 계산에 유용할 수 있음)
      continue;
    }

    result.boundary.push_back(p_virtual);
  }

  // [Step 5] 결과 확인: 2점 이상이어야 유효한 폴리라인
  result.success = (result.boundary.size() >= 2);
  return result;
}

}  // namespace track_planning
