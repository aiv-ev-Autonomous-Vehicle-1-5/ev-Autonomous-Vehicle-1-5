/**
 * @file pair_validator.cpp
 * @brief 좌/우 경계 폴리라인 쌍 유효성 검증 구현
 *
 * 파이프라인 Step (2): CorridorBuilder의 결과를 검증하여
 * 다음 단계(VirtualBoundary, CenterlineBuilder)에서 신뢰할 수 있는
 * 입력만 사용하도록 보장한다.
 *
 * 검증 실패 원인:
 *   - 교차(crossing): 좌/우 경계가 서로 역전됨 (w < 0)
 *   - 비평행(not parallel): 접선 방향 편차 과다 (angle_mean >= theta_mean_th)
 *   - 폭 범위 이탈: width_median < w_min 또는 > w_max
 *   - 폭 불균일: width_std > w_std_th (차로 폭이 일정하지 않음)
 */
#include "track_planning/corridor/pair_validator.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace track_planning
{

/**
 * @brief 좌/우 폴리라인 쌍의 유효성을 검증한다.
 *
 * 상세 알고리즘:
 *
 * [Step 1] 최소 점 수 확인
 *   - 각 측에 최소 2점이 있어야 폴리라인이 유효함
 *
 * [Step 2] 균일 리샘플링
 *   - resample_ds 간격으로 양쪽 폴리라인을 리샘플링
 *   - 짧은 쪽 길이(n)를 기준으로 비교 (길이 불일치 대응)
 *
 * [Step 3] 접선 계산 및 각도 편차 수집
 *   - polyline_tangents()로 각 점의 접선 벡터 계산
 *   - hL = heading(tL[i]), hR = heading(tR[i])
 *   - dtheta = |angle_diff(hL, hR)|  (방향 편차, [0, pi])
 *
 * [Step 4] 부호 있는 폭 계산
 *   - diff = right_rs[i] - left_rs[i]
 *   - w = cross(tL[i], diff)
 *     → w > 0: 우측이 좌측의 진행방향 오른쪽 (정상)
 *     → w < 0: 경계가 역전됨 (교차) → 즉시 무효 반환
 *
 * [Step 5] 교차 검사
 *   - widths 중 w < 0 이 하나라도 있으면 has_crossing = true → 무효
 *
 * [Step 6] 각도 통계 검사
 *   - angle_mean = sum(angles) / n
 *   - angle_max = max(angles)
 *   - angle_mean >= theta_mean_th OR angle_max >= theta_max_th → 무효
 *   (좌/우 경계가 평행하지 않으면 올바른 차로가 아님)
 *
 * [Step 7] 폭 통계 계산
 *   - width_median: 중앙값 (홀수/짝수 처리)
 *   - width_std: 표준편차 (분산의 제곱근)
 *
 * [Step 8] 폭 범위 검사
 *   - width_median < w_min OR > w_max → 무효 (너무 좁거나 넓은 차로)
 *
 * [Step 9] 폭 균일성 검사
 *   - width_std > w_std_th → 무효 (폭이 일정하지 않음)
 *
 * [Step 10] 모든 검사 통과 → result.valid = true
 */
PairResult PairValidator::validate(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  const PlanningParams & p)
{
  PairResult result;  // 기본값: valid = false

  // [Step 1] 최소 2점 확인 (폴리라인 유효 여부)
  if (left.size() < 2 || right.size() < 2) {
    return result;  // 점이 부족하면 검증 불가 → 무효
  }

  // [Step 2] 균일 리샘플링
  // ds 간격으로 양쪽을 리샘플링하여 점 개수와 간격을 통일
  const double ds = p.pair.resample_ds;   // 리샘플링 간격 [m]
  auto left_rs = resample_polyline(left, ds);
  auto right_rs = resample_polyline(right, ds);

  // 짧은 쪽 길이를 기준으로 비교 (좌/우 길이가 다를 수 있음)
  const size_t n = std::min(left_rs.size(), right_rs.size());
  if (n < 2) {
    return result;  // 리샘플링 후 점이 부족 → 무효
  }

  // [Step 3] 접선 벡터 계산
  // polyline_tangents: 각 점에서 전/후 점을 이용해 접선 방향 계산
  auto tL = polyline_tangents(left_rs);   // 좌측 각 점의 접선 단위벡터
  auto tR = polyline_tangents(right_rs);  // 우측 각 점의 접선 단위벡터

  // 각도 편차와 부호 있는 폭을 수집할 배열
  std::vector<double> angles;  // 각 점에서의 좌/우 접선 방향 편차 [rad]
  std::vector<double> widths;  // 각 점에서의 부호 있는 차로 폭 [m]
  angles.reserve(n);
  widths.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    // [Step 3] 접선 방향 편차 계산
    double hL = heading(tL[i]);   // 좌측 접선 방향각 [rad]
    double hR = heading(tR[i]);   // 우측 접선 방향각 [rad]
    // angle_diff: 두 각도의 최단 차이 (-pi ~ pi), abs로 절댓값
    double dtheta = std::abs(angle_diff(hL, hR));
    angles.push_back(dtheta);

    // [Step 4] 부호 있는 폭 계산
    // diff = right - left: 좌측 점에서 우측 점으로의 벡터
    Point2D diff = right_rs[i] - left_rs[i];
    // w = cross(tL, diff):
    //   양수(+): 우측 점이 좌측 접선방향의 오른쪽 → 정상 배치
    //   음수(-): 우측 점이 좌측 접선방향의 왼쪽 → 경계 교차 (비정상)
    double w = cross2(tL[i], diff);
    widths.push_back(w);
  }

  // [Step 5] 교차(crossing) 검사: 폭이 한 번이라도 음수이면 경계가 교차됨
  bool has_crossing = false;
  for (double w : widths) {
    if (w < 0.0) {
      has_crossing = true;
      break;
    }
  }
  if (has_crossing) {
    return result;  // 경계 교차 → 즉시 무효 반환
  }

  // [Step 6] 각도 통계 계산 및 검사
  double angle_sum = 0.0;
  double angle_max = 0.0;
  for (double a : angles) {
    angle_sum += a;
    angle_max = std::max(angle_max, a);
  }
  const double angle_mean = angle_sum / static_cast<double>(angles.size());

  // 평균 또는 최대 각도 편차가 임계값 초과 시 무효
  // (좌/우 경계가 평행하지 않으면 올바른 차로가 아님)
  if (angle_mean >= p.pair.theta_mean_th || angle_max >= p.pair.theta_max_th) {
    result.angle_mean = angle_mean;  // 진단용으로 기록
    return result;  // 비평행 → 무효
  }

  // [Step 7] 폭 통계 계산

  // 중앙값(median) 계산: 정렬 후 중간값 선택
  std::vector<double> sorted_w = widths;
  std::sort(sorted_w.begin(), sorted_w.end());
  double width_median;
  if (sorted_w.size() % 2 == 0) {
    // 짝수 개: 중간 두 값의 평균
    width_median = 0.5 * (sorted_w[sorted_w.size() / 2 - 1] + sorted_w[sorted_w.size() / 2]);
  } else {
    // 홀수 개: 정중앙 값
    width_median = sorted_w[sorted_w.size() / 2];
  }

  // 표준편차(std) 계산: sqrt(분산)
  double w_mean = 0.0;
  for (double w : widths) w_mean += w;
  w_mean /= static_cast<double>(widths.size());  // 산술 평균

  double w_var = 0.0;
  for (double w : widths) {
    double d = w - w_mean;
    w_var += d * d;  // 편차의 제곱합
  }
  w_var /= static_cast<double>(widths.size());  // 분산
  const double width_std = std::sqrt(w_var);    // 표준편차

  // [Step 8] 폭 범위 검사
  // width_median이 [w_min, w_max] 범위를 벗어나면 무효
  // (너무 좁거나 너무 넓은 차로는 올바르지 않음)
  if (width_median < p.pair.w_min || width_median > p.pair.w_max) {
    result.width_median = width_median;
    result.width_std = width_std;
    result.angle_mean = angle_mean;
    return result;  // 폭 범위 이탈 → 무효
  }

  // [Step 9] 폭 균일성 검사
  // width_std 가 임계값 초과 시 무효 (폭이 불균일한 차로)
  if (width_std > p.pair.w_std_th) {
    result.width_median = width_median;
    result.width_std = width_std;
    result.angle_mean = angle_mean;
    return result;  // 폭 불균일 → 무효
  }

  // [Step 10] 모든 검사 통과 → 유효한 쌍
  result.valid = true;
  result.width_median = width_median;  // 차로 폭 중앙값 [m] (w_hat 갱신에 사용)
  result.width_std = width_std;        // 차로 폭 표준편차 [m] (진단용)
  result.angle_mean = angle_mean;      // 접선 방향 평균 편차 [rad] (진단용)

  return result;
}

}  // namespace track_planning
