/**
 * @file path_postprocessor.hpp
 * @brief 경로 후처리기 — prune, smooth, resample, yaw 4단계 파이프라인
 *
 * ──────────────────────────────────────────────────────────────────
 * [전체 파이프라인에서의 위치]
 *
 *   MagneticPlanner가 costmap 위에서 greedy forward search로 생성한
 *   "원시 경로(raw path)"는 다음과 같은 문제를 가진다:
 *
 *   (1) 격자 해상도에 의한 지그재그 — costmap grid cell 단위로 이동하므로
 *       미세한 꺾임이 다수 발생한다.
 *   (2) 불필요한 중간점 — 거의 직선인 구간에도 매 cell마다 점이 존재한다.
 *   (3) 불균등한 점 간격 — 격자 대각선 vs 직교 이동에 따라 간격이 다르다.
 *   (4) yaw(헤딩) 정보 없음 — 차량 제어에는 각 waypoint의 목표 헤딩이 필요하다.
 *
 *   PathPostprocessor는 이 4가지 문제를 순차적으로 해결하여,
 *   차량 제어기(Pure Pursuit, Stanley 등)에 적합한
 *   "깨끗하고 균등한 경로 + yaw 배열"을 출력한다.
 *
 * ──────────────────────────────────────────────────────────────────
 * [4단계 파이프라인 요약]
 *
 *   raw_path (MagneticPlanner 출력)
 *       │
 *       ▼
 *   ① Prune (Douglas-Peucker 유사 단순화)
 *       │  - 직선 구간의 중간점을 제거하여 점 수를 대폭 줄임
 *       │  - max_dev 이하로 편차가 있는 중간점들은 건너뜀(shortcut)
 *       ▼
 *   ② Smooth (이동 평균 필터)
 *       │  - 남은 꺾임을 윈도우 크기 만큼의 이웃 평균으로 완화
 *       │  - 시작점/끝점은 보존하여 경로 연속성 유지
 *       ▼
 *   ③ Resample (등간격 리샘플링)
 *       │  - smoothing 후 불균등해진 점 간격을 ds 간격으로 균일하게 재배치
 *       │  - 제어기가 일정 간격 waypoint를 기대하므로 필수
 *       ▼
 *   ④ Yaw (접선 벡터 → 헤딩 각도)
 *       │  - 각 waypoint에서의 진행 방향(접선)을 구하고
 *       │  - atan2(dy, dx)로 yaw 각도 [rad]를 계산
 *       ▼
 *   PostprocessResult { path, yaw, valid }
 *       → SafetyChecker → 차량 제어기로 전달
 *
 * ──────────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_COSTMAP_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
#define CHAINING_COSTMAP_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_

#include "chaining_costmap_ver/common/types.hpp"

#include <vector>

namespace chaining_costmap_ver
{

/**
 * @class PathPostprocessor
 * @brief MagneticPlanner가 생성한 원시 경로를 차량 제어에 적합한 형태로 변환
 *
 * [사용 방법]
 *   PathPostprocessor pp;
 *   auto result = pp.process(raw_path, prune_max_dev, smooth_window, resample_ds);
 *   // result.path  → 등간격 waypoint 좌표 배열
 *   // result.yaw   → 각 waypoint의 목표 헤딩 [rad]
 *   // result.valid  → 경로 유효 여부
 */
class PathPostprocessor
{
public:
  /**
   * @brief 4단계 파이프라인 실행: prune → smooth → resample → yaw 계산
   *
   * @param raw_path       MagneticPlanner가 출력한 원시 경로 (Point2D 배열)
   * @param prune_max_dev  [m] prune 단계에서 허용하는 최대 수직 편차
   *                       - 값이 클수록 더 공격적으로 점을 제거 (직선화)
   *                       - 값이 작을수록 원본 형태를 더 보존
   *                       - 일반적으로 0.05 ~ 0.3m 범위
   * @param smooth_window  smooth 단계의 이동 평균 윈도우 크기 (홀수 권장)
   *                       - 값이 클수록 더 부드러운 경로 (그러나 원형 손실 증가)
   *                       - 값이 1 이하이면 스무딩 비활성화
   * @param resample_ds    [m] resample 단계의 등간격 거리
   *                       - 제어기의 lookahead 거리와 매칭하면 좋음
   *                       - 일반적으로 0.1 ~ 0.5m
   * @return PostprocessResult 후처리 완료된 경로 + yaw 배열
   */
  /**
   * @param kappa_max  [1/m] 최대 허용 곡률 (0.0이면 비활성화)
   *                   kappa_max = 1/R_min = 1/2.68 ≈ 0.373 for T870
   */
  PostprocessResult process(
    const std::vector<Point2D> & raw_path,
    double prune_max_dev,
    int smooth_window,
    double resample_ds,
    double kappa_max = 0.0);

private:
  /**
   * @brief ① Prune 단계 — Douglas-Peucker 유사 알고리즘으로 경로 단순화
   *
   * [알고리즘 원리]
   *   Douglas-Peucker와 유사하지만 "탐욕적(greedy)" 방식이다.
   *   시작점 i에서 가능한 한 먼 점 j까지 직선(i→j)을 긋고,
   *   i와 j 사이의 모든 중간점 k에 대해 수직 편차(perpendicular distance)를
   *   계산한다. 모든 중간점의 편차가 max_dev 이하이면 중간점들을 모두
   *   건너뛰고(shortcut) j로 점프한다.
   *
   *   이렇게 하면 거의 직선인 구간의 점 수가 대폭 줄어들고,
   *   실제 커브가 있는 구간의 점은 보존된다.
   *
   * [max_dev의 역할]
   *   "이 정도 편차는 직선으로 간주해도 된다"는 허용 오차이다.
   *   - max_dev = 0.05m → 5cm 이내 편차는 무시, 세밀한 커브 보존
   *   - max_dev = 0.3m  → 30cm 이내 편차까지 무시, 공격적 단순화
   *
   * @param pts     입력 경로 점 배열
   * @param max_dev [m] 허용 최대 수직 편차 — 이 값 이하면 중간점 제거
   * @return 단순화된 경로 점 배열 (점 수 ≤ 입력 점 수)
   */
  static std::vector<Point2D> prune(
    const std::vector<Point2D> & pts, double max_dev);

  /**
   * @brief ② Smooth 단계 — 이동 평균(Moving Average) 필터로 경로 평활화
   *
   * [알고리즘 원리]
   *   각 중간점 i에 대해, 윈도우 범위 [i - half, i + half] 내의
   *   점들의 x, y 좌표를 산술 평균하여 새 좌표로 대체한다.
   *
   *   이동 평균 필터는 고주파 노이즈(지그재그)를 제거하면서도
   *   저주파 형태(커브의 대략적 형태)는 보존하는 효과가 있다.
   *
   * [시작점/끝점 보존 이유]
   *   - 시작점: 차량의 현재 위치에 해당 → 이동시키면 안 됨
   *   - 끝점:   목표 지점에 해당 → 이동시키면 경로가 목표에 도달하지 못함
   *   - 따라서 index 0과 index (n-1)은 평균 처리에서 제외
   *
   * @param pts    입력 경로 점 배열
   * @param window 이동 평균 윈도우 크기 (예: 5이면 앞뒤 2개씩 총 5개 평균)
   * @return 평활화된 경로 점 배열 (점 수 동일, 시작/끝점 좌표 보존)
   */
  static std::vector<Point2D> smooth(
    const std::vector<Point2D> & pts, int window);

  /**
   * @brief ②½ Curvature Clamp — 최대 곡률 제한
   *
   * 각 triplet(i-1, i, i+1)의 Menger 곡률이 kappa_max를 초과하면
   * 중간점 i를 곡률 원의 중심 방향으로 밀어서 곡률을 낮춘다.
   * 차량의 최소 회전 반경을 보장하기 위한 후처리.
   *
   * @param pts       입력 경로
   * @param kappa_max [1/m] 최대 허용 곡률
   * @return 곡률이 제한된 경로
   */
  static std::vector<Point2D> curvature_clamp(
    const std::vector<Point2D> & pts, double kappa_max);
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
