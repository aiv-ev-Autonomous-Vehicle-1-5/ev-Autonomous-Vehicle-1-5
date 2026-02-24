/**
 * @file goal_selector.hpp
 * @brief A* 플래너의 목표 셀(goal)을 선택하는 GoalSelector 클래스 헤더
 *
 * ## 역할
 * - 현재 ego 위치와 속도를 바탕으로 A*의 목표 포인트를 결정한다.
 * - 두 가지 방법을 순서대로 시도하여 유효한 목표를 선택한다.
 *
 * ## 목표 선택 전략
 *
 * ### Method 1a: path_prev 룩어헤드 (시간 연속성 우선)
 *   - 이전 프레임에서 계획된 경로(path_prev)를 따라 lookahead 거리만큼 전진
 *   - 시간적 연속성을 제공하여 경로가 갑자기 바뀌는 것을 방지
 *   - path_prev가 없거나 목표 포인트가 occupied이면 실패 → Method 1b로 폴백
 *
 * ### Method 1b: 센터라인 룩어헤드
 *   - 트랙 센터라인을 따라 lookahead 거리만큼 전진
 *   - 목표 포인트가 occupied이면 끝점 → 역방향 탐색 순으로 폴백
 *   - 센터라인이 없거나 모두 occupied이면 실패 → Method 2로 폴백
 *
 * ### Method 2: 링 샘플링 (최후 수단)
 *   - ego 주변 lookahead 반경의 원(ring) 위에 n_samples 개 포인트 균등 배치
 *   - 각 포인트를 스코어링하여 최고점 선택
 *   - 스코어 = 0.4 * heading_score + 0.3 * progress_score + 0.3 * ref_score
 *
 * ## lookahead 거리 계산
 *   lookahead = lookahead_l0 + lookahead_kv * ego_speed
 *   속도가 높을수록 더 멀리 앞을 바라봄 (Pure Pursuit 스타일)
 */

#ifndef TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_
#define TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/**
 * @class GoalSelector
 * @brief A* 플래너의 목표 포인트를 선택하는 클래스
 *
 * 1a → 1b → 2 순서로 시도하며 처음 성공한 결과를 반환한다.
 */
class GoalSelector
{
public:
  /**
   * @brief A* 목표 포인트 선택
   *
   * @param centerline  트랙 센터라인 폴리라인 (월드 좌표)
   * @param path_prev   이전 프레임 경로 (시간 연속성용, 없으면 빈 벡터)
   * @param grid        주행 가능 마스크 (0=free, 100=occupied)
   * @param width       그리드 열 수
   * @param height      그리드 행 수
   * @param resolution  셀 크기(m/cell)
   * @param origin_x    그리드 원점 X(m)
   * @param origin_y    그리드 원점 Y(m)
   * @param ego_pos     자차 위치 (월드 좌표)
   * @param ego_heading 자차 진행 방향 단위벡터
   * @param ego_speed   자차 속도(m/s) — lookahead 거리 계산에 사용
   * @param p           플래닝 파라미터 (lookahead_l0, lookahead_kv, ring_samples)
   * @return            GoalResult (valid=true이면 목표 포인트 사용 가능)
   */
  GoalResult select(
    const std::vector<Point2D> & centerline,
    const std::vector<Point2D> & path_prev,
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos,
    const Point2D & ego_heading,
    double ego_speed,
    const PlanningParams & p);

private:
  /**
   * @brief Method 1: 참조 폴리라인을 따라 lookahead 거리의 목표 포인트 선택
   *
   * ## 탐색 로직
   *  1. 폴리라인을 처음부터 순회하며 누적 거리(accum)를 계산
   *  2. accum + seg_len >= lookahead 인 선분에서 보간으로 lookahead 위치 계산
   *  3. 해당 포인트가 free이면 반환 (score=1.0)
   *  4. occupied이면 break → 끝점 시도 (score=0.5)
   *  5. 끝점도 occupied이면 역방향으로 탐색 (score=0.3)
   *  6. 모두 실패하면 valid=false 반환
   *
   * @param ref_line    참조 폴리라인 (path_prev 또는 centerline)
   * @param lookahead   목표 거리(m)
   * @param method_id   결과의 method 필드에 저장할 식별자
   * @return            GoalResult (valid=false이면 폴백 필요)
   */
  static GoalResult method_lookahead(
    const std::vector<Point2D> & ref_line,
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    double lookahead,
    uint8_t method_id);

  /**
   * @brief Method 2: ego 주변 ring(원) 위의 포인트를 샘플링하여 최고 스코어 선택
   *
   * ## 샘플링
   *  - lookahead 반경의 원 위에 n_samples 개를 균등 배치
   *  - theta = -π + 2π * i / n_samples  (i = 0, ..., n_samples-1)
   *  - candidate = (ego.x + lookahead*cos(θ), ego.y + lookahead*sin(θ))
   *
   * ## 스코어 계산 (free 셀인 경우만)
   *  heading_score  = 1 - |wrap_pi(angle_to_candidate - heading_angle)| / π
   *                   (0=정반대 방향, 1=전방 방향)
   *  progress_score = dot(candidate - ego, ego_heading) / lookahead
   *                   (전방 진행도, 음수=후방)
   *  ref_score      = max(0, 1 - min_dist_to_ref / lookahead)
   *                   (센터라인에 가까울수록 높은 점수)
   *  total_score    = 0.4*heading + 0.3*progress + 0.3*ref
   *
   * @param ego_pos     자차 위치
   * @param ego_heading 자차 진행 방향 단위벡터
   * @param lookahead   링 반경(m)
   * @param n_samples   링 위 샘플 수
   * @param ref_line    참조 폴리라인 (ref_score 계산용)
   * @return            최고 스코어의 GoalResult
   */
  static GoalResult method_ring(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos,
    const Point2D & ego_heading,
    double lookahead,
    int n_samples,
    const std::vector<Point2D> & ref_line);

  /**
   * @brief 월드 포인트 pt가 그리드에서 free(0) 셀인지 확인
   *
   * 범위 밖 포인트는 false 반환 (보수적 처리).
   *
   * @param pt 확인할 월드 좌표
   * @return   free(0)이면 true
   */
  static bool is_free(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & pt);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_
