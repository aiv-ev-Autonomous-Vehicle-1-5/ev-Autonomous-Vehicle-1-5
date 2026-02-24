/**
 * @file corridor_builder.hpp
 * @brief 코리도(주행 경계) 빌더 — lane + cone 점들을 greedy chaining으로 연결
 *
 * 파이프라인 Step (1): Perception이 제공하는 차선점 + 콘 점을 좌/우 경계 폴리라인으로 변환.
 * 알고리즘:
 *   1. 참조점(c_end) + 접선(t_end) 결정 (이전 centerline 또는 ego heading)
 *   2. 좌측 경계 greedy chaining: seed → filter → score → select 반복
 *   3. 우측 경계 동일 과정
 *   4. 콘 우선(cone priority): 같은 단계에서 콘과 차선이 모두 있으면 콘을 우선 선택
 */
#ifndef TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_
#define TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <utility>
#include <vector>

namespace track_planning
{

class CorridorBuilder
{
public:
  /// 입력 데이터: 좌/우 차선점, 좌/우 콘, ego 상태, 이전 centerline
  struct Input
  {
    std::vector<Point2D> lane_left;        // 좌측 차선 경계점 (perception 출력)
    std::vector<Point2D> lane_right;       // 우측 차선 경계점 (perception 출력)
    std::vector<Point2D> cone_left;        // 좌측 콘 중심점 (DBSCAN 클러스터 중심)
    std::vector<Point2D> cone_right;       // 우측 콘 중심점
    Point2D ego_pos{0.0, 0.0};            // ego 위치 (base_link 원점, 차량 중심)
    Point2D ego_heading{1.0, 0.0};        // ego heading 단위벡터 (기본: +x 전방)
    std::vector<Point2D> centerline_prev;  // 이전 프레임의 centerline (cold start 시 비어있음)
  };

  /// 좌우 경계 폴리라인 생성 (메인 진입점)
  /// @param in  perception 입력 데이터
  /// @param p   플래닝 파라미터
  /// @return    좌/우 경계 폴리라인과 유효 여부
  CorridorPolylines build(const Input & in, const PlanningParams & p);

private:
  /// 참조점(c_end)과 참조 접선(t_end) 결정
  /// - centerline_prev가 충분히 길면 (warm start): 끝점 + 회귀 접선 사용
  /// - 비어있으면 (cold start): ego_pos + ego_heading 사용
  /// @return {c_end: 참조점, t_end: 참조 접선 단위벡터}
  std::pair<Point2D, Point2D> determine_reference(
    const Input & in, const PlanningParams & p);

  /// 한 쪽 경계를 greedy chaining으로 구축
  /// seed 찾기 → 반복: {filter → score → select → chain에 추가}
  /// @param lane_pts  차선 경계점 배열
  /// @param cone_pts  콘 중심점 배열
  /// @param c_end     참조점 (체이닝 기준점)
  /// @param t_end     참조 접선 단위벡터
  /// @param ego_pos   ego 위치 (seed 탐색 기준)
  /// @return          greedy chaining 으로 연결된 경계 폴리라인
  std::vector<Point2D> build_one_side(
    const std::vector<Point2D> & lane_pts,
    const std::vector<Point2D> & cone_pts,
    const Point2D & c_end,
    const Point2D & t_end,
    const Point2D & ego_pos,
    const PlanningParams & p);

  /// seed 점 찾기: x >= x_min인 점 중 ego에 가장 가까운 점
  /// 콘 우선: 콘에서 seed가 발견되면 차선 seed보다 우선 사용
  /// @param seed_out  찾은 seed 점 (출력)
  /// @return          seed 발견 여부
  bool find_seed(
    const std::vector<Point2D> & lane_pts,
    const std::vector<Point2D> & cone_pts,
    const Point2D & ego,
    double x_min,     // seed 탐색 최소 x 값 (ego 뒤쪽 제외)
    Point2D & seed_out);

  /// s/d 좌표계 기반 후보점 필터링
  /// s = dot(diff, t_end): 참조 접선 방향 전방 진행거리 (양수 = 전방)
  /// d = |cross(t_end, diff)|: 접선에 수직인 횡방향 거리
  /// 조건: s_min < s < s_max, d < d_max, dist(pt, p_k) < r_search
  /// @return  필터를 통과한 점의 인덱스 목록
  std::vector<size_t> filter_candidates(
    const std::vector<Point2D> & pts,
    const Point2D & c_end,   // 현재 참조점
    const Point2D & t_end,   // 현재 참조 접선
    const Point2D & p_k,     // 현재 chain 끝점 (원형 탐색 중심)
    const PlanningParams & p);

  /// 후보점에 점수를 매겨 최적 점 선택
  /// score = w_s*norm_s - w_d*norm_d - w_a*norm_a - w_p*norm_p
  ///   norm_s: 전방 진행 보상 [0,1]
  ///   norm_d: 횡방향 편차 패널티 [0,1]
  ///   norm_a: 방향 편차 패널티 [0,1]
  ///   norm_p: 예측 오차 패널티 [0,1]
  /// @param t_k  현재 chain 끝점의 로컬 접선 (예측 방향 기준)
  /// @return     최적 인덱스 (-1이면 선택 실패)
  int score_and_select(
    const std::vector<Point2D> & pts,
    const std::vector<size_t> & candidates,
    const Point2D & c_end,
    const Point2D & t_end,
    const Point2D & p_k,
    const Point2D & t_k,
    const PlanningParams & p);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_
