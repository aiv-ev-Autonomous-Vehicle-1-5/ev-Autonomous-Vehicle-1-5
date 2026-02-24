/**
 * @file inflation.hpp
 * @brief 2D 그리드에서 점유 셀을 원형 커널로 팽창(inflation)하는 유틸리티 헤더
 *
 * ## 역할
 * - 코스트맵에서 경계(boundary) 및 장애물(obstacle) 셀 주변을 팽창하여
 *   차량이 장애물/경계에 근접하면 높은 비용을 가지도록 만든다.
 * - A* 플래너는 팽창 영역(99)을 통과 가능하지만 높은 비용을 부여받아
 *   자연스럽게 장애물에서 멀어지는 경로를 선택하게 된다.
 *
 * ## 알고리즘 개요 (inflate_grid)
 *  1. 원형 마스크 사전 계산 : 반경 r_cells 내의 모든 (dx, dy) 오프셋 목록 생성
 *     조건: dx² + dy² ≤ r_cells²  (정수 연산으로 원 내부 판별)
 *  2. 소스 셀 수집 : grid에서 value >= occupied_th 인 셀 인덱스 목록 생성
 *     (반복 중 그리드를 수정하면 오류가 발생하므로 먼저 수집)
 *  3. 팽창 적용 : 각 소스 셀에서 원형 마스크의 모든 오프셋을 적용
 *     - 이미 점유(>= occupied_th)된 셀은 덮어쓰지 않음 (우선순위 보존)
 *     - 범위 밖 셀은 건너뜀
 */

#ifndef TRACK_PLANNING__COSTMAP__INFLATION_HPP_
#define TRACK_PLANNING__COSTMAP__INFLATION_HPP_

#include <cstdint>
#include <vector>

namespace track_planning
{
namespace inflation
{

/**
 * @brief 2D 그리드에서 점유 셀을 원형 커널로 팽창
 *
 * ## 파라미터 설명
 * @param grid           row-major 1D 그리드 (크기 = width * height)
 *                       -1=unknown, 0=free, 1~99=비용, 100=occupied
 * @param width          그리드 열(column) 수
 * @param height         그리드 행(row) 수
 * @param resolution     셀 하나의 실제 크기(m/cell)
 * @param radius         팽창 반경(m) — 셀 단위 반경 = ceil(radius / resolution)
 * @param occupied_th    이 값 이상인 셀을 팽창 소스로 간주
 *                       (예: 100으로 설정하면 완전 점유 셀만 팽창)
 * @param inflated_value 팽창 영역에 기록할 값 (예: 99)
 *                       occupied_th보다 작아야 소스 셀 값을 보존할 수 있음
 *
 * ## 주의사항
 * - radius <= 0 또는 resolution <= 0이면 아무 작업도 하지 않음
 * - 이미 occupied_th 이상인 셀은 inflated_value로 덮어쓰지 않음
 */
void inflate_grid(
  std::vector<int8_t> & grid,
  int width,
  int height,
  double resolution,
  double radius,
  int8_t occupied_th,
  int8_t inflated_value);

}  // namespace inflation
}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__INFLATION_HPP_
