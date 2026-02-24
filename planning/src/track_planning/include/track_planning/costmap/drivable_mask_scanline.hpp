/**
 * @file drivable_mask_scanline.hpp
 * @brief 스캔라인 채우기 방식으로 주행 가능 영역(Drivable Mask)을 생성하는 클래스 헤더
 *
 * ## 역할
 * - A* 플래너가 탐색할 수 있는 자유(free) 영역을 정의하는 그리드를 생성한다.
 * - 코리도(corridor) 좌/우 경계 사이를 주행 가능 영역으로, 그 외는 점유로 처리한다.
 * - 장애물/콘이 있는 셀은 다시 점유로 마킹하여 A*가 회피하도록 한다.
 *
 * ## 빌드 단계 (build() 참조)
 *  1. 그리드 전체를 100(occupied)으로 초기화
 *  2. 코리도 폴리라인을 resample_ds 간격으로 리샘플링
 *  3. 스캔라인 채우기: 각 그리드 열(X)에서
 *       - 좌/우 경계 폴리라인에서 Y값을 보간
 *       - Y 범위 내의 셀을 0(free)으로 채움
 *  4. 장애물/콘 포인트를 100(occupied)으로 래스터화
 *  5. 장애물을 obstacle_radius만큼 팽창
 *
 * ## 핵심 알고리즘: interpolate_y_at_x
 * - 폴리라인의 X값 범위에서 주어진 X에 해당하는 Y를 선형 보간
 * - 양방향 X(증가/감소) 폴리라인 모두 지원
 * - 수직 선분(dx ≈ 0)은 중간점 Y를 반환
 */

#ifndef TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_
#define TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/**
 * @class DrivableMaskScanline
 * @brief 스캔라인 채우기 기반 주행 가능 마스크 빌더
 *
 * A* 플래너의 탐색 공간을 정의하는 그리드를 생성한다.
 * 코리도 경계 사이를 free(0)으로, 장애물 영역을 occupied(100)으로 표시.
 */
class DrivableMaskScanline
{
public:
  /**
   * @struct Result
   * @brief build()의 출력 구조체
   *
   * @param grid       row-major 1D 그리드 (0=free, 100=occupied)
   * @param width      그리드 열(column) 수
   * @param height     그리드 행(row) 수
   * @param resolution 셀 크기(m/cell)
   * @param origin_x   그리드 좌하단 월드 X 좌표(m)
   * @param origin_y   그리드 좌하단 월드 Y 좌표(m)
   * @param valid      빌드 성공 여부
   */
  struct Result
  {
    std::vector<int8_t> grid;   // row-major: 0 = free(주행 가능), 100 = occupied(주행 불가)
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    bool valid = false;
  };

  /**
   * @brief 주행 가능 마스크 빌드 (5단계 파이프라인)
   *
   * 단계 1: 전체 그리드를 100(occupied)으로 초기화
   * 단계 2: 코리도 폴리라인을 그리드 해상도로 리샘플링
   * 단계 3: 스캔라인 채우기 — 각 열(X)에서 좌/우 경계 사이 행을 0(free)으로
   * 단계 4: 장애물/콘을 100(occupied)으로 래스터화
   * 단계 5: 장애물을 obstacle_radius만큼 팽창 (별도 버퍼 후 병합)
   *
   * @param corridor_left  좌측 코리도 경계 폴리라인 (최소 2개 포인트 필요)
   * @param corridor_right 우측 코리도 경계 폴리라인 (최소 2개 포인트 필요)
   * @param cones          콘 포인트 목록
   * @param obstacles      동적 장애물 포인트 목록
   * @param p              플래닝 파라미터 (ROI, inflation)
   * @return               빌드된 주행 마스크 결과
   */
  Result build(
    const std::vector<Point2D> & corridor_left,
    const std::vector<Point2D> & corridor_right,
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & obstacles,
    const PlanningParams & p);

private:
  /**
   * @brief 폴리라인에서 주어진 X 좌표에 해당하는 Y 값을 선형 보간
   *
   * 폴리라인의 인접한 두 점 사이에서 X가 해당 선분 범위 내에 있으면
   * Y를 선형 보간하여 반환한다.
   *
   * 예) pts = [(0,1), (2,3), (4,2)], x=1.0 → y=2.0 (첫 번째 선분)
   *
   * @param pts   폴리라인 포인트 목록
   * @param x     보간할 X 좌표 (월드 좌표)
   * @param y_out [출력] 보간된 Y 값
   * @return      폴리라인 X 범위 내에 있으면 true, 범위 밖이면 false
   *
   * @note 수직 선분(|dx| < 1e-12): 두 점의 Y 중간값 반환
   * @note X가 증가하는 폴리라인과 감소하는 폴리라인 모두 지원
   */
  static bool interpolate_y_at_x(
    const std::vector<Point2D> & pts,
    double x, double & y_out);

  /**
   * @brief 월드 좌표(wx, wy)를 그리드 셀(col, row)로 변환
   *
   * col = floor((wx - origin_x) / resolution)
   * row = floor((wy - origin_y) / resolution)
   *
   * @return 그리드 범위 내이면 true
   */
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y, double resolution,
    int width, int height,
    int & col, int & row);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_
