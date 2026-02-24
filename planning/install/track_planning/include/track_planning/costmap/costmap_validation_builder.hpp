/**
 * @file costmap_validation_builder.hpp
 * @brief 검증용 코스트맵(OccupancyGrid)을 5-레이어 구조로 빌드하는 클래스 헤더
 *
 * ## 역할
 * - 트랙 주행 가능 여부를 판단하기 위한 코스트맵을 구축한다.
 * - 센터라인이 장애물/경계에 충돌하는지 확인하는 데 사용된다 (DIRECT 모드 검증).
 *
 * ## 5-레이어 빌드 순서 (build() 참조)
 *  Layer 1 : Base — 전체 셀을 unknown(-1)으로 초기화
 *  Layer 2 : Boundary Barrier — 좌/우 코리도 폴리라인을 100(occupied)으로 래스터화
 *  Layer 3 : Boundary Inflation — 경계 장벽을 반경만큼 팽창하여 99(inflated)로 표시
 *  Layer 4 : Obstacle — 콘 + 동적장애물 포인트를 100(occupied)으로 래스터화
 *  Layer 5 : Obstacle Inflation — 장애물 셀을 반경만큼 팽창하여 99(inflated)로 표시
 *
 * ## 핵심 알고리즘
 * - world_to_grid  : 월드 좌표(m) → 그리드 셀 인덱스 (floor 기반)
 * - Bresenham line : 픽셀 단위 라인 그리기 (draw_line)
 * - rasterize_polyline : 폴리라인의 각 선분을 Bresenham으로 그리드에 그림
 * - rasterize_points   : 포인트 목록을 그리드 셀에 1:1 매핑
 * - inflate_grid (inflation.hpp) : 점유 셀 주변을 원형 커널로 팽창
 */

#ifndef TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_
#define TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <vector>

namespace track_planning
{

/**
 * @class CostmapValidationBuilder
 * @brief 센터라인 충돌 검증을 위한 OccupancyGrid 코스트맵 빌더
 *
 * build()로 코스트맵을 생성하고, check_centerline_collision()으로
 * 센터라인이 장애물/경계와 충돌하는지 검사한다.
 */
class CostmapValidationBuilder
{
public:
  /**
   * @struct Result
   * @brief build()의 출력 구조체
   *
   * @param grid_msg     ROS OccupancyGrid 메시지 (퍼블리시용)
   * @param cost_array   내부 비용 버퍼 (int8_t, row-major, -1=unknown, 0=free, 99=inflated, 100=occupied)
   * @param width        그리드 열(column) 수
   * @param height       그리드 행(row) 수
   * @param resolution   셀 하나의 크기(미터)
   * @param origin_x     그리드 좌하단 월드 X 좌표(m)
   * @param origin_y     그리드 좌하단 월드 Y 좌표(m)
   * @param valid        빌드 성공 여부
   */
  struct Result
  {
    nav_msgs::msg::OccupancyGrid grid_msg;
    std::vector<int8_t> cost_array;  // 내부 비용 버퍼 (직접 A*/충돌 검사에 사용)
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    bool valid = false;
  };

  /**
   * @brief 검증용 코스트맵을 5-레이어로 빌드
   *
   * @param corridor_left  좌측 코리도 경계 폴리라인 (월드 좌표)
   * @param corridor_right 우측 코리도 경계 폴리라인 (월드 좌표)
   * @param cones          콘(교통 콘) 포인트 목록 (월드 좌표)
   * @param obstacles      동적 장애물 포인트 목록 (월드 좌표)
   * @param p              플래닝 파라미터 (ROI 범위, inflation 반경 등)
   * @return               빌드된 코스트맵 결과
   */
  Result build(
    const std::vector<Point2D> & corridor_left,
    const std::vector<Point2D> & corridor_right,
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & obstacles,
    const PlanningParams & p);

  /**
   * @brief DIRECT 모드 검증: 센터라인이 코스트맵과 충돌하는지 확인
   *
   * 센터라인을 ds_check 간격으로 리샘플링한 후 각 점의 코스트를 조회한다.
   * cost_th 이상인 셀을 만나면 충돌로 판정한다.
   *
   * @param costmap    build()로 생성된 코스트맵
   * @param centerline 검사할 센터라인 폴리라인
   * @param ds_check   샘플링 간격(m)
   * @param cost_th    충돌 판정 임계값 (이 값 이상이면 충돌)
   * @return           true = 충돌 감지 (DIRECT 모드 실패)
   */
  static bool check_centerline_collision(
    const Result & costmap,
    const std::vector<Point2D> & centerline,
    double ds_check,
    int cost_th);

private:
  /**
   * @brief 월드 좌표(wx, wy)를 그리드 셀(col, row)로 변환
   *
   * 변환 공식: col = floor((wx - origin_x) / resolution)
   *           row = floor((wy - origin_y) / resolution)
   *
   * @param wx,wy      월드 좌표(m)
   * @param origin_x,origin_y  그리드 원점(m)
   * @param resolution 셀 크기(m)
   * @param width,height 그리드 크기(셀 단위)
   * @param col,row    [출력] 셀 인덱스
   * @return           그리드 범위 내에 있으면 true
   */
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y, double resolution,
    int width, int height,
    int & col, int & row);

  /**
   * @brief 연결된 선분으로 구성된 폴리라인을 그리드에 래스터화
   *
   * 인접한 두 점 사이를 Bresenham 알고리즘으로 선 그리기.
   * 그리드 범위를 벗어난 부분은 draw_line 내부에서 클리핑됨.
   *
   * @param grid       대상 그리드 버퍼
   * @param pts        폴리라인 점 목록
   * @param value      채울 값 (예: 100 = occupied)
   */
  static void rasterize_polyline(
    std::vector<int8_t> & grid, int width, int height,
    const std::vector<Point2D> & pts,
    double resolution, double origin_x, double origin_y,
    int8_t value);

  /**
   * @brief 포인트 목록을 그리드 셀에 1:1 래스터화 (셀 한 개씩)
   *
   * 각 포인트를 world_to_grid로 변환하고 해당 셀에 value를 기록.
   * 그리드 범위 외 포인트는 무시됨.
   *
   * @param grid   대상 그리드 버퍼
   * @param pts    포인트 목록
   * @param value  채울 값 (예: 100 = occupied)
   */
  static void rasterize_points(
    std::vector<int8_t> & grid, int width, int height,
    const std::vector<Point2D> & pts,
    double resolution, double origin_x, double origin_y,
    int8_t value);

  /**
   * @brief Bresenham 정수 라인 알고리즘으로 두 셀 사이를 직선으로 그림
   *
   * - dx, dy : 두 점 사이의 절댓값 차이
   * - sx, sy : 각 축의 이동 방향 (+1 or -1)
   * - err     : 오차 누산값 (err = dx + dy, 정확히는 dy가 음수)
   * - 그리드 범위를 체크하며 범위 내 셀만 value로 설정
   *
   * @param x0,y0  시작 셀
   * @param x1,y1  끝 셀
   * @param value  채울 값
   */
  static void draw_line(
    std::vector<int8_t> & grid, int width, int height,
    int x0, int y0, int x1, int y1,
    int8_t value);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_
