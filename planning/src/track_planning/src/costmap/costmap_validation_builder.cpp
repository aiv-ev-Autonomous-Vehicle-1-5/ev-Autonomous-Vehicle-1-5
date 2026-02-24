/**
 * @file costmap_validation_builder.cpp
 * @brief CostmapValidationBuilder 구현부
 *
 * ## 5-레이어 빌드 흐름
 *  1. Base : 전체 셀 = -1 (unknown)
 *  2. Boundary Barrier : 좌/우 경계 폴리라인 → 100 (occupied)
 *  3. Boundary Inflation : 경계 셀 주변 반경 내 → 99 (inflated)
 *  4. Obstacle : 콘 + 장애물 포인트 → 100 (occupied)
 *  5. Obstacle Inflation : 장애물 셀 주변 반경 내 → 99 (inflated)
 *
 * ## 좌표 변환
 *  월드(m)  →  그리드 셀(정수 인덱스) : floor((w - origin) / resolution)
 *  row-major 1D 인덱스 : idx = row * width + col
 *
 * ## Bresenham 라인
 *  정수 연산만으로 두 셀 사이의 모든 셀을 방문하는 알고리즘.
 *  대각선 이동도 포함하여 경계 폴리라인을 픽셀-완전하게 그린다.
 */

#include "track_planning/costmap/costmap_validation_builder.hpp"
#include "track_planning/costmap/inflation.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace track_planning
{

// ============================================================
// World → Grid conversion
// ============================================================
/**
 * @brief 월드 좌표(wx, wy)를 그리드 셀(col, row)로 변환
 *
 * floor()를 사용하여 음수 좌표에서도 올바른 셀 인덱스를 반환한다.
 * 예) wx=1.9m, origin=0.0m, res=0.1m → col=19
 *
 * @return 셀이 그리드 내에 있으면 true, 범위 밖이면 false
 */
bool CostmapValidationBuilder::world_to_grid(
  double wx, double wy,
  double origin_x, double origin_y, double resolution,
  int width, int height,
  int & col, int & row)
{
  // 월드 X 좌표에서 원점을 빼고 해상도로 나눠 열 인덱스 계산
  col = static_cast<int>(std::floor((wx - origin_x) / resolution));
  // 월드 Y 좌표에서 원점을 빼고 해상도로 나눠 행 인덱스 계산
  row = static_cast<int>(std::floor((wy - origin_y) / resolution));
  // 그리드 범위 [0, width) × [0, height) 내에 있는지 반환
  return (col >= 0 && col < width && row >= 0 && row < height);
}

// ============================================================
// Bresenham line drawing
// ============================================================
/**
 * @brief Bresenham 정수 라인 알고리즘으로 (x0,y0)→(x1,y1) 직선 그리기
 *
 * ## 알고리즘 원리
 *  dx = |x1 - x0|        (X축 이동 거리)
 *  dy = -|y1 - y0|       (Y축 이동 거리, 음수로 저장하여 누산에 활용)
 *  sx = x 이동 방향 (+1 or -1)
 *  sy = y 이동 방향 (+1 or -1)
 *  err = dx + dy          (초기 오차값)
 *
 *  매 스텝에서 2*err를 계산:
 *   - 2*err >= dy 이면 X축으로 한 칸 이동 (err += dy)
 *   - 2*err <= dx 이면 Y축으로 한 칸 이동 (err += dx)
 *  이를 통해 실수 연산 없이 직선에 가장 가까운 셀을 방문한다.
 *
 * @param grid   대상 그리드 버퍼 (row-major)
 * @param width  그리드 폭 (열 수)
 * @param height 그리드 높이 (행 수)
 * @param x0,y0 시작 셀 좌표
 * @param x1,y1 끝 셀 좌표
 * @param value  셀에 기록할 값
 */
void CostmapValidationBuilder::draw_line(
  std::vector<int8_t> & grid, int width, int height,
  int x0, int y0, int x1, int y1,
  int8_t value)
{
  int dx = std::abs(x1 - x0);           // X 방향 절대 거리
  int dy = -std::abs(y1 - y0);          // Y 방향 절대 거리 (음수로 저장)
  int sx = (x0 < x1) ? 1 : -1;         // X 이동 방향
  int sy = (y0 < y1) ? 1 : -1;         // Y 이동 방향
  int err = dx + dy;                    // 초기 오차값

  while (true) {
    // 현재 셀이 그리드 범위 내에 있을 때만 값 기록
    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
      // row-major 인덱스: y0 * width + x0
      grid[y0 * width + x0] = value;
    }

    // 끝점에 도달하면 루프 종료
    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    // X축 이동 조건: 오차가 Y 방향보다 크거나 같으면 X 방향으로 한 칸 이동
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    // Y축 이동 조건: 오차가 X 방향보다 작거나 같으면 Y 방향으로 한 칸 이동
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

// ============================================================
// Rasterize polyline (connected segments)
// ============================================================
/**
 * @brief 폴리라인의 각 선분을 Bresenham으로 그리드에 래스터화
 *
 * 연속된 두 점 사이를 draw_line()으로 연결하여
 * 폴리라인 전체를 그리드에 픽셀-완전하게 그린다.
 * 그리드 범위를 벗어난 부분은 draw_line 내부에서 자동 클리핑됨.
 *
 * @param grid   대상 그리드 버퍼
 * @param pts    폴리라인 포인트 목록 (월드 좌표, 최소 2개 필요)
 * @param value  셀에 기록할 값 (예: 100 = occupied)
 */
void CostmapValidationBuilder::rasterize_polyline(
  std::vector<int8_t> & grid, int width, int height,
  const std::vector<Point2D> & pts,
  double resolution, double origin_x, double origin_y,
  int8_t value)
{
  // 포인트가 2개 미만이면 선분을 그릴 수 없음
  if (pts.size() < 2) return;

  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    int c0, r0, c1, r1;
    // 각 선분의 두 끝점을 그리드 좌표로 변환
    // (범위 밖이어도 draw_line에서 클리핑하므로 반환값 무시)
    world_to_grid(pts[i].x, pts[i].y, origin_x, origin_y, resolution, width, height, c0, r0);
    world_to_grid(pts[i + 1].x, pts[i + 1].y, origin_x, origin_y, resolution, width, height, c1, r1);
    // Bresenham 라인으로 두 셀 사이를 연결
    draw_line(grid, width, height, c0, r0, c1, r1, value);
  }
}

// ============================================================
// Rasterize individual points
// ============================================================
/**
 * @brief 포인트 목록을 그리드 셀에 1:1로 래스터화
 *
 * 각 포인트를 world_to_grid로 변환하고 해당 셀에 value를 기록.
 * 콘(cone) 또는 장애물 포인트를 단일 셀로 마킹할 때 사용.
 * 그리드 범위 밖의 포인트는 무시됨.
 *
 * @param grid   대상 그리드 버퍼
 * @param pts    포인트 목록 (월드 좌표)
 * @param value  셀에 기록할 값 (예: 100 = occupied)
 */
void CostmapValidationBuilder::rasterize_points(
  std::vector<int8_t> & grid, int width, int height,
  const std::vector<Point2D> & pts,
  double resolution, double origin_x, double origin_y,
  int8_t value)
{
  for (const auto & pt : pts) {
    int col, row;
    // 월드 좌표 → 그리드 셀 변환 후 범위 내 셀만 기록
    if (world_to_grid(pt.x, pt.y, origin_x, origin_y, resolution, width, height, col, row)) {
      grid[row * width + col] = value;
    }
  }
}

// ============================================================
// Build validation costmap
// ============================================================
/**
 * @brief 5-레이어 구조의 검증용 코스트맵 빌드
 *
 * ## 빌드 순서
 *
 * [Layer 1: Base]
 *   - cost_array 전체를 -1(unknown)으로 초기화
 *   - A*가 아닌 경계/장애물 레이어가 덮어씀
 *
 * [Layer 2: Boundary Barrier]
 *   - 별도 버퍼(boundary_grid)에 좌/우 경계 폴리라인을 100으로 래스터화
 *   - 경계만 분리 저장하는 이유: 팽창 시 경계만 선택적으로 팽창하기 위함
 *
 * [Layer 3: Boundary Inflation]
 *   - boundary_grid를 inflate_grid()로 팽창 (반경 = boundary_radius)
 *   - 팽창값 = 99 (경계 100과 구별 가능, A*는 99 셀도 높은 비용으로 통과 가능)
 *   - 팽창 후 boundary_grid를 cost_array에 병합 (> 0 인 셀만 복사)
 *
 * [Layer 4: Obstacle]
 *   - 별도 버퍼(obstacle_grid)에 콘 + 장애물을 100으로 래스터화
 *
 * [Layer 5: Obstacle Inflation]
 *   - obstacle_grid를 inflate_grid()로 팽창 (반경 = obstacle_radius)
 *   - 팽창값 = 99
 *   - 팽창 후 obstacle_grid를 cost_array에 병합 (장애물이 경계보다 우선)
 *
 * [OccupancyGrid 메시지 조립]
 *   - cost_array를 그대로 grid_msg.data에 복사
 *   - frame_id = "base_link"
 *
 * @param corridor_left  좌측 경계 폴리라인 (월드 좌표)
 * @param corridor_right 우측 경계 폴리라인 (월드 좌표)
 * @param cones          콘 포인트 목록
 * @param obstacles      동적 장애물 포인트 목록
 * @param p              플래닝 파라미터 (ROI, inflation 반경)
 * @return               빌드 결과 (Result.valid == false이면 빈 그리드)
 */
CostmapValidationBuilder::Result CostmapValidationBuilder::build(
  const std::vector<Point2D> & corridor_left,
  const std::vector<Point2D> & corridor_right,
  const std::vector<Point2D> & cones,
  const std::vector<Point2D> & obstacles,
  const PlanningParams & p)
{
  Result result;
  // 그리드 해상도와 원점을 파라미터에서 가져옴
  result.resolution = p.roi.resolution;
  result.origin_x = p.roi.x_min;
  result.origin_y = p.roi.y_min;
  // ROI 범위를 해상도로 나눠 그리드 크기 계산 (ceil로 올림하여 모든 셀 포함)
  result.width = static_cast<int>(
    std::ceil((p.roi.x_max - p.roi.x_min) / p.roi.resolution));
  result.height = static_cast<int>(
    std::ceil((p.roi.y_max - p.roi.y_min) / p.roi.resolution));

  const int total = result.width * result.height;
  if (total <= 0) return result;  // 유효하지 않은 ROI

  // ---- Layer 1: Base unknown (-1) ----
  // 전체 그리드를 -1(unknown)으로 초기화
  // -1은 ROS OccupancyGrid에서 "미탐색 영역"을 의미
  result.cost_array.assign(static_cast<size_t>(total), -1);

  // ---- Layer 2: Boundary Barrier (occupied = 100) ----
  // 경계 팽창을 별도로 수행하기 위해 분리된 버퍼 사용
  std::vector<int8_t> boundary_grid(static_cast<size_t>(total), 0);

  // 좌측 코리도 경계를 100(occupied)으로 래스터화
  rasterize_polyline(
    boundary_grid, result.width, result.height,
    corridor_left, result.resolution, result.origin_x, result.origin_y, 100);
  // 우측 코리도 경계를 100(occupied)으로 래스터화
  rasterize_polyline(
    boundary_grid, result.width, result.height,
    corridor_right, result.resolution, result.origin_x, result.origin_y, 100);

  // ---- Layer 3: Boundary Inflation ----
  // boundary_radius > 0일 때만 팽창 수행
  if (p.inflation.boundary_radius > 0.0) {
    inflation::inflate_grid(
      boundary_grid, result.width, result.height,
      result.resolution, p.inflation.boundary_radius,
      100, 99);  // 팽창값 = 99 (100=실제 경계, 99=팽창 영역으로 구별)
  }

  // 경계 버퍼를 cost_array에 병합 (값이 0보다 큰 셀만 복사)
  for (int i = 0; i < total; ++i) {
    if (boundary_grid[i] > 0) {
      result.cost_array[i] = boundary_grid[i];
    }
  }

  // ---- Layer 4: Obstacle (cones + obstacles) occupied (100) ----
  // 장애물 팽창을 별도로 수행하기 위해 분리된 버퍼 사용
  std::vector<int8_t> obstacle_grid(static_cast<size_t>(total), 0);

  // 콘을 100(occupied)으로 래스터화
  rasterize_points(
    obstacle_grid, result.width, result.height,
    cones, result.resolution, result.origin_x, result.origin_y, 100);
  // 동적 장애물을 100(occupied)으로 래스터화
  rasterize_points(
    obstacle_grid, result.width, result.height,
    obstacles, result.resolution, result.origin_x, result.origin_y, 100);

  // ---- Layer 5: Obstacle Inflation ----
  // obstacle_radius > 0일 때만 팽창 수행
  if (p.inflation.obstacle_radius > 0.0) {
    inflation::inflate_grid(
      obstacle_grid, result.width, result.height,
      result.resolution, p.inflation.obstacle_radius,
      100, 99);  // 팽창값 = 99
  }

  // 장애물 버퍼를 cost_array에 병합
  // 장애물이 경계보다 높은 값을 가질 때만 덮어씀 (장애물 우선)
  for (int i = 0; i < total; ++i) {
    if (obstacle_grid[i] > 0 && obstacle_grid[i] > result.cost_array[i]) {
      result.cost_array[i] = obstacle_grid[i];
    }
  }

  // ---- Build OccupancyGrid message ----
  // ROS 메시지 헤더 설정 (base_link 기준 좌표계)
  result.grid_msg.header.frame_id = "base_link";
  // 그리드 해상도 (m/cell)
  result.grid_msg.info.resolution = static_cast<float>(result.resolution);
  // 그리드 가로/세로 셀 수
  result.grid_msg.info.width = static_cast<uint32_t>(result.width);
  result.grid_msg.info.height = static_cast<uint32_t>(result.height);
  // 그리드 원점(좌하단) 위치 (월드 좌표)
  result.grid_msg.info.origin.position.x = result.origin_x;
  result.grid_msg.info.origin.position.y = result.origin_y;
  result.grid_msg.info.origin.position.z = 0.0;
  result.grid_msg.info.origin.orientation.w = 1.0;  // 회전 없음 (단위 쿼터니언)

  // cost_array를 그대로 메시지 데이터로 복사
  result.grid_msg.data.resize(static_cast<size_t>(total));
  for (int i = 0; i < total; ++i) {
    result.grid_msg.data[i] = result.cost_array[i];
  }

  result.valid = true;
  return result;
}

// ============================================================
// DIRECT mode: check centerline collision
// ============================================================
/**
 * @brief DIRECT 모드 검증: 센터라인이 코스트맵 장애물/경계와 충돌하는지 확인
 *
 * ## 동작 방식
 *  1. 센터라인을 ds_check 간격으로 균등 리샘플링
 *  2. 각 샘플 포인트를 world_to_grid로 그리드 셀로 변환
 *  3. 그리드 범위 밖 포인트 → 충돌로 보수적 처리 (true 반환)
 *  4. cost_array[셀] >= cost_th 이면 충돌 감지 (true 반환)
 *  5. 모든 포인트가 통과하면 충돌 없음 (false 반환)
 *
 * @param costmap    build()로 생성된 코스트맵
 * @param centerline 검사할 센터라인 폴리라인 (최소 2개 포인트 필요)
 * @param ds_check   샘플링 간격(m), 작을수록 정밀하지만 계산량 증가
 * @param cost_th    충돌 판정 임계값 (이 값 이상 셀 = 충돌)
 * @return           true = 충돌 있음 (ASTAR 모드 전환 필요)
 */
bool CostmapValidationBuilder::check_centerline_collision(
  const Result & costmap,
  const std::vector<Point2D> & centerline,
  double ds_check,
  int cost_th)
{
  // 코스트맵이 유효하지 않거나 센터라인이 너무 짧으면 충돌로 처리
  if (!costmap.valid || centerline.size() < 2) return true;

  // 센터라인을 ds_check 간격으로 균등 리샘플링 (geometry.hpp의 resample_polyline 사용)
  auto sampled = resample_polyline(centerline, ds_check);

  for (const auto & pt : sampled) {
    int col, row;
    if (!world_to_grid(
          pt.x, pt.y,
          costmap.origin_x, costmap.origin_y, costmap.resolution,
          costmap.width, costmap.height,
          col, row))
    {
      // 포인트가 그리드 범위 밖 → 보수적으로 충돌 처리
      return true;
    }

    // row-major 1D 인덱스 계산
    const int idx = row * costmap.width + col;
    // 셀 비용이 임계값 이상이면 충돌
    if (costmap.cost_array[idx] >= static_cast<int8_t>(cost_th)) {
      return true;
    }
  }

  // 모든 샘플 포인트 통과 → 충돌 없음
  return false;
}

}  // namespace track_planning
