/**
 * @file drivable_mask_scanline.cpp
 * @brief DrivableMaskScanline 구현부 — 스캔라인 채우기로 주행 가능 영역 마스크 생성
 *
 * ## 스캔라인 채우기 원리
 *  - 래스터 그래픽스의 폴리곤 채우기 알고리즘을 1D(X축 방향)로 적용
 *  - 각 열(column, X 방향)에서 좌/우 경계의 Y값을 보간하여
 *    그 사이의 모든 행(row)을 free(0)로 채운다
 *  - 곡선 코리도에서도 각 X 슬라이스마다 독립적으로 채움
 *
 * ## 좌표 관계
 *  월드 X ↔ 그리드 col (열 방향, 가로)
 *  월드 Y ↔ 그리드 row (행 방향, 세로)
 *  그리드 원점(0,0) = 좌하단 (origin_x, origin_y)
 */

#include "track_planning/costmap/drivable_mask_scanline.hpp"
#include "track_planning/costmap/inflation.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace track_planning
{

// ============================================================
// World → Grid
// ============================================================
/**
 * @brief 월드 좌표(wx, wy)를 그리드 셀(col, row)로 변환
 *
 * floor()로 내림 처리하여 실수 좌표를 셀 인덱스로 매핑.
 * @return 셀이 그리드 범위 내이면 true
 */
bool DrivableMaskScanline::world_to_grid(
  double wx, double wy,
  double origin_x, double origin_y, double resolution,
  int width, int height,
  int & col, int & row)
{
  // 월드 X에서 원점을 빼고 해상도로 나눠 열 인덱스 계산
  col = static_cast<int>(std::floor((wx - origin_x) / resolution));
  // 월드 Y에서 원점을 빼고 해상도로 나눠 행 인덱스 계산
  row = static_cast<int>(std::floor((wy - origin_y) / resolution));
  // 그리드 범위 [0, width) × [0, height) 내에 있는지 반환
  return (col >= 0 && col < width && row >= 0 && row < height);
}

// ============================================================
// Interpolate Y at given X along a polyline
// ============================================================
/**
 * @brief 폴리라인에서 주어진 X 좌표에 해당하는 Y 값을 선형 보간
 *
 * ## 보간 방법
 *  폴리라인의 인접한 두 점 (x0, y0), (x1, y1)에 대해:
 *    t = (x - x0) / (x1 - x0)   ... 0 ≤ t ≤ 1
 *    y = y0 + t * (y1 - y0)      ... 선형 보간
 *
 * ## X 범위 판별
 *  (x0 ≤ x ≤ x1) 또는 (x1 ≤ x ≤ x0) 조건으로
 *  X가 증가하는 폴리라인과 감소하는 폴리라인 모두 처리
 *
 * ## 수직 선분 처리
 *  |x1 - x0| < 1e-12 (사실상 수직)이면 두 Y의 중간값 반환
 *
 * @param pts   폴리라인 포인트 목록 (최소 2개 필요)
 * @param x     보간할 월드 X 좌표
 * @param y_out [출력] 보간된 Y 값
 * @return      X가 폴리라인 범위 내이면 true
 */
bool DrivableMaskScanline::interpolate_y_at_x(
  const std::vector<Point2D> & pts,
  double x, double & y_out)
{
  if (pts.size() < 2) return false;

  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double x0 = pts[i].x;
    double x1 = pts[i + 1].x;

    // 현재 선분 [i, i+1]의 X 범위 안에 x가 포함되는지 확인
    // (x0 <= x <= x1) OR (x1 <= x <= x0) — 양방향 모두 처리
    if ((x0 <= x && x <= x1) || (x1 <= x && x <= x0)) {
      double dx = x1 - x0;
      if (std::abs(dx) < 1e-12) {
        // 수직 선분: 두 점의 Y 중간값 반환
        y_out = 0.5 * (pts[i].y + pts[i + 1].y);
        return true;
      }
      // 선형 보간 파라미터: t = (x - x0) / (x1 - x0)
      double t = (x - x0) / dx;
      // Y 보간: y = y0 + t * (y1 - y0)
      y_out = pts[i].y + t * (pts[i + 1].y - pts[i].y);
      return true;
    }
  }
  return false;  // x가 폴리라인의 X 범위 밖
}

// ============================================================
// Build drivable mask
// ============================================================
/**
 * @brief 스캔라인 채우기 방식으로 주행 가능 마스크 빌드
 *
 * ## 단계별 상세 설명
 *
 * ### 단계 1: 전체 그리드 초기화 (occupied = 100)
 *   - 기본적으로 모든 셀을 주행 불가(100)로 설정
 *   - 이후 코리도 내부만 free(0)으로 변경
 *
 * ### 단계 2: 코리도 폴리라인 리샘플링
 *   - resample_polyline()으로 그리드 해상도와 동일한 간격으로 리샘플링
 *   - 조밀한 포인트로 보간 정확도 향상
 *
 * ### 단계 3: 스캔라인 채우기
 *   - 각 그리드 열(col = 0, 1, ..., width-1)에 대해:
 *     * 열 중심의 월드 X 좌표: wx = origin_x + (col + 0.5) * resolution
 *     * 좌측/우측 경계에서 wx에 해당하는 Y 보간
 *     * y_low = min(y_left, y_right), y_high = max(y_left, y_right)
 *     * Y 범위를 행 인덱스로 변환 후 그리드 범위에 클램프
 *     * 해당 열의 [row_low, row_high] 행을 0(free)으로 채움
 *
 * ### 단계 4: 장애물/콘 래스터화 (100)
 *   - 콘과 장애물 포인트를 world_to_grid로 변환 후 100으로 마킹
 *   - 스캔라인으로 free가 된 셀을 다시 막음
 *
 * ### 단계 5: 장애물 팽창 및 병합
 *   - 별도의 obs_grid 버퍼에 장애물만 래스터화
 *   - inflate_grid()로 obstacle_radius만큼 팽창 (99로 표시)
 *   - free(0)인 셀이 팽창 영역이면 해당 팽창값으로 덮어씀
 *   - 이미 occupied(100)인 셀은 건드리지 않음
 *
 * @note 좌측이 우측보다 Y가 크거나 작을 수 있으므로 min/max로 정렬
 * @note 코리도 폴리라인이 2개 미만이면 빌드 실패 (valid=false)
 */
DrivableMaskScanline::Result DrivableMaskScanline::build(
  const std::vector<Point2D> & corridor_left,
  const std::vector<Point2D> & corridor_right,
  const std::vector<Point2D> & cones,
  const std::vector<Point2D> & obstacles,
  const PlanningParams & p)
{
  Result result;
  // 그리드 파라미터 설정
  result.resolution = p.roi.resolution;
  result.origin_x = p.roi.x_min;
  result.origin_y = p.roi.y_min;
  // ROI 범위를 해상도로 나눠 그리드 크기 계산
  result.width = static_cast<int>(
    std::ceil((p.roi.x_max - p.roi.x_min) / p.roi.resolution));
  result.height = static_cast<int>(
    std::ceil((p.roi.y_max - p.roi.y_min) / p.roi.resolution));

  const int total = result.width * result.height;
  if (total <= 0) return result;  // 유효하지 않은 ROI

  // 코리도 폴리라인이 너무 짧으면 스캔라인 채우기 불가
  if (corridor_left.size() < 2 || corridor_right.size() < 2) return result;

  // ---- 단계 1: 전체 그리드를 100(occupied)으로 초기화 ----
  // 기본적으로 모든 셀은 주행 불가 — 코리도 내부만 이후 free로 설정됨
  result.grid.assign(static_cast<size_t>(total), 100);

  // ---- 단계 2: 코리도 폴리라인을 그리드 해상도로 리샘플링 ----
  // 조밀한 포인트 간격으로 Y 보간 시 정확도 향상
  auto left_rs = resample_polyline(corridor_left, result.resolution);
  auto right_rs = resample_polyline(corridor_right, result.resolution);

  // ---- 단계 3: 스캔라인 채우기 — 각 열(X)에서 좌/우 경계 사이 행을 free로 ----
  for (int col = 0; col < result.width; ++col) {
    // 해당 열의 중심 월드 X 좌표 계산 (+0.5로 셀 중심 좌표 사용)
    const double wx = result.origin_x + (col + 0.5) * result.resolution;

    double y_left, y_right;
    // 좌측 경계에서 wx에 해당하는 Y 보간
    bool have_left = interpolate_y_at_x(left_rs, wx, y_left);
    // 우측 경계에서 wx에 해당하는 Y 보간
    bool have_right = interpolate_y_at_x(right_rs, wx, y_right);

    // 좌/우 중 하나라도 보간 실패이면 이 열은 채우지 않음
    if (!have_left || !have_right) continue;

    // y_low < y_high 보장 (좌측이 우측보다 위에 있을 수도 있음)
    double y_low = std::min(y_left, y_right);
    double y_high = std::max(y_left, y_right);

    // Y 범위를 행 인덱스로 변환
    int row_low = static_cast<int>(std::floor((y_low - result.origin_y) / result.resolution));
    int row_high = static_cast<int>(std::floor((y_high - result.origin_y) / result.resolution));

    // 그리드 범위 [0, height-1]에 클램프
    row_low = std::max(0, row_low);
    row_high = std::min(result.height - 1, row_high);

    // 해당 열의 Y 범위 내 모든 행을 free(0)로 채움
    for (int row = row_low; row <= row_high; ++row) {
      result.grid[row * result.width + col] = 0;  // 주행 가능 셀
    }
  }

  // ---- 단계 4: 장애물 + 콘을 100(occupied)으로 래스터화 ----
  // 스캔라인으로 free가 된 셀을 장애물이 있는 경우 다시 막음
  for (const auto & pt : cones) {
    int col, row;
    if (world_to_grid(pt.x, pt.y, result.origin_x, result.origin_y,
        result.resolution, result.width, result.height, col, row))
    {
      result.grid[row * result.width + col] = 100;  // 콘 위치 = 주행 불가
    }
  }
  for (const auto & pt : obstacles) {
    int col, row;
    if (world_to_grid(pt.x, pt.y, result.origin_x, result.origin_y,
        result.resolution, result.width, result.height, col, row))
    {
      result.grid[row * result.width + col] = 100;  // 장애물 위치 = 주행 불가
    }
  }

  // ---- 단계 5: 장애물 팽창 ----
  // 장애물 경계를 별도 버퍼에서 팽창하여 A*가 장애물 근처를 회피하도록 함
  if (p.inflation.obstacle_radius > 0.0) {
    // 팽창 전용 버퍼: 장애물 포인트만 포함 (경계는 포함하지 않음)
    std::vector<int8_t> obs_grid(static_cast<size_t>(total), 0);

    // 콘을 팽창 버퍼에 래스터화
    for (const auto & pt : cones) {
      int col, row;
      if (world_to_grid(pt.x, pt.y, result.origin_x, result.origin_y,
          result.resolution, result.width, result.height, col, row))
      {
        obs_grid[row * result.width + col] = 100;
      }
    }
    // 장애물을 팽창 버퍼에 래스터화
    for (const auto & pt : obstacles) {
      int col, row;
      if (world_to_grid(pt.x, pt.y, result.origin_x, result.origin_y,
          result.resolution, result.width, result.height, col, row))
      {
        obs_grid[row * result.width + col] = 100;
      }
    }

    // 팽창 적용: 장애물 주변 obstacle_radius 내의 셀을 99로 표시
    inflation::inflate_grid(
      obs_grid, result.width, result.height,
      result.resolution, p.inflation.obstacle_radius,
      100, 99);  // occupied_th=100, inflated_value=99

    // 팽창 결과를 메인 그리드에 병합
    // free(0)인 셀만 팽창값으로 덮어씀 (이미 occupied인 셀은 유지)
    for (int i = 0; i < total; ++i) {
      if (obs_grid[i] > 0 && result.grid[i] == 0) {
        result.grid[i] = obs_grid[i];  // free → 팽창 비용 셀
      }
    }
  }

  result.valid = true;
  return result;
}

}  // namespace track_planning
