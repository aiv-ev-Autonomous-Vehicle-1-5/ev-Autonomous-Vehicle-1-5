/**
 * @file inflation.cpp
 * @brief inflate_grid() 구현부 — 원형 커널 기반 그리드 팽창
 *
 * ## 처리 흐름
 *  1. 셀 단위 반경 계산: r_cells = ceil(radius / resolution)
 *  2. 원형 마스크(오프셋 목록) 사전 생성:
 *       - [-r_cells, r_cells] × [-r_cells, r_cells] 범위를 순회
 *       - dx² + dy² ≤ r_cells² 조건을 만족하는 (dx, dy) 오프셋만 저장
 *       - 정사각형이 아닌 원형으로 팽창되어 자연스러운 모양
 *  3. 소스 셀 수집:
 *       - 그리드 전체를 순회하며 value >= occupied_th 인 셀 인덱스 저장
 *       - 반복 도중 그리드 수정을 피하기 위해 미리 수집
 *  4. 팽창 적용:
 *       - 각 소스 셀 (src_row, src_col)에서 마스크 오프셋(dx, dy)을 적용
 *       - 대상 셀 (nr, nc) 계산 후 범위 체크
 *       - 이미 occupied_th 이상이면 보존 (더 낮은 값으로 덮어쓰지 않음)
 *       - 그렇지 않으면 inflated_value 기록
 */

#include "track_planning/costmap/inflation.hpp"

#include <cmath>
#include <vector>

namespace track_planning
{
namespace inflation
{

/**
 * @brief 2D 그리드에서 점유 셀을 원형 커널로 팽창
 *
 * ## 구현 세부사항
 *
 * ### 1단계: 셀 단위 반경 계산
 *   r_cells = ceil(radius / resolution)
 *   예) radius=0.5m, resolution=0.1m → r_cells=5 (5셀 반경 팽창)
 *   r_cells_sq = r_cells² (원 내부 판별용 정수 비교)
 *
 * ### 2단계: 원형 마스크 사전 계산
 *   struct Offset { int dx, dy; }
 *   [-r_cells, r_cells] 범위의 2D 격자를 순회하면서
 *   dx² + dy² ≤ r_cells² 조건을 만족하는 오프셋만 mask에 추가
 *   reserve()로 최대 (2r+1)² 크기만큼 메모리 예약
 *
 * ### 3단계: 소스 셀 수집
 *   그리드 전체를 선형 탐색하여 grid[i] >= occupied_th 인 인덱스 수집
 *   sources 벡터에 저장 (반복 중 그리드 수정 방지)
 *
 * ### 4단계: 팽창 적용
 *   각 소스 셀에서:
 *     src_row = src / width   (행 인덱스)
 *     src_col = src % width   (열 인덱스)
 *   각 오프셋(off.dx, off.dy)을 적용하여 이웃 셀(nr, nc) 계산
 *   범위 밖이면 continue
 *   이미 점유된 셀(>= occupied_th)은 건너뜀 (우선순위 보존)
 *   나머지 셀에 inflated_value 기록
 */
void inflate_grid(
  std::vector<int8_t> & grid,
  int width,
  int height,
  double resolution,
  double radius,
  int8_t occupied_th,
  int8_t inflated_value)
{
  // 반경 또는 해상도가 0 이하이면 팽창 불필요
  if (radius <= 0.0 || resolution <= 0.0) return;

  // 셀 단위 반경: 실제 반경(m)을 셀 크기로 나눠 올림
  const int r_cells = static_cast<int>(std::ceil(radius / resolution));
  // 원 내부 판별을 위한 반경의 제곱 (정수 비교로 sqrt 연산 회피)
  const int r_cells_sq = r_cells * r_cells;

  // ---- 1단계: 원형 마스크 오프셋 사전 계산 ----
  struct Offset { int dx, dy; };
  std::vector<Offset> mask;
  // 최대 크기: (2*r_cells+1)² 개의 오프셋 예약
  mask.reserve(static_cast<size_t>((2 * r_cells + 1) * (2 * r_cells + 1)));

  // [-r_cells, r_cells] 범위의 2D 격자 순회
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      // 정수 비교로 원 내부 판별 (sqrt 연산 불필요)
      if (dx * dx + dy * dy <= r_cells_sq) {
        mask.push_back({dx, dy});
      }
    }
  }

  // ---- 2단계: 소스 셀(점유 셀) 인덱스 수집 ----
  // 반복 중 그리드를 수정하면 팽창이 연쇄적으로 퍼지는 문제 발생
  // → 먼저 소스 목록을 수집한 후 별도로 팽창 적용
  std::vector<int> sources;
  const int total = width * height;
  sources.reserve(static_cast<size_t>(total / 10));  // 휴리스틱: 전체의 10% 정도 점유 예상

  for (int i = 0; i < total; ++i) {
    if (grid[i] >= occupied_th) {
      sources.push_back(i);  // 점유 셀 인덱스 저장
    }
  }

  // ---- 3단계: 팽창 적용 ----
  for (int src : sources) {
    // 1D 인덱스 → 2D (행, 열) 역변환
    const int src_row = src / width;   // 소스 셀의 행 인덱스
    const int src_col = src % width;   // 소스 셀의 열 인덱스

    // 원형 마스크의 각 오프셋을 소스 셀에 적용
    for (const auto & off : mask) {
      const int nr = src_row + off.dy;  // 이웃 셀의 행 인덱스
      const int nc = src_col + off.dx;  // 이웃 셀의 열 인덱스

      // 그리드 범위 밖이면 건너뜀
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;

      const int idx = nr * width + nc;
      // 이미 occupied_th 이상(실제 점유 셀)이거나
      // 이미 inflated_value 이상이면 덮어쓰지 않음 (높은 우선순위 보존)
      if (grid[idx] < occupied_th && grid[idx] < inflated_value) {
        grid[idx] = inflated_value;  // 팽창 영역에 inflated_value 기록
      }
    }
  }
}

}  // namespace inflation
}  // namespace track_planning
