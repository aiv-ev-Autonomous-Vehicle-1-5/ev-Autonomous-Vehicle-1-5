/**
 * @file path_postprocessor.cpp
 * @brief 경로 후처리기 — 구현부
 *
 * raw_path에 prune → smooth → resample → yaw 4단계를 적용하여
 * control 노드가 사용하기 좋은 최종 경로를 생성한다.
 */
#include "planning_mr_ver/postprocess/path_postprocessor.hpp"
#include "planning_mr_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace planning_mr_ver
{

/**
 * @brief Greedy Shortcut 가지치기
 *
 * 알고리즘:
 *   1. 첫 번째 점(i)부터 시작
 *   2. 가장 먼 점(j=끝)부터 역순으로, i→j 직선까지의
 *      모든 중간점(k)의 수직거리를 검사
 *   3. 모든 중간점이 max_dev 이내면 j로 바로 점프 (shortcut 성공)
 *   4. j에서 다시 2~3 반복
 *   5. 결과: 직선에 가까운 구간의 중간점들이 제거됨
 *
 * 수직거리 공식:
 *   perp = |dx * dir.y - dy * dir.x|  (외적의 절대값 / 단위벡터)
 */
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev)
{
  if (pts.size() <= 2) return pts;  // 2점 이하면 가지치기 불필요

  std::vector<Point2D> result;
  result.push_back(pts.front());  // 시작점 보존

  size_t i = 0;
  while (i < pts.size() - 1) {
    size_t best_j = i + 1;  // 최소한 다음 점으로는 이동

    // 가장 먼 점부터 역순으로 shortcut 시도 (greedy: 최대 점프 우선)
    for (size_t j = pts.size() - 1; j > i + 1; --j) {
      const Point2D & a = pts[i];
      const Point2D & b = pts[j];
      const double ab_len = dist(a, b);

      bool can_shortcut = true;
      if (ab_len < 1e-12) {
        can_shortcut = false;  // a와 b가 같은 위치
      } else {
        // a→b 방향의 단위벡터
        const Point2D dir = {(b.x - a.x) / ab_len, (b.y - a.y) / ab_len};

        // 모든 중간점(k)의 수직거리 검사
        for (size_t k = i + 1; k < j; ++k) {
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;
          // 외적으로 수직거리 계산: |d × dir| (dir이 단위벡터이므로 나누기 불필요)
          const double perp = std::abs(dx * dir.y - dy * dir.x);

          if (perp > max_dev) {
            can_shortcut = false;  // 이 중간점이 너무 멀어서 shortcut 불가
            break;
          }
        }
      }

      if (can_shortcut) {
        best_j = j;  // i에서 j로 바로 점프 가능
        break;         // greedy: 가장 먼 shortcut 찾으면 바로 채택
      }
    }

    result.push_back(pts[best_j]);
    i = best_j;  // j부터 다시 시작
  }

  return result;
}

/**
 * @brief 이동 평균(Moving Average) 스무딩
 *
 * 각 점(i)의 새 좌표 = 주변 [i-half, i+half] 범위 점들의 평균
 *
 * 특수 처리:
 *   - 시작점/끝점은 변경하지 않음 (경로 시작/끝 위치 보존)
 *   - 경계 근처: 윈도우가 잘릴 수 있음 (가용 점만 사용)
 */
std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window)
{
  if (pts.size() <= 2 || window <= 1) return pts;  // 스무딩 불필요

  const int half = window / 2;  // 편측 윈도우 크기 (예: window=5 → half=2)
  const int n = static_cast<int>(pts.size());

  std::vector<Point2D> result(pts.size());
  result.front() = pts.front();  // 시작점 보존
  result.back() = pts.back();    // 끝점 보존

  // 내부 점들만 스무딩 적용 (인덱스 1 ~ n-2)
  for (int i = 1; i < n - 1; ++i) {
    double sx = 0.0, sy = 0.0;
    int count = 0;

    // 윈도우 범위: [max(0, i-half), min(n-1, i+half)]
    const int lo = std::max(0, i - half);
    const int hi = std::min(n - 1, i + half);

    for (int j = lo; j <= hi; ++j) {
      sx += pts[j].x;
      sy += pts[j].y;
      ++count;
    }
    result[i] = {sx / count, sy / count};  // 평균 좌표
  }

  return result;
}

/**
 * @brief 4단계 후처리 파이프라인 실행
 *
 * 순서: prune → smooth → resample → yaw
 * 각 단계는 이전 단계의 출력을 입력으로 받는다.
 *
 * resample은 geometry.hpp의 resample_polyline()을 사용하고,
 * yaw는 polyline_tangents()로 접선 방향을 구한 뒤 atan2로 변환한다.
 */
PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds)
{
  PostprocessResult result;

  if (raw_path.size() < 2) return result;  // 점이 2개 미만이면 경로 생성 불가

  // Stage 1: Prune — 불필요한 중간점 제거 (Greedy Shortcut)
  auto pruned = prune(raw_path, prune_max_dev);

  // Stage 2: Smooth — 이동 평균 필터로 경로 부드럽게
  auto smoothed = smooth(pruned, smooth_window);

  // Stage 3: Resample — 균일 간격(resample_ds)으로 리샘플링
  result.path = resample_polyline(smoothed, resample_ds);

  if (result.path.size() < 2) return result;  // 리샘플 후 점 부족

  // Stage 4: Yaw — 각 점의 heading 각도 계산
  // 접선 벡터 → atan2로 라디안 각도 변환
  auto tangents = polyline_tangents(result.path);
  result.yaw.resize(result.path.size());
  for (size_t i = 0; i < tangents.size(); ++i) {
    result.yaw[i] = heading(tangents[i]);  // atan2(ty, tx)
  }

  result.valid = true;
  return result;
}

}  // namespace planning_mr_ver
