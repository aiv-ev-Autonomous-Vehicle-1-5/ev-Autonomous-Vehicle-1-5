/**
 * @file path_postprocessor.cpp
 * @brief PathPostprocessor 구현부 — prune, smooth, resample, yaw 4단계 파이프라인
 *
 * ## 각 단계별 목적
 *
 * prune()
 *  - 원시 경로의 불필요한 중간 포인트(이상점, 노이즈) 제거
 *  - 수직 편차 허용 범위 내에서 최대한 직선으로 단순화
 *
 * smooth()
 *  - 가지치기 후 남은 날카로운 꺾임 완화
 *  - 이동 평균으로 연속적이고 부드러운 경로 생성
 *
 * resample_polyline() [geometry.hpp]
 *  - 균일한 간격의 waypoint 생성
 *  - 제어기가 일정 간격의 포인트를 기대할 때 필수
 *
 * polyline_tangents() + heading() [geometry.hpp]
 *  - 각 waypoint에서의 차량 목표 방향(yaw) 계산
 *  - 전방 또는 중앙 차분으로 접선 방향 계산
 *
 * ## 의존 파일
 *  - geometry.hpp: resample_polyline(), polyline_tangents(), heading()
 *  - types.hpp: Point2D, PostprocessResult
 */

#include "track_planning/postprocess/path_postprocessor.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace track_planning
{

// ============================================================
// Greedy shortcut pruning
// ============================================================
/**
 * @brief Greedy shortcut pruning — 수직 편차 기반 중간 포인트 제거
 *
 * ## 알고리즘 상세
 *
 * result = [pts[0]]   // 시작점 항상 포함
 * i = 0               // 현재 앵커(고정) 포인트
 *
 * while i < pts.size()-1:
 *   best_j = i+1      // 기본값: 바로 다음 포인트
 *
 *   // 가장 먼 점(j)부터 역순으로 숏컷 시도
 *   for j = pts.size()-1 downto i+2:
 *     a = pts[i], b = pts[j]
 *     ab_len = dist(a, b)
 *
 *     if ab_len < 1e-12:
 *       can_shortcut = false  // 두 점이 거의 같은 위치 → 스킵
 *     else:
 *       dir = (b - a) / ab_len   // a→b 단위 방향 벡터
 *
 *       // 중간 포인트(k=i+1..j-1) 수직 거리 검사
 *       for k = i+1 to j-1:
 *         dx = pts[k].x - a.x
 *         dy = pts[k].y - a.y
 *         perp = |dx * dir.y - dy * dir.x|   // 2D 외적 = 수직 거리
 *         if perp > max_dev:
 *           can_shortcut = false
 *           break
 *
 *     if can_shortcut:
 *       best_j = j
 *       break   // 가장 먼 점을 찾으면 즉시 종료
 *
 *   result.append(pts[best_j])
 *   i = best_j   // 다음 앵커 포인트로 이동
 *
 * return result
 *
 * @note 시간복잡도: 최악 O(n³), 실제로는 경로가 단순하여 훨씬 빠름
 * @note 시작점(pts[0])은 result에 미리 추가, 끝점(pts.back())은 마지막 best_j
 */
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev)
{
  // 포인트가 2개 이하이면 제거할 중간점 없음
  if (pts.size() <= 2) return pts;

  std::vector<Point2D> result;
  result.push_back(pts.front());  // 시작점 항상 유지

  size_t i = 0;
  while (i < pts.size() - 1) {
    size_t best_j = i + 1;  // 기본값: 바로 다음 포인트 (숏컷 실패 시)

    // 가장 먼 j부터 역순으로 숏컷 가능성 탐색
    for (size_t j = pts.size() - 1; j > i + 1; --j) {
      // i→j 직선의 정보 계산
      const Point2D & a = pts[i];
      const Point2D & b = pts[j];
      const double ab_len = dist(a, b);  // i→j 거리

      bool can_shortcut = true;
      if (ab_len < 1e-12) {
        // 두 점이 사실상 같은 위치 → 수직 거리 계산 불가
        can_shortcut = false;
      } else {
        // a→b 방향의 단위 벡터
        const Point2D dir = {(b.x - a.x) / ab_len, (b.y - a.y) / ab_len};

        // 중간 포인트(i+1 ~ j-1) 수직 거리 검사
        for (size_t k = i + 1; k < j; ++k) {
          // k번 포인트에서 a까지의 벡터
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;
          // 수직 거리 = 2D 외적의 절댓값
          // cross2(pts[k]-a, dir) = dx*dir.y - dy*dir.x
          const double perp = std::abs(dx * dir.y - dy * dir.x);

          if (perp > max_dev) {
            // 중간 포인트가 허용 범위 밖 → 이 숏컷 불가
            can_shortcut = false;
            break;
          }
        }
      }

      if (can_shortcut) {
        best_j = j;  // 숏컷 허용: j가 다음 앵커
        break;       // 가장 먼 허용 j를 찾으면 즉시 종료
      }
    }

    // best_j를 결과에 추가하고 다음 앵커로 이동
    result.push_back(pts[best_j]);
    i = best_j;
  }

  return result;
}

// ============================================================
// Moving-average smoothing
// ============================================================
/**
 * @brief 이동 평균(Moving Average) 스무딩
 *
 * ## 알고리즘
 *  half = window / 2   (정수 나눗셈, window=5이면 half=2)
 *
 *  result[0] = pts[0]         // 시작점 보존
 *  result[n-1] = pts[n-1]     // 끝점 보존
 *
 *  for i = 1 to n-2:          // 중간 포인트만 스무딩
 *    lo = max(0, i - half)    // 윈도우 왼쪽 경계 (경계 클램프)
 *    hi = min(n-1, i + half)  // 윈도우 오른쪽 경계
 *    result[i] = mean(pts[lo..hi])  // X, Y 각각 평균
 *
 * ## 경계 처리
 *  - 시작점/끝점은 원본값 유지 (start/goal 위치 고정이 중요)
 *  - 경계 근처는 가용한 포인트 수로만 평균 (비대칭 윈도우, 별도 가중치 없음)
 *
 * ## 수렴성
 *  - window가 클수록 더 부드럽지만 경로가 짧아지는 경향
 *  - 여러 번 적용하면 Gaussian 스무딩에 수렴
 *
 * @param pts    입력 포인트 목록 (최소 3개 이상이어야 의미 있음)
 * @param window 윈도우 크기 (1이면 원본 그대로 반환)
 */
std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window)
{
  // 포인트가 2개 이하이거나 윈도우가 1이하면 스무딩 불필요
  if (pts.size() <= 2 || window <= 1) return pts;

  const int half = window / 2;  // 윈도우 절반 크기 (정수 나눗셈)
  const int n = static_cast<int>(pts.size());

  std::vector<Point2D> result(pts.size());
  // 양 끝점은 항상 원본 유지 (시작점=start, 끝점=goal)
  result.front() = pts.front();
  result.back() = pts.back();

  // 중간 포인트(1 ~ n-2)만 이동 평균 적용
  for (int i = 1; i < n - 1; ++i) {
    double sx = 0.0, sy = 0.0;  // X, Y 합산 누산기
    int count = 0;              // 윈도우 내 포인트 수

    // 윈도우 범위 계산 (경계 클램프)
    const int lo = std::max(0, i - half);     // 윈도우 왼쪽 경계
    const int hi = std::min(n - 1, i + half); // 윈도우 오른쪽 경계

    // 윈도우 내 포인트 합산
    for (int j = lo; j <= hi; ++j) {
      sx += pts[j].x;
      sy += pts[j].y;
      ++count;
    }
    // 평균으로 스무딩된 포인트 계산
    result[i] = {sx / count, sy / count};
  }

  return result;
}

// ============================================================
// Full postprocess pipeline
// ============================================================
/**
 * @brief 전체 후처리 파이프라인: prune → smooth → resample → yaw
 *
 * ## 파이프라인 흐름
 *
 *  입력: raw_path (DTR 센터라인의 원시 경로)
 *
 *  [단계 1: Prune]
 *   pruned = prune(raw_path, prune_max_dev)
 *   - 원시 경로의 불필요한 중간 포인트를 제거하고 직선에 가깝게 단순화
 *   - max_dev가 크면 더 직선화, 작으면 원래 경로에 가까움
 *
 *  [단계 2: Smooth]
 *   smoothed = smooth(pruned, smooth_window)
 *   - 이동 평균으로 날카로운 꺾임 완화
 *   - 양 끝점 보존
 *
 *  [단계 3: Resample]
 *   result.path = resample_polyline(smoothed, resample_ds)
 *   - resample_ds 간격의 균일한 waypoint 생성
 *   - geometry.hpp의 resample_polyline() 재사용
 *
 *  [단계 4: Yaw 계산]
 *   tangents = polyline_tangents(result.path)
 *   result.yaw[i] = heading(tangents[i])
 *   - 각 waypoint에서의 진행 방향(yaw) 각도(라디안) 계산
 *   - heading()은 방향 벡터를 atan2로 각도 변환
 *
 *  출력: PostprocessResult { path, yaw, valid }
 *   - path.size() == yaw.size() 보장
 *   - raw_path가 너무 짧으면 valid=false 반환
 *
 * @note 2개 미만의 포인트로 후처리 시도하면 valid=false 반환
 */
PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds)
{
  PostprocessResult result;

  // 입력 경로가 너무 짧으면 후처리 불가
  if (raw_path.size() < 2) return result;

  // ---- 단계 1: Greedy shortcut pruning ----
  // 원시 경로의 불필요한 중간 포인트(이상점) 제거
  auto pruned = prune(raw_path, prune_max_dev);

  // ---- 단계 2: 이동 평균 스무딩 ----
  // 가지치기 후 남은 꺾임 완화 (양 끝점 보존)
  auto smoothed = smooth(pruned, smooth_window);

  // ---- 단계 3: 균일 간격 재샘플링 ----
  // geometry.hpp의 resample_polyline() 재사용
  // resample_ds 간격으로 균일한 waypoint 생성
  result.path = resample_polyline(smoothed, resample_ds);

  // 재샘플링 결과가 너무 짧으면 실패 처리
  if (result.path.size() < 2) return result;

  // ---- 단계 4: 각 waypoint에서 yaw 각도 계산 ----
  // polyline_tangents(): 각 포인트의 접선 방향 단위벡터 계산
  auto tangents = polyline_tangents(result.path);
  result.yaw.resize(result.path.size());
  for (size_t i = 0; i < tangents.size(); ++i) {
    // heading(): 방향 벡터 → atan2 각도(라디안)
    result.yaw[i] = heading(tangents[i]);
  }

  result.valid = true;
  return result;
}

}  // namespace track_planning
