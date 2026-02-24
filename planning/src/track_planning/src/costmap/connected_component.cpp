/**
 * @file connected_component.cpp
 * @brief ConnectedComponent::filter_ego_component() 구현부
 *
 * ## BFS 알고리즘 상세
 *
 * ### 시드(seed) 결정
 *  - ego_pos를 floor()로 셀 인덱스 변환
 *  - ego 셀이 free(0)이면 그 셀을 시드로 사용
 *  - ego 셀이 occupied이면 5×5 이웃(search_r=5)에서
 *    유클리드 거리가 가장 작은 free 셀을 시드로 선택
 *  - 근처 free 셀이 없으면 함수 종료 (아무 작업도 하지 않음)
 *
 * ### BFS 탐색
 *  - std::queue<int>: 방문할 셀의 1D 인덱스를 저장
 *  - visited 벡터: 방문 여부 추적 (재방문 방지)
 *  - 4-방향 오프셋: dx[]={0,0,-1,1}, dy[]={-1,1,0,0} (상,하,좌,우)
 *  - 탐색 조건: 범위 내 && !visited && grid[ni]==0(free)
 *
 * ### 미도달 셀 처리
 *  - BFS 완료 후 grid[i]==0 && !visited[i] → grid[i]=100
 *  - 이로써 ego에서 연결되지 않은 모든 free 영역이 차단됨
 *
 * ## 성능
 *  - 시간복잡도: O(width × height) — 각 셀을 최대 한 번씩 방문
 *  - 공간복잡도: O(width × height) — visited 배열
 */

#include "track_planning/costmap/connected_component.hpp"

#include <cmath>
#include <queue>

namespace track_planning
{

/**
 * @brief ego에서 BFS로 도달 불가능한 free 셀을 occupied(100)으로 변환
 *
 * ## 구현 단계
 *
 * ### 단계 1: ego 위치 → 그리드 셀 변환
 *   ego_col = floor((ego_pos.x - origin_x) / resolution)
 *   ego_row = floor((ego_pos.y - origin_y) / resolution)
 *   그리드 범위 밖이면 함수 종료
 *
 * ### 단계 2: 시드 셀 결정
 *   ego 셀(ego_idx)이 free(==0)이면 직접 시드로 사용.
 *   occupied이면 5셀 반경 내 이웃 탐색:
 *     - dr ∈ [-5, 5], dc ∈ [-5, 5]
 *     - d2 = dr² + dc² 로 거리 비교 (sqrt 불필요)
 *     - 가장 가까운 free 셀을 seed_idx로 업데이트
 *   근처 free 셀이 없으면 (seed_idx 셀이 여전히 occupied) 조기 반환
 *
 * ### 단계 3: BFS 탐색 (4-방향)
 *   visited[seed_idx] = true 로 초기화
 *   큐에 seed_idx 추가
 *   while (큐가 비지 않음):
 *     cur = 큐 앞 원소 꺼냄
 *     cur_row = cur / width, cur_col = cur % width
 *     for d in {상, 하, 좌, 우}:
 *       nr = cur_row + dy[d], nc = cur_col + dx[d]
 *       범위 밖이면 skip
 *       already visited이면 skip
 *       grid[ni] != 0 (occupied)이면 skip
 *       visited[ni] = true, 큐에 ni 추가
 *
 * ### 단계 4: 미도달 free 셀 차단
 *   for i in [0, total):
 *     if grid[i] == 0 && !visited[i]:
 *       grid[i] = 100  // 도달 불가 free → occupied
 */
void ConnectedComponent::filter_ego_component(
  std::vector<int8_t> & grid,
  int width, int height,
  double resolution, double origin_x, double origin_y,
  const Point2D & ego_pos)
{
  const int total = width * height;
  if (total <= 0) return;

  // ---- 단계 1: ego 위치 → 그리드 셀 변환 ----
  // floor()로 내림하여 셀 인덱스 계산
  const int ego_col = static_cast<int>(std::floor((ego_pos.x - origin_x) / resolution));
  const int ego_row = static_cast<int>(std::floor((ego_pos.y - origin_y) / resolution));

  // ego 위치가 그리드 범위 밖이면 처리 불가
  if (ego_col < 0 || ego_col >= width || ego_row < 0 || ego_row >= height) return;

  // ego의 1D 인덱스 계산
  const int ego_idx = ego_row * width + ego_col;

  // ---- 단계 2: 시드 셀 결정 ----
  int seed_idx = ego_idx;  // 기본값: ego 셀 자체를 시드로 사용

  if (grid[ego_idx] != 0) {
    // ego 셀이 occupied인 경우 (팽창 영역 등) → 근처 free 셀 탐색
    int best_dist_sq = std::numeric_limits<int>::max();
    const int search_r = 5;  // 탐색 반경: 5셀

    for (int dr = -search_r; dr <= search_r; ++dr) {
      for (int dc = -search_r; dc <= search_r; ++dc) {
        int nr = ego_row + dr;
        int nc = ego_col + dc;
        // 그리드 범위 체크
        if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;
        int ni = nr * width + nc;
        if (grid[ni] == 0) {  // free 셀 발견
          int d2 = dr * dr + dc * dc;  // 유클리드 거리² (정수 비교)
          if (d2 < best_dist_sq) {
            best_dist_sq = d2;
            seed_idx = ni;  // 더 가까운 free 셀로 업데이트
          }
        }
      }
    }
    // 주변에 free 셀이 없으면 필터링 불가 — 함수 종료
    if (grid[seed_idx] != 0) return;
  }

  // ---- 단계 3: BFS 탐색 (4-방향 연결) ----
  std::vector<bool> visited(static_cast<size_t>(total), false);
  std::queue<int> q;
  q.push(seed_idx);
  visited[seed_idx] = true;

  // 4-방향 이웃 오프셋 (상, 하, 좌, 우)
  // dx: 열 방향 이동 (좌:-1, 우:+1, 상하:0)
  // dy: 행 방향 이동 (상:-1, 하:+1, 좌우:0)
  const int dx[4] = {0, 0, -1, 1};
  const int dy[4] = {-1, 1, 0, 0};

  while (!q.empty()) {
    const int cur = q.front();
    q.pop();

    // 1D 인덱스 → 2D (행, 열) 역변환
    const int cur_row = cur / width;
    const int cur_col = cur % width;

    // 4방향 이웃 탐색
    for (int d = 0; d < 4; ++d) {
      const int nr = cur_row + dy[d];  // 이웃 셀의 행
      const int nc = cur_col + dx[d];  // 이웃 셀의 열

      // 그리드 범위 체크
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;

      const int ni = nr * width + nc;
      if (visited[ni]) continue;       // 이미 방문한 셀 skip
      if (grid[ni] != 0) continue;     // occupied 셀은 통과 불가 (0인 free만 탐색)

      // 방문 표시 후 큐에 추가
      visited[ni] = true;
      q.push(ni);
    }
  }

  // ---- 단계 4: ego에서 도달 불가능한 free 셀을 occupied로 변환 ----
  // free(0)이지만 BFS에서 방문되지 않은 셀 → 고립된 영역 → 차단
  for (int i = 0; i < total; ++i) {
    if (grid[i] == 0 && !visited[i]) {
      grid[i] = 100;  // 도달 불가 free → occupied
    }
  }
}

}  // namespace track_planning
