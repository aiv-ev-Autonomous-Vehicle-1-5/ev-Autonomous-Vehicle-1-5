/**
 * @file centerline_builder.cpp
 * @brief 코리도 경계에서 중심선(centerline) 생성 구현
 *
 * 파이프라인 Step (4): 좌/우 경계 상태에 따라 센터라인 생성.
 *
 * Case 별 처리 방법:
 *
 *   Case 1 — build_from_pair() [both_ok = true]:
 *     DTR(Delaunay Triangulation-based Racing) 방식:
 *     1. 좌/우 경계를 균일 리샘플링
 *     2. Constrained Delaunay Triangulation (CDT)
 *     3. 혼합 삼각형 필터링 (좌+우 꼭짓점 포함 + 기하 조건)
 *     4. 외심(circumcenter) 추출
 *     5. Greedy nearest-neighbor 체이닝
 *     6. 이동평균 스무딩 + 리샘플링
 *
 *   Case 2 — build_from_one_side() [virtual_used = true]:
 *     한쪽만 실측, 반대쪽은 VirtualBoundary에서 생성된 경우
 *     - 실측 경계(visible)의 각 점에서 법선 방향으로 w_hat/2 이동
 *
 *   Case 3 — 실패 [빈 CenterlineResult 반환]
 *
 * 참고: arXiv:2505.24320v1 (DTR 논문)
 *   - 논문: raw LiDAR → wall segmentation으로 경계 클래스 분류
 *   - 우리: 이미 좌/우 corridor로 분류 완료 → 세그먼트 분류 불필요
 */
#include "track_planning/corridor/centerline_builder.hpp"
#include "track_planning/common/geometry.hpp"

// CDT: Constrained Delaunay Triangulation (header-only, MIT license)
// artem-ogre/CDT — https://github.com/artem-ogre/CDT
#include "track_planning/third_party/CDT/CDT.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace track_planning
{

// ============================================================
// 메인 진입점
// ============================================================
CenterlineResult CenterlineBuilder::build(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  bool both_ok,
  bool virtual_used,
  bool visible_is_left,
  double w_hat,
  const PlanningParams & params)
{
  // Case 1: 양쪽 경계가 모두 유효 → DTR 외심 방식
  if (both_ok && left.size() >= 2 && right.size() >= 2) {
    return build_from_pair(left, right, params);
  }

  // Case 2: 가상 경계 사용 → 한쪽에서 오프셋 방법
  if (virtual_used) {
    const auto & visible = visible_is_left ? left : right;
    if (visible.size() >= 2) {
      return build_from_one_side(visible, visible_is_left, w_hat, params);
    }
  }

  // Case 3: 실패
  return CenterlineResult{};
}

// ============================================================
// Case 1: DTR 외심 기반 센터라인
//
// 알고리즘 (6단계):
//   1. 리샘플링: 양쪽 경계를 resample_ds 간격으로 균일화
//   2. CDT: 좌측 [0..n_left-1], 우측 [n_left..n_total-1] 정점 + 제약 엣지
//      닫힌 다각형(시작-시작, 끝-끝 연결)으로 eraseOuterTrianglesAndHoles()
//   3. 삼각형 필터링:
//      Filter A — 경계 클래스 혼합: 좌+우 꼭짓점 모두 포함
//      Filter B — 기하 조건: (isosceles_like && pointed) || large_area
//   4. 외심(circumcenter) 계산 + 전방 필터 (x > x_filter_min)
//   5. Greedy NN 체이닝: ego(0,0)에서 시작, 가장 가까운 미방문 외심 연결
//   6. 이동평균 스무딩 + 리샘플링
//
// 기하 필터 상세:
//   - sides 정렬: s[0] <= s[1] <= s[2]
//   - isosceles_like: s[2]/s[1] < tri_isosceles_ratio (두 긴 변이 비슷)
//   - pointed: s[2]/s[0] > tri_pointed_ratio (가늘고 긴 삼각형)
//   - large_area: area > tri_min_area
//   → 트랙을 가로지르는 얇은 삼각형의 외심이 중앙선에 가까움
// ============================================================
CenterlineResult CenterlineBuilder::build_from_pair(
  const std::vector<Point2D> & left,
  const std::vector<Point2D> & right,
  const PlanningParams & params)
{
  CenterlineResult result;
  const double resample_ds = params.postprocess.resample_ds;

  // ---- Step 1: 리샘플링 (리샘플 결과가 원본보다 부족하면 원본 사용) ----
  auto left_rs = resample_polyline(left, resample_ds);
  auto right_rs = resample_polyline(right, resample_ds);
  if (left_rs.size() < left.size()) left_rs = left;
  if (right_rs.size() < right.size()) right_rs = right;

  const size_t n_left = left_rs.size();
  const size_t n_right = right_rs.size();
  const size_t n_total = n_left + n_right;

  if (n_left < 2 || n_right < 2) {
    return result;
  }

  // ---- Step 1.5: 경계 교차 검사 ----
  // 좌/우 경계가 교차하면 CDT가 비정상 삼각형 생성 → 조기 종료
  if (polylines_cross(left_rs, right_rs)) {
    return result;
  }

  // ---- Step 2: Constrained Delaunay Triangulation ----
  // 정점 배열: [0..n_left-1] = 좌측, [n_left..n_total-1] = 우측
  std::vector<CDT::V2d<double>> verts;
  verts.reserve(n_total);
  for (const auto & p : left_rs) {
    verts.push_back({p.x, p.y});
  }
  for (const auto & p : right_rs) {
    verts.push_back({p.x, p.y});
  }

  // 제약 엣지: 각 경계의 연속 점 + 양 끝 닫기
  std::vector<CDT::Edge> edges;
  edges.reserve(n_left + n_right);

  // 좌측 경계 엣지: 0→1→...→n_left-1
  for (size_t i = 0; i + 1 < n_left; ++i) {
    edges.push_back(CDT::Edge(
      static_cast<CDT::VertInd>(i),
      static_cast<CDT::VertInd>(i + 1)));
  }
  // 우측 경계 엣지: n_left→n_left+1→...→n_total-1
  for (size_t i = 0; i + 1 < n_right; ++i) {
    edges.push_back(CDT::Edge(
      static_cast<CDT::VertInd>(n_left + i),
      static_cast<CDT::VertInd>(n_left + i + 1)));
  }
  // 닫기 엣지: 좌측 끝 ↔ 우측 끝, 좌측 시작 ↔ 우측 시작
  edges.push_back(CDT::Edge(
    static_cast<CDT::VertInd>(n_left - 1),
    static_cast<CDT::VertInd>(n_total - 1)));
  edges.push_back(CDT::Edge(
    static_cast<CDT::VertInd>(0),
    static_cast<CDT::VertInd>(n_left)));

  // CDT 실행 (입력이 퇴화 시 예외 가능 → catch)
  CDT::Triangulation<double> cdt;
  try {
    cdt.insertVertices(verts);
    cdt.insertEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();
  } catch (...) {
    return result;  // CDT 실패 → 빈 결과
  }

  // ---- Step 3+4: 삼각형 필터링 + 외심 계산 (인덱스 기반) ----
  // eraseOuterTrianglesAndHoles() 후 인덱스가 재매핑됨 → 0-based 유효
  // Filter A (혼합 클래스) + Filter B (기하 조건) 유지, Filter C (x_filter) 삭제
  std::unordered_set<size_t> valid_set;
  std::unordered_map<size_t, Point2D> cc_map;

  for (size_t t = 0; t < cdt.triangles.size(); ++t) {
    const auto & tri = cdt.triangles[t];
    const auto v0 = tri.vertices[0];
    const auto v1 = tri.vertices[1];
    const auto v2 = tri.vertices[2];

    // Filter A: 경계 클래스 혼합 (좌+우 꼭짓점 모두 포함해야 통과)
    const bool has_left =
      (v0 < n_left) || (v1 < n_left) || (v2 < n_left);
    const bool has_right =
      (v0 >= n_left) || (v1 >= n_left) || (v2 >= n_left);
    if (!has_left || !has_right) {
      continue;
    }

    // 꼭짓점 좌표 추출
    const Point2D a{cdt.vertices[v0].x, cdt.vertices[v0].y};
    const Point2D b{cdt.vertices[v1].x, cdt.vertices[v1].y};
    const Point2D c{cdt.vertices[v2].x, cdt.vertices[v2].y};

    // Filter B: 기하 조건
    double sides[3] = {dist(a, b), dist(b, c), dist(c, a)};
    std::sort(sides, sides + 3);

    if (sides[0] < 1e-12) {
      continue;
    }

    const bool isosceles_like = (sides[2] / sides[1] < params.centerline.tri_isosceles_ratio);
    const bool pointed = (sides[2] / sides[0] > params.centerline.tri_pointed_ratio);
    const double area = 0.5 * std::abs(cross2(b - a, c - a));
    const bool large_area = (area > params.centerline.tri_min_area);

    if (!((isosceles_like && pointed) || large_area)) {
      continue;
    }

    valid_set.insert(t);
    cc_map[t] = circumcenter(a, b, c);
  }

  // ---- Step 5: 유효 삼각형 부족 시 실패 ----
  if (valid_set.size() < 2) {
    return result;
  }

  // ---- Step 6: 삼각형 인접 그래프 기반 체이닝 (DTR 논문 원본) ----
  // Triangle::neighbors[3] = 엣지를 공유하는 이웃 삼각형 인덱스
  // 인접한 유효 삼각형의 외심을 연결 = Voronoi 엣지 = medial axis

  // 시작 삼각형: ego(0,0)에서 외심이 가장 가까운 유효 삼각형
  size_t start_tri;
  {
    const Point2D ego{0.0, 0.0};
    start_tri = *valid_set.begin();
    double min_d = dist(ego, cc_map[start_tri]);
    for (size_t t : valid_set) {
      const double d = dist(ego, cc_map[t]);
      if (d < min_d) {
        min_d = d;
        start_tri = t;
      }
    }
  }

  // 양방향 인접 워크 — visited를 공유하여 겹침/사이클 방지
  std::unordered_set<size_t> walked;
  walked.insert(start_tri);

  auto walk = [&](size_t first) -> std::vector<size_t> {
    std::vector<size_t> path;
    size_t cur = first;
    while (!walked.count(cur)) {
      walked.insert(cur);
      path.push_back(cur);
      const auto & tri = cdt.triangles[cur];
      size_t next = cdt.triangles.size();
      for (int k = 0; k < 3; ++k) {
        const auto nb = static_cast<size_t>(tri.neighbors[k]);
        if (nb < cdt.triangles.size() && valid_set.count(nb) && !walked.count(nb)) {
          next = nb;
          break;
        }
      }
      if (next >= cdt.triangles.size()) break;
      cur = next;
    }
    return path;
  };

  // 시작 삼각형의 유효 이웃 탐색
  std::vector<size_t> start_nbs;
  {
    const auto & tri = cdt.triangles[start_tri];
    for (int k = 0; k < 3; ++k) {
      const auto nb = static_cast<size_t>(tri.neighbors[k]);
      if (nb < cdt.triangles.size() && valid_set.count(nb)) {
        start_nbs.push_back(nb);
      }
    }
  }

  // 양방향 워크 → 외심 체인 조립: reverse(path_b) + start + path_a
  std::vector<size_t> path_a, path_b;
  if (start_nbs.size() >= 1) {
    path_a = walk(start_nbs[0]);
  }
  if (start_nbs.size() >= 2) {
    path_b = walk(start_nbs[1]);
  }

  std::vector<Point2D> chain;
  chain.reserve(path_b.size() + 1 + path_a.size());
  for (auto it = path_b.rbegin(); it != path_b.rend(); ++it) {
    chain.push_back(cc_map[*it]);
  }
  chain.push_back(cc_map[start_tri]);
  for (size_t t : path_a) {
    chain.push_back(cc_map[t]);
  }

  if (chain.size() < 2) {
    return result;
  }

  // ---- Step 7: 리샘플링 + 이동평균 스무딩 ----
  result.center = resample_polyline(chain, resample_ds);

  // 3점 이동평균 스무딩 (첫/끝 점은 고정)
  if (result.center.size() >= 3) {
    std::vector<Point2D> smoothed;
    smoothed.reserve(result.center.size());
    smoothed.push_back(result.center.front());
    for (size_t i = 1; i + 1 < result.center.size(); ++i) {
      smoothed.push_back({
        (result.center[i - 1].x + result.center[i].x + result.center[i + 1].x) / 3.0,
        (result.center[i - 1].y + result.center[i].y + result.center[i + 1].y) / 3.0
      });
    }
    smoothed.push_back(result.center.back());
    result.center = smoothed;
  }

  result.valid = (result.center.size() >= 2);
  return result;
}

// ============================================================
// Case 2: 한쪽(visible) 경계에서 법선 방향 w_hat/2 오프셋으로 중심선 생성
// ============================================================
CenterlineResult CenterlineBuilder::build_from_one_side(
  const std::vector<Point2D> & visible,
  bool visible_is_left,
  double w_hat,
  const PlanningParams & params)
{
  CenterlineResult result;
  const double resample_ds = params.postprocess.resample_ds;

  auto vis_rs = resample_polyline(visible, resample_ds);
  if (vis_rs.size() < 2) {
    return result;
  }

  auto tangents = polyline_tangents(vis_rs);

  const double sgn = visible_is_left ? -1.0 : 1.0;
  const double offset = sgn * (w_hat / 2.0);

  result.center.reserve(vis_rs.size());
  for (size_t i = 0; i < vis_rs.size(); ++i) {
    const Point2D n = rotate90(tangents[i]);
    result.center.push_back(vis_rs[i] + offset * n);
  }

  if (result.center.size() >= 2) {
    result.center = resample_polyline(result.center, resample_ds);
  }

  result.valid = (result.center.size() >= 2);
  return result;
}

}  // namespace track_planning
