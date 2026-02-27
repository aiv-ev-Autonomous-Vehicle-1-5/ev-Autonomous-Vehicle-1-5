/**
 * @file cdt_centerline.cpp
 * @brief CDT 기반 Centerline 추출 알고리즘 구현
 *
 * DTR 논문의 circumcenter 필터링을 적용하여, 좌/우 경계 체인 사이의
 * 중심선(centerline)을 기하학적으로 추출한다.
 *
 * ── 의존 라이브러리 ──
 *   artem-ogre/CDT (https://github.com/artem-ogre/CDT)
 *   MPL-2.0 라이센스, header-only Constrained Delaunay Triangulation.
 *   CMake FetchContent로 빌드 시 자동 다운로드된다.
 */
#include "chaining_CDT/planner/cdt_centerline.hpp"
#include "chaining_CDT/common/params.hpp"

// artem-ogre/CDT 라이브러리 헤더
#include <CDT.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace chaining_CDT
{

// ── 내부 헬퍼 (익명 네임스페이스) ──
namespace
{

/**
 * @brief 삼각형의 외심(circumcenter) 계산
 *
 * 외심 = 세 꼭짓점에서 등거리인 점 = 외접원의 중심.
 * 좌/우 경계 사이의 삼각형이면, 외심은 자연스럽게 경계 사이 중앙에 위치한다.
 *
 * 공식:
 *   D = 2 * (Ax(By-Cy) + Bx(Cy-Ay) + Cx(Ay-By))
 *   Ux = ((Ax²+Ay²)(By-Cy) + (Bx²+By²)(Cy-Ay) + (Cx²+Cy²)(Ay-By)) / D
 *   Uy = ((Ax²+Ay²)(Cx-Bx) + (Bx²+By²)(Ax-Cx) + (Cx²+Cy²)(Bx-Ax)) / D
 *
 * @param ax,ay  꼭짓점 A 좌표
 * @param bx,by  꼭짓점 B 좌표
 * @param cx,cy  꼭짓점 C 좌표
 * @param[out] ux,uy  외심 좌표 (valid일 때만 유효)
 * @return true: 정상 계산됨, false: 세 점이 동일 직선(degenerate) → 외심 없음
 */
bool circumcenter(
  double ax, double ay, double bx, double by, double cx, double cy,
  double & ux, double & uy)
{
  const double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(D) < 1e-12) return false;  // degenerate triangle

  const double a2 = ax * ax + ay * ay;
  const double b2 = bx * bx + by * by;
  const double c2 = cx * cx + cy * cy;

  ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / D;
  uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / D;
  return true;
}

/**
 * @brief 두 점 사이의 유클리드 거리의 제곱
 *
 * sqrt를 생략하여 비교 연산에서 성능을 절약한다.
 */
inline double dist_sq(double x1, double y1, double x2, double y2)
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return dx * dx + dy * dy;
}

/**
 * @brief 삼각형 면적 (부호 없는 절대값)
 *
 * Shoelface 공식의 절반:
 *   Area = |Ax(By-Cy) + Bx(Cy-Ay) + Cx(Ay-By)| / 2
 */
inline double triangle_area(
  double ax, double ay, double bx, double by, double cx, double cy)
{
  return 0.5 * std::abs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
}

}  // anonymous namespace

// ══════════════════════════════════════════════════════════════
//  CDTCenterlineExtractor::extract() — 메인 알고리즘
// ══════════════════════════════════════════════════════════════
std::vector<Point2D> CDTCenterlineExtractor::extract(
  const std::vector<ChainPoint> & left_component,
  const std::vector<ChainPoint> & right_component,
  const PlanningParams & params) const
{
  // ── 입력 검증 ──
  // CDT는 최소 3개 이상의 정점이 필요하고,
  // 좌/우 각각 최소 2개 이상이어야 제약 간선을 구성할 수 있다.
  if (left_component.size() < 2 || right_component.size() < 2) {
    return {};
  }

  const auto & cp = params.cdt_planner;
  const size_t N_left = left_component.size();
  const size_t N_right = right_component.size();
  const size_t N_total = N_left + N_right;

  // ================================================================
  // Step 1: 정점(vertices) + 제약 간선(constraint edges) 구성
  // ================================================================
  // left 점들: 인덱스 [0, N_left)
  // right 점들: 인덱스 [N_left, N_total)
  std::vector<CDT::V2d<double>> vertices;
  vertices.reserve(N_total);

  for (const auto & p : left_component) {
    vertices.push_back(CDT::V2d<double>{p.x, p.y});
  }
  for (const auto & p : right_component) {
    vertices.push_back(CDT::V2d<double>{p.x, p.y});
  }

  // 제약 간선: 각 체인의 연속 쌍 (i, i+1)
  // 이 간선들이 CDT에서 반드시 유지되어야 하는 "벽"이 된다.
  // eraseOuterTrianglesAndHoles()가 이 제약 간선 바깥의 삼각형을 제거한다.
  std::vector<CDT::Edge> edges;
  edges.reserve((N_left - 1) + (N_right - 1));

  // left chain 연속 쌍
  for (size_t i = 0; i + 1 < N_left; ++i) {
    edges.push_back(CDT::Edge(
      static_cast<CDT::VertInd>(i),
      static_cast<CDT::VertInd>(i + 1)));
  }
  // right chain 연속 쌍
  for (size_t i = 0; i + 1 < N_right; ++i) {
    edges.push_back(CDT::Edge(
      static_cast<CDT::VertInd>(N_left + i),
      static_cast<CDT::VertInd>(N_left + i + 1)));
  }

  // ================================================================
  // Step 1.5: 중복 정점 제거 + 간선 인덱스 재매핑
  // ================================================================
  // 체이닝 리샘플 과정에서 동일 좌표의 점이 생길 수 있다.
  // CDT 라이브러리는 중복 정점을 허용하지 않으므로 (DuplicateVertexError),
  // 내장 유틸리티로 제거하고 제약 간선 인덱스를 자동 재매핑한다.
  CDT::RemoveDuplicatesAndRemapEdges(vertices, edges);

  // 중복 제거 후 정점이 3개 미만이면 CDT 불가
  if (vertices.size() < 3) {
    return {};
  }

  // ================================================================
  // Step 2: CDT 수행
  // ================================================================
  CDT::Triangulation<double> cdt(
    CDT::VertexInsertionOrder::Auto,
    CDT::IntersectingConstraintEdges::TryResolve,
    0.0);
  cdt.insertVertices(vertices);
  cdt.insertEdges(edges);
  cdt.eraseOuterTrianglesAndHoles();  // 제약 외부 삼각형 제거

  const auto & triangles = cdt.triangles;
  const auto & verts = cdt.vertices;

  if (triangles.empty()) {
    return {};
  }

  // ================================================================
  // Step 3 + 4: 외심 계산 + 기하학적 필터링
  // ================================================================
  // 각 삼각형에 대해:
  //   (1) 외심 계산
  //   (2) DTR 논문의 3가지 조건으로 필터링:
  //       - Isosceles-like: sides[1]/sides[2] < iso_ratio_max
  //       - Pointedness: sides[2]/sides[0] > pointed_ratio_min
  //       - Area: 면적 > area_min
  //       조건: (isosceles AND pointedness) OR (area > area_min)
  struct Circumcenter {
    double x, y;
  };
  std::vector<Circumcenter> circumcenters;
  circumcenters.reserve(triangles.size());

  for (const auto & tri : triangles) {
    const auto & A = verts[tri.vertices[0]];
    const auto & B = verts[tri.vertices[1]];
    const auto & C = verts[tri.vertices[2]];

    // ── Step 3: 외심 계산 ──
    double ux, uy;
    if (!circumcenter(A.x, A.y, B.x, B.y, C.x, C.y, ux, uy)) {
      continue;  // degenerate triangle → 스킵
    }

    // ── Step 4: 기하학적 필터링 ──
    // 변 길이의 제곱 계산 (sqrt 생략)
    double s0_sq = dist_sq(A.x, A.y, B.x, B.y);
    double s1_sq = dist_sq(B.x, B.y, C.x, C.y);
    double s2_sq = dist_sq(C.x, C.y, A.x, A.y);

    // 오름차순 정렬: sides[0] <= sides[1] <= sides[2]
    double sides_sq[3] = {s0_sq, s1_sq, s2_sq};
    std::sort(sides_sq, sides_sq + 3);

    // 최소 변 길이가 0이면 degenerate → 스킵
    if (sides_sq[0] < 1e-12) continue;

    // Isosceles-like: sides[1]/sides[2] < iso_ratio_max
    // 제곱 비교: (s1/s2)² < ratio² → s1_sq * ratio_sq_inv > s2_sq 형태는
    // 복잡하므로 sqrt를 사용한다 (삼각형 수가 적어 성능 영향 미미)
    const double s0 = std::sqrt(sides_sq[0]);
    const double s1 = std::sqrt(sides_sq[1]);
    const double s2 = std::sqrt(sides_sq[2]);

    const bool is_isosceles = (s1 / s2) < cp.iso_ratio_max;
    const bool is_pointed = (s2 / s0) > cp.pointed_ratio_min;
    const double area = triangle_area(A.x, A.y, B.x, B.y, C.x, C.y);
    const bool area_ok = area > cp.area_min;

    // DTR 논문 조건: (isosceles AND pointedness) OR area
    if ((is_isosceles && is_pointed) || area_ok) {
      circumcenters.push_back({ux, uy});
    }
  }

  if (circumcenters.empty()) {
    return {};
  }

  // ================================================================
  // Step 5: Greedy Nearest-Neighbor Centerline 연결
  // ================================================================
  // ego(0,0)에 가장 가까운 외심에서 시작하여,
  // 매 스텝마다 가장 가까운 미방문 외심을 선택한다.
  // 조건:
  //   1. 최대 거리: max_circumcenter_dist 이내
  //   2. 전방 검사: dot(heading, to_next) > forward_dot_min

  const size_t N_cc = circumcenters.size();
  std::vector<bool> visited(N_cc, false);
  std::vector<Point2D> centerline;
  centerline.reserve(N_cc);

  // ego(0,0)에 가장 가까운 외심 찾기
  size_t start_idx = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < N_cc; ++i) {
    const double d2 = circumcenters[i].x * circumcenters[i].x +
                       circumcenters[i].y * circumcenters[i].y;
    if (d2 < best_dist_sq) {
      best_dist_sq = d2;
      start_idx = i;
    }
  }

  // 시작점 추가
  visited[start_idx] = true;
  centerline.push_back({circumcenters[start_idx].x, circumcenters[start_idx].y});

  // 초기 heading: ego(0,0) → 시작점 방향
  // 시작점이 ego와 매우 가까우면 기본 전방 방향(1,0) 사용
  double hx, hy;
  if (best_dist_sq > 1e-6) {
    const double d = std::sqrt(best_dist_sq);
    hx = circumcenters[start_idx].x / d;
    hy = circumcenters[start_idx].y / d;
  } else {
    hx = 1.0;
    hy = 0.0;
  }

  size_t current = start_idx;
  const double max_dist_sq = cp.max_circumcenter_dist * cp.max_circumcenter_dist;

  // Greedy nearest-neighbor 루프
  while (true) {
    double best_d2 = std::numeric_limits<double>::max();
    size_t best_next = N_cc;  // invalid sentinel

    const double cx = circumcenters[current].x;
    const double cy = circumcenters[current].y;

    for (size_t i = 0; i < N_cc; ++i) {
      if (visited[i]) continue;

      const double dx = circumcenters[i].x - cx;
      const double dy = circumcenters[i].y - cy;
      const double d2 = dx * dx + dy * dy;

      // 조건 1: 최대 거리
      if (d2 > max_dist_sq) continue;
      if (d2 < 1e-12) continue;  // 자기 자신과 동일한 위치

      // 조건 2: 전방 검사 (dot product)
      const double d = std::sqrt(d2);
      const double dot = (dx * hx + dy * hy) / d;
      if (dot < cp.forward_dot_min) continue;

      // 가장 가까운 후보 선택
      if (d2 < best_d2) {
        best_d2 = d2;
        best_next = i;
      }
    }

    // 더 이상 연결할 외심이 없으면 종료
    if (best_next >= N_cc) break;

    // heading 업데이트: 현재 → 다음 방향
    const double dx = circumcenters[best_next].x - cx;
    const double dy = circumcenters[best_next].y - cy;
    const double d = std::sqrt(best_d2);
    hx = dx / d;
    hy = dy / d;

    // 다음 점 추가
    visited[best_next] = true;
    centerline.push_back({circumcenters[best_next].x, circumcenters[best_next].y});
    current = best_next;
  }

  return centerline;
}

}  // namespace chaining_CDT
