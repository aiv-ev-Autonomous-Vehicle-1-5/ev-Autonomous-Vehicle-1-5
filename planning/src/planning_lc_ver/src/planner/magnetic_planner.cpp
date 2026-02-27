/**
 * @file magnetic_planner.cpp
 * @brief Magnetic Resistance Planner — 구현부
 *
 * ──────────────────────────────────────────────────────────────────
 * [이 파일의 역할]
 *
 * MagneticPlanner 클래스의 세 가지 핵심 함수를 구현한다:
 *
 *   1. world_to_grid / grid_to_world  — 좌표 변환 유틸리티
 *   2. find_best_forward_cell          — 전방 원뿔 내 최소 비용 셀 탐색
 *   3. plan                            — 메인 greedy 전진 탐색 루프
 *
 * [알고리즘 흐름 요약]
 *
 *   plan() 진입
 *     │
 *     ├─ ego (0,0) 에서 시작, 초기 heading 설정
 *     │
 *     └─ for (step = 0; step < max_steps; ++step)
 *          │
 *          ├─ find_best_forward_cell()   ← 전방 원뿔 내 최적 셀 탐색
 *          │    (비용 최소 → 정렬도 최대 순으로 tiebreak)
 *          │
 *          ├─ Heading Damping            ← 이전 heading과 새 방향을 가중 혼합
 *          │    new_hdg = damp * old_hdg + (1-damp) * move_dir
 *          │    (급격한 방향 전환 방지, 경로 부드럽게 만듦)
 *          │
 *          ├─ Max Steer Clamp            ← 회전각 제한 (회전 행렬 적용)
 *          │    |delta_angle| > max_steer_rad 이면 clamped 각도로 회전
 *          │    (차량 조향 물리 한계 반영)
 *          │
 *          ├─ heading 갱신, 현재 위치 갱신, 경로에 추가
 *          │
 *          └─ 경계 도달 체크 → 도달 시 break
 * ──────────────────────────────────────────────────────────────────
 */
#include "planning_lc_ver/planner/magnetic_planner.hpp"
#include "planning_lc_ver/common/geometry.hpp"

#include <cmath>
#include <limits>

namespace planning_lc_ver
{

// ============================================================================
// 좌표 변환 함수
// ============================================================================

/**
 * world_to_grid — 월드 좌표(미터) → 그리드 인덱스(정수) 변환
 *
 * [변환 원리]
 *   그리드는 origin (좌측 하단)에서 시작하여,
 *   x방향(col)과 y방향(row)으로 resolution 간격의 셀로 분할된다.
 *
 *   예: origin_x = -5.0, resolution = 0.05 일 때
 *       wx = 0.0 → col = (0.0 - (-5.0)) / 0.05 = 100
 *       → 0.0m 지점은 왼쪽 끝에서 100번째 열에 해당
 *
 * [주의] static_cast<int>는 floor 방향으로 절삭하므로,
 *        음수 좌표가 origin 밖으로 나가면 음수 인덱스가 되어
 *        범위 체크에서 false를 반환한다.
 */
bool MagneticPlanner::world_to_grid(
  double wx, double wy,
  double origin_x, double origin_y,
  double resolution,
  int rows, int cols,
  int & row, int & col)
{
  // x축(전방/후방) → 열(col) 인덱스로 변환
  col = static_cast<int>((wx - origin_x) / resolution);
  // y축(좌/우)     → 행(row) 인덱스로 변환
  row = static_cast<int>((wy - origin_y) / resolution);
  // 그리드 범위 안에 있는지 확인 (0 이상, 최대 행/열 미만)
  return (row >= 0 && row < rows && col >= 0 && col < cols);
}

/**
 * grid_to_world — 그리드 인덱스(정수) → 월드 좌표(미터) 변환
 *
 * [셀 중심 좌표 반환]
 *   +0.5를 더하는 이유: 셀의 "좌측 하단 꼭짓점"이 아닌 "중심점"을 반환하기 위해.
 *
 *   예: col=100, origin_x=-5.0, resolution=0.05
 *       wx = -5.0 + (100 + 0.5) * 0.05 = -5.0 + 5.025 = 0.025
 *       → 100번째 열의 중심은 0.025m 지점
 *
 *   만약 +0.5 없이 col * resolution만 쓰면 셀의 왼쪽 끝 좌표가 되어
 *   경로 포인트가 셀 경계에 치우치게 된다.
 *
 *       col 범위   |  0.5 없이  | 0.5 추가
 *       [0.0, 0.05) |  0.000    | 0.025   ← 중심이 더 정확
 */
Point2D MagneticPlanner::grid_to_world(
  int row, int col,
  double origin_x, double origin_y,
  double resolution)
{
  return {
    origin_x + (col + 0.5) * resolution,   // col → x 좌표 (셀 중심)
    origin_y + (row + 0.5) * resolution    // row → y 좌표 (셀 중심)
  };
}

// ============================================================================
// 전방 원뿔 탐색 — Greedy 셀 선택의 핵심
// ============================================================================

/**
 * find_best_forward_cell — 전방 원뿔 내에서 최소 비용 셀을 찾는 함수
 *
 * [동작 원리: 단계별 설명]
 *
 *   Step 1. 탐색 영역 설정
 *     - search_radius를 그리드 셀 단위(r_cells)로 변환
 *     - 현재 위치 주변 (cur_row ± r_cells, cur_col ± r_cells) 사각형 탐색
 *     - 그리드 경계를 넘지 않도록 min/max로 클램핑
 *
 *   Step 2. 각 셀에 대해 3가지 필터 적용
 *     (a) 거리 필터: 자기 자신(d≈0)이거나 search_radius 밖이면 skip
 *     (b) 원뿔 필터: heading과의 alignment가 cos(half_cone)보다 작으면 skip
 *         → 전방 원뿔 바깥의 셀은 무시
 *     (c) 비용 비교: 통과한 셀 중 cost가 가장 낮은 것을 선택
 *         → 동일 cost이면 alignment(heading 일치도)가 높은 것을 선택
 *
 *   [전방 원뿔(Forward Cone) 시각화]
 *
 *                  heading 방향
 *                      ↑
 *                     /|\
 *                    / | \
 *                   /  |  \     ← forward_cone_deg (예: 60°)
 *                  /   |   \       → 반각: 30°
 *                 / 30°|30° \
 *                /     |     \
 *               /      |      \
 *              +---[현재 위치]---+
 *                  (search_radius)
 *
 *   alignment = cos(heading과 셀 방향 사이의 각도)
 *   cos(30°) ≈ 0.866 → alignment > 0.866이면 원뿔 내부
 *
 *   [셀 선택 기준 (우선순위)]
 *
 *   1차: cost 최소  — 장애물에서 멀수록 cost가 낮음
 *   2차: alignment 최대 (tiebreaker) — heading과 일직선일수록 선호
 *
 *   이렇게 하면 안전한 셀(cost 낮음) 중에서도 가능한 직진 방향에 가까운
 *   셀을 선택하여 불필요한 지그재그를 방지한다.
 */
Point2D MagneticPlanner::find_best_forward_cell(
  const CostmapResult & costmap,
  const Point2D & current_pos,
  const Point2D & hdg,
  const PlanningParams & params,
  bool & found)
{
  found = false;
  // 지금까지 발견한 최적 셀의 비용 (초기값: 무한대 → 어떤 셀이든 갱신 가능)
  double best_cost = std::numeric_limits<double>::max();
  // 지금까지 발견한 최적 셀의 alignment (초기값: -1 → 어떤 양수 alignment이든 갱신 가능)
  double best_dot = -1.0;
  Point2D best_pos{0.0, 0.0};

  // ── Step 1: 탐색 영역 준비 ──
  const double r = params.planner.search_radius;    // 탐색 반경 [m]
  const double r_sq = r * r;                        // 거리 비교 시 sqrt 회피용 (제곱 비교)
  // 반경을 그리드 셀 개수로 변환 (올림: 부족하지 않게)
  int r_cells = static_cast<int>(std::ceil(r / costmap.resolution));

  // 전방 원뿔의 반각(half cone angle)을 라디안으로 변환
  // 예: forward_cone_deg=60° → half=30° → cos(30°)=0.866
  const double half_cone_rad = params.planner.forward_cone_deg * 0.5 * M_PI / 180.0;
  const double cos_half_cone = std::cos(half_cone_rad);

  // 현재 위치를 그리드 인덱스로 변환 (범위 밖이면 탐색 불가)
  int cur_row, cur_col;
  if (!world_to_grid(current_pos.x, current_pos.y,
      costmap.origin_x, costmap.origin_y,
      costmap.resolution, costmap.rows, costmap.cols,
      cur_row, cur_col)) {
    return best_pos;   // 그리드 밖이면 빈 결과 반환
  }

  // 탐색 사각형의 행/열 범위 (그리드 경계로 클램핑)
  int row_min = std::max(0, cur_row - r_cells);
  int row_max = std::min(costmap.rows - 1, cur_row + r_cells);
  int col_min = std::max(0, cur_col - r_cells);
  int col_max = std::min(costmap.cols - 1, cur_col + r_cells);

  // ── Step 2: 사각형 내 모든 셀을 순회하며 최적 셀 탐색 ──
  for (int row = row_min; row <= row_max; ++row) {
    for (int col = col_min; col <= col_max; ++col) {
      // 그리드 인덱스 → 월드 좌표 (셀 중심)
      Point2D cell_pos = grid_to_world(
        row, col, costmap.origin_x, costmap.origin_y, costmap.resolution);

      // 현재 위치에서 해당 셀까지의 벡터 (dx, dy)
      double dx = cell_pos.x - current_pos.x;
      double dy = cell_pos.y - current_pos.y;
      double d_sq = dx * dx + dy * dy;   // 거리의 제곱

      // 필터 (a): 자기 자신(거리≈0) 또는 반경 밖이면 건너뜀
      if (d_sq < 1e-12 || d_sq > r_sq) continue;

      // ── 전방 원뿔 판정 ──
      double d = std::sqrt(d_sq);
      // heading 벡터와 셀 방향 벡터의 내적 (정규화 전)
      double dot_val = hdg.x * dx + hdg.y * dy;
      // alignment = cos(heading과 셀 방향 사이의 각도)
      // 1.0이면 정확히 heading 방향, 0.0이면 직각, -1.0이면 반대 방향
      double alignment = dot_val / d;

      // 필터 (b): 전방 원뿔 밖이면 건너뜀
      // alignment <= cos_half_cone → 각도가 half_cone_rad 이상 벌어져 있음
      if (alignment <= cos_half_cone) continue;

      // 해당 셀의 비용 값 (row-major 1D 배열 접근)
      double cost = costmap.data[row * costmap.cols + col];

      // 필터 (c): cost_ceiling 이상인 셀은 후보에서 제외
      // 콘 중심 근처(비용 ≥ 95) 등 위험 영역은 절대 경로로 선택하지 않음
      if (cost >= params.planner.cost_ceiling) continue;

      // ── 셀 선택 (greedy 비교) ──
      // 1차: cost가 더 낮으면 무조건 갱신
      // 2차: cost가 같으면 alignment(heading 정렬도)가 더 높은 쪽을 선택
      //      → 같은 안전도의 셀 중에서 직진 방향에 가까운 셀을 선호
      if (cost < best_cost || (cost == best_cost && alignment > best_dot)) {
        best_cost = cost;
        best_dot = alignment;
        best_pos = cell_pos;
        found = true;
      }
    }
  }

  return best_pos;
}

// ============================================================================
// 메인 경로 생성 — Greedy 전진 탐색 루프
// ============================================================================

/**
 * plan — costmap 위에서 greedy 전진 탐색으로 경로 생성
 *
 * [알고리즘 상세 흐름]
 *
 *   1. ego 위치 (0,0)에서 출발, 초기 heading 설정
 *   2. 매 스텝마다:
 *      a) find_best_forward_cell()로 전방 원뿔 내 최소 비용 셀 탐색
 *      b) 못 찾으면 경로 종료 (전방에 갈 곳이 없음)
 *      c) 이동 방향 계산 (현재→다음 셀의 단위 벡터)
 *      d) Heading Damping 적용 (급회전 방지)
 *      e) Max Steer Rate Clamp 적용 (차량 조향 한계)
 *      f) heading 갱신, 위치 갱신, 경로에 추가
 *      g) 그리드 경계 근처면 종료
 *   3. 원시 경로(raw_path) 반환 → PathPostprocessor에서 후처리
 *
 * [Heading Damping (헤딩 감쇠) — 왜 필요한가?]
 *
 *   damping 없이 매 스텝마다 최소 비용 셀 방향으로 즉시 heading을 바꾸면:
 *     → 좌우 비용이 비슷한 구간에서 지그재그(oscillation)가 발생
 *     → 경로가 들쭉날쭉해져서 차량 제어가 어려워짐
 *
 *   damping을 적용하면:
 *     new_hdg = damp * old_hdg + (1-damp) * move_dir
 *     → 이전 heading의 관성(inertia)을 유지하면서 새 방향으로 서서히 전환
 *     → damp=0.5: 이전 방향과 새 방향을 50:50으로 혼합
 *     → damp=0.8: 이전 방향 80% 유지 → 매우 부드럽지만 반응 느림
 *     → damp=0.2: 이전 방향 20% 유지 → 빠른 반응이지만 요동 가능
 *
 *   [비유] 스프링 댐퍼: heading을 "스프링"으로 새 방향에 끌리게 하되,
 *          "댐퍼"로 급격한 변화를 억제하는 것과 같다.
 *
 * [Max Steer Rate Clamp (최대 조향각 제한) — 어떻게 동작하는가?]
 *
 *   heading damping 후에도 한 스텝당 회전각이 차량의 물리적 조향 한계를
 *   초과할 수 있다. 이를 방지하기 위해 회전각을 clamp한다.
 *
 *   1. delta_angle = atan2(cross(old_hdg, new_hdg), dot(old_hdg, new_hdg))
 *      → old_hdg에서 new_hdg까지의 부호 있는 회전 각도 계산
 *      → cross: 회전 방향(양수=반시계, 음수=시계)
 *      → dot: cos(각도) 성분
 *
 *   2. |delta_angle| > max_steer_rad 이면:
 *      → clamped = sign(delta_angle) * max_steer_rad
 *      → old_hdg를 clamped 각도만큼 회전시켜 new_hdg를 재계산
 *
 *   3. 회전 행렬 적용:
 *      [cos(θ)  -sin(θ)] [hdg.x]   [hdg.x·cosθ - hdg.y·sinθ]
 *      [sin(θ)   cos(θ)] [hdg.y] = [hdg.x·sinθ + hdg.y·cosθ]
 *
 *      → 2D 회전 행렬 R(θ)를 old_hdg 벡터에 곱해서
 *        정확히 θ 라디안만큼만 회전한 새 heading을 얻는다.
 *
 *   [왜 회전 행렬인가?]
 *   단순히 각도를 더하는 것보다 회전 행렬을 사용하면:
 *   - atan2/sin/cos 왕복 변환 없이 벡터를 직접 회전
 *   - 단위 벡터 성질이 자동으로 보존됨 (정규화 불필요)
 *   - 수치적으로 안정적
 */
std::vector<Point2D> MagneticPlanner::plan(
  const CostmapResult & costmap,
  const PlanningParams & params)
{
  std::vector<Point2D> raw_path;

  // costmap이 유효하지 않으면 빈 경로 반환
  if (!costmap.valid) return raw_path;

  // ── 초기 설정 ──
  // ego 차량 위치 = 원점 (0, 0) — base_link 좌표계 기준
  Point2D current_pos{0.0, 0.0};
  // 초기 heading 방향을 파라미터에서 읽어 정규화
  // 기본값: (1.0, 0.0) → 전방(+x) 방향
  Point2D hdg = normalize(
    Point2D{params.planner.heading_init_x, params.planner.heading_init_y});

  // 시작점을 경로에 추가
  raw_path.push_back(current_pos);

  // ── Greedy 전진 탐색 루프 ──
  // 매 스텝: 전방 원뿔에서 최적 셀 선택 → heading 갱신 → 이동
  for (int step = 0; step < params.planner.max_steps; ++step) {

    // ── (a) 전방 원뿔 내 최적 셀 탐색 ──
    bool found = false;
    Point2D next_pos = find_best_forward_cell(
      costmap, current_pos, hdg, params, found);

    // 전방에 유효한 셀이 없으면 경로 종료
    // (막다른 길, 그리드 밖, 또는 모든 셀이 원뿔 밖)
    if (!found) break;

    // ── (b) 이동 방향 벡터 계산 ──
    // 현재 위치 → 다음 셀까지의 단위 벡터
    Point2D move_dir = normalize(next_pos - current_pos);
    // 이동 거리가 거의 0이면 종료 (같은 셀에 갇힘 방지)
    if (norm(move_dir) < 1e-6) break;

    // ──────────────────────────────────────────────────────────
    // (c) Heading Damping (헤딩 감쇠)
    //
    // 이전 heading(hdg)과 새 이동 방향(move_dir)을 가중 혼합.
    //
    //   blended = damp * hdg + (1 - damp) * move_dir
    //
    // damp 값이 클수록 이전 heading의 관성이 강하여 경로가 부드러워지지만,
    // 장애물에 대한 회피 반응이 느려진다.
    // damp 값이 작으면 새 방향에 빠르게 반응하지만 경로가 들쭉날쭉해질 수 있다.
    //
    // 예시 (damp=0.5):
    //   이전 heading = (1, 0) [직진], move_dir = (0.7, 0.7) [좌측 45°]
    //   blended = 0.5*(1,0) + 0.5*(0.7,0.7) = (0.85, 0.35)
    //   → normalize → 약 15° 좌회전 (45°의 절반보다 적게 회전)
    // ──────────────────────────────────────────────────────────
    const double damp = params.planner.heading_damping;
    Point2D blended{
      damp * hdg.x + (1.0 - damp) * move_dir.x,
      damp * hdg.y + (1.0 - damp) * move_dir.y
    };
    Point2D new_hdg = normalize(blended);

    // ──────────────────────────────────────────────────────────
    // (d) Max Turn Rate Clamp (최대 조향각 제한)
    //
    // heading damping 후에도 한 스텝당 회전각이 차량의 물리적 조향 한계를
    // 초과할 수 있다. 이때 회전 행렬로 최대 허용 각도까지만 회전시킨다.
    //
    // 1. delta_angle: old_hdg → new_hdg 사이의 부호 있는 회전각
    //    - atan2(cross, dot) 형태로 [-π, π] 범위의 정확한 각도를 얻음
    //    - cross2 > 0: 반시계(좌회전), < 0: 시계(우회전)
    //
    // 2. |delta_angle| > max_steer_rad 이면:
    //    - clamped 각도로 old_hdg를 회전 → 최대 회전만 허용
    //
    // 3. 2D 회전 행렬 R(θ):
    //    new_hdg.x = hdg.x * cos(θ) - hdg.y * sin(θ)
    //    new_hdg.y = hdg.x * sin(θ) + hdg.y * cos(θ)
    //
    // 이 clamp가 없으면: costmap에 갑작스러운 비용 변화가 있을 때
    // heading이 한 스텝에 90° 이상 꺾이는 비현실적 경로가 생길 수 있다.
    // 실제 차량(T870 카트)은 조향각에 물리적 한계가 있으므로 이를 반영한다.
    // ──────────────────────────────────────────────────────────
    const double max_steer_rad = params.planner.max_steer_per_step_deg * M_PI / 180.0;
    // old_hdg에서 new_hdg까지의 부호 있는 회전 각도 계산
    // cross2(a,b) = a.x*b.y - a.y*b.x → 외적의 z성분 (회전 방향 판별)
    // dot2(a,b) = a.x*b.x + a.y*b.y   → 내적 (cos 각도)
    double delta_angle = std::atan2(
      cross2(hdg, new_hdg),    // sin(θ) 성분 — 회전 방향 부호
      dot2(hdg, new_hdg));     // cos(θ) 성분 — 각도 크기

    if (std::fabs(delta_angle) > max_steer_rad) {
      // 회전각이 한계를 초과 → 부호를 유지하면서 max_steer_rad로 클램핑
      double clamped = (delta_angle > 0.0) ? max_steer_rad : -max_steer_rad;
      // 2D 회전 행렬의 cos/sin 요소 계산
      double cos_a = std::cos(clamped);
      double sin_a = std::sin(clamped);
      // old_hdg를 clamped 각도만큼 회전 → 새 heading
      // R(θ) * hdg = (hdg.x*cosθ - hdg.y*sinθ, hdg.x*sinθ + hdg.y*cosθ)
      new_hdg = {hdg.x * cos_a - hdg.y * sin_a,
                 hdg.x * sin_a + hdg.y * cos_a};
    }

    // ── heading 및 위치 갱신 ──
    hdg = new_hdg;
    current_pos = next_pos;
    raw_path.push_back(current_pos);

    // ── (e) 그리드 경계 도달 체크 ──
    // 경계에서 2셀 이내이면 종료 (경계 셀의 비용값은 신뢰할 수 없으므로)
    int r, c;
    if (!world_to_grid(current_pos.x, current_pos.y,
        costmap.origin_x, costmap.origin_y,
        costmap.resolution, costmap.rows, costmap.cols,
        r, c)) {
      break;  // 그리드 완전히 밖 → 종료
    }
    // 경계에서 2셀 마진 이내이면 경로 종료
    // (경계 근처 셀은 costmap 생성 시 데이터가 불완전할 수 있음)
    if (r <= 1 || r >= costmap.rows - 2 ||
        c <= 1 || c >= costmap.cols - 2) {
      break;
    }
  }

  return raw_path;
}

}  // namespace planning_lc_ver
