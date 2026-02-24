/**
 * @file params.hpp
 * @brief 플래닝 파이프라인의 전체 파라미터 정의 및 YAML 로딩
 *
 * PlanningParams 구조체 하나에 모든 파라미터를 중첩 구조체로 관리.
 * load(rclcpp::Node*) 메서드를 호출하면 ROS 2 파라미터 서버에서
 * planning.yaml의 값을 선언(declare)하고 가져온다(get).
 *
 * 차량 점 모델(Vehicle Point Model):
 *   v3 명세서에서 차량을 점으로 모델링하고, 차량 폭과 안전 마진을
 *   inflation 반경에 흡수시킨다.
 *   r_infl = vehicle.width/2 + safety.margin + extra_margin
 */
#ifndef TRACK_PLANNING__COMMON__PARAMS_HPP_
#define TRACK_PLANNING__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace track_planning
{

struct PlanningParams
{
  // ============================================================
  // ROI & Grid — costmap의 영역 범위 및 해상도
  // ============================================================
  struct ROI
  {
    double x_min = -1.0;     // ego 뒤쪽 (m)
    double x_max = 10.0;     // ego 앞쪽 (m)
    double y_min = -4.0;     // 우측 (m)
    double y_max = 4.0;      // 좌측 (m)
    double resolution = 0.10; // 그리드 셀 크기 (m/cell)
  } roi;

  // ============================================================
  // Vehicle — T870 차량 제원
  // ============================================================
  struct Vehicle
  {
    double width = 0.50;         // 차폭 (m)
    double wheelbase = 0.87;     // 축거 (m) — 앞바퀴~뒷바퀴 거리
    double delta_max = 0.314;    // 최대 조향각 (rad, ~18도)

    /// 최소 회전반경: R_min = L / tan(δ_max)
    /// Ackermann 기하학에서 유도 — 곡률 feasibility 판정에 사용
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ============================================================
  // Safety — 안전 마진 (차량 점 모델)
  // ============================================================
  struct Safety
  {
    double margin = 0.10;             // 기본 안전 마진 (m)
    double margin_boundary = 0.0;     // 경계 inflation 추가 마진
    double margin_obstacle = 0.0;     // 장애물 inflation 추가 마진
  } safety;

  // ============================================================
  // Corridor: Seed — 체이닝 시작점 탐색 조건
  // ============================================================
  struct CorridorSeed
  {
    double x_seed_min = 0.5;     // seed를 찾기 시작할 최소 x 좌표 (m)
  } corridor_seed;

  // ============================================================
  // Corridor: Filter — 후보점 필터링 (s/d 좌표계)
  // ============================================================
  struct CorridorFilter
  {
    double s_min = 0.05;   // 최소 전방 진행거리 (m) — 뒤로 가는 점 제외
    double s_max = 2.0;    // 최대 전방 탐색거리 (m)
    double d_max = 2.0;    // 최대 횡방향 거리 (m)
    double r_search = 2.5; // 원형 탐색 반경 (m, 선택적)
  } corridor_filter;

  // ============================================================
  // Corridor: Scoring — 후보점 점수 산정 가중치
  // ============================================================
  struct CorridorScore
  {
    double w_s = 1.0;      // 전방 진행 가중치 (클수록 먼 점 선호)
    double w_d = 0.8;      // 횡방향 편차 패널티
    double w_a = 0.5;      // 각도 편차 패널티
    double w_p = 0.3;      // 예측 오차 패널티
    double theta_max = 1.047;  // 각도 정규화 상한 (~60도)
    double step_pred = 0.5;    // 예측 스텝 거리 (m)
  } corridor_score;

  // ============================================================
  // Corridor: Top-K — 상위 K개 후보 중 최종 선택
  // ============================================================
  struct CorridorTopK
  {
    bool enable = false;  // Top-K 활성화 여부
    int k_top = 5;        // 상위 K개
  } corridor_topk;

  // ============================================================
  // Corridor: Reference Tangent — 참조 접선 회귀
  // ============================================================
  struct CorridorRefTangent
  {
    int n_reg = 7;  // 회귀에 사용할 점 개수 (regress_tangent 윈도우)
  } corridor_ref_tangent;

  // ============================================================
  // Corridor: Cold Start — 이전 centerline 없을 때 초기화
  // ============================================================
  struct CorridorColdStart
  {
    int min_centerline_points = 3;       // centerline_prev 최소 점 수
    double theta_heading_gate_deg = 60.0; // heading 기준 후보 필터 각도 (deg)
  } corridor_cold_start;

  // ============================================================
  // Corridor: General — 일반 제한
  // ============================================================
  struct CorridorGeneral
  {
    int max_points_side = 100;  // 한쪽 경계의 최대 점 수
  } corridor_general;

  // ============================================================
  // Pair Validator — 좌우 경계 쌍 검증 기준
  // ============================================================
  struct Pair
  {
    double resample_ds = 0.2;    // 검증용 리샘플 간격 (m)
    double theta_mean_th = 0.35; // 접선 각도차 평균 상한 (~20도)
    double theta_max_th = 0.70;  // 접선 각도차 최대 상한 (~40도)
    double w_min = 0.8;          // 최소 허용 폭 (m)
    double w_max = 3.0;          // 최대 허용 폭 (m)
    double w_std_th = 0.5;       // 폭 표준편차 상한 (m)
  } pair;

  // ============================================================
  // Virtual Boundary — 가상 경계 생성
  // ============================================================
  struct Virtual
  {
    double default_track_width = 1.5;  // 기본 트랙 폭 (m, 대회 규격)
    double ema_alpha = 0.3;            // EMA 스무딩 계수 (0~1)
    double min_corridor_width = 0.6;   // 최소 코리도 폭 (m)
  } virt;

  // ============================================================
  // Inflation — 셀 팽창 반경 (차량 점 모델)
  // ============================================================
  struct Inflation
  {
    double boundary_extra_margin = 0.0;  // 경계 inflation 추가 마진 (YAML)
    double obstacle_extra_margin = 0.0;  // 장애물 inflation 추가 마진 (YAML)

    // load() 시 자동 계산: vehicle.width/2 + safety.margin + extra
    double boundary_radius = 0.0;  // 경계 inflation 반경 (m)
    double obstacle_radius = 0.0;  // 장애물 inflation 반경 (m)
  } inflation;

  // ============================================================
  // Costmap Validation — DIRECT 모드 충돌 검사
  // ============================================================
  struct CostmapValid
  {
    double ds_check = 0.1;  // centerline 충돌 검사 샘플링 간격 (m)
    int cost_th = 80;        // 충돌로 판정할 cost 임계값
  } costmap_valid;

  // ============================================================
  // Goal Selection — A* 모드 목표점 선택
  // ============================================================
  struct Goal
  {
    double lookahead_l0 = 3.0;  // 기본 lookahead 거리 (m)
    double lookahead_kv = 0.5;  // 속도 비례 계수: L = L0 + kv * v
    int ring_samples = 36;      // Method2 ring sampling 후보 수
  } goal;

  // ============================================================
  // Speed — 속도 제한
  // ============================================================
  struct Speed
  {
    double v_max = 1.60;     // 최대 속도 (m/s, T870 하드웨어 제한)
    double a_lat_max = 2.0;  // 최대 횡가속도 (m/s²) — 곡률 속도 제한에 사용
  } speed;

  // ============================================================
  // Postprocess — 경로 후처리
  // ============================================================
  struct Postprocess
  {
    double resample_ds = 0.10;   // 출력 경로 점 간격 (m)
    int smooth_window = 5;       // 이동평균 윈도우 크기
    double prune_max_dev = 0.15; // pruning 최대 횡편차 (m)
  } postprocess;

  // ============================================================
  // Mode Selector — DIRECT / ASTAR 모드 전환 조건
  // ============================================================
  struct ModeSelector
  {
    double l_check = 5.0;          // 전방 충돌 검사 거리 (m)
    double centerline_jump_th = 1.0; // centerline 프레임간 점프 임계값 (m)
    bool enable_astar = true;       // A* 모드 활성화 (false면 항상 DIRECT)
  } mode_selector;

  // ============================================================
  // Timeouts — 입력 데이터 stale 판정 기준 (ms)
  // ============================================================
  struct Timeouts
  {
    int odom_ms = 100;        // odometry 타임아웃 (ms)
    int perception_ms = 300;  // perception(lane/cone) 타임아웃 (ms)
    int plan_ms = 100;        // 계획 주기 타임아웃 (ms)
  } timeouts;

  // ============================================================
  // load() — ROS 2 파라미터 서버에서 값 읽기
  // ============================================================

  /**
   * @brief 모든 파라미터를 ROS 2 노드에 declare하고 값을 로딩
   *
   * 내부에서 lambda 'p'를 사용: declare_parameter() + get_parameter() 원라인 처리.
   * YAML 파일에 값이 있으면 해당 값 사용, 없으면 코드의 기본값 사용.
   * 마지막에 inflation 반경을 자동 계산 (차량 점 모델):
   *   r = vehicle.width/2 + safety.margin + extra_margin
   *
   * @param node  파라미터를 선언할 ROS 2 노드 포인터
   */
  void load(rclcpp::Node * node)
  {
    // 파라미터 선언 + 읽기를 한 줄로 처리하는 lambda
    auto p = [&](const std::string & name, auto default_val) {
      node->declare_parameter(name, rclcpp::ParameterValue(default_val));
      return node->get_parameter(name).get_value<decltype(default_val)>();
    };

    // ROI
    roi.x_min = p("roi.x_min", roi.x_min);
    roi.x_max = p("roi.x_max", roi.x_max);
    roi.y_min = p("roi.y_min", roi.y_min);
    roi.y_max = p("roi.y_max", roi.y_max);
    roi.resolution = p("roi.resolution", roi.resolution);

    // Vehicle
    vehicle.width = p("vehicle.width", vehicle.width);
    vehicle.wheelbase = p("vehicle.wheelbase", vehicle.wheelbase);
    vehicle.delta_max = p("vehicle.delta_max", vehicle.delta_max);

    // Safety
    safety.margin = p("safety.margin", safety.margin);
    safety.margin_boundary = p("safety.margin_boundary", safety.margin_boundary);
    safety.margin_obstacle = p("safety.margin_obstacle", safety.margin_obstacle);

    // Corridor: Seed
    corridor_seed.x_seed_min = p("corridor.seed.x_seed_min", corridor_seed.x_seed_min);

    // Corridor: Filter
    corridor_filter.s_min = p("corridor.filter.s_min", corridor_filter.s_min);
    corridor_filter.s_max = p("corridor.filter.s_max", corridor_filter.s_max);
    corridor_filter.d_max = p("corridor.filter.d_max", corridor_filter.d_max);
    corridor_filter.r_search = p("corridor.filter.r_search", corridor_filter.r_search);

    // Corridor: Scoring
    corridor_score.w_s = p("corridor.score.w_s", corridor_score.w_s);
    corridor_score.w_d = p("corridor.score.w_d", corridor_score.w_d);
    corridor_score.w_a = p("corridor.score.w_a", corridor_score.w_a);
    corridor_score.w_p = p("corridor.score.w_p", corridor_score.w_p);
    corridor_score.theta_max = p("corridor.score.theta_max", corridor_score.theta_max);
    corridor_score.step_pred = p("corridor.score.step_pred", corridor_score.step_pred);

    // Corridor: Top-K
    corridor_topk.enable = p("corridor.topk.enable", corridor_topk.enable);
    corridor_topk.k_top = p("corridor.topk.k_top", corridor_topk.k_top);

    // Corridor: Reference Tangent
    corridor_ref_tangent.n_reg = p("corridor.ref_tangent.n_reg", corridor_ref_tangent.n_reg);

    // Corridor: Cold Start
    corridor_cold_start.min_centerline_points =
      p("corridor.cold_start.min_centerline_points", corridor_cold_start.min_centerline_points);
    corridor_cold_start.theta_heading_gate_deg =
      p("corridor.cold_start.theta_heading_gate_deg", corridor_cold_start.theta_heading_gate_deg);

    // Corridor: General
    corridor_general.max_points_side =
      p("corridor.general.max_points_side", corridor_general.max_points_side);

    // Pair
    pair.resample_ds = p("pair.resample_ds", pair.resample_ds);
    pair.theta_mean_th = p("pair.theta_mean_th", pair.theta_mean_th);
    pair.theta_max_th = p("pair.theta_max_th", pair.theta_max_th);
    pair.w_min = p("pair.w_min", pair.w_min);
    pair.w_max = p("pair.w_max", pair.w_max);
    pair.w_std_th = p("pair.w_std_th", pair.w_std_th);

    // Virtual
    virt.default_track_width = p("virtual.default_track_width", virt.default_track_width);
    virt.ema_alpha = p("virtual.ema_alpha", virt.ema_alpha);
    virt.min_corridor_width = p("virtual.min_corridor_width", virt.min_corridor_width);

    // Inflation
    inflation.boundary_extra_margin =
      p("inflation.boundary_extra_margin", inflation.boundary_extra_margin);
    inflation.obstacle_extra_margin =
      p("inflation.obstacle_extra_margin", inflation.obstacle_extra_margin);

    // 차량 점 모델: inflation 반경 자동 계산
    // r = (차폭/2) + 기본마진 + 구간별_추가마진 + YAML_추가마진
    const double r_base = vehicle.width / 2.0 + safety.margin;
    inflation.boundary_radius = r_base + safety.margin_boundary + inflation.boundary_extra_margin;
    inflation.obstacle_radius = r_base + safety.margin_obstacle + inflation.obstacle_extra_margin;

    // Costmap Validation
    costmap_valid.ds_check = p("costmap_valid.ds_check", costmap_valid.ds_check);
    costmap_valid.cost_th = p("costmap_valid.cost_th", costmap_valid.cost_th);

    // Goal
    goal.lookahead_l0 = p("goal.lookahead_l0", goal.lookahead_l0);
    goal.lookahead_kv = p("goal.lookahead_kv", goal.lookahead_kv);
    goal.ring_samples = p("goal.ring_samples", goal.ring_samples);

    // Speed
    speed.v_max = p("speed.v_max", speed.v_max);
    speed.a_lat_max = p("speed.a_lat_max", speed.a_lat_max);

    // Postprocess
    postprocess.resample_ds = p("postprocess.resample_ds", postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);

    // Mode Selector
    mode_selector.l_check = p("mode_selector.l_check", mode_selector.l_check);
    mode_selector.centerline_jump_th =
      p("mode_selector.centerline_jump_th", mode_selector.centerline_jump_th);
    mode_selector.enable_astar = p("mode_selector.enable_astar", mode_selector.enable_astar);

    // Timeouts
    timeouts.odom_ms = p("timeouts.odom_ms", timeouts.odom_ms);
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);
    timeouts.plan_ms = p("timeouts.plan_ms", timeouts.plan_ms);

    RCLCPP_INFO(
      node->get_logger(),
      "PlanningParams loaded: vehicle_w=%.2f, r_min=%.2f, infl_boundary=%.3f, infl_obstacle=%.3f",
      vehicle.width, vehicle.r_min(), inflation.boundary_radius, inflation.obstacle_radius);
  }
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__PARAMS_HPP_
