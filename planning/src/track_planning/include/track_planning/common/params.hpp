/**
 * @file params.hpp
 * @brief 플래닝 파이프라인의 전체 파라미터 정의 및 YAML 로딩
 *
 * PlanningParams 구조체 하나에 모든 파라미터를 중첩 구조체로 관리.
 * load(rclcpp::Node*) 메서드를 호출하면 ROS 2 파라미터 서버에서
 * planning.yaml의 값을 선언(declare)하고 가져온다(get).
 *
 * 파이프라인 흐름:
 *   Stale 검사 → 입력 파싱 → 코리더 빌드 → 가상 경계 →
 *   DTR 센터라인 → 후처리 → 안전 검사 → 퍼블리시
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
  // ROI — 코리더 구축 및 가상 경계 필터링 범위 (ego-centric)
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
  // Safety — 안전 마진
  //   가상 경계 생성 시 최소 차로 폭 판정(vehicle.width + margin)에 사용
  // ============================================================
  struct Safety
  {
    double margin = 0.10;  // 기본 안전 마진 (m)
  } safety;

  // ============================================================
  // Corridor: Seed — 체이닝 시작점 탐색 조건
  // ============================================================
  struct CorridorSeed
  {
    double r_seed        = 5.0;   // 최대 탐색 반지름 [m] (1m씩 확장하며 이 값까지 탐색)
    double r_seed_step   = 1.0;   // 반경 확장 단계 [m] (1m → 2m → ... → r_seed)
    double forward_range = 1.047; // 전방 탐색 각도 범위 [rad] (~60도, 접선 기준 좌우 허용)
    double w_dist        = 1.0;   // 점수 가중치: 이전 seed로부터의 거리 (멀수록 높은 점수)
    double w_center      = 1.0;   // 점수 가중치: 예측 중앙선 근접도 (가까울수록 높은 점수)
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
  // Corridor: General — 일반 제한
  // ============================================================
  struct CorridorGeneral
  {
    int max_points_side = 100;  // 한쪽 경계의 최대 점 수
  } corridor_general;

  // ============================================================
  // Virtual Boundary — 가상 경계 생성
  // ============================================================
  struct Virtual
  {
    double default_track_width = 1.5;  // 기본 트랙 폭 (m, 대회 규격)
    double min_corridor_width = 0.6;   // 최소 코리도 폭 (m)
  } virt;

  // ============================================================
  // Centerline — DTR 센터라인 빌드 파라미터
  //   삼각형 기하 필터 조건 (인접 그래프 체이닝은 파라미터 불필요)
  // ============================================================
  struct Centerline
  {
    double tri_isosceles_ratio = 1.5;   // 삼각형 이등변 비율 (sides[2]/sides[1] < ratio)
    double tri_pointed_ratio = 2.0;     // 삼각형 가늘기 비율 (sides[2]/sides[0] > ratio)
    double tri_min_area = 0.01;         // [m²] 삼각형 최소 면적
  } centerline;

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
  // Timeouts — 입력 데이터 stale 판정 기준 (ms)
  // ============================================================
  struct Timeouts
  {
    int perception_ms = 300;  // perception(lane/cone) 타임아웃 (ms)
  } timeouts;

  // ============================================================
  // load() — ROS 2 파라미터 서버에서 값 읽기
  // ============================================================

  /**
   * @brief 모든 파라미터를 ROS 2 노드에 declare하고 값을 로딩
   *
   * 내부에서 lambda 'p'를 사용: declare_parameter() + get_parameter() 원라인 처리.
   * YAML 파일에 값이 있으면 해당 값 사용, 없으면 코드의 기본값 사용.
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

    // Corridor: Seed
    corridor_seed.r_seed        = p("corridor.seed.r_seed",        corridor_seed.r_seed);
    corridor_seed.r_seed_step   = p("corridor.seed.r_seed_step",   corridor_seed.r_seed_step);
    corridor_seed.forward_range = p("corridor.seed.forward_range", corridor_seed.forward_range);
    corridor_seed.w_dist        = p("corridor.seed.w_dist",        corridor_seed.w_dist);
    corridor_seed.w_center      = p("corridor.seed.w_center",      corridor_seed.w_center);

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

    // Corridor: General
    corridor_general.max_points_side =
      p("corridor.general.max_points_side", corridor_general.max_points_side);

    // Virtual
    virt.default_track_width = p("virtual.default_track_width", virt.default_track_width);
    virt.min_corridor_width = p("virtual.min_corridor_width", virt.min_corridor_width);

    // Centerline
    centerline.tri_isosceles_ratio =
      p("centerline.tri_isosceles_ratio", centerline.tri_isosceles_ratio);
    centerline.tri_pointed_ratio =
      p("centerline.tri_pointed_ratio", centerline.tri_pointed_ratio);
    centerline.tri_min_area =
      p("centerline.tri_min_area", centerline.tri_min_area);

    // Speed
    speed.v_max = p("speed.v_max", speed.v_max);
    speed.a_lat_max = p("speed.a_lat_max", speed.a_lat_max);

    // Postprocess
    postprocess.resample_ds = p("postprocess.resample_ds", postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);

    // Timeouts
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);

    RCLCPP_INFO(
      node->get_logger(),
      "PlanningParams loaded: vehicle_w=%.2f, r_min=%.2f, v_max=%.2f",
      vehicle.width, vehicle.r_min(), speed.v_max);
  }
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__PARAMS_HPP_
