/**
 * @file params.hpp
 * @brief MR 플래너 파이프라인의 모든 파라미터를 관리하는 구조체
 *
 * planning_mr.yaml에서 로드되는 파라미터들을 구조체로 관리한다.
 * load() 함수로 ROS 2 파라미터 서버에서 값을 읽어와 멤버에 저장한다.
 *
 * 파라미터 그룹:
 *   1. Costmap    — costmap 생성 관련 (그리드 크기, 자력 감쇠 등)
 *   2. Planner    — Greedy 전진 탐색 관련 (탐색 반경, 최대 스텝 등)
 *   3. Vehicle    — T870 차량 제원 (차폭, 축간거리, 최대 조향각)
 *   4. Safety     — 안전 마진
 *   5. Speed      — 속도 제한 (최대 속도, 최대 횡가속도)
 *   6. Postprocess — 경로 후처리 (리샘플 간격, 스무딩, 가지치기)
 *   7. Timeouts   — perception 데이터 타임아웃
 */
#ifndef PLANNING_MR_VER__COMMON__PARAMS_HPP_
#define PLANNING_MR_VER__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace planning_mr_ver
{

struct PlanningParams
{
  // ============================================================
  // Costmap — Magnetic Resistance costmap 파라미터
  //
  // 콘과 차선을 S극 자석으로 모델링하여 costmap을 생성한다.
  // 각 자석의 자력(cost)은 중심에서 가장 강하고, 거리에 따라 감쇠한다.
  // 감쇠 공식: cost = cost_max / (1 + alpha * d_eff²)
  //   여기서 d_eff = max(0, 거리 - inner_radius)
  // ============================================================
  struct Costmap
  {
    double size_x = 10.0;          ///< [m] grid 전체 폭 (ego 중심: -5 ~ +5)
    double size_y = 10.0;          ///< [m] grid 전체 높이 (ego 중심: -5 ~ +5)
    double resolution = 0.05;      ///< [m/cell] 셀 크기 (200×200 = 40,000 cells)
    double cone_cost_max = 100.0;  ///< 콘 중심의 최대 cost (S극 자력 세기)
    double lane_cost_max = 50.0;   ///< 차선 경계점의 최대 cost (콘보다 약함)
    double cone_radius = 0.65;     ///< [m] 콘 클러스터 반지름 (이 내부는 cost = max, flat zone)
    double alpha = 2.0;            ///< 감쇠율 계수. 클수록 빠르게 감쇠 (좁은 영향 범위)
    double cost_threshold = 2.0;   ///< cutoff: 이 값 미만이면 0 처리 (연산 절약)
  } costmap;

  // ============================================================
  // Planner — Greedy 전진 탐색 파라미터
  //
  // ego(0,0)에서 heading 방향으로 전방 180°를 탐색하며
  // cost가 가장 낮은 셀로 한 칸씩 이동하여 경로를 생성한다.
  // ============================================================
  struct Planner
  {
    double search_radius = 0.30;   ///< [m] 전방 180° 탐색 반경 (이 원 안에서 최소 cost 셀 선택)
    int max_steps = 200;           ///< 최대 경로 포인트 수 (이 이상이면 탐색 종료)
    double heading_init_x = 1.0;   ///< 초기 heading X (+x = 전방, 차량 진행 방향)
    double heading_init_y = 0.0;   ///< 초기 heading Y (0 = 직진)
  } planner;

  // ============================================================
  // Vehicle — T870 전동 카트 제원
  // ============================================================
  struct Vehicle
  {
    double width = 0.50;       ///< [m] 차량 폭
    double wheelbase = 0.87;   ///< [m] 축간 거리 (앞바퀴~뒷바퀴 중심 간 거리)
    double delta_max = 0.314;  ///< [rad] 최대 조향각 (~18°)

    /**
     * @brief 최소 회전 반경 계산: r_min = wheelbase / tan(delta_max)
     *
     * Ackermann 조향 기하학에서 유도된다.
     * 이 값보다 작은 곡률 반경의 경로는 차량이 물리적으로 따라갈 수 없다.
     */
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ============================================================
  // Safety — 안전 마진
  // ============================================================
  struct Safety
  {
    double margin = 0.10;  ///< [m] 장애물과의 안전 마진 (차체 + 여유)
  } safety;

  // ============================================================
  // Speed — 속도 제한
  //
  // 곡선 구간에서는 횡가속도 제한(a_lat_max)으로 속도를 줄인다.
  // v_curve = sqrt(a_lat_max / curvature)
  // ============================================================
  struct Speed
  {
    double v_max = 1.60;      ///< [m/s] 최대 주행 속도
    double a_lat_max = 2.0;   ///< [m/s²] 최대 횡가속도 (곡선 주행 시 속도 제한 기준)
  } speed;

  // ============================================================
  // Postprocess — 경로 후처리
  //
  // raw_path에 대해 prune → smooth → resample → yaw 4단계 후처리를 수행한다.
  // ============================================================
  struct Postprocess
  {
    double resample_ds = 0.10;    ///< [m] 출력 경로 포인트 간격
    int smooth_window = 5;        ///< 이동 평균 스무딩 윈도우 크기 (홀수 권장)
    double prune_max_dev = 0.15;  ///< [m] 가지치기 최대 횡편차 (이 이내면 중간점 생략)
  } postprocess;

  // ============================================================
  // Timeouts — 인식 데이터 타임아웃
  // ============================================================
  struct Timeouts
  {
    int perception_ms = 300;  ///< [ms] 이 시간 내 인식 데이터 미수신 시 STALE 판정
  } timeouts;

  // ============================================================
  // load() — ROS 2 파라미터 서버에서 값 읽기
  // ============================================================
  /**
   * @brief ROS 2 노드에서 파라미터를 선언(declare)하고 읽어온다
   *
   * planning_mr.yaml의 값이 있으면 그 값을, 없으면 위의 기본값(default)을 사용한다.
   * 이 함수는 노드 생성자에서 1번만 호출된다.
   *
   * @param node 파라미터를 선언할 ROS 2 노드 포인터
   */
  void load(rclcpp::Node * node)
  {
    // 람다 헬퍼: 파라미터 선언 + 값 읽기를 한 줄로 처리
    //   name = "costmap.size_x" 같은 점(.)으로 구분된 파라미터 이름
    //   default_val = 위의 멤버 기본값
    //   declare_parameter(): 파라미터 선언 (yaml에 있으면 그 값, 없으면 default)
    //   get_parameter(): 선언된 파라미터 값 읽기
    auto p = [&](const std::string & name, auto default_val) {
      node->declare_parameter(name, rclcpp::ParameterValue(default_val));
      return node->get_parameter(name).get_value<decltype(default_val)>();
    };

    // Costmap 파라미터 로드
    costmap.size_x         = p("costmap.size_x",         costmap.size_x);
    costmap.size_y         = p("costmap.size_y",         costmap.size_y);
    costmap.resolution     = p("costmap.resolution",     costmap.resolution);
    costmap.cone_cost_max  = p("costmap.cone_cost_max",  costmap.cone_cost_max);
    costmap.lane_cost_max  = p("costmap.lane_cost_max",  costmap.lane_cost_max);
    costmap.cone_radius    = p("costmap.cone_radius",    costmap.cone_radius);
    costmap.alpha          = p("costmap.alpha",           costmap.alpha);
    costmap.cost_threshold = p("costmap.cost_threshold", costmap.cost_threshold);

    // Planner 파라미터 로드
    planner.search_radius  = p("planner.search_radius",  planner.search_radius);
    planner.max_steps      = p("planner.max_steps",      planner.max_steps);
    planner.heading_init_x = p("planner.heading_init_x", planner.heading_init_x);
    planner.heading_init_y = p("planner.heading_init_y", planner.heading_init_y);

    // Vehicle 파라미터 로드
    vehicle.width     = p("vehicle.width",     vehicle.width);
    vehicle.wheelbase = p("vehicle.wheelbase", vehicle.wheelbase);
    vehicle.delta_max = p("vehicle.delta_max", vehicle.delta_max);

    // Safety 파라미터 로드
    safety.margin = p("safety.margin", safety.margin);

    // Speed 파라미터 로드
    speed.v_max     = p("speed.v_max",     speed.v_max);
    speed.a_lat_max = p("speed.a_lat_max", speed.a_lat_max);

    // Postprocess 파라미터 로드
    postprocess.resample_ds   = p("postprocess.resample_ds",   postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);

    // Timeouts 파라미터 로드
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);

    // 로드된 핵심 파라미터 로그 출력
    RCLCPP_INFO(
      node->get_logger(),
      "MR PlanningParams loaded: grid=%.0fx%.0f res=%.3f cone_max=%.0f cone_r=%.2f alpha=%.1f",
      costmap.size_x, costmap.size_y, costmap.resolution,
      costmap.cone_cost_max, costmap.cone_radius, costmap.alpha);
  }
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__PARAMS_HPP_
