/**
 * @file params.hpp
 * @brief LC 플래너 파이프라인의 모든 파라미터를 관리하는 구조체
 *
 * planning_mr_ver 기반 + Chainer 섹션 추가.
 * planning_lc.yaml에서 로드되는 파라미터들을 구조체로 관리한다.
 */
#ifndef PLANNING_LC_VER__COMMON__PARAMS_HPP_
#define PLANNING_LC_VER__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace planning_lc_ver
{

struct PlanningParams
{
  // ============================================================
  // Costmap — Magnetic Resistance costmap 파라미터
  // ============================================================
  struct Costmap
  {
    double size_x = 10.0;
    double size_y = 10.0;
    double resolution = 0.05;
    double cone_cost_max = 100.0;
    double lane_cost_max = 50.0;
    double cone_radius = 0.65;
    double sigma = 1.0;
    double cost_threshold = 2.0;
  } costmap;

  // ============================================================
  // Planner — Greedy 전진 탐색 파라미터
  // ============================================================
  struct Planner
  {
    double search_radius = 0.30;
    int max_steps = 200;
    double heading_init_x = 1.0;
    double heading_init_y = 0.0;
    double forward_cone_deg = 60.0;
    double max_steer_per_step_deg = 30.0;
    double heading_damping = 0.5;
  } planner;

  // ============================================================
  // Vehicle — T870 전동 카트 제원
  // ============================================================
  struct Vehicle
  {
    double width = 0.50;
    double wheelbase = 0.87;
    double delta_max = 0.314;
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ============================================================
  // Safety — 안전 마진
  // ============================================================
  struct Safety
  {
    double margin = 0.10;
  } safety;

  // ============================================================
  // Speed — 속도 제한
  // ============================================================
  struct Speed
  {
    double v_max = 1.60;
    double a_lat_max = 2.0;
  } speed;

  // ============================================================
  // Postprocess — 경로 후처리
  // ============================================================
  struct Postprocess
  {
    double resample_ds = 0.10;
    int smooth_window = 5;
    double prune_max_dev = 0.15;
  } postprocess;

  // ============================================================
  // SensorTf — velodyne → base_link 좌표 오프셋
  // ============================================================
  struct SensorTf
  {
    double tf_x = 0.7;
    double tf_y = 0.0;
    double tf_z = 0.7;
  } sensor_tf;

  // ============================================================
  // Timeouts — 인식 데이터 타임아웃
  // ============================================================
  struct Timeouts
  {
    int perception_ms = 300;
  } timeouts;

  // ============================================================
  // Chainer — LineChainer 파라미터 (신규)
  //
  // DFS 기반 nearest-neighbor 체이닝에서 사용하는 탐색 파라미터.
  // 후보가 없으면 search_radius를 search_radius_step씩 증가시키며
  // search_radius_max까지 확장한다.
  // ============================================================
  struct Chainer
  {
    double search_radius = 1.0;       ///< [m] 초기 탐색 반경
    double search_radius_step = 0.5;  ///< [m] 반경 증가 스텝
    double search_radius_max = 5.0;   ///< [m] 최대 탐색 반경
    double forward_angle = M_PI / 2;  ///< [rad] 전방 각도 (90°, ±45° cone)
    int min_regress_pts = 2;          ///< PCA regression 최소 점 수
    int max_regress_pts = 7;          ///< PCA regression 최대 점 수
    double resample_ds = 0.1;         ///< [m] 체인 리샘플 간격
  } chainer;

  // ============================================================
  // load() — ROS 2 파라미터 서버에서 값 읽기
  // ============================================================
  void load(rclcpp::Node * node)
  {
    auto p = [&](const std::string & name, auto default_val) {
      node->declare_parameter(name, rclcpp::ParameterValue(default_val));
      return node->get_parameter(name).get_value<decltype(default_val)>();
    };

    // Costmap
    costmap.size_x         = p("costmap.size_x",         costmap.size_x);
    costmap.size_y         = p("costmap.size_y",         costmap.size_y);
    costmap.resolution     = p("costmap.resolution",     costmap.resolution);
    costmap.cone_cost_max  = p("costmap.cone_cost_max",  costmap.cone_cost_max);
    costmap.lane_cost_max  = p("costmap.lane_cost_max",  costmap.lane_cost_max);
    costmap.cone_radius    = p("costmap.cone_radius",    costmap.cone_radius);
    costmap.sigma          = p("costmap.sigma",           costmap.sigma);
    costmap.cost_threshold = p("costmap.cost_threshold", costmap.cost_threshold);

    // Planner
    planner.search_radius  = p("planner.search_radius",  planner.search_radius);
    planner.max_steps      = p("planner.max_steps",      planner.max_steps);
    planner.heading_init_x = p("planner.heading_init_x", planner.heading_init_x);
    planner.heading_init_y = p("planner.heading_init_y", planner.heading_init_y);
    planner.forward_cone_deg       = p("planner.forward_cone_deg",       planner.forward_cone_deg);
    planner.max_steer_per_step_deg = p("planner.max_steer_per_step_deg", planner.max_steer_per_step_deg);
    planner.heading_damping        = p("planner.heading_damping",        planner.heading_damping);

    // Vehicle
    vehicle.width     = p("vehicle.width",     vehicle.width);
    vehicle.wheelbase = p("vehicle.wheelbase", vehicle.wheelbase);
    vehicle.delta_max = p("vehicle.delta_max", vehicle.delta_max);

    // Safety
    safety.margin = p("safety.margin", safety.margin);

    // Speed
    speed.v_max     = p("speed.v_max",     speed.v_max);
    speed.a_lat_max = p("speed.a_lat_max", speed.a_lat_max);

    // Postprocess
    postprocess.resample_ds   = p("postprocess.resample_ds",   postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);

    // SensorTf
    sensor_tf.tf_x = p("sensor_tf.tf_x", sensor_tf.tf_x);
    sensor_tf.tf_y = p("sensor_tf.tf_y", sensor_tf.tf_y);
    sensor_tf.tf_z = p("sensor_tf.tf_z", sensor_tf.tf_z);

    // Timeouts
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);

    // Chainer (신규)
    chainer.search_radius      = p("chainer.search_radius",      chainer.search_radius);
    chainer.search_radius_step = p("chainer.search_radius_step", chainer.search_radius_step);
    chainer.search_radius_max  = p("chainer.search_radius_max",  chainer.search_radius_max);
    chainer.forward_angle      = p("chainer.forward_angle",      chainer.forward_angle);
    chainer.min_regress_pts    = p("chainer.min_regress_pts",    chainer.min_regress_pts);
    chainer.max_regress_pts    = p("chainer.max_regress_pts",    chainer.max_regress_pts);
    chainer.resample_ds        = p("chainer.resample_ds",        chainer.resample_ds);

    RCLCPP_INFO(
      node->get_logger(),
      "LC PlanningParams loaded: grid=%.0fx%.0f res=%.3f chain_r=%.1f chain_max=%.1f",
      costmap.size_x, costmap.size_y, costmap.resolution,
      chainer.search_radius, chainer.search_radius_max);
  }
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COMMON__PARAMS_HPP_
