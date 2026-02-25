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
  // ============================================================
  struct Costmap
  {
    double size_x = 10.0;          // [m] grid 전체 폭 (ego 중심: -5 ~ +5)
    double size_y = 10.0;          // [m] grid 전체 높이
    double resolution = 0.05;      // [m/cell] 셀 크기 (200x200 = 40,000 cells)
    double cone_cost_max = 100.0;  // 콘 중심 최대 cost
    double lane_cost_max = 50.0;   // 차선 경계점 최대 cost
    double cone_radius = 0.65;     // [m] 콘 클러스터 반지름 (이 내부는 cost=max)
    double alpha = 2.0;            // 감쇠율: cost = max / (1 + α·d_eff²)
    double cost_threshold = 2.0;   // cutoff: 미만이면 0 처리
  } costmap;

  // ============================================================
  // Planner — Greedy 전진 탐색 파라미터
  // ============================================================
  struct Planner
  {
    double search_radius = 0.30;   // [m] 전방 180° 탐색 반경
    int max_steps = 200;           // 최대 경로 포인트 수
    double heading_init_x = 1.0;   // 초기 heading X (+x = 전방)
    double heading_init_y = 0.0;   // 초기 heading Y
  } planner;

  // ============================================================
  // Vehicle — T870 차량 제원
  // ============================================================
  struct Vehicle
  {
    double width = 0.50;
    double wheelbase = 0.87;
    double delta_max = 0.314;
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ============================================================
  // Safety
  // ============================================================
  struct Safety
  {
    double margin = 0.10;
  } safety;

  // ============================================================
  // Speed
  // ============================================================
  struct Speed
  {
    double v_max = 1.60;
    double a_lat_max = 2.0;
  } speed;

  // ============================================================
  // Postprocess
  // ============================================================
  struct Postprocess
  {
    double resample_ds = 0.10;
    int smooth_window = 5;
    double prune_max_dev = 0.15;
  } postprocess;

  // ============================================================
  // Timeouts
  // ============================================================
  struct Timeouts
  {
    int perception_ms = 300;
  } timeouts;

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
    costmap.alpha          = p("costmap.alpha",           costmap.alpha);
    costmap.cost_threshold = p("costmap.cost_threshold", costmap.cost_threshold);

    // Planner
    planner.search_radius  = p("planner.search_radius",  planner.search_radius);
    planner.max_steps      = p("planner.max_steps",      planner.max_steps);
    planner.heading_init_x = p("planner.heading_init_x", planner.heading_init_x);
    planner.heading_init_y = p("planner.heading_init_y", planner.heading_init_y);

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

    // Timeouts
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);

    RCLCPP_INFO(
      node->get_logger(),
      "MR PlanningParams loaded: grid=%.0fx%.0f res=%.3f cone_max=%.0f cone_r=%.2f alpha=%.1f",
      costmap.size_x, costmap.size_y, costmap.resolution,
      costmap.cone_cost_max, costmap.cone_radius, costmap.alpha);
  }
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__PARAMS_HPP_
