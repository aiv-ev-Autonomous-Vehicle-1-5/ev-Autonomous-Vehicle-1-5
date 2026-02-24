#ifndef TRACK_PLANNING__COMMON__PARAMS_HPP_
#define TRACK_PLANNING__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace track_planning
{

struct PlanningParams
{
  // ---- ROI & Grid ----
  struct ROI
  {
    double x_min = -1.0;
    double x_max = 10.0;
    double y_min = -4.0;
    double y_max = 4.0;
    double resolution = 0.10;
  } roi;

  // ---- Vehicle ----
  struct Vehicle
  {
    double width = 0.50;         // m
    double wheelbase = 0.87;     // m (T870)
    double delta_max = 0.314;    // rad (~18 deg, T870 max steering)
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ---- Safety ----
  struct Safety
  {
    double margin = 0.10;             // base safety margin (m)
    double margin_boundary = 0.0;     // extra margin for boundary inflation
    double margin_obstacle = 0.0;     // extra margin for obstacle inflation
  } safety;

  // ---- Corridor: Seed ----
  struct CorridorSeed
  {
    double x_seed_min = 0.5;     // min x to start seed search
  } corridor_seed;

  // ---- Corridor: Filter ----
  struct CorridorFilter
  {
    double s_min = 0.05;         // min forward progress
    double s_max = 2.0;          // max forward search range
    double d_max = 2.0;          // max lateral distance
    double r_search = 2.5;       // circular search radius (optional)
  } corridor_filter;

  // ---- Corridor: Scoring ----
  struct CorridorScore
  {
    double w_s = 1.0;            // weight: forward progress
    double w_d = 0.8;            // weight: lateral deviation
    double w_a = 0.5;            // weight: angle deviation
    double w_p = 0.3;            // weight: prediction error
    double theta_max = 1.047;    // ~60 deg, max angle for normalization
    double step_pred = 0.5;      // prediction step distance
  } corridor_score;

  // ---- Corridor: Top-K ----
  struct CorridorTopK
  {
    bool enable = false;
    int k_top = 5;
  } corridor_topk;

  // ---- Corridor: Reference Tangent ----
  struct CorridorRefTangent
  {
    int n_reg = 7;               // regression window size
  } corridor_ref_tangent;

  // ---- Corridor: Cold Start ----
  struct CorridorColdStart
  {
    int min_centerline_points = 3;
    double theta_heading_gate_deg = 60.0;
  } corridor_cold_start;

  // ---- Corridor: General ----
  struct CorridorGeneral
  {
    int max_points_side = 100;
  } corridor_general;

  // ---- Pair Validator ----
  struct Pair
  {
    double resample_ds = 0.2;    // resample interval for validation
    double theta_mean_th = 0.35; // ~20 deg
    double theta_max_th = 0.70;  // ~40 deg
    double w_min = 0.8;          // min corridor width (m)
    double w_max = 3.0;          // max corridor width (m)
    double w_std_th = 0.5;       // max width std (m)
  } pair;

  // ---- Virtual Boundary ----
  struct Virtual
  {
    double default_track_width = 1.5;  // m (competition spec)
    double ema_alpha = 0.3;
    double min_corridor_width = 0.6;   // m
  } virt;

  // ---- Inflation ----
  struct Inflation
  {
    // Computed: vehicle.width/2 + safety.margin + extra
    double boundary_extra_margin = 0.0;
    double obstacle_extra_margin = 0.0;

    // Convenience: computed at load time
    double boundary_radius = 0.0;
    double obstacle_radius = 0.0;
  } inflation;

  // ---- Costmap Validation (DIRECT mode) ----
  struct CostmapValid
  {
    double ds_check = 0.1;       // centerline sampling interval (m)
    int cost_th = 80;            // cost threshold for collision
  } costmap_valid;

  // ---- Goal Selection ----
  struct Goal
  {
    double lookahead_l0 = 3.0;   // base lookahead (m)
    double lookahead_kv = 0.5;   // speed-dependent gain
    int ring_samples = 36;       // Method2 ring sample count
  } goal;

  // ---- Speed ----
  struct Speed
  {
    double v_max = 1.60;         // m/s (T870 max)
    double a_lat_max = 2.0;     // m/s^2 lateral acceleration limit
  } speed;

  // ---- Postprocess ----
  struct Postprocess
  {
    double resample_ds = 0.10;   // m, output path spacing
    int smooth_window = 5;       // moving average window size
    double prune_max_dev = 0.15; // m, max lateral deviation for pruning
  } postprocess;

  // ---- Mode Selector ----
  struct ModeSelector
  {
    double l_check = 5.0;        // forward check distance (m)
    double centerline_jump_th = 1.0;  // centerline jump threshold (m)
    bool enable_astar = true;
  } mode_selector;

  // ---- Timeouts ----
  struct Timeouts
  {
    int odom_ms = 100;
    int perception_ms = 300;
    int plan_ms = 100;
  } timeouts;

  /// Declare and load all parameters from a ROS 2 node
  void load(rclcpp::Node * node)
  {
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

    // Compute inflation radii (vehicle point model)
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
