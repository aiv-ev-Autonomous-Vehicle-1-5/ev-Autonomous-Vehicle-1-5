#ifndef TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_
#define TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

// Pipeline modules
#include "track_planning/corridor/corridor_builder.hpp"
#include "track_planning/corridor/pair_validator.hpp"
#include "track_planning/corridor/virtual_boundary.hpp"
#include "track_planning/corridor/centerline_builder.hpp"
#include "track_planning/costmap/costmap_validation_builder.hpp"
#include "track_planning/costmap/drivable_mask_scanline.hpp"
#include "track_planning/costmap/connected_component.hpp"
#include "track_planning/goal/goal_selector.hpp"
#include "track_planning/planner/astar_planner.hpp"
#include "track_planning/postprocess/path_postprocessor.hpp"

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <track_msgs/msg/lane_boundary_array.hpp>
#include <track_msgs/msg/cone_array.hpp>
#include <track_msgs/msg/obstacle_array.hpp>
#include <track_msgs/msg/planner_status.hpp>
#include <track_msgs/msg/system_state.hpp>

#include <vector>

namespace track_planning
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options);

private:
  // ---- Timer callback: runs full pipeline at 10Hz ----
  void on_timer();

  // ---- Input parsing helpers ----
  void parse_lanes(
    std::vector<Point2D> & left, std::vector<Point2D> & right) const;
  void parse_cones(
    std::vector<Point2D> & cone_left,
    std::vector<Point2D> & cone_right,
    std::vector<Point2D> & cone_all) const;
  void parse_obstacles(std::vector<Point2D> & out) const;

  // ---- Stale check ----
  bool check_stale() const;

  // ---- Parameters ----
  PlanningParams params_;

  // ---- Pipeline modules ----
  CorridorBuilder corridor_builder_;
  PairValidator pair_validator_;
  VirtualBoundary virtual_boundary_;
  CenterlineBuilder centerline_builder_;
  CostmapValidationBuilder costmap_builder_;
  DrivableMaskScanline drivable_builder_;
  GoalSelector goal_selector_;
  AstarPlanner astar_planner_;
  PathPostprocessor postprocessor_;

  // ---- Persistent state (frame-to-frame) ----
  double w_hat_{0.0};
  bool w_hat_initialized_{false};
  std::vector<Point2D> path_prev_;
  std::vector<Point2D> centerline_prev_;

  // ---- Latest input data ----
  track_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;
  track_msgs::msg::ConeArray::UniquePtr last_cones_;
  track_msgs::msg::ObstacleArray::UniquePtr last_obstacles_;
  nav_msgs::msg::Odometry::UniquePtr last_odom_;
  track_msgs::msg::SystemState::UniquePtr last_state_;

  // ---- Input timestamps ----
  rclcpp::Time stamp_lanes_;
  rclcpp::Time stamp_cones_;
  rclcpp::Time stamp_obstacles_;
  rclcpp::Time stamp_odom_;

  // ---- Subscriptions ----
  rclcpp::Subscription<track_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;
  rclcpp::Subscription<track_msgs::msg::ConeArray>::SharedPtr sub_cones_;
  rclcpp::Subscription<track_msgs::msg::ObstacleArray>::SharedPtr sub_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<track_msgs::msg::SystemState>::SharedPtr sub_state_;

  // ---- Publishers: core ----
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<track_msgs::msg::PlannerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_speed_;

  // ---- Publishers: debug ----
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_left_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_corridor_right_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_centerline_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dbg_pair_valid_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dbg_virtual_used_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dbg_path_mode_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__NODES__LOCAL_PLANNER_NODE_HPP_
