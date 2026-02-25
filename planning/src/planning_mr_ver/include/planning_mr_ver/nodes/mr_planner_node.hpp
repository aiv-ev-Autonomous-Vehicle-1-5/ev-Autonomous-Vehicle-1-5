#ifndef PLANNING_MR_VER__NODES__MR_PLANNER_NODE_HPP_
#define PLANNING_MR_VER__NODES__MR_PLANNER_NODE_HPP_

#include "planning_mr_ver/common/types.hpp"
#include "planning_mr_ver/common/params.hpp"
#include "planning_mr_ver/costmap/costmap_generator.hpp"
#include "planning_mr_ver/planner/magnetic_planner.hpp"
#include "planning_mr_ver/postprocess/path_postprocessor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <track_msgs/msg/lane_boundary_array.hpp>
#include <track_msgs/msg/cone_array.hpp>
#include <track_msgs/msg/planner_status.hpp>

#include <vector>

namespace planning_mr_ver
{

class MRPlannerNode : public rclcpp::Node
{
public:
  explicit MRPlannerNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  // 입력 파싱: costmap에서는 L/R 분리 불필요 → 단일 벡터로 수집
  void parse_lanes(std::vector<Point2D> & all_lane_pts) const;
  void parse_cones(std::vector<Point2D> & all_cones) const;
  bool check_stale() const;

  // Parameters
  PlanningParams params_;

  // Pipeline modules
  CostmapGenerator costmap_generator_;
  MagneticPlanner  magnetic_planner_;
  PathPostprocessor postprocessor_;

  // Latest input data (UniquePtr for zero-copy)
  track_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;
  track_msgs::msg::ConeArray::UniquePtr         last_cones_;

  // Input timestamps
  rclcpp::Time stamp_lanes_;
  rclcpp::Time stamp_cones_;

  // Subscriptions
  rclcpp::Subscription<track_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;
  rclcpp::Subscription<track_msgs::msg::ConeArray>::SharedPtr sub_cones_;

  // Core publishers (same topics as track_planning → drop-in)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<track_msgs::msg::PlannerStatus>::SharedPtr pub_status_;

  // Debug publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dbg_costmap_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_raw_path_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__NODES__MR_PLANNER_NODE_HPP_
