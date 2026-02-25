#include "planning_mr_ver/nodes/mr_planner_node.hpp"
#include "planning_mr_ver/common/geometry.hpp"
#include "planning_mr_ver/common/debug_publish.hpp"
#include "planning_mr_ver/safety/safety_checker.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace planning_mr_ver
{

MRPlannerNode::MRPlannerNode(const rclcpp::NodeOptions & options)
: Node("mr_planner_node", options),
  stamp_lanes_(0, 0, RCL_ROS_TIME),
  stamp_cones_(0, 0, RCL_ROS_TIME)
{
  params_.load(this);

  // Subscriptions (동일 토픽 → drop-in replacement)
  sub_lanes_ = create_subscription<track_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", rclcpp::QoS(1),
    [this](track_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();
      last_lanes_ = std::move(msg);
    });

  sub_cones_ = create_subscription<track_msgs::msg::ConeArray>(
    "/perception/cones", rclcpp::QoS(1),
    [this](track_msgs::msg::ConeArray::UniquePtr msg) {
      stamp_cones_ = now();
      last_cones_ = std::move(msg);
    });

  // Core publishers
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/path", rclcpp::QoS(1));
  pub_status_ = create_publisher<track_msgs::msg::PlannerStatus>(
    "/planning/status", rclcpp::QoS(1));

  // Debug publishers
  pub_dbg_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/debug/costmap", rclcpp::QoS(1));
  pub_dbg_raw_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/raw_path", rclcpp::QoS(1));

  // 10Hz timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MRPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "MRPlannerNode initialized (10 Hz, MR costmap)");
}

void MRPlannerNode::parse_lanes(
  std::vector<Point2D> & all_lane_pts) const
{
  if (!last_lanes_) return;
  for (const auto & b : last_lanes_->boundaries) {
    for (const auto & p : b.points) {
      all_lane_pts.push_back({p.x, p.y});
    }
  }
}

void MRPlannerNode::parse_cones(
  std::vector<Point2D> & all_cones) const
{
  if (!last_cones_) return;
  for (const auto & c : last_cones_->cones) {
    all_cones.push_back({c.position.x, c.position.y});
  }
}

bool MRPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_cones_) {
    const double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  return !have_perception;
}

void MRPlannerNode::on_timer()
{
  const auto stamp = now();
  const std::string frame_id = "base_link";

  // ======== Stage 0: Stale Gate ========
  if (check_stale()) {
    auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
    status_msg->header.stamp = stamp;
    status_msg->header.frame_id = frame_id;
    status_msg->status = track_msgs::msg::PlannerStatus::STALE;
    status_msg->reason = "input_stale";
    pub_status_->publish(std::move(status_msg));
    return;
  }

  // ======== Stage 1: Input Parse ========
  std::vector<Point2D> all_cones, all_lane_pts;
  parse_cones(all_cones);
  parse_lanes(all_lane_pts);

  // ======== Stage 2: Costmap Generation ========
  auto costmap = costmap_generator_.generate(all_cones, all_lane_pts, params_);

  // ======== Stage 3: Magnetic Planner ========
  auto raw_path = magnetic_planner_.plan(costmap, params_);

  // ======== Stage 4: Postprocess ========
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds);

  // ======== Stage 5: Safety Check ========
  auto safety = safety_checker::check(pp_result, params_);

  // ======== Stage 6: Publish ========

  // Core: path
  auto path_msg = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(pp_result.path, frame_id, stamp));
  pub_path_->publish(std::move(path_msg));

  // Core: status
  auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
  status_msg->header.stamp = stamp;
  status_msg->header.frame_id = frame_id;
  status_msg->status = static_cast<uint8_t>(safety.state);
  status_msg->reason = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // Debug: costmap as OccupancyGrid (Lazy Publishing)
  if (pub_dbg_costmap_->get_subscription_count() > 0 && costmap.valid) {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.stamp = stamp;
    grid_msg->header.frame_id = frame_id;
    grid_msg->info.resolution = static_cast<float>(costmap.resolution);
    grid_msg->info.width = costmap.cols;
    grid_msg->info.height = costmap.rows;
    grid_msg->info.origin.position.x = costmap.origin_x;
    grid_msg->info.origin.position.y = costmap.origin_y;
    grid_msg->info.origin.orientation.w = 1.0;
    grid_msg->data.resize(costmap.rows * costmap.cols);
    for (size_t i = 0; i < costmap.data.size(); ++i) {
      grid_msg->data[i] = static_cast<int8_t>(
        std::clamp(costmap.data[i], 0.0, 100.0));
    }
    pub_dbg_costmap_->publish(std::move(grid_msg));
  }

  // Debug: raw path (before postprocess)
  if (pub_dbg_raw_path_->get_subscription_count() > 0) {
    pub_dbg_raw_path_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(raw_path, frame_id, stamp)));
  }
}

}  // namespace planning_mr_ver

RCLCPP_COMPONENTS_REGISTER_NODE(planning_mr_ver::MRPlannerNode)
