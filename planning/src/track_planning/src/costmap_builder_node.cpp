#include "track_planning/costmap_builder_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace track_planning
{

CostmapBuilderNode::CostmapBuilderNode(const rclcpp::NodeOptions & options)
: Node("costmap_builder", options)
{
  // Topic parameters
  input_topic_ = declare_parameter<std::string>("input_topic", "/pointcloud/clustered_split");
  output_topic_ = declare_parameter<std::string>("output_topic", "/planning/costmap");
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

  // Grid geometry
  origin_x_ = declare_parameter<double>("origin_x", -1.0);
  origin_y_ = declare_parameter<double>("origin_y", -4.0);
  grid_width_ = declare_parameter<double>("grid_width", 11.0);
  grid_height_ = declare_parameter<double>("grid_height", 8.0);
  resolution_ = declare_parameter<double>("resolution", 0.10);

  // Obstacle
  obstacle_radius_ = declare_parameter<double>("obstacle_radius", 0.30);
  occupied_value_ = declare_parameter<int>("occupied_value", 100);
  min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);

  // Inflation
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.45);
  inflation_cost_max_ = declare_parameter<int>("inflation_cost_max", 99);
  inflation_cost_min_ = declare_parameter<int>("inflation_cost_min", 1);

  // Staleness
  stale_threshold_sec_ = declare_parameter<double>("stale_threshold_sec", 0.5);

  // Compute grid dimensions
  grid_cols_ = static_cast<int>(std::ceil(grid_width_ / resolution_));
  grid_rows_ = static_cast<int>(std::ceil(grid_height_ / resolution_));
  grid_data_.resize(static_cast<size_t>(grid_cols_ * grid_rows_), 0);

  // Subscriptions / Publishers
  auto sensor_qos = rclcpp::SensorDataQoS();
  sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, sensor_qos,
    std::bind(&CostmapBuilderNode::pointcloud_callback, this, std::placeholders::_1));

  auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
  pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, costmap_qos);

  RCLCPP_INFO(get_logger(),
    "CostmapBuilder started | in: %s | out: %s | frame: %s | grid: %dx%d | res: %.2fm",
    input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
    grid_cols_, grid_rows_, resolution_);
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void CostmapBuilderNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Staleness check
  const double age = (this->now() - msg->header.stamp).seconds();
  if (age > stale_threshold_sec_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Input stale: %.3f s > %.3f threshold", age, stale_threshold_sec_);
    return;
  }

  reset_grid();
  rasterize_obstacles(*msg);
  inflate_obstacles();
  publish_costmap(msg->header);
}

// ---------------------------------------------------------------------------
// Step 1: Reset
// ---------------------------------------------------------------------------

void CostmapBuilderNode::reset_grid()
{
  std::fill(grid_data_.begin(), grid_data_.end(), static_cast<int8_t>(0));
}

// ---------------------------------------------------------------------------
// Step 2: Rasterize cluster centroids as circular obstacles
// ---------------------------------------------------------------------------

void CostmapBuilderNode::rasterize_obstacles(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  const size_t n_points = static_cast<size_t>(cloud.width) * cloud.height;
  if (n_points == 0U) {
    return;
  }

  // Iterate PointCloud2 fields
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<int32_t> it_cid(cloud, "cluster_id");

  // Accumulate per-cluster centroid sums
  struct Accum
  {
    double sx = 0.0;
    double sy = 0.0;
    int count = 0;
  };
  std::unordered_map<int32_t, Accum> clusters;

  for (size_t i = 0; i < n_points; ++i, ++it_x, ++it_y, ++it_cid) {
    const int32_t cid = *it_cid;
    if (cid < 0) {
      continue;  // skip noise
    }
    auto & c = clusters[cid];
    c.sx += static_cast<double>(*it_x);
    c.sy += static_cast<double>(*it_y);
    c.count += 1;
  }

  // Rasterize each centroid as a filled circle
  const int obs_cells = static_cast<int>(std::ceil(obstacle_radius_ / resolution_));
  const double obs_r2 = obstacle_radius_ * obstacle_radius_;

  for (const auto & [cid, acc] : clusters) {
    if (acc.count < min_cluster_points_) {
      continue;
    }

    const double cx = acc.sx / acc.count;
    const double cy = acc.sy / acc.count;

    int center_col = 0;
    int center_row = 0;
    if (!world_to_grid(cx, cy, center_col, center_row)) {
      continue;  // centroid outside grid
    }

    for (int dr = -obs_cells; dr <= obs_cells; ++dr) {
      for (int dc = -obs_cells; dc <= obs_cells; ++dc) {
        const int c = center_col + dc;
        const int r = center_row + dr;
        if (c < 0 || c >= grid_cols_ || r < 0 || r >= grid_rows_) {
          continue;
        }
        const double dx = dc * resolution_;
        const double dy = dr * resolution_;
        if (dx * dx + dy * dy <= obs_r2) {
          grid_data_[static_cast<size_t>(grid_index(c, r))] =
            static_cast<int8_t>(occupied_value_);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Step 3: Inflate obstacles (linear cost decay)
// ---------------------------------------------------------------------------

void CostmapBuilderNode::inflate_obstacles()
{
  // Collect occupied cell positions
  std::vector<std::pair<int, int>> occupied_cells;
  for (int r = 0; r < grid_rows_; ++r) {
    for (int c = 0; c < grid_cols_; ++c) {
      if (grid_data_[static_cast<size_t>(grid_index(c, r))] ==
        static_cast<int8_t>(occupied_value_))
      {
        occupied_cells.emplace_back(c, r);
      }
    }
  }

  const int inf_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const double inv_inf = 1.0 / inflation_radius_;

  for (const auto & [oc, or_] : occupied_cells) {
    for (int dr = -inf_cells; dr <= inf_cells; ++dr) {
      for (int dc = -inf_cells; dc <= inf_cells; ++dc) {
        const int c = oc + dc;
        const int r = or_ + dr;
        if (c < 0 || c >= grid_cols_ || r < 0 || r >= grid_rows_) {
          continue;
        }

        const size_t idx = static_cast<size_t>(grid_index(c, r));

        // Skip already-occupied cells
        if (grid_data_[idx] == static_cast<int8_t>(occupied_value_)) {
          continue;
        }

        const double dist = std::hypot(dc * resolution_, dr * resolution_);
        if (dist > inflation_radius_) {
          continue;
        }

        // Linear decay: max cost at obstacle edge, min cost at inflation boundary
        const double ratio = dist * inv_inf;  // 0.0 (at obstacle) -> 1.0 (at boundary)
        const int cost = inflation_cost_max_ -
          static_cast<int>((inflation_cost_max_ - inflation_cost_min_) * ratio);

        // Keep highest cost from overlapping inflations
        if (static_cast<int8_t>(cost) > grid_data_[idx]) {
          grid_data_[idx] = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Step 4: Publish OccupancyGrid
// ---------------------------------------------------------------------------

void CostmapBuilderNode::publish_costmap(const std_msgs::msg::Header & header)
{
  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  msg->header.stamp = header.stamp;
  msg->header.frame_id = frame_id_;

  msg->info.resolution = static_cast<float>(resolution_);
  msg->info.width = static_cast<uint32_t>(grid_cols_);
  msg->info.height = static_cast<uint32_t>(grid_rows_);
  msg->info.origin.position.x = origin_x_;
  msg->info.origin.position.y = origin_y_;
  msg->info.origin.position.z = 0.0;
  msg->info.origin.orientation.w = 1.0;

  msg->data.assign(grid_data_.begin(), grid_data_.end());

  pub_costmap_->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool CostmapBuilderNode::world_to_grid(
  double wx, double wy, int & col, int & row) const
{
  col = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  row = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return (col >= 0 && col < grid_cols_ && row >= 0 && row < grid_rows_);
}

int CostmapBuilderNode::grid_index(int col, int row) const
{
  return row * grid_cols_ + col;
}

}  // namespace track_planning

RCLCPP_COMPONENTS_REGISTER_NODE(track_planning::CostmapBuilderNode)
