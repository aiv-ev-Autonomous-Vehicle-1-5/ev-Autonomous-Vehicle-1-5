#ifndef TRACK_PLANNING__COSTMAP_BUILDER_NODE_HPP_
#define TRACK_PLANNING__COSTMAP_BUILDER_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace track_planning
{

class CostmapBuilderNode : public rclcpp::Node
{
public:
  explicit CostmapBuilderNode(const rclcpp::NodeOptions & options);

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void reset_grid();
  void rasterize_obstacles(const sensor_msgs::msg::PointCloud2 & cloud);
  void inflate_obstacles();
  void publish_costmap(const std_msgs::msg::Header & header);

  bool world_to_grid(double wx, double wy, int & col, int & row) const;
  int grid_index(int col, int row) const;

  // Subscribers / Publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;

  // Internal costmap storage (0=FREE, 100=OCCUPIED, 1-99=inflation)
  std::vector<int8_t> grid_data_;

  // Topic parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  // Grid geometry parameters
  double origin_x_;
  double origin_y_;
  double grid_width_;
  double grid_height_;
  double resolution_;
  int grid_cols_;
  int grid_rows_;

  // Obstacle parameters
  double obstacle_radius_;
  int occupied_value_;
  int min_cluster_points_;

  // Inflation parameters
  double inflation_radius_;
  int inflation_cost_max_;
  int inflation_cost_min_;

  // Staleness
  double stale_threshold_sec_;
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP_BUILDER_NODE_HPP_
