#ifndef CLUSTER_SPLITTER__CLUSTER_SPLITTER_NODE_HPP_
#define CLUSTER_SPLITTER__CLUSTER_SPLITTER_NODE_HPP_

#include <cstdint>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cluster_splitter
{

class ClusterSplitterNode : public rclcpp::Node
{
public:
  struct PointRecord
  {
    float x;
    float y;
    float z;
  };

  struct AxisStats
  {
    double axis_length;
    double ux;
    double uy;
    std::vector<std::pair<double, size_t>> projection;
  };

  explicit ClusterSplitterNode(const rclcpp::NodeOptions & options);

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void split_cluster_recursive(
    const std::vector<PointRecord> & points,
    const std::vector<size_t> & indices,
    int depth,
    std::vector<std::vector<size_t>> & split_groups) const;

  bool split_by_projection_gap(
    const std::vector<size_t> & indices,
    const AxisStats & stats,
    std::vector<size_t> & left,
    std::vector<size_t> & right) const;

  bool split_by_kmeans(
    const std::vector<PointRecord> & points,
    const std::vector<size_t> & indices,
    std::vector<size_t> & left,
    std::vector<size_t> & right) const;

  AxisStats compute_axis_stats(
    const std::vector<PointRecord> & points,
    const std::vector<size_t> & indices) const;

  bool is_oversized_cluster(
    const std::vector<size_t> & indices,
    const AxisStats & stats) const;

  void validate_cone_clusters(
    const std::vector<PointRecord> & points,
    std::vector<int32_t> & relabeled,
    int32_t & next_cluster_id) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;

  int min_points_to_consider_split_;
  int oversized_point_count_;
  int min_subcluster_points_;
  int max_recursive_depth_;
  int kmeans_max_iterations_;

  double oversized_axis_length_m_;
  double valley_min_gap_m_;
  double kmeans_min_centroid_distance_m_;

  bool preserve_noise_points_;
  bool enable_kmeans_fallback_;

  bool enable_cone_validation_;
  double cone_diameter_m_;
  double cone_max_width_m_;
  double cone_min_width_m_;
  double cone_max_height_m_;
  double cone_min_height_m_;
  int cone_min_points_;
};

}  // namespace cluster_splitter

#endif  // CLUSTER_SPLITTER__CLUSTER_SPLITTER_NODE_HPP_
