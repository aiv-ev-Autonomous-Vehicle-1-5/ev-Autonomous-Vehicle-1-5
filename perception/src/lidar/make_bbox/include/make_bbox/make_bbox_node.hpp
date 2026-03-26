#ifndef MAKE_BBOX__MAKE_BBOX_NODE_HPP_
#define MAKE_BBOX__MAKE_BBOX_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ev_msgs/msg/b_box_array.hpp>

namespace make_bbox
{

class MakeBBoxNode : public rclcpp::Node
{
public:
  explicit MakeBBoxNode(const rclcpp::NodeOptions & options);

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<ev_msgs::msg::BBoxArray>::SharedPtr pub_bboxes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  float max_size_x_;
  float max_size_y_;
  float max_size_z_;
  float min_size_z_;  // 납작한 바닥 노이즈 클러스터 필터링용
  float max_center_z_;  // 클러스터 중심 z가 이 값 초과 시 필터링
  float min_center_z_;  // 클러스터 중심 z가 이 값 미만 시 필터링
  float max_base_height_ratio_;  // 밑변/높이 비율 상한 (라바콘 형상 필터)

  // 클러스터 분할 파라미터
  bool enable_split_;
  float split_cone_diameter_;
  int split_kmeans_max_iter_;
  int split_min_points_;
};

}  // namespace make_bbox

#endif  // MAKE_BBOX__MAKE_BBOX_NODE_HPP_
