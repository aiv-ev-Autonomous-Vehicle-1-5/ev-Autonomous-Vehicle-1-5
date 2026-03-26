/**
 * @file debug_publisher.hpp
 * @brief [Stage 7] 디버그 토픽 발행 유틸리티
 *
 * on_timer()의 Stage 7에서 호출되는 디버그 시각화 로직을 분리한 헤더.
 * costmap, raw_path, chains, seeds 등의 디버그 마커 생성/발행.
 * 모든 함수는 lazy publishing (구독자가 있을 때만 발행).
 *
 * [구현 파일] src/nodes/debug_publisher.cpp
 */
#ifndef CHAINING_COSTMAP_VER__NODES__DEBUG_PUBLISHER_HPP_
#define CHAINING_COSTMAP_VER__NODES__DEBUG_PUBLISHER_HPP_

#include "chaining_costmap_ver/common/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace chaining_costmap_ver
{

/**
 * @brief costmap을 OccupancyGrid로 발행 (구독자 있을 때만)
 */
void publish_debug_costmap(
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub,
  const CostmapResult & costmap,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

/**
 * @brief obstacle_cost 이상인 셀을 빨간색 CUBE_LIST로 발행
 */
void publish_debug_obstacle_wall(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const CostmapResult & costmap,
  double obstacle_cost,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

/**
 * @brief 곡률 초과 지점을 노란색→빨간색 구체로 발행
 */
void publish_debug_curvature(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const PostprocessResult & pp_result,
  double r_min,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

/**
 * @brief seeds & goals MarkerArray 생성
 */
visualization_msgs::msg::MarkerArray make_seeds_markers(
  const DirectionChainResult & dc_result,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__NODES__DEBUG_PUBLISHER_HPP_
