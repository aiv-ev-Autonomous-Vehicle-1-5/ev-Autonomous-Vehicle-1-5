/**
 * @file input_parser.hpp
 * @brief [Stage 1] bbox/차선 ROS 메시지 → ChainPoint 벡터 변환
 *
 * BBox(LiDAR 장애물)와 LaneBoundary(카메라 차선)를
 * 단일 ChainPoint 벡터로 변환하는 유틸리티.
 * LiDAR bbox에는 sensor_tf 오프셋 보정이 적용됨.
 *
 * [구현 파일] src/nodes/input_parser.cpp
 */
#ifndef CHAINING_COSTMAP_VER__NODES__INPUT_PARSER_HPP_
#define CHAINING_COSTMAP_VER__NODES__INPUT_PARSER_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"

#include <ev_msgs/msg/lane_boundary_array.hpp>
#include <ev_msgs/msg/b_box_array.hpp>

#include <vector>

namespace chaining_costmap_ver
{

/**
 * @brief bbox/차선 ROS 메시지를 단일 ChainPoint 벡터로 변환
 *
 * [변환 규칙]
 *   BBox → ChainPoint: sensor_tf 오프셋 적용 (velodyne→base_link), type=CONE
 *   LaneBoundary → ChainPoint: 그대로 사용 (base_link 기준), type=LANE
 *
 * @param bboxes    LiDAR bbox 메시지 (nullptr 가능)
 * @param lanes     카메라 차선 메시지 (nullptr 가능)
 * @param params    센서 TF 오프셋 정보
 * @param[out] all_pts  변환된 ChainPoint들이 추가될 벡터
 */
void parse_input(
  const ev_msgs::msg::BBoxArray * bboxes,
  const ev_msgs::msg::LaneBoundaryArray * lanes,
  const PlanningParams & params,
  std::vector<ChainPoint> & all_pts);

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__NODES__INPUT_PARSER_HPP_
