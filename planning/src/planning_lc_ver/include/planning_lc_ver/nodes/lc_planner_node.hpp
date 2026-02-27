/**
 * @file lc_planner_node.hpp
 * @brief LC Planner 메인 노드 — LineChainer 파이프라인
 *
 * ──────────────────────────────────────────────────────────────
 * 파이프라인 (10Hz on_timer 콜백):
 *   Stage 0: Stale 검사
 *   Stage 1: 입력 파싱 (콘/차선 → ChainedPoint 벡터)
 *   Stage 2: LineChainer (DFS 체이닝 + 리샘플링)
 *   Stage 3: Costmap 생성 (체인 기반)
 *   Stage 4: Magnetic Planner (Greedy 전진 탐색)
 *   Stage 5: 후처리 (prune, smooth, resample, yaw)
 *   Stage 6: 안전 검사 (곡률, 속도 제한)
 *   Stage 7: 퍼블리시
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_LC_VER__NODES__LC_PLANNER_NODE_HPP_
#define PLANNING_LC_VER__NODES__LC_PLANNER_NODE_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"
#include "planning_lc_ver/chainer/line_chainer.hpp"
#include "planning_lc_ver/costmap/costmap_generator.hpp"
#include "planning_lc_ver/planner/magnetic_planner.hpp"
#include "planning_lc_ver/postprocess/path_postprocessor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <ev_msgs/msg/lane_boundary_array.hpp>
#include <ev_msgs/msg/b_box_array.hpp>

#include <vector>

namespace planning_lc_ver
{

class LCPlannerNode : public rclcpp::Node
{
public:
  explicit LCPlannerNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  /**
   * @brief 콘/차선 메시지를 단일 ChainedPoint 벡터로 수집
   *
   * 모든 점에 PointType 태그만 붙이고, L/R 분리는 하지 않는다.
   * 좌/우 구분은 LineChainer가 seed 선택(y>0/y<0)으로 수행.
   */
  void parse_input(std::vector<ChainedPoint> & all_pts) const;

  bool check_stale() const;

  // ── Parameters ──
  PlanningParams params_;

  // ── Pipeline modules ──
  LineChainer        line_chainer_;
  CostmapGenerator   costmap_generator_;
  MagneticPlanner    magnetic_planner_;
  PathPostprocessor  postprocessor_;

  // ── Latest input data ──
  ev_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;
  ev_msgs::msg::BBoxArray::UniquePtr          last_bboxes_;

  // ── Input timestamps ──
  rclcpp::Time stamp_lanes_;
  rclcpp::Time stamp_bboxes_;

  // ── Subscriptions ──
  rclcpp::Subscription<ev_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;
  rclcpp::Subscription<ev_msgs::msg::BBoxArray>::SharedPtr sub_bboxes_;

  // ── Core publishers ──
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  // ── Debug publishers ──
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dbg_costmap_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_raw_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_left_chain_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_right_chain_;

  // ── Timer ──
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__NODES__LC_PLANNER_NODE_HPP_
