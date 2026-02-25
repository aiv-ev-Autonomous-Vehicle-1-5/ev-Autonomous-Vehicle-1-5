/**
 * @file mr_planner_node.hpp
 * @brief MR Planner 메인 노드 — 파이프라인 오케스트레이션
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: ROS 2 ComposableNode로서 전체 MR 플래너 파이프라인을 조율한다.
 *
 * 파이프라인 (10Hz on_timer 콜백):
 *   Stage 0: Stale 검사 (perception 데이터 타임아웃)
 *   Stage 1: 입력 파싱 (콘, 차선 → Point2D 벡터)
 *   Stage 2: Costmap 생성 (CostmapGenerator)
 *   Stage 3: Greedy 전진 탐색 (MagneticPlanner → raw_path)
 *   Stage 4: 후처리 (PathPostprocessor → prune, smooth, resample, yaw)
 *   Stage 5: 안전 검사 (safety_checker → 곡률, 속도 제한)
 *   Stage 6: 퍼블리시 (path, status, debug costmap, debug raw_path)
 *
 * 토픽 인터페이스 (track_planning과 동일 → drop-in replacement):
 *   구독: /perception/lane_boundaries, /perception/cones
 *   발행: /planning/path, /planning/status
 *   디버그: /planning/debug/costmap, /planning/debug/raw_path
 * ──────────────────────────────────────────────────────────────
 */
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
  /**
   * @brief 생성자 — 파라미터 로드, 구독/발행/타이머 초기화
   * @param options ROS 2 노드 옵션 (ComposableNode에서 전달)
   */
  explicit MRPlannerNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 10Hz 타이머 콜백 — 전체 파이프라인 실행
   *
   * Stage 0~6을 순차적으로 실행하여 경로를 생성하고 퍼블리시한다.
   */
  void on_timer();

  /**
   * @brief 차선 메시지를 파싱하여 모든 차선 점을 하나의 벡터로 수집
   *
   * CDT 버전과 달리 L/R 분리가 불필요하다.
   * costmap에서는 모든 차선 점이 동일한 S극 자석이므로 단일 벡터로 수집.
   */
  void parse_lanes(std::vector<Point2D> & all_lane_pts) const;

  /**
   * @brief 콘 메시지를 파싱하여 모든 콘 위치를 수집
   */
  void parse_cones(std::vector<Point2D> & all_cones) const;

  /**
   * @brief perception 데이터 타임아웃 검사
   *
   * 차선과 콘 둘 다 타임아웃이면 STALE로 판정한다.
   * 둘 중 하나라도 유효하면 정상.
   *
   * @return true: STALE (인식 데이터 없음), false: 정상
   */
  bool check_stale() const;

  // ── Parameters ──
  PlanningParams params_;  ///< 모든 파라미터 (planning_mr.yaml에서 로드)

  // ── Pipeline modules ──
  CostmapGenerator costmap_generator_;   ///< Stage 2: costmap 생성
  MagneticPlanner  magnetic_planner_;    ///< Stage 3: Greedy 전진 탐색
  PathPostprocessor postprocessor_;      ///< Stage 4: 경로 후처리

  // ── Latest input data ──
  // UniquePtr: Intra-process Zero-copy를 위해 소유권 이전 방식 사용
  track_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;  ///< 최근 차선 데이터
  track_msgs::msg::ConeArray::UniquePtr         last_cones_;  ///< 최근 콘 데이터

  // ── Input timestamps ──
  // 데이터 수신 시각을 기록하여 stale 판정에 사용
  rclcpp::Time stamp_lanes_;  ///< 차선 데이터 수신 시각
  rclcpp::Time stamp_cones_;  ///< 콘 데이터 수신 시각

  // ── Subscriptions ──
  rclcpp::Subscription<track_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;
  rclcpp::Subscription<track_msgs::msg::ConeArray>::SharedPtr sub_cones_;

  // ── Core publishers (track_planning과 동일 토픽 → drop-in replacement) ──
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;                ///< /planning/path
  rclcpp::Publisher<track_msgs::msg::PlannerStatus>::SharedPtr pub_status_;   ///< /planning/status

  // ── Debug publishers ──
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dbg_costmap_;  ///< RViz2 costmap
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_raw_path_;          ///< 후처리 전 경로

  // ── Timer ──
  rclcpp::TimerBase::SharedPtr timer_;  ///< 10Hz (100ms) 주기 타이머
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__NODES__MR_PLANNER_NODE_HPP_
