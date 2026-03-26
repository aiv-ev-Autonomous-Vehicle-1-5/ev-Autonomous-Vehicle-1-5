/**
 * @file yolo_lane_cluster_node.hpp
 * @brief 차선 전처리 노드 — 시드 기반 좌/우 판별 + 가상 차선 생성 + LEFT/RIGHT 라벨링
 *
 * camera(yolo_instance_seg)에서 클러스터링된 차선 boundary를 받아서:
 *   1) 왼쪽/오른쪽 시드로 각 클러스터를 좌/우 판별
 *   2) 양쪽 다 보이면 총 경로 길이가 긴 쪽을 채택, track_width 안쪽 오프셋으로 반대편 가상 차선 생성
 *      한쪽만 보이면 기존 로직대로 가상 반대편 차선 생성
 *   3) 모든 boundary에 lane_side 라벨 (LEFT/RIGHT) 설정
 *   4) 가공된 LaneBoundaryArray를 planning에 전달
 *
 * [데이터 흐름]
 *   /perception/raw_lane_boundaries (카메라) → yolo_lane_cluster → /perception/lane_boundaries (planning)
 */
#ifndef YOLO_LANE_CLUSTER__YOLO_LANE_CLUSTER_NODE_HPP_
#define YOLO_LANE_CLUSTER__YOLO_LANE_CLUSTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ev_msgs/msg/lane_boundary.hpp>
#include <ev_msgs/msg/lane_boundary_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace yolo_lane_cluster
{

// ============================================================================
// 타입 정의
// ============================================================================

/// 차선이 어느 쪽인지
enum class LaneSide : uint8_t
{
  LEFT  = 0,   ///< 왼쪽 차선 (ego 기준 y > 0)
  RIGHT = 1    ///< 오른쪽 차선 (ego 기준 y < 0)
};

/// 시드 상태 — 프레임 간 유지
struct SeedState
{
  double center_x = 0.0;   ///< 탐색 중심 x [m] — 이전 프레임 매칭 클러스터의 x_min
  double center_y = 0.0;   ///< 탐색 중심 y [m] — 초기 오프셋 고정 (좌: +0.75, 우: -0.75)
  bool has_match = false;   ///< 이전 프레임에서 매칭 성공 여부
};

// ============================================================================
// 파라미터 구조체
// ============================================================================

struct YoloLaneClusterParams
{
  // 시드 초기 위치
  double seed_init_x    = 0.0;    ///< 시드 초기 x 위치 [m]
  double seed_left_y    = 0.75;   ///< 왼쪽 시드 초기 y 오프셋 [m]
  double seed_right_y   = -0.75;  ///< 오른쪽 시드 초기 y 오프셋 [m]

  // 탐색 직사각형 (시드 중심 기준)
  double search_rect_width  = 2.0;  ///< y 방향 폭 [m]
  double search_rect_height = 8.0;  ///< x 방향 높이 [m]

  // 가상 차선
  double track_width = 1.5;  ///< 트랙 폭 [m] — 가상 차선 오프셋 거리
};

// ============================================================================
// 노드 클래스
// ============================================================================

class YoloLaneClusterNode : public rclcpp::Node
{
public:
  explicit YoloLaneClusterNode(const rclcpp::NodeOptions & options);

private:
  // ── 콜백 ──
  void on_lane_boundaries(const ev_msgs::msg::LaneBoundaryArray::SharedPtr msg);

  // ── 시드 매칭 (seed_tracker.cpp) ──
  /// 시드의 탐색 직사각형 내에서 가장 가까운 클러스터 인덱스 반환 (-1 = 매칭 없음)
  int match_cluster_to_seed(
    const SeedState & seed,
    const ev_msgs::msg::LaneBoundaryArray & msg,
    int exclude_idx) const;

  /// 매칭된 클러스터의 x_min으로 시드 중심 업데이트
  void update_seed(SeedState & seed, const ev_msgs::msg::LaneBoundary & boundary);

  /// 시드를 초기 위치로 리셋
  void reset_seed(SeedState & seed, LaneSide side);

  // ── 가상 차선 생성 (virtual_lane_gen.cpp) ──
  /// 실제 차선에서 track_width만큼 안쪽으로 오프셋하여 가상 차선 생성
  ev_msgs::msg::LaneBoundary generate_virtual_lane(
    const ev_msgs::msg::LaneBoundary & real_lane,
    LaneSide real_side) const;

  /// 경계점 배열의 총 경로 길이 (유클리디안 거리 합) [m]
  static double path_length(const ev_msgs::msg::LaneBoundary & bd);

  // ── 디버그 (debug_publisher.cpp) ──
  /// lazy Marker 토픽으로 lane points + seed 위치 시각화
  void publish_debug_markers(
    const ev_msgs::msg::LaneBoundaryArray & output,
    bool left_matched, bool right_matched,
    bool left_virtual, bool right_virtual);

  // ── ROS 인터페이스 ──
  rclcpp::Subscription<ev_msgs::msg::LaneBoundaryArray>::SharedPtr sub_;
  rclcpp::Publisher<ev_msgs::msg::LaneBoundaryArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

  // ── 상태 ──
  SeedState left_seed_;
  SeedState right_seed_;
  YoloLaneClusterParams params_;
};

}  // namespace yolo_lane_cluster

#endif  // YOLO_LANE_CLUSTER__YOLO_LANE_CLUSTER_NODE_HPP_
