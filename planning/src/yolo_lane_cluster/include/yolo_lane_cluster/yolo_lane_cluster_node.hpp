/**
 * @file yolo_lane_cluster_node.hpp
 * @brief 차선 전처리 노드 — seed 기반 클러스터 선택 + backbone chaining + 가상 차선 생성
 *
 * camera(yolo_instance_seg)에서 클러스터링된 차선 boundary를 받아서:
 *   1) LEFT/RIGHT seed로 가장 가까운 클러스터를 좌/우 차선으로 선택
 *   2) 선택된 클러스터의 x_min 포인트를 chaining seed로 하여 backbone chaining
 *   3) 긴 쪽 채택 + track_width 안쪽 오프셋으로 반대편 가상 차선 생성
 *   4) 선택된 클러스터의 x_min (x,y)를 다음 프레임 seed 중심으로 저장
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

#include <unordered_set>
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

/// 포인트 풀의 개별 포인트 — 모든 클러스터에서 flat화
struct LanePoint
{
  double x = 0.0;
  double y = 0.0;
  int32_t label = -1;   ///< 원래 클러스터(boundary) 인덱스
};

/// Seed 추적 상태 — 프레임 간 유지
struct SeedState
{
  double center_x = 0.0;   ///< seed 중심 x [m]
  double center_y = 0.0;   ///< seed 중심 y [m]
};

// ============================================================================
// 파라미터 구조체
// ============================================================================

/// Backbone chaining 파라미터
struct ChainerParams
{
  double d_max             = 2.5;     ///< [m] 탐색 최대 거리
  double forward_cone_deg  = 120.0;   ///< [deg] 전방 cone 전체 각도 (±60°)
  double lateral_gate      = 1.3;     ///< [m] 횡방향 오차 한계

  double alpha       = 1.2;   ///< 거리 비용 (C_d)
  double beta        = 1.2;   ///< 방향 오차 비용 (C_a)
  double gamma       = 0.7;   ///< 횡오차 비용 (C_lat)
  double lambda_side = 0.5;   ///< side preference (C_side)

  int    max_backtrack_count = 5;
  double backtrack_w_curv    = 1.0;
  double backtrack_w_dist    = 1.0;

  int    max_chain_len = 300;
};

struct YoloLaneClusterParams
{
  // 시드 초기 위치
  double seed_left_y    = 0.8;
  double seed_right_y   = -0.8;

  // 시드 타임아웃 — 이 시간 동안 chaining 미성공 시 seed 초기 위치로 리셋
  double seed_timeout_sec = 3.0;

  // Holdover — chaining 실패 시 이전 결과 재발행
  int holdover_frames = 3;   ///< 최대 holdover 프레임 수

  // 가상 차선
  double track_width = 1.6;

  // Chainer
  ChainerParams chainer;
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

  // ── Backbone Chaining (lane_chainer.cpp) ──

  /// 포인트 풀에서 seed 좌표에 가장 가까운 포인트 인덱스 반환 (-1 = 없음)
  int find_seed(
    const std::vector<LanePoint> & points,
    double seed_x, double seed_y) const;

  /// seed에서 전방(+x) greedy chaining → backbone 인덱스 배열
  std::vector<int> extract_backbone(
    const std::vector<LanePoint> & points,
    int seed_idx,
    bool is_left) const;

  /// 단방향 greedy chaining 헬퍼
  std::vector<int> chain_one_direction(
    const std::vector<LanePoint> & points,
    const std::vector<bool> & owner,
    int seed_idx,
    double vx, double vy,
    bool is_left,
    std::unordered_set<int> & visited_set,
    int remaining_len) const;

  /// 기본 비용함수: w = α·C_d + β·C_a + γ·C_lat
  double compute_cost(
    const LanePoint & pi,
    const LanePoint & pj,
    double vx, double vy) const;

  /// 확장 비용함수: w' = w + λ·C_side
  double compute_cost_prime(
    const LanePoint & pi,
    const LanePoint & pj,
    double vx, double vy,
    bool is_left) const;

  /// 좌/우 backbone overlap 해소 (backtracking)
  void resolve_overlaps(
    const std::vector<LanePoint> & points,
    std::vector<int> & left_bb,
    std::vector<int> & right_bb) const;

  /// 3-node 윈도우 backtracking 비용
  double compute_backtrack_cost(
    const std::vector<LanePoint> & points,
    const std::vector<int> & backbone,
    int overlap_pos) const;

  /// truncate 후 재chaining
  void rechain_from(
    const std::vector<LanePoint> & points,
    std::vector<int> & backbone,
    int rechain_pos,
    bool is_left,
    const std::unordered_set<int> & excluded_set) const;

  /// backbone 인덱스 배열 → LaneBoundary 변환
  ev_msgs::msg::LaneBoundary backbone_to_boundary(
    const std::vector<LanePoint> & points,
    const std::vector<int> & backbone,
    const std_msgs::msg::Header & header) const;

  // ── 가상 차선 생성 (virtual_lane_gen.cpp) ──
  ev_msgs::msg::LaneBoundary generate_virtual_lane(
    const ev_msgs::msg::LaneBoundary & real_lane,
    LaneSide real_side) const;

  static void filter_virtual_lane_outliers(
    ev_msgs::msg::LaneBoundary & vl,
    double angle_threshold_deg = 30.0);

  static double path_length(const ev_msgs::msg::LaneBoundary & bd);

  // ── 디버그 (debug_publisher.cpp) ──
  void publish_debug_markers(
    const ev_msgs::msg::LaneBoundaryArray & output,
    bool left_chained, bool right_chained,
    bool left_virtual, bool right_virtual);

  // ── ROS 인터페이스 ──
  rclcpp::Subscription<ev_msgs::msg::LaneBoundaryArray>::SharedPtr sub_;
  rclcpp::Publisher<ev_msgs::msg::LaneBoundaryArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

  // ── 상태 (프레임 간 유지) ──
  SeedState left_seed_;
  SeedState right_seed_;
  rclcpp::Time left_seed_last_seen_;    ///< 왼쪽 seed 마지막 chaining 성공 시각
  rclcpp::Time right_seed_last_seen_;   ///< 오른쪽 seed 마지막 chaining 성공 시각

  // ── Holdover: 이전 프레임 결과 버퍼 ──
  ev_msgs::msg::LaneBoundaryArray last_output_;   ///< 마지막 유효 출력
  int holdover_remaining_ = 0;                     ///< 남은 holdover 프레임 수

  // ── 파라미터 ──
  YoloLaneClusterParams params_;
};

}  // namespace yolo_lane_cluster

#endif  // YOLO_LANE_CLUSTER__YOLO_LANE_CLUSTER_NODE_HPP_
