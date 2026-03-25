// ============================================================================
// bbox_tracker_node.hpp — BBox 트래커 노드
// ============================================================================
// LiDAR bbox 검출에 ego-motion 기반 트래킹을 적용하여,
// LiDAR 최소 인식거리 사각지대의 bbox를 예측 유지한다.
//
// [파이프라인]
//   1) /perception/raw_bboxes 수신 (raw bbox)
//   2) /t870/control_command 수신 (steering, speed)
//   3) 매 사이클:
//      a) Predict: bicycle model로 ego-motion 계산 → 기존 트랙 좌표 보정
//      b) Match:   새 검출과 기존 트랙 최근접 매칭 (동적 거리 threshold)
//      c) Update:  매칭 트랙 갱신, 미매칭 miss++, 고정 N프레임 초과 시 삭제
//      d) Publish:  모든 활성 트랙을 /perception/bboxes로 발행
//
// [Bicycle Model]
//   dθ = (v * tan(δ) / L) * dt
//   dx = v * cos(dθ/2) * dt
//   dy = v * sin(dθ/2) * dt
// ============================================================================

#ifndef BBOX_TRACKER__BBOX_TRACKER_NODE_HPP_
#define BBOX_TRACKER__BBOX_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ev_msgs/msg/b_box_array.hpp>
#include <ev_msgs/msg/b_box.hpp>
#include <t870_msgs/msg/control_command.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>

namespace bbox_tracker
{

struct Track
{
  double x = 0.0;          ///< 위치 x (base_link)
  double y = 0.0;          ///< 위치 y (base_link)
  double size_x = 0.0;     ///< bbox X 크기
  double size_y = 0.0;     ///< bbox Y 크기
  double size_z = 0.0;     ///< bbox Z 크기
  int label = -1;          ///< 클러스터 ID
  int miss_count = 0;      ///< 연속 미검출 횟수
  int track_id = 0;        ///< 고유 트랙 ID
};

class BBoxTrackerNode : public rclcpp::Node
{
public:
  explicit BBoxTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- 콜백 ----
  void on_bboxes(ev_msgs::msg::BBoxArray::UniquePtr msg);
  void on_control(const t870_msgs::msg::ControlCommand::SharedPtr msg);

  // ---- 트래킹 로직 ----

  /// Bicycle model로 ego-motion 계산, 기존 트랙 좌표를 현재 base_link로 변환
  void predict(double dt);

  /// 새 검출과 기존 트랙을 최근접 거리로 매칭 + 고정 프레임 수명 관리
  void match_and_update(const ev_msgs::msg::BBoxArray & detections, double dt);

  /// 모든 활성 트랙을 BBoxArray로 발행
  void publish_tracks(const std_msgs::msg::Header & header);

  // ---- 파라미터 ----
  double wheelbase_{0.87};          ///< [m] 차량 축간거리
  double min_match_dist_{0.1};      ///< [m] 동적 매칭 최소 거리
  int max_miss_count_{5};           ///< 고정 미검출 허용 프레임 수

  // ---- 상태 ----
  std::vector<Track> tracks_;       ///< 활성 트랙 목록
  double last_speed_{0.0};          ///< 직전 속도 명령 [m/s]
  double last_steering_{0.0};       ///< 직전 조향 명령 [rad]
  rclcpp::Time last_predict_time_{0, 0, RCL_ROS_TIME};  ///< 직전 predict 시각
  int next_track_id_{0};            ///< 트랙 ID 발번

  // ---- ROS2 통신 ----
  rclcpp::Subscription<ev_msgs::msg::BBoxArray>::SharedPtr sub_bboxes_;
  rclcpp::Subscription<t870_msgs::msg::ControlCommand>::SharedPtr sub_control_;
  rclcpp::Publisher<ev_msgs::msg::BBoxArray>::SharedPtr pub_tracked_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_dbg_tracks_;      ///< 전체 트랙 (초록=검출, 빨강=예측)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_dbg_predicted_;   ///< 예측 유지 중 트랙만 (miss_count > 0)
};

}  // namespace bbox_tracker

#endif  // BBOX_TRACKER__BBOX_TRACKER_NODE_HPP_
