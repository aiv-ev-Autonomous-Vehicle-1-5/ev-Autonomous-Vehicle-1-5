// ============================================================================
// pure_pursuit_relative_node.hpp — Pure Pursuit 제어 노드 (오케스트레이터)
// ============================================================================
//
// [개요]
//   상대좌표(base_link 기준) Marker(POINTS)를 입력받아 Pure Pursuit 알고리즘으로
//   T870 차량의 조향각(steering)과 속도(speed)를 계산하는 제어 노드.
//
// [데이터 흐름]
//   /planning/path   (visualization_msgs/Marker POINTS, base_link 기준 상대좌표)
//   /planning/status (std_msgs/String — "OK", "FAIL - ..." 등 planning 상태)
//     → [이 노드: Pure Pursuit 계산]
//       → /t870/control_command  (t870_msgs/ControlCommand)  — 실차용
//       → /erp42/control_command (erp42_msgs/ControlCommand) — Gazebo 시뮬레이션용 (lazy)
//       → /pp_debug/lookahead_point (Marker SPHERE)          — 디버그 (lazy)
//       → /pp_debug/pursuit_arc     (Marker LINE_STRIP)      — 디버그 (lazy)
//
// [왜 "Relative" 버전인가?]
//   planning 모듈이 이미 base_link 기준 상대좌표로 경로를 생성하므로,
//   GPS/odometry 없이도 동작한다.
//
// [모듈 구조]
//   이 노드는 다음 모듈들을 조합하여 동작한다:
//     - common/params.hpp          : 파라미터 관리
//     - pursuit/path_query         : 경로 분석 (최근접, 목표점, 잔여거리, 곡률)
//     - pursuit/speed_planning     : 속도 계획 (lookahead, 속도, rate limit)
//     - pursuit/steering           : 조향각 계산
//     - nodes/command_publisher    : T870/ERP42 이중 명령 발행
//     - debug/debug_visualizer     : RViz2 시각화
// ============================================================================

#ifndef PP_CONTROLLER_CPP__NODES__PURE_PURSUIT_RELATIVE_NODE_HPP_
#define PP_CONTROLLER_CPP__NODES__PURE_PURSUIT_RELATIVE_NODE_HPP_

#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include "ev_msgs/msg/b_box_array.hpp"

#include "pp_controller_cpp/common/params.hpp"
#include "pp_controller_cpp/nodes/command_publisher.hpp"
#include "pp_controller_cpp/debug/debug_visualizer.hpp"

namespace pp_controller_cpp
{

class PurePursuitRelativeNode : public rclcpp::Node
{
public:
  explicit PurePursuitRelativeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ======================== 콜백 ========================

  /// /planning/path 토픽 콜백: 최신 경로 저장
  void on_path(const visualization_msgs::msg::Marker::SharedPtr msg);

  /// 메인 제어 루프 (50Hz)
  void on_timer();

  // ======================== 유틸리티 ========================

  /// 경로가 유효(신선)한지 판단
  /// true = 비어있지 않고 timeout 이내, false = 없거나 오래됨
  bool path_fresh() const;

  /// 정지 명령 발행 (speed=0, steering=0) — 즉시 정지 아닌 점진적 감속
  void publish_emergency_decel(double dt);

  /// 경로 잔여 길이를 이동 평균 필터링 (drop 의심 시 buffer 동결, N프레임 확정 시 즉시 반영)
  double compute_filtered_remaining(double raw_remaining);

  /// 전방 ROI에 bbox 장애물이 없는지 판단 (CREEP 모드 조건)
  bool is_forward_clear() const;

  // ======================== 멤버 변수 ========================

  // --- 파라미터 ---
  PurePursuitParams params_;

  // --- 상태 ---
  std::vector<geometry_msgs::msg::Point> latest_points_;  // 최근 수신 경로 점 배열
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};      // 경로 마지막 수신 시각 (stale 검사용)
  rclcpp::Time last_path_stamp_{0, 0, RCL_ROS_TIME};    // 경로 메시지의 원본 센서 타임스탬프 (delay 전파용)
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};   // 직전 제어 루프 시각
  std::string latest_status_;                              // 최신 planning 상태
  double last_cmd_speed_{0.0};                             // 직전 속도 명령 [m/s]
  int    fail_counter_{0};                                  // FAIL 연속 카운터
  std::deque<double> path_length_buffer_;                   // 경로 길이 이동 평균 버퍼
  int path_drop_counter_{0};                                // 급락 연속 카운터
  std::vector<double> pending_raws_;                        // drop 의심 중 보류된 raw 값들

  // --- CREEP 상태 ---
  ev_msgs::msg::BBoxArray::SharedPtr latest_bboxes_;       // 최신 bbox 데이터

  // --- ROS2 통신 객체 ---
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<ev_msgs::msg::BBoxArray>::SharedPtr bbox_sub_;
  CommandPublisher cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- 디버그 시각화 ---
  DebugVisualizer debug_viz_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_dbg_creep_roi_;  ///< CREEP ROI 영역 (lazy)
};

}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__NODES__PURE_PURSUIT_RELATIVE_NODE_HPP_
