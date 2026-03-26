// ============================================================================
// command_publisher.hpp — T870/ERP42 이중 제어 명령 발행기
// ============================================================================
// T870 실차와 ERP42 Gazebo 시뮬레이터에 동시에 제어 명령을 발행한다.
// ERP42는 구독자가 있을 때만 발행 (lazy).
// 제어 명령(ControlCommand)에 원본 센서 타임스탬프(velodyne_points)를
// header.stamp으로 전파하여 ros2 topic delay로 파이프라인 지연 측정이 가능하다.
// ============================================================================

#ifndef PP_CONTROLLER_CPP__NODES__COMMAND_PUBLISHER_HPP_
#define PP_CONTROLLER_CPP__NODES__COMMAND_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "t870_msgs/msg/control_command.hpp"
#include "erp42_msgs/msg/control_command.hpp"

namespace pp_controller_cpp
{

class CommandPublisher
{
public:
  using T870Pub  = rclcpp::Publisher<t870_msgs::msg::ControlCommand>::SharedPtr;
  using ERP42Pub = rclcpp::Publisher<erp42_msgs::msg::ControlCommand>::SharedPtr;

  CommandPublisher(T870Pub t870_pub, ERP42Pub erp42_pub);

  /// T870 + ERP42 양쪽에 (speed, steering) 명령 발행
  /// ERP42는 구독자가 있을 때만 발행 (lazy)
  /// v_cmd < 1e-3 일 때 ERP42에 brake=75 적용
  /// stamp: 원본 센서 타임스탬프 전파 (ros2 topic delay 측정용)
  void publish(double speed, double steering, const builtin_interfaces::msg::Time & stamp);

private:
  T870Pub  t870_pub_;
  ERP42Pub erp42_pub_;
};

}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__NODES__COMMAND_PUBLISHER_HPP_
