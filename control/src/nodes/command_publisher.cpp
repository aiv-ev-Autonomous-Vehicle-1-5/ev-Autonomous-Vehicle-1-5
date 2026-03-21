// ============================================================================
// command_publisher.cpp — T870/ERP42 이중 제어 명령 발행기 구현
// ============================================================================

#include "pp_controller_cpp/nodes/command_publisher.hpp"

namespace pp_controller_cpp
{

CommandPublisher::CommandPublisher(T870Pub t870_pub, ERP42Pub erp42_pub)
  : t870_pub_(std::move(t870_pub))
  , erp42_pub_(std::move(erp42_pub))
{
}

void CommandPublisher::publish(double speed, double steering)
{
  // T870 실차 명령
  t870_msgs::msg::ControlCommand cmd;
  cmd.speed = speed;
  cmd.steering = steering;
  t870_pub_->publish(cmd);

  // ERP42 Gazebo 시뮬레이션 명령 (구독자가 있을 때만)
  if (erp42_pub_->get_subscription_count() > 0) {
    erp42_msgs::msg::ControlCommand erp_cmd;
    erp_cmd.speed = speed;
    erp_cmd.steering = steering;
    erp_cmd.brake = (speed < 1e-3) ? 75 : 0;
    erp42_pub_->publish(erp_cmd);
  }
}

}  // namespace pp_controller_cpp
