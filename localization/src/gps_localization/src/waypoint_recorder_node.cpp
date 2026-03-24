#include "gps_localization/waypoint_recorder.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<gps_localization::WaypointRecorder>());

  rclcpp::shutdown();
  return 0;
}
