#include "gps_localization/utm_localizer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<gps_localization::UtmLocalizer>());

  rclcpp::shutdown();
  return 0;
}