#include "gps_localization/utm_localizer.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UtmLocalizer>());
    rclcpp::shutdown();
    return 0;
}