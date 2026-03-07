#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class UtmLocalizer : public rclcpp::Node{
public:
    UtmLocalizer();

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void latLonToUTM(double lat, double lon, double &x, double &y);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    bool origin_set_;
    double origin_x_;
    double origin_y_;

    double prev_x_;
    double prev_y_;
    bool has_prev_;
    double jump_threshold_;

    double covariance_threshold_;
};