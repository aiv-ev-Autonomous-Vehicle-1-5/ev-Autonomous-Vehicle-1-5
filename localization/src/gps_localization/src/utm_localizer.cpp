#include "gps_localization/utm_localizer.hpp"
#include <cmath>

UtmLocalizer::UtmLocalizer()
: Node("utm_localizer"), origin_set_(false)
{
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix", 10,
        std::bind(&UtmLocalizer::gpsCallback, this, std::placeholders::_1)
    );

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/local_pose", 10
    );
}

void UtmLocalizer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    double utm_x, utm_y;
    latLonToUTM(msg->latitude, msg->longitude, utm_x, utm_y);

    if (!origin_set_)
    {
        origin_x_ = utm_x;
        origin_y_ = utm_y;
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Origin set.");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "base_link";

    pose.pose.position.x = utm_x - origin_x_;
    pose.pose.position.y = utm_y - origin_y_;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.w = 1.0;  // yaw=0 고정

    pose_pub_->publish(pose);
}

void UtmLocalizer::latLonToUTM(double lat, double lon, double &x, double &y)
{
    const double a = 6378137.0;
    const double f = 1 / 298.257223563;
    const double k0 = 0.9996;

    const double e = sqrt(f * (2 - f));
    const double latRad = lat * M_PI / 180.0;
    const double lonRad = lon * M_PI / 180.0;

    int zone = 52; // 한국
    double lonOrigin = (zone - 1) * 6 - 180 + 3;
    double lonOriginRad = lonOrigin * M_PI / 180.0;

    double N = a / sqrt(1 - pow(e * sin(latRad), 2));
    double T = pow(tan(latRad), 2);
    double C = pow(e * cos(latRad), 2);
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = a * ((1 - e * e / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256) * latRad
        - (3 * e * e / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024) * sin(2 * latRad));

    x = k0 * N * A + 500000.0;
    y = k0 * M;
}