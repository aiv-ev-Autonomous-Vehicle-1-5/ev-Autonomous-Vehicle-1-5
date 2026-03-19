#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>
#include <utility>   // std::pair

namespace gps_localization {

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

class UtmLocalizer : public rclcpp::Node {
public:
  explicit UtmLocalizer();

private:
  /*
  ============================================================
  파이프라인
  ------------------------------------------------------------
  1) /fix (lat,lon,cov) 수신
  2) 공분산 필터(sigma_xy): 너무 크면 무시
  3) lat/lon -> UTM
  4) 최초 1회 origin 설정(utm 기준)
     - origin 설정 후 waypoint(lat/lon)를 UTM->local(map)로 1회 변환 캐싱
  5) local(map) = (utm - origin)
  6) 점프 필터: 이전 local과 거리 > jump_threshold면 무시
  7) yaw 추정: 이동방향(atan2(dy,dx))
     - 저속이면 yaw 업데이트 freeze
     - yaw 급점프(outlier) 무시
  8) 전체 waypoint를 헤딩 기준 상대 좌표(base_link)로 변환하여 /local_path (Marker POINTS)로 발행
  ============================================================
  */

  // -------- ROS callbacks --------
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // -------- Waypoint I/O --------
  bool loadWaypointsLatLonCsv(
      const std::string& file_path,
      std::vector<std::pair<double,double>>& out_latlon);

  // origin 설정 이후 1회 실행: lat/lon -> UTM -> local(map) 캐싱
  void convertWaypointsToLocal();
  bool waypointsReady() const { return waypoints_local_ready_ && !waypoints_local_.empty(); }

  // 전체 waypoint를 헤딩 기준 상대 좌표(base_link)로 변환해 Path 발행
  void publishWaypointsBaseLink(
      const Vec2& P_local,
      double yaw,
      const rclcpp::Time& stamp);

  // -------- Math helpers --------
  void latLonToUTM(double lat, double lon, double &x, double &y);

  static double normalizeAngle(double a);
  static Vec2 worldToBase(const Vec2& P_world, double yaw, const Vec2& G_world);

  // ============================================================
  // Members
  // ============================================================

  // -------- ROS I/O --------
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // -------- Parameters --------
  std::string fix_topic_{"/ublox_gps_node/fix"};
  std::string waypoint_file_;

  double jump_threshold_{2.0};          // GPS 점프 필터 거리(m)
  double covariance_threshold_{0.5};    // sigma_xy(m) 임계값

  // -------- Origin / current state --------
  bool origin_set_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};

  bool has_prev_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};

  // yaw 추정 상태
  bool yaw_valid_{false};
  double yaw_{0.0};
  double min_move_m_{0.2};              // 이동거리 < 이 값이면 yaw freeze
  double max_yaw_jump_rad_{M_PI / 4.0}; // yaw 점프 outlier 임계값

  // -------- Waypoints --------
  bool waypoints_loaded_{false};
  bool waypoints_local_ready_{false};

  // raw: lat/lon
  std::vector<std::pair<double,double>> waypoints_latlon_;

  // local(map) 캐시
  std::vector<Vec2> waypoints_local_;
};

} // namespace gps_localization