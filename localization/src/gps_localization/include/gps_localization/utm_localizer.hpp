#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

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
  로컬라이제이션 파이프라인(실전 버전)
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
  8) /local_pose (frame=map) 발행
  9) waypoint(local map) -> base_link 변환하여 /local_path (frame=base_link) 발행
     - 컨트롤은 /local_path만 받아 조향각 계산
  10) timeout/goal:
     - /fix timeout 시 빈 path 발행
     - goal 근처면 빈 path 발행
  ============================================================
  */

  // -------- ROS callbacks --------
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // /fix timeout 감시(타이머)
  void watchdogTick();
  void publishEmptyPath(const rclcpp::Time& stamp);

  // goal 도착 판정
  bool reachedGoal(const Vec2& P_local) const;

  // -------- Waypoint I/O --------
  bool loadWaypointsLatLonCsv(
      const std::string& file_path,
      std::vector<std::pair<double,double>>& out_latlon);

  // origin 설정 이후 1회 실행: lat/lon -> UTM -> local(map) 캐싱
  void convertWaypointsToLocal();
  bool waypointsReady() const { return waypoints_local_ready_ && !waypoints_local_.empty(); }

  // -------- Target / Path build --------
  int findNearestIdx(const Vec2& P, int start_hint) const;
  int findLookaheadIdx(int nearest_idx, double lookahead_m) const;

  // local(map) waypoint들을 base_link로 변환해 Path 발행
  void publishLocalPathBaseLink(
      const Vec2& P_local,
      double yaw,
      int start_idx,
      const rclcpp::Time& stamp);

  // -------- Math helpers --------
  void latLonToUTM(double lat, double lon, double &x, double &y);

  static double normalizeAngle(double a);
  static geometry_msgs::msg::Quaternion yawToQuat(double yaw);
  static Vec2 worldToBase(const Vec2& P_world, double yaw, const Vec2& G_world);

  // ============================================================
  // Members
  // ============================================================

  // -------- ROS I/O --------
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // -------- Parameters --------
  std::string fix_topic_{"/ublox_gps_node/fix"};
  std::string waypoint_file_;

  double jump_threshold_{2.0};          // GPS 점프 필터 거리(m)
  double covariance_threshold_{0.5};    // sigma_xy(m) 임계값

  double lookahead_m_{6.0};             // lookahead 거리(m)
  int path_points_{30};                 // local_path에 넣을 점 개수

  double fix_timeout_s_{0.5};           // /fix가 이 시간 이상 끊기면 timeout
  double goal_tolerance_m_{2.0};        // 마지막 waypoint와 이 거리 이내면 도착

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

  // timeout 감시용
  rclcpp::Time last_fix_stamp_{0, 0, RCL_ROS_TIME};
  bool has_fix_stamp_{false};

  // -------- Waypoints --------
  bool waypoints_loaded_{false};
  bool waypoints_local_ready_{false};

  bool filter_forward_only_{true};   // x>0 점만 path에 넣기
  double forward_min_x_{0.2};        // 너무 가까운 점(0~0.2m)은 제거

  // raw: lat/lon
  std::vector<std::pair<double,double>> waypoints_latlon_;

  // local(map) 캐시
  std::vector<Vec2> waypoints_local_;

  // 진행 인덱스(뒤로 돌아가는 현상 완화)
  int progress_idx_{0};
};

} // namespace gps_localization