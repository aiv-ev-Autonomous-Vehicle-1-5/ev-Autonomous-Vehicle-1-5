#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>
#include <fstream>

namespace gps_localization {

class WaypointRecorder : public rclcpp::Node {
public:
  explicit WaypointRecorder();
  ~WaypointRecorder();

private:
  /*
  ============================================================
  파이프라인
  ------------------------------------------------------------
  1) /fix (lat,lon,cov) 수신
  2) 공분산 필터(sigma_xy): 너무 크면 무시
  3) lat/lon -> UTM 변환
  4) 최신 유효 UTM 좌표 캐싱
  5) 타이머 주기마다:
     - 새 유효 fix가 있으면
     - 이전 기록점과 min_distance 이상 떨어져 있으면
     - CSV에 "index,utm_x,utm_y" 한 줄 추가
  ============================================================
  */

  // -------- ROS callbacks --------
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void timerCallback();

  // -------- Math helpers --------
  // 한국 기준 UTM zone 52 고정 변환
  static void latLonToUTM(double lat, double lon, double &x, double &y);

  // ============================================================
  // Members
  // ============================================================

  // -------- ROS I/O --------
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr record_timer_;

  // -------- Parameters --------
  std::string fix_topic_{"/ublox_gps_node/fix"};
  std::string output_file_;
  double record_interval_{0.5};           // 기록 주기(s)
  double covariance_threshold_{0.5};      // sigma_xy(m) 임계값
  double min_distance_{0.3};              // 연속 기록 최소 거리(m)

  // -------- State --------
  std::ofstream fout_;
  int index_{0};                          // CSV 행 인덱스 (1-based)

  bool has_valid_fix_{false};             // GPS 콜백에서 유효 fix를 받았는지
  double latest_utm_x_{0.0};
  double latest_utm_y_{0.0};

  bool has_last_recorded_{false};         // 이전 기록점 존재 여부
  double last_recorded_x_{0.0};
  double last_recorded_y_{0.0};
};

} // namespace gps_localization
