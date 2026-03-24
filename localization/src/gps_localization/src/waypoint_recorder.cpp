#include "gps_localization/waypoint_recorder.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

using std::placeholders::_1;

namespace gps_localization {

WaypointRecorder::WaypointRecorder()
: Node("waypoint_recorder"){
  /*
  ------------------------------------------------------------
  파라미터
  - output_file  : 저장할 CSV 경로 (빈 문자열이면 타임스탬프 기반 자동 생성)
  - record_interval : 기록 주기(s)
  - covariance_threshold : 공분산 기반 sigma_xy 임계값(m)
  - min_distance : 연속 기록 최소 거리(m), 정차 시 중복 방지
  ------------------------------------------------------------
  */
  declare_parameter<std::string>("fix_topic", fix_topic_);
  declare_parameter<std::string>("output_file", "");
  declare_parameter<double>("record_interval", record_interval_);
  declare_parameter<double>("covariance_threshold", covariance_threshold_);
  declare_parameter<double>("min_distance", min_distance_);

  fix_topic_            = get_parameter("fix_topic").as_string();
  output_file_          = get_parameter("output_file").as_string();
  record_interval_      = get_parameter("record_interval").as_double();
  covariance_threshold_ = get_parameter("covariance_threshold").as_double();
  min_distance_         = get_parameter("min_distance").as_double();

  // output_file이 비어 있으면 타임스탬프 기반 파일명 자동 생성
  if (output_file_.empty()) {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    std::ostringstream oss;
    oss << "waypoints_"
        << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << ".csv";
    output_file_ = oss.str();
  }

  // CSV 파일 열기 (append 모드 — 기존 파일이 있으면 이어서 기록)
  fout_.open(output_file_, std::ios::app);
  if (!fout_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", output_file_.c_str());
    return;
  }

  // 1) /fix 구독
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic_, rclcpp::QoS(10),
    std::bind(&WaypointRecorder::gpsCallback, this, _1));

  // 2) 기록 타이머
  record_timer_ = create_wall_timer(
    std::chrono::duration<double>(record_interval_),
    std::bind(&WaypointRecorder::timerCallback, this));

  RCLCPP_INFO(get_logger(),
    "waypoint_recorder started. output=%s  interval=%.2fs  min_dist=%.2fm",
    output_file_.c_str(), record_interval_, min_distance_);
}

WaypointRecorder::~WaypointRecorder(){
  if (fout_.is_open()) {
    fout_.flush();
    fout_.close();
    RCLCPP_INFO(get_logger(), "Saved %d waypoints to %s", index_, output_file_.c_str());
  }
}

// ============================================================
// GPS 콜백: 공분산 필터 통과 시 최신 UTM 좌표 캐싱
// ============================================================
void WaypointRecorder::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){

  // 공분산 필터
  if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    const double cov_xx = msg->position_covariance[0];
    const double cov_yy = msg->position_covariance[4];
    const double v = 0.5 * (cov_xx + cov_yy);
    if (v >= 0.0) {
      const double sigma_xy = std::sqrt(v);
      if (sigma_xy > covariance_threshold_) {
        return;   // GPS 품질 불량 → 무시
      }
    }
  }

  // lat/lon → UTM
  double utm_x = 0.0, utm_y = 0.0;
  latLonToUTM(msg->latitude, msg->longitude, utm_x, utm_y);

  latest_utm_x_ = utm_x;
  latest_utm_y_ = utm_y;
  has_valid_fix_ = true;
}

// ============================================================
// 타이머 콜백: 주기마다 유효 fix → CSV 기록
// ============================================================
void WaypointRecorder::timerCallback(){
  if (!has_valid_fix_) return;
  if (!fout_.is_open()) return;

  // 이전 기록점과 거리 검사 (정차 시 중복 방지)
  if (has_last_recorded_) {
    const double dx = latest_utm_x_ - last_recorded_x_;
    const double dy = latest_utm_y_ - last_recorded_y_;
    if (std::hypot(dx, dy) < min_distance_) {
      return;   // 너무 가까움 → 스킵
    }
  }

  // CSV 한 줄 기록: index,utm_x,utm_y
  ++index_;
  fout_ << index_ << ","
        << std::fixed << std::setprecision(9)
        << latest_utm_x_ << ","
        << latest_utm_y_ << "\n";
  fout_.flush();

  last_recorded_x_   = latest_utm_x_;
  last_recorded_y_   = latest_utm_y_;
  has_last_recorded_  = true;
  has_valid_fix_      = false;   // 다음 유효 fix 대기

  if (index_ % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Recorded %d waypoints", index_);
  }
}

// ============================================================
// lat/lon → UTM (zone 52 고정, 한국 기준)
// utm_localizer.cpp와 동일한 변환 로직
// ============================================================
void WaypointRecorder::latLonToUTM(double lat, double lon, double &x, double &y){
  const double a  = 6378137.0;
  const double f  = 1.0 / 298.257223563;
  const double k0 = 0.9996;

  const double e  = std::sqrt(f * (2 - f));
  const double e2 = e * e;
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;

  double latRad = lat * M_PI / 180.0;
  double lonRad = lon * M_PI / 180.0;

  int zone = 52;
  double lonOrigin    = (zone - 1) * 6 - 180 + 3;
  double lonOriginRad = lonOrigin * M_PI / 180.0;

  double N = a / std::sqrt(1 - e2 * std::pow(std::sin(latRad), 2));
  double T = std::pow(std::tan(latRad), 2);
  double C = (e2 / (1 - e2)) * std::pow(std::cos(latRad), 2);
  double A = std::cos(latRad) * (lonRad - lonOriginRad);

  double M =
      a * ((1 - e2/4 - 3*e4/64 - 5*e6/256) * latRad
      - (3*e2/8 + 3*e4/32 + 45*e6/1024) * std::sin(2*latRad)
      + (15*e4/256 + 45*e6/1024) * std::sin(4*latRad)
      - (35*e6/3072) * std::sin(6*latRad));

  x = k0 * N * (A + (1 - T + C) * std::pow(A,3)/6
      + (5 - 18*T + T*T + 72*C - 58*(e2/(1-e2))) * std::pow(A,5)/120)
      + 500000.0;

  y = k0 * (M + N * std::tan(latRad) *
      (A*A/2
      + (5 - T + 9*C + 4*C*C) * std::pow(A,4)/24
      + (61 - 58*T + T*T + 600*C - 330*(e2/(1-e2))) * std::pow(A,6)/720));
}

} // namespace gps_localization
