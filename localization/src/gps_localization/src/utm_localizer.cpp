#include "gps_localization/utm_localizer.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

using std::placeholders::_1;

namespace gps_localization {

UtmLocalizer::UtmLocalizer()
: Node("utm_localizer"){
  /*
  ------------------------------------------------------------
  파라미터
  - waypoint_file: "index,utm_x,utm_y" 형식 CSV 파일 경로
  - jump_threshold: GPS 튐 거리 임계값(m)
  - covariance_threshold: 공분산 기반 sigma_xy 임계값(m)
  ------------------------------------------------------------
  */
  // ---- parameters ----
  declare_parameter<std::string>("fix_topic", "/ublox_gps_node/fix");
  declare_parameter<std::string>("waypoint_file", "");
  declare_parameter<double>("jump_threshold", jump_threshold_);
  declare_parameter<double>("covariance_threshold", covariance_threshold_);

  // ---- get parameters ----
  fix_topic_ = get_parameter("fix_topic").as_string();
  waypoint_file_ = get_parameter("waypoint_file").as_string();
  jump_threshold_ = get_parameter("jump_threshold").as_double();
  covariance_threshold_ = get_parameter("covariance_threshold").as_double();

  // 1) /fix 구독
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic_, rclcpp::QoS(10),
    std::bind(&UtmLocalizer::gpsCallback, this, _1));

  // 2) /local_path 퍼블리시 (frame=base_link) — 전체 waypoint의 헤딩 기준 상대 좌표 (Marker POINTS)
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/local_path", rclcpp::QoS(10));

  // 3) waypoint CSV 로드 (index,utm_x,utm_y 형식)
  //    origin이 정해져야 local(map)로 변환 가능하므로 UTM 값만 미리 로드
  if (!waypoint_file_.empty()) {
    waypoints_loaded_ = loadWaypointsUtmCsv(waypoint_file_);
    if (waypoints_loaded_) {
      RCLCPP_INFO(get_logger(), "Loaded waypoint UTM: %zu", waypoints_utm_.size());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to load waypoint file: %s", waypoint_file_.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "waypoint_file parameter is empty. Path publish will be skipped.");
  }

  RCLCPP_INFO(get_logger(), "utm_localizer started. fix_topic=%s", fix_topic_.c_str());

}

void UtmLocalizer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){

  // ------------------------------------------------------------
  // (0) 공분산 필터
  // - msg->position_covariance는 3x3(평면은 xx, yy가 중요)
  // - sigma_xy = sqrt(0.5*(cov_xx + cov_yy))
  // ------------------------------------------------------------
  if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    const double cov_xx = msg->position_covariance[0];
    const double cov_yy = msg->position_covariance[4];
    const double v = 0.5 * (cov_xx + cov_yy);
    if (v >= 0.0) {
      const double sigma_xy = std::sqrt(v);
      if (sigma_xy > covariance_threshold_) {
        RCLCPP_WARN(get_logger(),
          "GPS covariance too large. sigma_xy=%.3f m > %.3f -> ignored",
          sigma_xy, covariance_threshold_);
        return;
      }
    }
  }

  // ------------------------------------------------------------
  // (1) 위경도 -> UTM
  // ------------------------------------------------------------
  double utm_x = 0.0, utm_y = 0.0;
  latLonToUTM(msg->latitude, msg->longitude, utm_x, utm_y);

  // ------------------------------------------------------------
  // (2) 최초 1회 origin 설정
  //     origin_set_ 된 순간, waypoint들을 local(map)로 변환해서 캐싱한다.
  // ------------------------------------------------------------
  if (!origin_set_) {
    origin_x_ = utm_x;
    origin_y_ = utm_y;
    origin_set_ = true;
    RCLCPP_INFO(get_logger(), "Origin set: (%.3f, %.3f)", origin_x_, origin_y_);

    if (waypoints_loaded_) {
      convertWaypointsToLocal();
      if (waypointsReady()) {
        RCLCPP_INFO(get_logger(), "Waypoints converted to local(map): %zu", waypoints_local_.size());
      } else {
        RCLCPP_WARN(get_logger(), "Waypoints local conversion failed/empty.");
      }
    }
  }

  // ------------------------------------------------------------
  // (3) origin 기준 local(map) 좌표
  // ------------------------------------------------------------
  const double local_x = utm_x - origin_x_;
  const double local_y = utm_y - origin_y_;
  Vec2 P_local{local_x, local_y};

  // ------------------------------------------------------------
  // (4) GPS 점프 필터: 이전 local과 거리 비교
  // ------------------------------------------------------------
  if (has_prev_) {
    const double dx = local_x - prev_x_;
    const double dy = local_y - prev_y_;
    const double dist = std::hypot(dx, dy);

    if (dist > jump_threshold_) {
      RCLCPP_WARN(get_logger(), "GPS jump detected! dist=%.2f m > %.2f -> ignored",
                  dist, jump_threshold_);
      return;
    }
  }

  // ------------------------------------------------------------
  // (5) yaw(heading) 추정
  // - IMU가 없으니 "이동 방향(course over ground)"로 yaw를 만든다.
  // - 너무 조금 움직였으면(yaw가 튀기 쉬움) yaw 업데이트 freeze
  // - 갑자기 큰 yaw 점프도 outlier로 보고 무시
  // ------------------------------------------------------------
  if (has_prev_) {
    const double dx = local_x - prev_x_;
    const double dy = local_y - prev_y_;
    const double dist = std::hypot(dx, dy);

    if (dist >= min_move_m_) {
      const double new_yaw = std::atan2(dy, dx);

      if (!yaw_valid_) {
        yaw_ = new_yaw;
        yaw_valid_ = true;
      } else {
        const double dyaw = normalizeAngle(new_yaw - yaw_);
        if (std::fabs(dyaw) <= max_yaw_jump_rad_) {
          yaw_ = new_yaw;
        }
      }
    }
  }

  // ------------------------------------------------------------
  // (6) 이전 값 업데이트 (jump filter 통과한 값만 저장)
  // ------------------------------------------------------------
  prev_x_ = local_x;
  prev_y_ = local_y;
  has_prev_ = true;

  // ------------------------------------------------------------
  // (7) /local_path 발행 (frame=base_link)
  // - 전체 waypoint를 헤딩 기준 상대 좌표로 변환하여 발행
  // ------------------------------------------------------------
  if (yaw_valid_ && waypointsReady()) {
    publishWaypointsBaseLink(P_local, yaw_, msg->header.stamp);
  }
}

bool UtmLocalizer::loadWaypointsUtmCsv(const std::string& file_path){
  std::ifstream fin(file_path);
  if (!fin.is_open()) return false;

  waypoints_utm_.clear();

  std::string line;
  while (std::getline(fin, line)) {
    line.erase(std::remove_if(line.begin(), line.end(), [](unsigned char c){ return c=='\r'; }), line.end());
    if (line.empty()) continue;
    if (line[0] == '#') continue;

    // "index,utm_x,utm_y" 파싱
    std::stringstream ss(line);
    std::string idx_str, x_str, y_str;
    if (!std::getline(ss, idx_str, ',')) continue;
    if (!std::getline(ss, x_str, ',')) continue;
    if (!std::getline(ss, y_str)) continue;

    try {
      Vec2 wp;
      wp.x = std::stod(x_str);
      wp.y = std::stod(y_str);
      waypoints_utm_.push_back(wp);
    } catch (...) {
      continue;
    }
  }
  return !waypoints_utm_.empty();
}

void UtmLocalizer::convertWaypointsToLocal(){
  waypoints_local_.clear();
  waypoints_local_ready_ = false;

  if (!origin_set_ || !waypoints_loaded_) return;

  waypoints_local_.reserve(waypoints_utm_.size());

  // waypoint는 이미 UTM 좌표이므로 origin만 빼면 local(map)
  for (const auto& wp : waypoints_utm_) {
    Vec2 w_local;
    w_local.x = wp.x - origin_x_;
    w_local.y = wp.y - origin_y_;
    waypoints_local_.push_back(w_local);
  }

  waypoints_local_ready_ = !waypoints_local_.empty();
}

void UtmLocalizer::publishWaypointsBaseLink(const Vec2& P_local, double yaw, const rclcpp::Time& stamp){
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = "base_link";
  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3;  // point 크기
  marker.scale.y = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  const int n = static_cast<int>(waypoints_local_.size());
  marker.points.reserve(n);

  // 전체 waypoint를 헤딩 기준 상대 좌표(base_link)로 변환
  for (int i = 0; i < n; ++i) {
    Vec2 G_base = worldToBase(P_local, yaw, waypoints_local_[i]);

    geometry_msgs::msg::Point pt;
    pt.x = G_base.x;
    pt.y = G_base.y;
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  marker_pub_->publish(marker);
}

double UtmLocalizer::normalizeAngle(double a){
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

Vec2 UtmLocalizer::worldToBase(const Vec2& P_world, double yaw, const Vec2& G_world){
  // R(-yaw) * (G - P)
  const double dx = G_world.x - P_world.x;
  const double dy = G_world.y - P_world.y;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  Vec2 out;
  out.x =  c * dx + s * dy;
  out.y = -s * dx + c * dy;
  return out;
}

/*
------------------------------------------------------------
latLonToUTM
- 한국 기준 zone 52 고정
------------------------------------------------------------
*/
void UtmLocalizer::latLonToUTM(double lat, double lon, double &x, double &y){
  const double a = 6378137.0;              // WGS84 장반경
  const double f = 1 / 298.257223563;      // 편평률
  const double k0 = 0.9996;                // scale factor

  const double e = std::sqrt(f * (2 - f));
  const double e2 = e * e;
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;

  double latRad = lat * M_PI / 180.0;
  double lonRad = lon * M_PI / 180.0;

  int zone = 52;
  double lonOrigin = (zone - 1) * 6 - 180 + 3;
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