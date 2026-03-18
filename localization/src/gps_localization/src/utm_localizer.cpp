#include "gps_localization/utm_localizer.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>

using std::placeholders::_1;

namespace gps_localization {

UtmLocalizer::UtmLocalizer()
: Node("utm_localizer"){
  /*
  ------------------------------------------------------------
  파라미터
  - waypoint_file: "lat,lon" 형식 CSV 파일 경로
  - jump_threshold: GPS 튐 거리 임계값(m)
  - covariance_threshold: 공분산 기반 sigma_xy 임계값(m)
  - lookahead_m: 목표점(lookahead) 거리(m)
  - path_points: local_path에 넣을 포인트 수
  ------------------------------------------------------------
  */
  // ---- parameters ----
  declare_parameter<std::string>("fix_topic", "/ublox_gps_node/fix");
  declare_parameter<std::string>("waypoint_file", "");
  declare_parameter<double>("jump_threshold", jump_threshold_);
  declare_parameter<double>("covariance_threshold", covariance_threshold_);
  declare_parameter<double>("lookahead_m", lookahead_m_);
  declare_parameter<int>("path_points", path_points_);
  declare_parameter<bool>("filter_forward_only", filter_forward_only_);
  declare_parameter<double>("forward_min_x", forward_min_x_);


  // 새로 추가 (timeout, goal)
  declare_parameter<double>("fix_timeout_s", fix_timeout_s_);
  declare_parameter<double>("goal_tolerance_m", goal_tolerance_m_);

  // ---- get parameters ----
  fix_topic_ = get_parameter("fix_topic").as_string();
  waypoint_file_ = get_parameter("waypoint_file").as_string();
  jump_threshold_ = get_parameter("jump_threshold").as_double();
  covariance_threshold_ = get_parameter("covariance_threshold").as_double();
  lookahead_m_ = get_parameter("lookahead_m").as_double();
  path_points_ = get_parameter("path_points").as_int();
  fix_timeout_s_ = get_parameter("fix_timeout_s").as_double();
  goal_tolerance_m_ = get_parameter("goal_tolerance_m").as_double();
  filter_forward_only_ = get_parameter("filter_forward_only").as_bool();
  forward_min_x_ = get_parameter("forward_min_x").as_double();

  // 1) /fix 구독 (너가 실제로 쓰는 토픽이 /ublox_gps_node/fix라면 여기만 바꿈
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic_, rclcpp::QoS(10),
    std::bind(&UtmLocalizer::gpsCallback, this, _1));

  // 2) /local_pose 퍼블리시 (frame=map)
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/gps_pose", rclcpp::QoS(10));

  // 3) /local_path 퍼블리시 (frame=base_link)  -> 컨트롤 입력
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", rclcpp::QoS(10));

  // 4) waypoint 파일은 origin이 정해져야 local(map)로 변환 가능하므로,
  //    여기서는 "lat/lon 목록만" 미리 로드해둔다.
  if (!waypoint_file_.empty()) {
    waypoints_loaded_ = loadWaypointsLatLonCsv(waypoint_file_, waypoints_latlon_);
    if (waypoints_loaded_) {
      RCLCPP_INFO(get_logger(), "Loaded waypoint lat/lon: %zu", waypoints_latlon_.size());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to load waypoint file: %s", waypoint_file_.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "waypoint_file parameter is empty. Path publish will be skipped.");
  }

  // ---------------------------------------------
  // fix timeout 감시 타이머
  // - /fix 가 끊기면 컨트롤 보호를 위해 빈 path publish
  // ---------------------------------------------
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&UtmLocalizer::watchdogTick, this)
  );

  RCLCPP_INFO(get_logger(),
  "utm_localizer started. fix_topic=%s, lookahead=%.1f, path_points=%d, timeout=%.2fs, goal_tol=%.1fm",
  fix_topic_.c_str(), lookahead_m_, path_points_, fix_timeout_s_, goal_tolerance_m_);

}

void UtmLocalizer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){

  // timeout 감시용 stamp 갱신
  last_fix_stamp_ = now();   // 수신 시각 기준으로 timeout 판단
  has_fix_stamp_ = true;

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

    // origin이 있어야 waypoint를 local(map)로 만들 수 있음
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
        } else {
          // outlier면 yaw 유지
          // (GPS가 순간적으로 튀거나 방향이 급격히 바뀐 것처럼 보이는 경우 방지)
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
  // (7) /local_pose 발행 (frame=map)
  // - planning/debug용, 또는 로깅용
  // ------------------------------------------------------------
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = msg->header.stamp;
  pose.header.frame_id = "map";

  pose.pose.position.x = local_x;
  pose.pose.position.y = local_y;
  pose.pose.position.z = 0.0;

  if (yaw_valid_) {
    pose.pose.orientation = yawToQuat(yaw_);
  } else {
    pose.pose.orientation.w = 1.0; // yaw 아직 없으면 identity
  }

  pose_pub_->publish(pose);

  // ------------------------------------------------------------
  // (8) /local_path 발행 (frame=base_link)
  // - 컨트롤은 base_link 좌표계에서 path를 받으면 바로 조향각 계산 가능
  // ------------------------------------------------------------
  if (yaw_valid_ && waypointsReady()) {

    // 도착 판정: 마지막 웨이포인트 근처면 빈 path
    if(reachedGoal(P_local)){
      publishEmptyPath(msg->header.stamp);
      return;
    }

    // nearest를 찾을 때 progress_idx_ 부근부터 찾으면 "뒤로 돌아가는 현상"이 줄어듦
    const int nearest = findNearestIdx(P_local, progress_idx_);
    progress_idx_ = std::max(progress_idx_, nearest);

    const int start_idx = findLookaheadIdx(nearest, lookahead_m_);
    publishLocalPathBaseLink(P_local, yaw_, start_idx, msg->header.stamp);
  }
}

bool UtmLocalizer::loadWaypointsLatLonCsv(const std::string& file_path,
                                         std::vector<std::pair<double,double>>& out_latlon){
  std::ifstream fin(file_path);
  if (!fin.is_open()) return false;

  out_latlon.clear();

  std::string line;
  while (std::getline(fin, line)) {
    // 공백 제거
    line.erase(std::remove_if(line.begin(), line.end(), [](unsigned char c){ return c=='\r'; }), line.end());
    if (line.empty()) continue;
    if (line[0] == '#') continue;

    // "lat,lon" 파싱
    std::stringstream ss(line);
    std::string a, b;
    if (!std::getline(ss, a, ',')) continue;
    if (!std::getline(ss, b)) continue;

    try {
      const double lat = std::stod(a);
      const double lon = std::stod(b);
      out_latlon.emplace_back(lat, lon);
    } catch (...) {
      // 헤더/문자열 라인이면 스킵
      continue;
    }
  }
  return !out_latlon.empty();
}

void UtmLocalizer::convertWaypointsToLocal(){
  waypoints_local_.clear();
  waypoints_local_ready_ = false;

  if (!origin_set_ || !waypoints_loaded_) return;

  waypoints_local_.reserve(waypoints_latlon_.size());

  for (const auto& ll : waypoints_latlon_) {
    double wx_utm = 0.0, wy_utm = 0.0;
    latLonToUTM(ll.first, ll.second, wx_utm, wy_utm);

    Vec2 w_local;
    w_local.x = wx_utm - origin_x_;
    w_local.y = wy_utm - origin_y_;
    waypoints_local_.push_back(w_local);
  }

  waypoints_local_ready_ = !waypoints_local_.empty();
  progress_idx_ = 0;
}

int UtmLocalizer::findNearestIdx(const Vec2& P, int start_hint) const{
  const int n = static_cast<int>(waypoints_local_.size());
  if (n == 0) return 0;

  // 힌트 인덱스 주변부터 앞으로만 탐색(뒤로 돌아가는 현상 완화)
  int start = std::clamp(start_hint, 0, n-1);

  int best = start;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (int i = start; i < n; ++i) {
    const double dx = waypoints_local_[i].x - P.x;
    const double dy = waypoints_local_[i].y - P.y;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
    // 너무 멀어지는 구간이면 조기 종료(선택적, 과도한 최적화는 피하려면 제거해도 됨)
  }
  return best;
}

int UtmLocalizer::findLookaheadIdx(int nearest_idx, double lookahead_m) const{
  const int n = static_cast<int>(waypoints_local_.size());
  if (n == 0) return 0;

  int idx = std::clamp(nearest_idx, 0, n-1);
  double acc = 0.0;

  // nearest부터 앞으로 가면서 누적 거리 >= lookahead가 되는 지점 선택
  for (int i = idx; i < n-1; ++i) {
    const double dx = waypoints_local_[i+1].x - waypoints_local_[i].x;
    const double dy = waypoints_local_[i+1].y - waypoints_local_[i].y;
    acc += std::hypot(dx, dy);
    if (acc >= lookahead_m) return i+1;
  }
  return n-1;
}

void UtmLocalizer::publishLocalPathBaseLink(const Vec2& P_local, double yaw, int start_idx, const rclcpp::Time& stamp){
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "base_link";

  const int n = static_cast<int>(waypoints_local_.size());
  int s = std::clamp(start_idx, 0, std::max(0, n-1));
  int e = std::min(n, s + path_points_);

  path.poses.reserve(e - s);

  // local(map) waypoint들을 base_link로 변환해서 Path에 넣음
  for (int i = s; i < e; ++i) {
    Vec2 G_base = worldToBase(P_local, yaw, waypoints_local_[i]);

    // 차량 앞쪽 점만 남기기
    if(filter_forward_only_ && G_base.x < forward_min_x_){
      continue;
    }

    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = G_base.x;
    p.pose.position.y = G_base.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0; // path 점 자체는 방향 불필요(컨트롤에서 곡률 계산하면 됨)

    path.poses.push_back(p);
  }

  if (path.poses.empty()) {
    // fallback: start_idx 점 하나라도 넣기
    Vec2 G_base = worldToBase(P_local, yaw, waypoints_local_[s]);
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = G_base.x;
    p.pose.position.y = G_base.y;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  path_pub_->publish(path);
}

double UtmLocalizer::normalizeAngle(double a){
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

geometry_msgs::msg::Quaternion UtmLocalizer::yawToQuat(double yaw){
  geometry_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
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

void UtmLocalizer::watchdogTick(){
  if (!has_fix_stamp_) return;

  const rclcpp::Time now_t = now();
  const double dt = (now_t - last_fix_stamp_).seconds();

  if (dt > fix_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "/fix timeout: dt=%.2fs > %.2fs -> publish empty path",
      dt, fix_timeout_s_);
    publishEmptyPath(now_t);
  }
}

void UtmLocalizer::publishEmptyPath(const rclcpp::Time& stamp){
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "base_link";
  path.poses.clear();
  path_pub_->publish(path);
}

bool UtmLocalizer::reachedGoal(const Vec2& P_local) const
{
  if (!waypointsReady()) return false;
  const Vec2& G_last = waypoints_local_.back();
  const double d = std::hypot(G_last.x - P_local.x, G_last.y - P_local.y);
  return d <= goal_tolerance_m_;
}

} // namespace gps_localization