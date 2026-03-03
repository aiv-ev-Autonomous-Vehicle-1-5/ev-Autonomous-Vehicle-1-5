#include "gps_localization/utm_localizer.hpp"
#include <cmath>

/*
------------------------------------------------------------
UtmLocalizer 생성자
- 노드 이름: utm_localizer
- GPS(/fix) 구독
- 로컬 좌표(/local_pose) 퍼블리시
------------------------------------------------------------
*/
UtmLocalizer::UtmLocalizer()
: Node("utm_localizer"),
  origin_set_(false),
  has_prev_(false),          // 이전 좌표 존재 여부
  jump_threshold_(2.0),       // 2m 이상 점프하면 무시
  covariance_threshold_(0.5) {       
    // GPS 토픽 구독
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix", 10,
        std::bind(&UtmLocalizer::gpsCallback, this, std::placeholders::_1)
    );

    // 로컬 좌표 퍼블리셔
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/local_pose", 10
    );
}


/*
------------------------------------------------------------
gpsCallback
- GPS(lat, lon) → UTM 변환
- 최초 1회 origin 설정
- 이전 좌표와 거리 비교하여 튐 필터 적용
- 정상 값이면 /local_pose publish
------------------------------------------------------------
*/
void UtmLocalizer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    double utm_x, utm_y;

    if(msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN){
        double cov_x = msg->position_covariance[0];
        double cov_y = msg->position_covariance[4];
        double horizontal_cov = std::sqrt(cov_x + cov_y);

        if(horizontal_cov > covariance_threshold_){
            RCLCPP_WARN(this->get_logger(), "GPS covariance too large. cov=%.3f -> ignored",
            horizontal_cov);
            return;
        }
    }

    // 1. 위경도 → UTM 변환
    latLonToUTM(msg->latitude, msg->longitude, utm_x, utm_y);

    // 2. 최초 한 번 origin 설정
    if (!origin_set_) {
        origin_x_ = utm_x;
        origin_y_ = utm_y;
        origin_set_ = true;

        RCLCPP_INFO(this->get_logger(), "Origin set.");
    }

    // 3. origin 기준 상대좌표 계산
    double local_x = utm_x - origin_x_;
    double local_y = utm_y - origin_y_;

    // 4. GPS 튐 필터 (이전 좌표와 거리 비교)
    if (has_prev_) {
        double dx = local_x - prev_x_;
        double dy = local_y - prev_y_;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist > jump_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                        "GPS jump detected! dist=%.2f m → ignored", dist);
            return;  // 튄 값은 publish하지 않음
        }
    }

    // 5. 정상 값이면 이전 값 업데이트
    prev_x_ = local_x;
    prev_y_ = local_y;
    has_prev_ = true;

    // 6. PoseStamped 생성
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "map";   // origin 기준 전역 좌표이므로 map이 더 적절

    pose.pose.position.x = local_x;
    pose.pose.position.y = local_y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.w = 1.0;  // yaw는 아직 사용 안함

    // 7. publish
    pose_pub_->publish(pose);
}


/*
------------------------------------------------------------
latLonToUTM
- 위도(lat), 경도(lon)를 UTM 좌표(x,y)로 변환
- 현재는 한국 기준 zone 52 고정
- 단순 Transverse Mercator 근사식 사용
------------------------------------------------------------
*/
void UtmLocalizer::latLonToUTM(double lat, double lon, double &x, double &y){
    // WGS84 기준 상수
    const double a = 6378137.0;              // 지구 장반경
    const double f = 1 / 298.257223563;      // 편평률
    const double k0 = 0.9996;                // UTM scale factor

    const double e = sqrt(f * (2 - f));

    // 위도/경도 → 라디안
    const double latRad = lat * M_PI / 180.0;
    const double lonRad = lon * M_PI / 180.0;

    // 한국 UTM zone 52
    int zone = 52;
    double lonOrigin = (zone - 1) * 6 - 180 + 3;
    double lonOriginRad = lonOrigin * M_PI / 180.0;

    double N = a / sqrt(1 - pow(e * sin(latRad), 2));
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = a * ((1 - e * e / 4
        - 3 * pow(e, 4) / 64
        - 5 * pow(e, 6) / 256) * latRad);

    // 최종 UTM 좌표 계산
    x = k0 * N * A + 500000.0;
    y = k0 * M;
}