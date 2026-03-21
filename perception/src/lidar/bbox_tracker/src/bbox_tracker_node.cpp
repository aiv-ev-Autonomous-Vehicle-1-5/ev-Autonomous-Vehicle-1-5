// ============================================================================
// bbox_tracker_node.cpp — BBox 트래커 노드 구현
// ============================================================================

#include "bbox_tracker/bbox_tracker_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bbox_tracker
{

BBoxTrackerNode::BBoxTrackerNode(const rclcpp::NodeOptions & options)
  : Node("bbox_tracker_node", options)
{
  // 파라미터
  declare_parameter<double>("wheelbase", wheelbase_);
  declare_parameter<double>("min_match_dist", min_match_dist_);
  declare_parameter<int>("max_miss_count", max_miss_count_);
  declare_parameter<std::string>("input_topic", "/perception/bboxes");
  declare_parameter<std::string>("output_topic", "/tracked/bboxes");
  declare_parameter<std::string>("control_topic", "/t870/control_command");

  wheelbase_ = get_parameter("wheelbase").as_double();
  min_match_dist_ = get_parameter("min_match_dist").as_double();
  max_miss_count_ = get_parameter("max_miss_count").as_int();
  const auto input_topic = get_parameter("input_topic").as_string();
  const auto output_topic = get_parameter("output_topic").as_string();
  const auto control_topic = get_parameter("control_topic").as_string();

  // Subscriber
  sub_bboxes_ = create_subscription<ev_msgs::msg::BBoxArray>(
    input_topic, rclcpp::QoS(10).best_effort(),
    [this](ev_msgs::msg::BBoxArray::UniquePtr msg) { on_bboxes(std::move(msg)); });

  sub_control_ = create_subscription<t870_msgs::msg::ControlCommand>(
    control_topic, rclcpp::QoS(10).best_effort(),
    std::bind(&BBoxTrackerNode::on_control, this, std::placeholders::_1));

  // Publisher
  pub_tracked_ = create_publisher<ev_msgs::msg::BBoxArray>(
    output_topic, rclcpp::QoS(10).best_effort());

  pub_dbg_tracks_ = create_publisher<visualization_msgs::msg::Marker>(
    "/tracker/debug/tracks", rclcpp::QoS(1).best_effort());
  pub_dbg_predicted_ = create_publisher<visualization_msgs::msg::Marker>(
    "/tracker/debug/predicted", rclcpp::QoS(1).best_effort());

  RCLCPP_INFO(get_logger(),
    "bbox_tracker started: in=%s out=%s ctrl=%s wheelbase=%.2f max_miss=%d",
    input_topic.c_str(), output_topic.c_str(), control_topic.c_str(),
    wheelbase_, max_miss_count_);
}

// ===========================================================================
// on_control: 제어 명령 저장
// ===========================================================================
void BBoxTrackerNode::on_control(const t870_msgs::msg::ControlCommand::SharedPtr msg)
{
  last_speed_ = msg->speed;
  last_steering_ = msg->steering;
}

// ===========================================================================
// on_bboxes: 메인 트래킹 사이클 (검출 수신 시 실행)
// ===========================================================================
void BBoxTrackerNode::on_bboxes(ev_msgs::msg::BBoxArray::UniquePtr msg)
{
  const auto now = this->now();

  // dt 계산
  double dt = 0.1;  // 기본값 (첫 프레임)
  if (last_predict_time_.nanoseconds() > 0) {
    dt = std::clamp((now - last_predict_time_).seconds(), 1e-3, 0.5);
  }
  last_predict_time_ = now;

  // 1) Predict: ego-motion으로 기존 트랙 좌표 보정
  predict(dt);

  // 2) Match + Update: 새 검출과 매칭
  match_and_update(*msg, dt);

  // 3) Publish: 활성 트랙 발행
  publish_tracks(msg->header);
}

// ===========================================================================
// predict: Bicycle model로 ego-motion 보정
// ===========================================================================
// 자차가 (dx, dy, dθ)만큼 이동 → 기존 트랙은 역변환 적용
// ===========================================================================
void BBoxTrackerNode::predict(double dt)
{
  if (std::abs(last_speed_) < 1e-4) return;  // 정지 시 skip

  // Bicycle model: 자차 이동량
  const double v = last_speed_;
  const double delta = last_steering_;
  const double dtheta = (v * std::tan(delta) / wheelbase_) * dt;
  const double dx = v * std::cos(dtheta * 0.5) * dt;
  const double dy = v * std::sin(dtheta * 0.5) * dt;

  const double cos_th = std::cos(dtheta);
  const double sin_th = std::sin(dtheta);

  // 기존 트랙을 현재 base_link로 변환 (자차 이동의 역변환)
  for (auto & t : tracks_) {
    const double rx = t.x - dx;
    const double ry = t.y - dy;
    t.x =  cos_th * rx + sin_th * ry;
    t.y = -sin_th * rx + cos_th * ry;
  }
}

// ===========================================================================
// match_and_update: 검출-트랙 최근접 매칭 + 상태 갱신
// ===========================================================================
void BBoxTrackerNode::match_and_update(const ev_msgs::msg::BBoxArray & detections, double dt)
{
  const int n_det = static_cast<int>(detections.bboxes.size());
  const int n_trk = static_cast<int>(tracks_.size());
  // 동적 threshold: 속도 × dt (한 프레임 이동 거리), 최소 0.1m 보장
  const double dynamic_thresh = std::max(min_match_dist_, std::abs(last_speed_) * dt);
  const double thresh2 = dynamic_thresh * dynamic_thresh;

  // 매칭 결과: detection → track index (-1 = 미매칭)
  std::vector<int> det_to_trk(n_det, -1);
  std::vector<bool> trk_matched(n_trk, false);

  // 간단한 greedy nearest neighbor 매칭
  for (int d = 0; d < n_det; ++d) {
    const double dx_d = detections.bboxes[d].position.x;
    const double dy_d = detections.bboxes[d].position.y;

    double best_d2 = std::numeric_limits<double>::max();
    int best_t = -1;

    for (int t = 0; t < n_trk; ++t) {
      if (trk_matched[t]) continue;
      const double ddx = dx_d - tracks_[t].x;
      const double ddy = dy_d - tracks_[t].y;
      const double d2 = ddx * ddx + ddy * ddy;
      if (d2 < best_d2 && d2 < thresh2) {
        best_d2 = d2;
        best_t = t;
      }
    }

    if (best_t >= 0) {
      det_to_trk[d] = best_t;
      trk_matched[best_t] = true;
    }
  }

  // 매칭된 트랙: 검출값으로 갱신
  for (int d = 0; d < n_det; ++d) {
    if (det_to_trk[d] >= 0) {
      auto & t = tracks_[det_to_trk[d]];
      const auto & b = detections.bboxes[d];
      t.x = b.position.x;
      t.y = b.position.y;
      t.size_x = b.size_x;
      t.size_y = b.size_y;
      t.size_z = b.size_z;
      t.label = b.label;
      t.miss_count = 0;
    }
  }

  // 미매칭 트랙: miss 증가
  for (int t = 0; t < n_trk; ++t) {
    if (!trk_matched[t]) {
      tracks_[t].miss_count++;
    }
  }

  // 오래된 트랙 삭제
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const Track & t) { return t.miss_count > max_miss_count_; }),
    tracks_.end());

  // 미매칭 검출: 새 트랙 생성
  for (int d = 0; d < n_det; ++d) {
    if (det_to_trk[d] < 0) {
      const auto & b = detections.bboxes[d];
      Track t;
      t.x = b.position.x;
      t.y = b.position.y;
      t.size_x = b.size_x;
      t.size_y = b.size_y;
      t.size_z = b.size_z;
      t.label = b.label;
      t.miss_count = 0;
      t.track_id = next_track_id_++;
      tracks_.push_back(t);
    }
  }
}

// ===========================================================================
// publish_tracks: 활성 트랙을 BBoxArray + 디버그 마커로 발행
// ===========================================================================
void BBoxTrackerNode::publish_tracks(const std_msgs::msg::Header & header)
{
  // BBoxArray 발행
  ev_msgs::msg::BBoxArray out;
  out.header = header;
  out.bboxes.reserve(tracks_.size());

  for (const auto & t : tracks_) {
    ev_msgs::msg::BBox b;
    b.position.x = t.x;
    b.position.y = t.y;
    b.position.z = 0.0;
    b.size_x = t.size_x;
    b.size_y = t.size_y;
    b.size_z = t.size_z;
    b.label = t.label;
    out.bboxes.push_back(b);
  }
  pub_tracked_->publish(out);

  // 디버그 마커 (lazy)
  if (pub_dbg_tracks_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "tracked_bboxes";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.points.reserve(tracks_.size());
    m.colors.reserve(tracks_.size());

    for (const auto & t : tracks_) {
      geometry_msgs::msg::Point p;
      p.x = t.x;
      p.y = t.y;
      p.z = 0.0;
      m.points.push_back(p);

      // 검출 중 = 초록, 예측 유지 = 빨강
      std_msgs::msg::ColorRGBA c;
      c.a = 1.0f;
      if (t.miss_count == 0) {
        c.r = 0.0f; c.g = 1.0f; c.b = 0.0f;  // 초록
      } else {
        c.r = 1.0f; c.g = 0.0f; c.b = 0.0f;  // 빨강
      }
      m.colors.push_back(c);
    }
    pub_dbg_tracks_->publish(m);
  }

  // 예측 유지 중 트랙만 별도 발행 (miss_count > 0인 트랙만)
  if (pub_dbg_predicted_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "predicted_only";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.0f;
    m.color.a = 0.7f;

    for (const auto & t : tracks_) {
      if (t.miss_count > 0) {
        geometry_msgs::msg::Point p;
        p.x = t.x;
        p.y = t.y;
        p.z = 0.0;
        m.points.push_back(p);
      }
    }

    pub_dbg_predicted_->publish(m);
  }
}

}  // namespace bbox_tracker

// 컴포넌트 등록
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bbox_tracker::BBoxTrackerNode)
