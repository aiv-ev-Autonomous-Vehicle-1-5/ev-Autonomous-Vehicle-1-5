// ============================================================================
// params.hpp — Pure Pursuit 파라미터 구조체 (header-only)
// ============================================================================
// ROS2 파라미터를 구조화하여 관리한다.
// load(rclcpp::Node*) 메서드로 파라미터를 declare/get 하고 멤버에 캐싱한다.
//
// 서브구조체:
//   Topics   — 토픽 이름
//   Vehicle  — 차량 물리 파라미터
//   Lookahead — 속도 연동 lookahead 범위
//   Speed    — 속도 제어 관련 파라미터
//   Safety   — 안전 제한 파라미터
// ============================================================================

#ifndef PP_CONTROLLER_CPP__COMMON__PARAMS_HPP_
#define PP_CONTROLLER_CPP__COMMON__PARAMS_HPP_

#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace pp_controller_cpp
{

struct PurePursuitParams
{
  // --- 토픽 관련 ---
  struct Topics {
    std::string path_topic{"/planning/path"};   // planning 모듈이 발행하는 경로 토픽
    std::string cmd_topic{"/t870/control_command"};  // 제어 명령 출력 토픽
  } topics;

  // --- 차량/알고리즘 파라미터 ---
  struct Vehicle {
    double wheelbase{0.87};     // T870 축간거리 [m]
    double delta_max{0.314};    // 최대 조향각 [rad] (≈18도)
  } vehicle;

  // --- Lookahead 파라미터 ---
  struct Lookahead {
    double min{0.8};            // 속도 연동 lookahead 최소값 [m]
    double max{1.6};            // 속도 연동 lookahead 최대값 [m]
    double speed_gain{0.6};     // 속도 1m/s 증가당 lookahead 증가량 [m]
  } lookahead;

  // --- 속도 제어 파라미터 ---
  struct Speed {
    double min{0.4};                    // 최소 주행 속도 [m/s]
    double max{1.2};                    // 최대 주행 속도 [m/s]
    double lateral_accel_limit{0.9};    // 곡률 기반 감속용 최대 횡가속 [m/s^2]
    double preview_distance{2.5};       // 선감속용 전방 curvature preview 거리 [m]
    double accel_rate{0.8};             // 가속 rate limit [m/s^2]
    double decel_rate{1.8};             // 감속 rate limit [m/s^2]
    int    path_length_filter_size{10}; // 경로 길이 이동 평균 윈도우 크기
    double stop_margin{1.5};            // 경로 끝 정지 여유거리 [m]
  } speed;

  // --- 안전 파라미터 ---
  struct Safety {
    double path_timeout_sec{0.5};       // 경로 타임아웃 [초]
    double min_x_target{0.05};          // 목표점 최소 전방 거리 [m]
    double emergency_decel_rate{3.0};   // 긴급 정지 감속 rate limit [m/s^2]
    int    emergency_stop_count{10};    // FAIL 연속 N회 시 긴급 감속 시작
  } safety;

  // --- CREEP 모드 ---
  // 경로 실패 시 전방에 장애물이 없으면 저속 직진
  struct Creep {
    double speed{0.2};                  // [m/s] CREEP 시 직진 속도
    double roi_x{2.0};                  // [m] 전방 판정 거리
    double roi_y{0.8};                  // [m] 좌우 판정 폭 (±)
  } creep;

  // =========================================================================
  // load: ROS2 파라미터를 declare/get 하여 멤버에 캐싱
  // =========================================================================
  // launch 파일이나 yaml에서 오버라이드 가능.
  // 예: ros2 run pp_controller_cpp pure_pursuit_relative_node
  //       --ros-args -p lookahead_min:=1.0 -p speed_max:=1.5
  // =========================================================================
  void load(rclcpp::Node * node)
  {
    // --- 토픽 ---
    node->declare_parameter<std::string>("path_topic", topics.path_topic);
    node->declare_parameter<std::string>("cmd_topic",  topics.cmd_topic);
    topics.path_topic = node->get_parameter("path_topic").as_string();
    topics.cmd_topic  = node->get_parameter("cmd_topic").as_string();

    // --- 차량 ---
    node->declare_parameter<double>("wheelbase", vehicle.wheelbase);
    node->declare_parameter<double>("delta_max", vehicle.delta_max);
    vehicle.wheelbase = node->get_parameter("wheelbase").as_double();
    vehicle.delta_max = node->get_parameter("delta_max").as_double();

    // --- Lookahead (이전 고정 lookahead 설정과의 호환용 legacy 파라미터 포함) ---
    node->declare_parameter<double>("lookahead", 1.2);
    node->declare_parameter<double>("lookahead_min", lookahead.min);
    node->declare_parameter<double>("lookahead_max", lookahead.max);
    node->declare_parameter<double>("lookahead_speed_gain", lookahead.speed_gain);

    const double legacy_lookahead = node->get_parameter("lookahead").as_double();
    lookahead.min        = node->get_parameter("lookahead_min").as_double();
    lookahead.max        = node->get_parameter("lookahead_max").as_double();
    lookahead.speed_gain = node->get_parameter("lookahead_speed_gain").as_double();

    if (lookahead.min <= 0.0) {
      lookahead.min = legacy_lookahead;
    }
    if (lookahead.max <= 0.0) {
      lookahead.max = legacy_lookahead;
    }
    if (lookahead.max < lookahead.min) {
      std::swap(lookahead.min, lookahead.max);
    }

    // --- 속도 (이전 고정 speed 설정과의 호환용 legacy 파라미터 포함) ---
    node->declare_parameter<double>("speed", 0.3);
    node->declare_parameter<double>("speed_min", speed.min);
    node->declare_parameter<double>("speed_max", speed.max);
    node->declare_parameter<double>("lateral_accel_limit", speed.lateral_accel_limit);
    node->declare_parameter<double>("preview_distance", speed.preview_distance);
    node->declare_parameter<double>("accel_rate", speed.accel_rate);
    node->declare_parameter<double>("decel_rate", speed.decel_rate);
    node->declare_parameter<int>("path_length_filter_size", speed.path_length_filter_size);

    const double legacy_speed = node->get_parameter("speed").as_double();
    speed.max = node->get_parameter("speed_max").as_double();
    if (speed.max <= 0.0) {
      speed.max = std::max(0.0, legacy_speed);
    }
    speed.min = std::clamp(node->get_parameter("speed_min").as_double(), 0.0, speed.max);
    speed.lateral_accel_limit = std::max(1e-3, node->get_parameter("lateral_accel_limit").as_double());
    speed.preview_distance    = std::max(lookahead.min, node->get_parameter("preview_distance").as_double());
    speed.accel_rate = std::max(1e-3, node->get_parameter("accel_rate").as_double());
    speed.decel_rate = std::max(1e-3, node->get_parameter("decel_rate").as_double());
    speed.path_length_filter_size = std::max(1, static_cast<int>(node->get_parameter("path_length_filter_size").as_int()));

    node->declare_parameter<double>("stop_margin", speed.stop_margin);
    speed.stop_margin = std::max(0.0, node->get_parameter("stop_margin").as_double());

    // --- 안전 ---
    node->declare_parameter<double>("path_timeout_sec", safety.path_timeout_sec);
    node->declare_parameter<double>("min_x_target", safety.min_x_target);
    node->declare_parameter<double>("emergency_decel_rate", safety.emergency_decel_rate);
    node->declare_parameter<int>("emergency_stop_count", safety.emergency_stop_count);
    safety.path_timeout_sec     = node->get_parameter("path_timeout_sec").as_double();
    safety.min_x_target         = node->get_parameter("min_x_target").as_double();
    safety.emergency_decel_rate = std::max(1e-3, node->get_parameter("emergency_decel_rate").as_double());
    safety.emergency_stop_count = std::max(1, static_cast<int>(node->get_parameter("emergency_stop_count").as_int()));

    // CREEP
    node->declare_parameter<double>("creep_speed", creep.speed);
    node->declare_parameter<double>("creep_roi_x", creep.roi_x);
    node->declare_parameter<double>("creep_roi_y", creep.roi_y);
    creep.speed = node->get_parameter("creep_speed").as_double();
    creep.roi_x = node->get_parameter("creep_roi_x").as_double();
    creep.roi_y = node->get_parameter("creep_roi_y").as_double();
  }
};

}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__COMMON__PARAMS_HPP_
