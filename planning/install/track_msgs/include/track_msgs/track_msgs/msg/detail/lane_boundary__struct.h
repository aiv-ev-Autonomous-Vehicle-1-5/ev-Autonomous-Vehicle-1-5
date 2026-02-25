// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LEFT'.
/**
  * 좌/우 상수 및 필드
  * 좌측 차선 경계 상수
 */
enum
{
  track_msgs__msg__LaneBoundary__LEFT = 0
};

/// Constant 'RIGHT'.
/**
  * 우측 차선 경계 상수
 */
enum
{
  track_msgs__msg__LaneBoundary__RIGHT = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LaneBoundary in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/LaneBoundary.msg
  * =============================================================
  * 단일 차선 경계선 — 순서대로 정렬된 경계점 배열
  *
  * 용도:
  *   - 카메라 기반 차선 인식 결과로 검출된 경계선을 표현
  *   - CorridorBuilder에서 좌/우 경계 체이닝의 입력으로 사용
  *   - 콘(Cone)이 없는 구간에서 경계 대체 역할
  *
  * 좌/우 구분:
  *   - side == LEFT (0)  : 좌측 차선 경계
  *   - side == RIGHT (1) : 우측 차선 경계
  *   - parse_lanes()에서 side 필드 기반으로 lane_left / lane_right 분리
  *
  * 대회 규격:
  *   - 차선: 10cm 흰색 테이프, 5cm 검은색 경계선
  *   - 차로 폭: 1.5m
  * =============================================================
 */
typedef struct track_msgs__msg__LaneBoundary
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 순서 정렬된 경계점 배열 — 차량에서 가까운 점부터
  geometry_msgs__msg__Point__Sequence points;
  /// 이 경계의 좌/우 구분 (LEFT 또는 RIGHT)
  uint8_t side;
  /// 검출 신뢰도 (0.0 ~ 1.0) — 향후 가중치 적용 가능
  float confidence;
} track_msgs__msg__LaneBoundary;

// Struct for a sequence of track_msgs__msg__LaneBoundary.
typedef struct track_msgs__msg__LaneBoundary__Sequence
{
  track_msgs__msg__LaneBoundary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__LaneBoundary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
