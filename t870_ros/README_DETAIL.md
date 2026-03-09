# t870_ros 상세 문서

Henes T870 전동차 플랫폼을 ROS 2에서 제어하기 위한 드라이버 패키지입니다.
시리얼 통신을 통해 차량의 속도/조향 제어 및 상태 피드백을 처리하며, RQT 기반 GUI 플러그인을 제공합니다.

---

## 패키지 구조

```
t870_ros/
├── src/
│   ├── t870_cmake/          # CMake 헬퍼 매크로 (C++17, 컴파일 옵션)
│   ├── t870_msgs/           # 커스텀 메시지/서비스 정의
│   ├── t870_serial/         # 시리얼 통신 드라이버 (핵심 패키지)
│   ├── t870_rqt_plugin/     # RQT GUI 플러그인 (제어 패널 + 피드백 모니터)
│   └── t870_util/           # 유틸리티 (로깅, 예외 처리)
├── docker/                  # Docker 빌드 파일
└── .github/workflows/       # CI/CD (colcon build, Docker build)
```

---

## 시스템 아키텍처

```
┌─────────────────────────┐
│   ROS 2 Application     │
│   (Planning / Control)  │
└────────────┬────────────┘
             │
      ┌──────▼──────┐
      │  ROS 2 I/F  │
      └──────┬──────┘
             │
┌────────────▼────────────┐
│   SerialBridge Node     │  ← 50Hz (20ms) 주기 통신
│   (serial_bridge)       │
├─────────────────────────┤
│  Sub: /t870/control_command (ControlCommand)
│  Pub: /t870/feedback        (Feedback)
│  Srv: /t870/mode_command    (ModeCommand)
└────────────┬────────────┘
             │
      ┌──────▼──────┐
      │  SerialPort  │  ← POSIX termios (8N1)
      │  /dev/ttyXXX │
      └──────┬──────┘
             │
      ╔══════▼══════╗
      ║  T870 PCU   ║  ← 115200 baud (기본)
      ╚═════════════╝
```

---

## 패키지별 상세 설명

### 1. t870_msgs (메시지/서비스 정의)

T870 플랫폼과의 ROS 2 통신 인터페이스를 정의합니다.

#### ControlCommand.msg

```
float64 speed      # 목표 속도 (m/s)
float64 steering   # 목표 조향각 (rad)
```

#### Feedback.msg

```
std_msgs/Header header
bool    manual_mode       # true=수동, false=자동
bool    emergency_stop    # 긴급 정지 상태
uint8   GEAR_FORWARD=0
uint8   GEAR_NEUTRAL=1
uint8   GEAR_BACKWARD=2
uint8   gear              # 현재 기어
float64 speed             # 현재 속도 (m/s)
float64 steering          # 현재 조향각 (rad)
uint8   heartbeat         # 헬스 체크 카운터 (0~255 순환)
```

#### ModeCommand.srv

```
# Request
bool  manual_mode     # 수동 모드 설정
bool  emergency_stop  # 긴급 정지 설정
uint8 gear            # 기어 선택 (0=전진, 1=중립, 2=후진)
---
# Response
bool success          # 처리 결과
```

---

### 2. t870_serial (시리얼 통신 드라이버)

T870 PCU와 시리얼 포트를 통해 통신하며, ROS 2 메시지와 차량 프로토콜 간 변환을 담당합니다.

#### 핵심 클래스

| 클래스 | 파일 | 역할 |
|--------|------|------|
| `SerialPort` | `serial_port.hpp/cpp` | POSIX API 기반 시리얼 포트 관리 (open, read, write) |
| `SerialBridge` | `serial_bridge.hpp/cpp` | ROS 2 노드. 토픽/서비스 처리 + 시리얼 패킷 변환 |

#### 시리얼 프로토콜 (TX/RX 공통, 13바이트)

| Index | 필드            | 설명                          |
|-------|----------------|-------------------------------|
| 0     | STX_S          | 시작 바이트 `0x53` ('S')       |
| 1     | STX_T          | 시작 바이트 `0x54` ('T')       |
| 2     | STX_X          | 시작 바이트 `0x58` ('X')       |
| 3     | CONTROL_MODE   | 0=수동, 1=자동                 |
| 4     | EMERGENCY_STOP | 긴급 정지 플래그               |
| 5     | GEAR           | 0=전진, 1=중립, 2=후진         |
| 6-7   | SPEED          | 속도 (2바이트 정수)            |
| 8-9   | STEERING       | 조향 (2바이트 정수)            |
| 10    | HEARTBEAT      | 헬스 체크 카운터               |
| 11-12 | ETX            | 종료 바이트 `0x0D 0x0A` (CR+LF)|

#### 변환 계수

| 상수        | 값                  | 용도           |
|------------|---------------------|---------------|
| MPS2BYTE   | 555.555555556       | m/s → raw     |
| RAD2BYTE   | -5729.57795131      | rad → raw     |
| BYTE2MPS   | 0.00019634954       | raw → m/s     |
| BYTE2RAD   | -0.00017453292      | raw → rad     |

#### SerialBridge 동작 흐름

1. **초기화**: 파라미터 로드 → SerialPort 생성 → 포트 오픈 → 타이머/퍼블리셔/서브스크라이버/서비스 생성
2. **timer_callback() (20ms 주기)**:
   - `transmit_command()` → TX 패킷을 시리얼 포트로 전송
   - `receive_feedback()` → RX 패킷 수신 → STX/ETX 검증 → Feedback 메시지 퍼블리시
   - heartbeat 카운터 증가
3. **control_command_callback()**: 수신한 ControlCommand 메시지의 속도/조향 값을 clamp 후 raw 바이트로 변환하여 TX 패킷에 저장
4. **mode_command_callback()**: ModeCommand 서비스 요청에 따라 TX 패킷의 제어모드/긴급정지/기어 필드 업데이트

#### 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `port_path` | `/dev/ttyUSB0` | 시리얼 포트 경로 |
| `baud_rate` | `115200` | 보드레이트 (9600 또는 115200) |
| `max_speed_mps` | `1.60` | 최대 속도 제한 (m/s) |
| `max_steering_deg` | `18.00` | 최대 조향각 제한 (도) |
| `steering_offset_deg` | `0.00` | 조향 오프셋 보정 (도) |

---

### 3. t870_rqt_plugin (RQT GUI 플러그인)

Qt5 기반 RQT 플러그인으로 차량 제어 및 모니터링 GUI를 제공합니다.

#### Control Panel Plugin

차량을 수동으로 제어하기 위한 패널입니다.

- **제어 모드 선택**: Manual / Auto 라디오 버튼
- **속도 제어**: 슬라이더 + 스핀박스 (0.0 ~ 1.6 m/s)
- **조향 제어**: 슬라이더 + 스핀박스 (-20.0 ~ 20.0 도, 내부적으로 rad 변환)
- **기어 선택**: Forward / Neutral / Backward
- **긴급 정지**: E-Stop 체크박스
- **Apply Mode 버튼**: `/t870/mode_command` 서비스 호출

ControlCommand 토픽은 20ms(50Hz) 타이머로 자동 퍼블리시됩니다.

#### Feedback Monitor Plugin

차량 상태를 실시간으로 모니터링하는 패널입니다.

- 차량 파라미터 표시 (max_speed, max_steering, steering_offset)
- 제어 모드 (Manual/Auto), 긴급 정지 상태
- 현재 기어, 속도 (m/s), 조향각 (rad)
- 하트비트 카운터

시작 시 `serial_bridge` 또는 `gazebo_bridge` 노드를 자동 탐색하여 파라미터를 읽어옵니다.

#### 스레드 모델

- ROS 2 콜백은 `MultiThreadedExecutor`에서 별도 스레드로 실행
- GUI 업데이트는 `QMetaObject::invokeMethod(Qt::QueuedConnection)`으로 Qt 메인 스레드에서 처리
- `shutdownPlugin()`에서 스레드 안전하게 종료

---

### 4. t870_util (유틸리티)

#### 로깅 시스템

ANSI 컬러 출력 및 파일 기록을 지원하는 로깅 매크로입니다.

```cpp
#include "t870_util/log.hpp"

T870_DEBUG("디버그 메시지: %d", value);    // CYAN
T870_INFO("정보 메시지: %s", name);        // GREEN
T870_WARN("경고 메시지");                  // YELLOW
T870_ERROR("에러 메시지: %f", val);        // RED

// 파일 기록 활성화
activate_log_file("output.log");
set_log_type(LogType::FILE);
```

내부적으로 `LogManager` (mutex 보호)가 `LogHandlerTerminal` 또는 `LogHandlerFile` 핸들러를 관리합니다.

#### 예외 클래스

`std::runtime_error`를 상속한 커스텀 예외 클래스입니다.

```cpp
#include "t870_util/exception.hpp"

throw t870::Exception("SerialError", "포트를 열 수 없습니다");
```

---

### 5. t870_cmake (CMake 매크로)

`t870_package()` 매크로를 제공하여 프로젝트 전체에서 일관된 빌드 설정을 적용합니다.

```cmake
find_package(t870_cmake REQUIRED)
t870_package()
```

적용 내용:
- C++17 표준
- 컴파일 경고: `-Wall -Wextra -Wpedantic`
- `ament_cmake_auto` 자동 의존성 탐색
- `ROS_DISTRO` 컴파일 정의

---

## ROS 2 인터페이스 요약

| 이름                      | 타입    | 메시지 타입      | QoS                  | 주기   |
|--------------------------|---------|-----------------|----------------------|--------|
| `/t870/control_command`  | Topic   | ControlCommand  | Reliable, KeepLast(1) | 50Hz   |
| `/t870/feedback`         | Topic   | Feedback        | Reliable, KeepLast(1) | 50Hz   |
| `/t870/mode_command`     | Service | ModeCommand     | Reliable, KeepLast(1) | on-demand |

---

## 빌드

```bash
cd ~/ev_ws/t870_ros
colcon build --symlink-install
source install/setup.bash
```

개별 패키지:

```bash
colcon build --symlink-install --packages-select t870_serial
colcon build --symlink-install --packages-select t870_rqt_plugin
```

의존성 설치:

```bash
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

---

## 실행

### 시리얼 브리지

```bash
ros2 launch t870_serial serial_bridge.launch.py
```

커스텀 파라미터:

```bash
ros2 launch t870_serial serial_bridge.launch.py \
  serial_bridge_parameter:=/path/to/custom.param.yaml
```

### RQT 플러그인

```bash
rqt
```

메뉴: `Plugins` → `Henes T870` → `Control Panel` 또는 `Feedback Monitor`

---

## 주요 설계 특징

- **ComposableNode 지원**: `rclcpp_components` 등록으로 intra-process 통신 가능
- **50Hz 고정 주기 통신**: 타이머 기반 안정적인 제어 루프
- **POSIX 시리얼 API**: termios 기반 raw mode (8N1), 13바이트 blocking read
- **Qt 스레드 안전**: QueuedConnection 패턴으로 GUI 업데이트
- **파라미터 기반 설정**: YAML 파일로 하드코딩 없이 관리
- **CI/CD**: GitHub Actions로 colcon build 및 Docker build 자동화

---

## 의존성

- ROS 2 Humble
- C++17
- rclcpp, std_msgs, rosidl_default_generators
- Qt5 (Widgets, Core) - RQT 플러그인용
- rqt_gui_cpp - RQT 플러그인용

## 라이선스

Apache License 2.0

## Author

- **Minkyu Kil** (Sejong Univ. AIV) - mgkilprg@gmail.com
