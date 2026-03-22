# t870_rqt_plugin 패키지 파이프라인

## 패키지 정보
- **Package**: t870_rqt_plugin
- **Description**: Henes T870 ROS2 RQT GUI 플러그인 (제어 패널 + 피드백 모니터)

---

## 플러그인

### 1. ControlPanelPlugin (t870_control_panel 노드)

수동 제어용 RQT GUI 플러그인.

#### 기능
- 속도 슬라이더 (0~1.6 m/s)
- 조향 슬라이더 (-20~20 deg)
- 모드 선택 (수동/자동)
- E-Stop 제어
- 기어 선택
- 20ms 타이머로 제어 명령 발행

#### 데이터 흐름

```
속도/조향 슬라이더 입력
  -> 20ms 타이머 (50Hz)
  -> /t870/control_command 발행

모드/E-Stop/기어 변경
  -> /t870/mode_command 서비스 요청
```

---

### 2. FeedbackMonitorPlugin (t870_feedback_monitor 노드)

차량 피드백 실시간 모니터링용 RQT GUI 플러그인.

#### 기능
- 차량 피드백 실시간 모니터링
- 파라미터 표시 (serial_bridge 또는 gazebo_bridge 노드)
- 20ms spin_some

#### 데이터 흐름

```
/t870/feedback 구독
  -> GUI 상태 표시 갱신 (모드, E-Stop, 기어, 속도, 조향, 하트비트)
```
