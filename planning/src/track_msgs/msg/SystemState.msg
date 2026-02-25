# =============================================================
# track_msgs/msg/SystemState.msg
# =============================================================
# 시스템 전체 상태 (FSM: Finite State Machine)
# 현재 미사용, 향후 상위 시스템 통합 시 사용 예정
#
# 상태 전이 예시:
#   OFFLINE → MANUAL → AUTO_STANDBY → AUTO_ACTIVE
#   AUTO_ACTIVE → AUTO_HOLD (일시정지)
#   AUTO_ACTIVE → INFEASIBLE (경로 추종 불가)
#   * → EMERGENCY_STOP (비상 정지, 어디서든 전이 가능)
#
# 각 상태 설명:
#   OFFLINE (0)        — 시스템 비활성 (초기화 전)
#   MANUAL (1)         — 수동 조종 모드 (조이스틱/RC)
#   AUTO_STANDBY (2)   — 자율주행 대기 (경로 수신 대기 중)
#   AUTO_ACTIVE (3)    — 자율주행 활성 (경로 추종 중)
#   AUTO_HOLD (4)      — 자율주행 일시정지 (장애물/신호 대기)
#   INFEASIBLE (5)     — 경로 추종 불가 (감속 후 정지)
#   EMERGENCY_STOP (6) — 비상 정지 (즉시 정지)
# =============================================================

std_msgs/Header header              # 타임스탬프 + 좌표계 프레임

# 시스템 상태 상수 정의
uint8 OFFLINE        = 0            # 시스템 비활성
uint8 MANUAL         = 1            # 수동 조종 모드
uint8 AUTO_STANDBY   = 2            # 자율주행 대기
uint8 AUTO_ACTIVE    = 3            # 자율주행 활성 (경로 추종 중)
uint8 AUTO_HOLD      = 4            # 자율주행 일시정지
uint8 INFEASIBLE     = 5            # 경로 추종 불가
uint8 EMERGENCY_STOP = 6            # 비상 정지
uint8 state                         # 현재 시스템 상태
