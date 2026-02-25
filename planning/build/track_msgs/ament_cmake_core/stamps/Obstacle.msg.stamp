# =============================================================
# track_msgs/msg/Obstacle.msg
# =============================================================
# 단일 장애물 (정적 또는 동적) — 현재 미사용, 향후 확장용
#
# 용도 (예정):
#   - PE 드럼, 보행자 등 장애물의 위치/크기/속도 정보
#   - 장애물 회피 플래너에서 사용 예정
#
# 대회 규격 참고:
#   - PE 드럼: 직경 500mm, 높이 840mm
#   - 교통 콘: 현재 Cone.msg로 별도 처리
# =============================================================

geometry_msgs/Point position        # 장애물 중심 좌표 (x, y, z) [m]
geometry_msgs/Vector3 dimensions    # 장애물 크기 (width, depth, height) [m]
geometry_msgs/Vector3 velocity      # 추정 속도 (vx, vy, vz) [m/s] — 동적 장애물용

# 장애물 유형 상수
uint8 TYPE_STATIC  = 0              # 정적 장애물 (PE 드럼, 고정 장벽 등)
uint8 TYPE_DYNAMIC = 1              # 동적 장애물 (보행자, 다른 차량 등)
uint8 type                          # 이 장애물의 유형
