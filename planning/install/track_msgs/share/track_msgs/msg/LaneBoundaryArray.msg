# =============================================================
# track_msgs/msg/LaneBoundaryArray.msg
# =============================================================
# 차선 경계 배열 — 한 프레임에서 인식된 모든 차선 경계를 묶어서 전달
#
# 토픽: /perception/lane_boundaries
# 발행: Perception 노드 (카메라 차선 인식 결과)
# 구독: LocalPlannerNode → parse_lanes()에서 좌/우 분리
#
# 처리 흐름:
#   Perception → LaneBoundaryArray 발행
#     → LocalPlannerNode.parse_lanes()
#       → boundary.side == LEFT  → lane_left (좌측 차선 경계점)
#       → boundary.side == RIGHT → lane_right (우측 차선 경계점)
#     → CorridorBuilder.build() 입력으로 전달
#
# 참고:
#   - boundaries 배열에는 보통 2개(좌1, 우1)의 경계가 포함
#   - 한쪽 차선만 인식된 경우 1개만 포함될 수 있음
# =============================================================

std_msgs/Header header              # 타임스탬프 + 좌표계 프레임 (base_link)
LaneBoundary[] boundaries           # 차선 경계 배열 (가변 길이)
