# =============================================================
# track_msgs/msg/ConeArray.msg
# =============================================================
# 콘 배열 — 한 프레임에서 검출된 모든 콘을 묶어서 전달
#
# 토픽: /perception/cones
# 발행: Perception 노드 (LiDAR DBSCAN 클러스터링 결과)
# 구독: LocalPlannerNode → parse_cones()에서 좌/우 분리
#
# 처리 흐름:
#   Perception → ConeArray 발행
#     → LocalPlannerNode.parse_cones()
#       → cone.position.y >= 0 → cone_left (좌측 콘)
#       → cone.position.y <  0 → cone_right (우측 콘)
#     → CorridorBuilder.build() 입력으로 전달
# =============================================================

std_msgs/Header header              # 타임스탬프 + 좌표계 프레임 (base_link)
Cone[] cones                        # 검출된 콘 배열 (가변 길이)
