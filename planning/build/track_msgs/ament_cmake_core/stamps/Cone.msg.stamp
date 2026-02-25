# =============================================================
# track_msgs/msg/Cone.msg
# =============================================================
# 단일 콘(라바콘/교통콘) 정보
#
# 용도:
#   - LiDAR 기반 DBSCAN 클러스터링 결과로 검출된 개별 콘을 표현
#   - CorridorBuilder에서 좌/우 경계 체이닝의 기준점으로 사용
#   - 차선 경계점(LaneBoundary)보다 높은 우선순위로 처리됨 (Cone Priority)
#
# 좌표계:
#   - base_link 기준 (전방 +x, 좌측 +y, 위쪽 +z)
#   - position.y >= 0 이면 좌측 콘, < 0 이면 우측 콘으로 분류
#
# 대회 규격:
#   - PE 드럼: 직경 500mm, 높이 840mm
#   - 교통 콘: 최소 3개 연속 배치 (장애물 회피 구간)
# =============================================================

geometry_msgs/Point position        # 콘 중심 좌표 (x, y, z) [m] — DBSCAN 클러스터 무게중심
geometry_msgs/Vector3 dimensions    # 콘 크기 (width, depth, height) [m] — 바운딩 박스
float32 confidence                  # 검출 신뢰도 (0.0 ~ 1.0) — 현재 미사용, 향후 필터링용
int32 label                         # 클러스터 라벨 또는 클래스 ID — DBSCAN 출력
