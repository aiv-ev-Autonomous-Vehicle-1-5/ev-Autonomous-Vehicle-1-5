std_msgs/Header header              # 타임스탬프 + 좌표계 프레임 (base_link)
geometry_msgs/Point[] points        # 순서 정렬된 경계점 배열 [m] — 차량에서 가까운 점부터

float32 confidence                  # 검출 신뢰도 (0.0 ~ 1.0)
