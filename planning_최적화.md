종합 병목 분석 및 최적화 방안
공통 발견: bbox_cost_max 100→1000 변경이 핵심 원인
costmap, entry 모두에서 bbox_cost_max=1000 (기본값 100의 10배)이 가우시안 유효 반경을 2.80m→3.53m으로 늘려 패치 면적이 58% 증가한 것이 전체 성능 저하의 가장 큰 원인.

Stage별 최적화 요약
1. Costmap (58.9%) — apply_source() 이중 루프
우선순위	방안	변경 범위	예상 효과
P0	cost_threshold 2→10~20 상향 (yaml만)	yaml 1줄	30-50% 감소
P1	원형 클리핑 if(d2 > r2) continue 추가	코드 1줄	15-20% 감소
P2	포화 셀 조기 스킵 if(grid[idx]>=cost_max) continue	코드 1줄	10-30% 감소
P3	sqrt 제거 (d² 기반 flat zone 판정)	코드 수줄	15-25% 감소
P4	row-level 원형 col 범위 클리핑	코드 5줄	5-10% 감소
P5	fprintf/fflush 제거	코드 삭제	1-3% 감소
2. Entry (12.4%) — apply_entry_walls() 가상 bbox 반복
우선순위	방안	변경 범위	예상 효과
P0	step 0.3m→1.0m 증가	코드 1줄	60-70% 감소
P1	entry wall 전용 cost_max 별도 설정 (낮추기)	코드 수줄	30% 감소
P2	직선 래스터화 방식으로 교체 (장기)	함수 재작성	80% 감소
3. A* (6.9%) — plan() 매 호출 배열 재할당 + 비효율적 탐색
우선순위	방안	변경 범위	예상 효과
P0	g_score/parent를 멤버 변수로 재사용 (generation counter)	코드 수십줄	0.1-0.5ms 절감
P1	Octile Distance 휴리스틱으로 변경 (sqrt 제거)	코드 3줄	탐색 노드 20-40% 감소
P2	closed set 배열 추가 (중복 삽입 방지)	코드 5줄	open set 크기 제한
P3	휴리스틱 인라인 (idx→row,col 나눗셈 제거)	코드 수줄	0.1-0.3ms
4. Post (6.2%) — curvature_clamp max_iter 과다 설정
우선순위	방안	변경 범위	예상 효과
P0	curvature_clamp_max_iter 500→30 (yaml만)	yaml 1줄	P99 3-4ms 감소
P1	curvature_clamp 진동 감지 조기 종료	코드 5줄	P99 0.5-1ms 추가
P2	curvature_clamp sqrt를 제곱 비교로 대체	코드 수줄	0.2ms
즉시 적용 가능한 Quick Wins (yaml/코드 최소 변경)
#	대상	변경	예상 total 영향
1	costmap cost_threshold	yaml: 2→15	평균 -3~5ms
2	post curvature_clamp_max_iter	yaml: 500→30	P99 -3~4ms
3	entry wall step	코드 1줄: resolution*2 → 1.0	평균 -1~1.5ms
4	costmap 원형 클리핑	코드 1줄 추가	평균 -1~2ms
5	costmap 포화 셀 스킵	코드 1줄 추가	평균 -0.5~1ms
1+2번만 yaml 수정으로 평균 total 16.7ms → ~12ms, P99 45ms → ~30ms 수준으로 개선 예상.
1~5번 모두 적용 시 평균 ~8-10ms, P99 ~20-25ms 달성 가능 예상.

어떤 것부터 적용할까요? yaml 파라미터 변경만 먼저 할지, 코드 수정까지 같이 갈지 알려주세요.