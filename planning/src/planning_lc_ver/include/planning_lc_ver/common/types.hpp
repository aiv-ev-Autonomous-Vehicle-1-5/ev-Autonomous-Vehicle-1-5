/**
 * @file types.hpp
 * @brief planning_lc_ver 패키지 전체에서 사용하는 공통 자료구조 정의
 *
 * planning_mr_ver 기반 + LineChainer를 위한 타입 확장:
 *   - PointType: 콘/차선 구분
 *   - ChainedPoint: 체이닝된 점 (타입 정보 포함)
 *   - ChainResult: 좌/우 체인 결과
 *
 * ──────────────────────────────────────────────────────────────────
 * [파이프라인 전체 흐름과 타입 매핑]
 *
 *  1) 인지 입력 (Perception)
 *     - LiDAR DBSCAN → 콘(PE 드럼/교통 콘)의 중심 좌표
 *     - 카메라 차선 인식 → 차선 경계점 좌표
 *     → 이 데이터들이 ChainPoint로 통합됨
 *
 *  2) DirectionChainer (경계 체이닝)
 *     - ChainPoint[] 입력 → Component→Backbone→Branch 추출
 *     - 좌/우 각각 SideResult로 출력
 *     - 최종 결과: DirectionChainResult (left + right SideResult)
 *
 *  3) CostmapGenerator (비용 지도 생성)
 *     - SideResult.component → ChainedPoint[]로 변환
 *     - 콘/차선 타입에 따라 서로 다른 cost 파라미터 적용
 *     - 출력: CostmapResult (2D grid 비용 지도)
 *
 *  4) MagneticPlanner (경로 탐색)
 *     - CostmapResult 위에서 greedy forward search
 *     - 출력: Point2D[] (최소 비용 경로 좌표열)
 *
 *  5) PathPostprocessor (후처리)
 *     - 경로 pruning, smoothing, resampling
 *     - 출력: PostprocessResult (최종 경로 + yaw)
 *
 *  6) SafetyChecker (안전 검사)
 *     - 곡률/속도 한계 체크 → PlannerState 결정
 * ──────────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_LC_VER__COMMON__TYPES_HPP_
#define PLANNING_LC_VER__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace planning_lc_ver
{

// ============================================================================
// 기본 타입 (Basic Types)
// — 패키지 전반에서 공통으로 사용하는 최소 단위 구조체
// ============================================================================

/**
 * @brief 2D 좌표를 나타내는 기본 점 구조체
 *
 * ego 차량 좌표계(base_link) 기준:
 *   x = 전방(+) / 후방(-)
 *   y = 좌측(+) / 우측(-)
 *
 * [왜 Point2D가 필요한가?]
 * - CostmapGenerator, MagneticPlanner, PathPostprocessor 등
 *   "센서 타입에 무관하게" 좌표만 다루는 모듈에서 사용됨.
 * - ChainedPoint나 ChainPoint는 센서 메타정보(type, confidence 등)를
 *   포함하지만, planner 이후 단계에서는 순수 좌표만 필요하므로
 *   to_point2d() 변환을 통해 이 구조체로 내려온다.
 */
struct Point2D
{
  double x = 0.0;  ///< X 좌표 [m] — 양수가 차량 전방, 음수가 후방
  double y = 0.0;  ///< Y 좌표 [m] — 양수가 좌측, 음수가 우측 (ROS 좌표계 관례)
};

// ============================================================================
// 코스트맵 결과 (Costmap Output)
// — CostmapGenerator → MagneticPlanner 사이의 인터페이스
// ============================================================================

/**
 * @brief CostmapGenerator의 출력 결과를 담는 구조체
 *
 * [역할]
 * 좌/우 경계 체인(ChainedPoint[])으로부터 2D 그리드 비용 지도를 생성한 결과.
 * MagneticPlanner가 이 코스트맵 위에서 greedy forward search를 수행한다.
 *
 * [비용 지도의 원리 — 자기장 저항 모델]
 * - 각 경계점(콘/차선)을 "자석"처럼 취급하여, 가까울수록 비용(저항)이 높다.
 * - 콘은 물리적 크기가 있으므로 flat zone(반지름 내 최대 비용)이 적용되고,
 *   차선은 점 형태라 flat zone 없이 거리 기반 감쇠만 적용된다.
 * - 플래너는 비용이 낮은 셀을 따라가므로, 자연스럽게 경계 사이의
 *   중앙(차로 중심)으로 경로가 유도된다.
 *
 * [데이터 레이아웃]
 * data[]는 1차원 배열이지만, 실제로는 rows x cols 크기의 2D 그리드.
 * grid[row][col] = data[row * cols + col] 로 접근한다 (row-major 순서).
 *
 * [좌표 변환: 그리드 인덱스 ↔ 실제 좌표]
 *   world_x = origin_x + col * resolution
 *   world_y = origin_y + row * resolution
 * origin은 그리드의 좌하단 모서리에 해당하는 실제 좌표(base_link 기준).
 */
struct CostmapResult
{
  // --- 비용 데이터 ---
  std::vector<double> data;   ///< row-major flat grid: data[row * cols + col]
                               ///< 값이 클수록 "위험"하여 플래너가 기피한다.
                               ///< 0.0 = 비용 없음(자유 공간), 1.0 = 최대 비용(장애물 위)

  // --- 그리드 크기 ---
  int rows = 0;               ///< 그리드 행 수 — y축 방향 셀 개수
  int cols = 0;               ///< 그리드 열 수 — x축 방향 셀 개수

  // --- 해상도 & 원점 ---
  double resolution = 0.05;   ///< 셀 하나의 실제 크기 [m/cell]
                               ///< 0.05m = 5cm → 차선 폭(1.5m)을 30셀로 표현 가능
  double origin_x = -5.0;     ///< 그리드 좌하단의 x 좌표 [m] (base_link 기준)
                               ///< 음수 = 차량 후방도 포함 (후방 5m까지)
  double origin_y = -5.0;     ///< 그리드 좌하단의 y 좌표 [m] (base_link 기준)
                               ///< 음수 = 차량 우측도 포함 (우측 5m까지)

  // --- 유효성 ---
  bool valid = false;          ///< true: 코스트맵 정상 생성됨, false: 입력 부족 등으로 실패
};

// ============================================================================
// 후처리 결과 (PostProcess Output)
// — PathPostprocessor → SafetyChecker / 제어기 사이의 인터페이스
// ============================================================================

/**
 * @brief PathPostprocessor의 출력 결과를 담는 구조체
 *
 * [역할]
 * MagneticPlanner가 출력한 raw 경로는 격자 단위라 지그재그가 심하다.
 * PostprocessResult는 아래 3단계 후처리를 거친 최종 경로:
 *   1) Prune  — 불필요한 중복/역방향 점 제거
 *   2) Smooth — 저역통과 필터(이동 평균 등)로 부드럽게
 *   3) Resample — 일정 간격으로 재샘플링 (제어기가 등간격 경로를 기대하므로)
 *
 * [path와 yaw의 관계]
 * path[i]와 yaw[i]는 1:1 대응. yaw[i]는 path[i]에서 path[i+1] 방향의
 * 헤딩 각도(rad, atan2 기준). 제어기(pure pursuit/stanley 등)가 조향각을
 * 계산할 때 yaw 정보를 참조한다.
 */
struct PostprocessResult
{
  std::vector<Point2D> path;  ///< 후처리 완료된 경로 좌표 [m]
                               ///< path[0]이 차량에 가장 가까운 점,
                               ///< path[N-1]이 가장 먼 미래 지점
  std::vector<double> yaw;    ///< 각 경로점의 헤딩 [rad]
                               ///< atan2(dy, dx) 기준, -π ~ +π 범위
                               ///< path와 같은 크기(size)를 가짐
  bool valid = false;          ///< true: 후처리 성공, false: 입력 경로 부족 등
};

// ============================================================================
// 플래너 상태 (Planner State)
// — SafetyChecker가 판단하여 제어기에 전달하는 상태 머신
// ============================================================================

/**
 * @brief 플래너의 현재 상태를 나타내는 열거형
 *
 * [왜 필요한가?]
 * 경로가 생성되었더라도, 그 경로가 안전한지(곡률이 너무 급하진 않은지,
 * 센서 데이터가 오래되진 않았는지 등)를 SafetyChecker가 판단한다.
 * 이 상태에 따라 제어기는 "경로 추종", "정지", "비상 정지" 등
 * 서로 다른 행동을 취한다.
 *
 * [상태 전이 예시]
 *   OK → INFEASIBLE: 전방 장애물이 너무 가까워 회피 경로를 생성 불가
 *   OK → STALE: 센서 데이터가 일정 시간 이상 갱신되지 않음
 *   STOP → OK: 정지 조건 해소 후 새 경로 생성 성공
 */
enum class PlannerState : uint8_t
{
  OK = 0,          ///< 정상 — 경로 추종 가능, 제어기에 경로 전달
  STOP = 1,        ///< 정지 — 안전한 감속 정지 (예: 경로 끝 도달, 속도 초과)
  INFEASIBLE = 2,  ///< 경로 생성 불가 — 장애물로 막혀 있는 등 해결 불가 상황
                    ///< 제어기는 현재 위치에서 비상 정지 수행
  STALE = 3        ///< 데이터 만료 — 센서 입력이 timeout 이상 갱신 안 됨
                    ///< 마지막 유효 경로를 유지하되, 속도를 점진적으로 줄임
};

// ============================================================================
// LineChainer 확장 타입
// — 콘과 차선을 구분하여 체이닝하기 위한 타입들
// — 파이프라인에서 DirectionChainer(v2)가 이 타입들을 확장하여 사용함
// ============================================================================

/**
 * @brief 경계점의 원본 타입 (콘 vs 차선)
 *
 * costmap에서 콘은 cone_cost_max + cone_radius (flat zone),
 * 차선은 lane_cost_max (flat zone 없음)로 차별 적용된다.
 *
 * [왜 콘과 차선을 구분해야 하는가?]
 * 콘(PE 드럼, 직경 500mm)은 물리적 크기가 있어서, 콘 중심에서 반지름만큼의
 * 영역은 무조건 진입 불가 → costmap에서 flat zone(최대 비용 영역)을 만든다.
 * 반면 차선(10cm 흰색 테이프)은 "넘어가면 안 되는 선"이지만 물리적 두께가
 * 거의 없으므로, flat zone 없이 선 위치에서만 높은 비용을 부여한다.
 * 이 차이를 코스트맵이 구분하려면 원본 타입 정보가 필요하다.
 */
enum class PointType : uint8_t
{
  CONE = 0,  ///< LiDAR DBSCAN 결과 (PE 드럼/교통 콘)
              ///< 물리적 크기 있음 → costmap에서 flat zone 적용
  LANE = 1   ///< 카메라 차선 인식 결과
              ///< 점 형태 → costmap에서 거리 기반 감쇠만 적용
};

/**
 * @brief 체이닝된 경계점 — 위치 + 원본 타입 정보
 *
 * LineChainer가 출력하는 점. CostmapGenerator에서 type에 따라
 * 콘/차선 각각의 cost 파라미터를 적용한다.
 *
 * [파이프라인에서의 위치]
 * DirectionChainer의 SideResult.component[]에 담긴 ChainPoint들이
 * to_chained_point()를 통해 ChainedPoint로 변환된 후,
 * CostmapGenerator에 전달된다.
 * ChainPoint보다 가벼운 구조체 — confidence, label, size 등
 * 메타정보를 버리고 (x, y, type)만 남긴 것.
 * CostmapGenerator는 이 3가지 정보만으로 비용 지도를 생성할 수 있다.
 */
struct ChainedPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< 콘/차선 구분 — costmap 비용 계산에 사용

  /// Point2D로 변환 (costmap/planner 모듈과의 호환용)
  /// 타입 정보를 버리고 순수 좌표만 필요할 때 사용
  Point2D to_point2d() const { return {x, y}; }
};

/**
 * @brief LineChainer의 출력 결과 — 좌/우 체인
 *
 * left_chain: 좌측 corridor (left seed에서 출발한 DFS chain)
 * right_chain: 우측 corridor (right seed에서 출발한 DFS chain)
 * 리샘플링까지 완료된 상태로 CostmapGenerator에 전달된다.
 *
 * [왜 좌/우를 분리하는가?]
 * 자율주행에서 "주행 가능 영역"은 좌측 경계와 우측 경계 사이의 corridor다.
 * 이 두 체인이 코스트맵의 좌/우 벽 역할을 하며, 플래너는 이 사이를 지나간다.
 * 좌/우가 따로 있어야 각각 독립적으로 체이닝 실패를 처리할 수 있다.
 * (예: 한쪽만 콘이 보이는 경우에도 한쪽 체인만으로 부분적 경로 생성 가능)
 *
 * [참고] DirectionChainer(v2)에서는 이 구조체 대신 DirectionChainResult 사용.
 * ChainResult는 v1(LineChainer) 호환을 위해 남아있다.
 */
struct ChainResult
{
  std::vector<ChainedPoint> left_chain;   ///< 좌측 경계 체인 (리샘플 완료)
                                           ///< left_chain[0]이 차량에 가장 가까운 점
  std::vector<ChainedPoint> right_chain;  ///< 우측 경계 체인 (리샘플 완료)
                                           ///< right_chain[0]이 차량에 가장 가까운 점
  bool valid = false;                      ///< 최소 한쪽 체인이라도 생성되었으면 true
};

// ============================================================================
// DirectionChainer v2 타입 (Component → Backbone → Branch)
// — v1(LineChainer)의 단순 DFS 체이닝을 개선한 방향성 체이닝 시스템
// — 3단계: (1) Connected Component 추출 → (2) Backbone 선정 → (3) Branch 분기
// ============================================================================

/**
 * @brief DirectionChainer 입력 포인트 — 위치 + 메타정보
 *
 * BBox/LaneBoundary를 통합한 내부 표현.
 * LineChainer의 ChainedPoint보다 confidence/label/size 정보가 추가됨.
 *
 * [왜 ChainedPoint보다 필드가 많은가?]
 * DirectionChainer는 체이닝 품질을 높이기 위해 추가 정보를 활용한다:
 *   - confidence: 신뢰도가 낮은 점은 체이닝 우선순위를 낮춘다
 *   - label: 같은 클러스터에서 온 점들을 그룹으로 처리할 수 있다
 *   - size_x/y: 콘의 AABB 크기 → 코스트맵에서 flat zone 반지름 결정에 활용 가능
 * 체이닝이 끝난 후 CostmapGenerator에 넘길 때는 to_chained_point()로
 * 메타정보를 제거하고 (x, y, type)만 전달한다.
 *
 * [데이터 흐름]
 *   LiDAR DBSCAN → BBox 메시지 → ChainPoint (type=CONE, label=cluster_id)
 *   카메라 차선  → LaneBoundary → ChainPoint (type=LANE, label=-1)
 */
struct ChainPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< 콘/차선 구분
  float confidence = 1.0f;    ///< 원본 신뢰도 [0.0~1.0]
                                ///< 콘: DBSCAN의 클러스터 밀도 기반 점수
                                ///< 차선: 인식 알고리즘의 confidence score
  int32_t label = -1;         ///< 원본 cluster_id
                                ///< 콘: DBSCAN이 부여한 클러스터 번호 (0, 1, 2, ...)
                                ///< 차선: -1 (클러스터 개념 없음)
  double size_x = 0.0;        ///< AABB(축 정렬 바운딩 박스) X 크기 [m]
                                ///< 콘만 유효 — PE 드럼은 약 0.5m, 교통 콘은 약 0.3m
                                ///< 차선: 0 (크기 개념 없음)
  double size_y = 0.0;        ///< AABB Y 크기 [m] (콘만 유효, 차선: 0)

  /// ChainedPoint로 변환 (CostmapGenerator 호환용)
  /// 체이닝 완료 후, 비용 지도 생성 단계에서 메타정보를 버릴 때 사용
  ChainedPoint to_chained_point() const { return {x, y, type}; }
  /// Point2D로 변환 — 순수 좌표만 필요한 경우
  Point2D to_point2d() const { return {x, y}; }
};

/**
 * @brief Undirected 그래프 — component 추출 전용 (사전 구성)
 *
 * directed edge는 사전 구성하지 않는다.
 * greedy chaining 중에 kNN + 3게이트 + w' 계산을 동적으로 수행한다.
 *
 * [왜 무방향 그래프인가?]
 * Component 추출 단계에서는 "어떤 점들이 서로 이웃인가"만 알면 된다.
 * 방향(앞→뒤)은 아직 불필요하다. BFS/DFS로 연결 컴포넌트를 찾을 때
 * 방향이 있으면 한쪽에서 도달 못하는 노드가 생기므로 무방향이 맞다.
 *
 * [3단계 chaining에서의 역할]
 *   1단계 — undirected 그래프로 Connected Component 추출
 *           (가까운 점들끼리 하나의 그룹으로 묶음)
 *   2단계 — 각 component 내에서 greedy chaining (kNN + 방향 게이트)
 *           → Backbone 선정
 *   3단계 — backbone에서 분기되는 Branch 추출
 *
 * [왜 directed edge를 사전 구성하지 않는가?]
 * 체이닝 과정에서 현재 진행 방향(heading)이 계속 변하므로,
 * "전방"의 정의가 동적으로 바뀐다. 사전에 고정된 방향 그래프를
 * 만들면 이 동적 특성을 반영할 수 없다.
 */
struct ChainingGraph
{
  std::vector<std::vector<int>> undirected;  ///< [node_index] → [이웃 node_index 리스트]
                                              ///< undirected[3] = {1, 5, 7} 이면
                                              ///< 노드 3은 노드 1, 5, 7과 연결됨
};

/**
 * @brief Branch 정보 — backbone에서 분기된 가지
 *
 * [왜 Branch가 필요한가?]
 * 실제 경기장에서 콘/차선이 항상 일직선으로 배치되지 않는다.
 * 곡선 구간이나 장애물 회피 구간에서는 경계점이 "갈래"처럼 갈라질 수 있다.
 * Backbone은 가장 긴/신뢰도 높은 주 경계선이고,
 * Branch는 backbone에서 갈라져 나간 부수적 경계선이다.
 *
 * [실전 예시]
 * 교통 콘이 3개 연속 배치된 구간에서, 일부 콘이 살짝 비틀어져 있으면
 * backbone은 주 방향을 따라가고, 비틀어진 콘들은 branch로 분류된다.
 * Branch의 점들도 코스트맵에 반영되어 경로가 해당 영역을 피하게 된다.
 *
 * [score의 의미]
 * score가 높을수록 "좋은" branch다. 현재는 누적 비용의 역수로 계산하며,
 * 낮은 비용으로 길게 이어진 branch일수록 score가 높다.
 * 주로 디버깅/시각화 목적으로 사용된다.
 */
struct BranchInfo
{
  int parent_backbone_idx = -1;              ///< backbone 배열에서 이 branch가 갈라져 나온
                                              ///< 분기점의 인덱스. backbone[parent_backbone_idx]가
                                              ///< branch의 시작점과 연결됨.
  std::vector<ChainPoint> points;            ///< branch를 구성하는 포인트 배열
                                              ///< points[0]이 backbone 분기점 바로 다음 점,
                                              ///< points[N-1]이 branch의 끝 점
  double score = 0.0;                        ///< branch 품질 점수 (누적 비용 역수 등)
                                              ///< 높을수록 신뢰도 높은 분기
};

/**
 * @brief Greedy chaining 종료 이유
 *
 * [왜 종료 이유를 기록하는가?]
 * 체이닝이 왜 끝났는지에 따라 후속 처리 전략이 달라진다:
 *   - NO_CANDIDATE/MAX_LEN: 정상 종료 → 그대로 사용
 *   - ALL_GATED: 방향 전환이 너무 급한 점만 남음 → 경고 로그
 *   - CYCLE: 루프 감지 → 루프 구간 제거 필요
 * 디버깅 시에도 "왜 체인이 짧은가?"를 빠르게 진단할 수 있다.
 *
 * [3-게이트 시스템 (ALL_GATED 관련)]
 * Greedy chaining에서 다음 후보를 선택할 때 3가지 조건(게이트)을 통과해야 함:
 *   1) 방향 게이트: 현재 진행 방향 대비 후보의 각도가 허용 범위 내인가
 *   2) 거리 게이트: 후보까지의 거리가 적절한가 (너무 멀지 않은가)
 *   3) 연속성 게이트: 체인의 부드러움이 유지되는가
 */
enum class StopReason : uint8_t
{
  NO_CANDIDATE,   ///< 전방에 후보 점이 아예 없음 (kNN 범위 내 미발견)
                   ///< 보통 도로 끝이나 센서 범위 밖에서 발생
  ALL_GATED,      ///< 후보 점은 있으나, 3-게이트(방향/거리/연속성) 모두 탈락
                   ///< 급격한 방향 전환 구간에서 주로 발생
  CYCLE,          ///< 이미 방문한 노드를 다시 방문하려 함 (루프 감지)
                   ///< 원형 배치된 콘 등에서 발생 가능
  MAX_LEN         ///< 파라미터 max_chain_len에 도달하여 강제 종료
                   ///< 무한 체이닝 방지용 안전장치
};

/**
 * @brief 한쪽 side(좌 또는 우)의 chaining 결과
 *
 * [구조 이해]
 * DirectionChainer는 좌/우 경계를 각각 독립적으로 체이닝한다.
 * SideResult는 한쪽 경계의 전체 결과를 담으며, 3가지 레벨로 구성된다:
 *
 *   component (전체)
 *   └── backbone (주 경계선)
 *       └── branches[] (가지들)
 *
 * [데이터 흐름]
 *   component[] → to_chained_point() → CostmapGenerator (실제 사용)
 *   backbone[], branches[] → RViz2 MarkerArray (디버깅/시각화)
 *
 * [seed와 goal]
 * seed: 차량에 가장 가까운 경계점 (체이닝 시작점)
 *   → 차량 바로 옆의 콘이나 차선점이 seed가 됨
 * goal: greedy chaining이 도달한 마지막 점 (자동 결정)
 *   → 센서가 볼 수 있는 가장 먼 경계점
 */
struct SideResult
{
  // --- 코스트맵 전달용 (실제 파이프라인에서 사용) ---
  std::vector<ChainPoint> component;       ///< side 전체 리샘플 포인트
                                            ///< backbone + 모든 branch의 점을 합친 뒤
                                            ///< 일정 간격으로 리샘플링한 결과.
                                            ///< CostmapGenerator에 전달되는 최종 데이터.

  // --- 디버깅/시각화용 ---
  std::vector<ChainPoint> backbone;        ///< 주 경계선: seed에서 시작하여 greedy하게
                                            ///< 이어나간 가장 긴 체인.
                                            ///< backbone[0]=seed, backbone[N-1]=goal.
                                            ///< RViz2에서 LINE_STRIP으로 시각화.
  std::vector<BranchInfo> branches;        ///< backbone에서 분기된 가지들.
                                            ///< 각 branch는 backbone의 특정 인덱스에서
                                            ///< 갈라져 나온 부수적 경계선.

  // --- 메타 정보 ---
  int seed_idx = -1;                       ///< 원본 포인트 배열에서 seed의 인덱스
                                            ///< seed는 차량에 가장 가까운 경계점
  int goal_idx = -1;                       ///< greedy chain이 도달한 마지막 노드의 인덱스
                                            ///< 체이닝이 StopReason에 의해 멈춘 위치
  StopReason stop_reason = StopReason::NO_CANDIDATE;
                                            ///< 체이닝이 왜 종료되었는지 기록
                                            ///< 디버깅 시 "왜 체인이 짧은가?" 진단용
};

/**
 * @brief DirectionChainer의 출력 결과 — 좌/우 side 전체
 *
 * [파이프라인에서의 위치]
 * DirectionChainer의 최종 출력. 이 결과가 CostmapGenerator로 전달된다.
 *   DirectionChainer → DirectionChainResult → CostmapGenerator
 *
 * [valid의 의미]
 * 최소 한쪽(left 또는 right)의 backbone이 성공적으로 생성되면 valid=true.
 * 양쪽 모두 실패하면 valid=false → CostmapGenerator를 호출하지 않고,
 * PlannerState::INFEASIBLE로 전이한다.
 *
 * [한쪽만 성공한 경우]
 * 예를 들어 좌측에만 콘이 보이는 경우, left.backbone은 있지만
 * right.backbone이 비어있을 수 있다. 이때 CostmapGenerator는
 * 좌측 경계만으로 코스트맵을 생성하고, 우측은 "열린 공간"으로 처리한다.
 * 경로는 좌측 경계를 피하면서 우측으로 약간 치우친 형태가 된다.
 */
struct DirectionChainResult
{
  SideResult left;            ///< 좌측 경계 체이닝 결과 (y > 0 방향)
  SideResult right;           ///< 우측 경계 체이닝 결과 (y < 0 방향)
  bool valid = false;         ///< 최소 한쪽 backbone 생성 성공 여부
                               ///< false이면 경로 생성 불가 → INFEASIBLE
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COMMON__TYPES_HPP_
