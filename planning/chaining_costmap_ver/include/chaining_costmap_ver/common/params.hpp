/**
 * @file params.hpp
 * @brief LC 플래너 파이프라인의 모든 파라미터를 관리하는 구조체
 *
 * planning_lc.yaml에서 로드되는 파라미터들을 구조체로 관리한다.
 *
 * ── 전체 구조 ──
 *   PlanningParams (최상위 구조체)
 *     ├── Costmap      : 가우시안 비용 지도 생성 파라미터
 *     ├── AStar        : A* 경로 탐색 파라미터
 *     ├── Vehicle       : T870 전동 카트의 물리적 제원 (폭, 축거, 최대 조향각)
 *     ├── Postprocess   : 생성된 경로의 리샘플링 / 스무딩 / 가지치기
 *     ├── SensorTf      : LiDAR → base_link 좌표 변환 오프셋
 *     ├── Timeouts      : 인식 데이터 유효 시간
 *     └── Chainer       : DirectionChainer v2 — 좌/우 차선 경계 체이닝
 *
 * ── 로딩 흐름 ──
 *   1. lc_planner_node 가 생성될 때 load(this) 호출
 *   2. load()가 ROS 2 파라미터 서버(declare_parameter + get_parameter)를 통해
 *      planning_lc.yaml 의 값을 읽어 이 구조체에 채움
 *   3. yaml에 값이 없으면 여기 정의된 기본값(default)이 그대로 사용됨
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__PARAMS_HPP_
#define CHAINING_COSTMAP_VER__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace chaining_costmap_ver
{

struct PlanningParams
{
  // ============================================================
  // Costmap — 가우시안 비용 지도 생성 파라미터
  //
  // 좌/우 경계 체인의 각 포인트에서 가우시안 비용을 확산시켜
  // 2D 격자 비용 지도(costmap)를 생성한다.
  //
  // 콘(CONE): flat zone(cone_radius) + 가우시안 감쇠, 비용 높음(100)
  // 차선(LANE): 가우시안 감쇠만, 비용 낮음(50) → A*가 필요시 차선을 넘을 수 있음
  //
  // 이를 통해 차선→콘 트랙 전환 구간에서 자연스러운 경로 생성 가능.
  // ============================================================
  struct Costmap
  {
    // [m] costmap X(전방) 전체 크기. base_link 기준 -size_x/2 ~ +size_x/2 범위.
    // 16.0m = 전방 8m, 후방 8m. LiDAR 탐지 범위에 맞춰 조절.
    double size_x = 16.0;

    // [m] costmap Y(좌우) 전체 크기. base_link 기준 -size_y/2 ~ +size_y/2 범위.
    // 10.0m = 좌우 각 5m. 차로 폭 1.5m 기준 충분한 범위.
    double size_y = 10.0;

    // [m/cell] 격자 해상도. 셀 하나의 실제 크기.
    // 0.15m = 차로 폭 1.5m가 약 10셀 → 경로 선택에 충분한 해상도.
    // 작게 하면 정밀하지만 셀 수 증가 → A* 탐색 시간 증가.
    double resolution = 0.15;

    // [무차원] 콘의 최대 비용 값. 가우시안 중심(또는 flat zone)에서의 비용.
    // 100.0 = 콘 근처에 높은 비용 → A*가 콘을 강하게 회피.
    double cone_cost_max = 100.0;

    // [무차원] 차선의 최대 비용 값.
    // 50.0 = 콘(100)보다 낮음 → 차선을 넘는 것이 콘을 넘는 것보다 비용이 낮음.
    // 이 차이로 인해 차선→콘 트랙 전환이 자연스럽게 이루어진다.
    double lane_cost_max = 50.0;

    // [m] 콘의 flat zone(최대 비용 유지) 반경.
    // 0.65m = PE 드럼 직경(500mm)의 약간 바깥.
    // 이 반경 이내에서는 비용이 cone_cost_max로 일정.
    // 이 반경 밖에서부터 가우시안 감쇠 시작.
    double cone_radius = 0.65;

    // [m] 가우시안 확산의 표준편차(σ).
    // cost(d) = cost_max * exp(-d² / (2σ²)).
    // 1.0m = 1σ 거리에서 비용이 약 60%로 감소. 3σ(3m)에서 거의 0.
    // 줄이면 콘/차선 근처만 높은 비용 (날카로운 장벽),
    // 키우면 넓게 퍼짐 (부드러운 경로 유도).
    double sigma = 1.0;

    // [무차원] 비용 임계값. 이 값 이하의 비용은 0으로 처리.
    // 2.0 = 가우시안 꼬리의 미세한 비용을 무시 → 연산량 절감.
    double cost_threshold = 2.0;

    // [m] ego 쪽 가상 콘 시작점의 횡방향 오프셋.
    // ego 좌측(0, +ego_y)→left_seed, ego 우측(0, -ego_y)→right_seed
    double entry_wall_ego_y = 0.3;
  } costmap;

  // ============================================================
  // AStar — A* 경로 탐색 파라미터
  //
  // costmap 위에서 8방향 격자 탐색으로 최적 경로를 찾는다.
  // g(n) = 누적 이동 비용 + costmap_cost × cost_weight
  // h(n) = 유클리드 거리 (admissible heuristic)
  // f(n) = g(n) + h(n) → min-heap으로 최소 f 우선 확장
  // ============================================================
  struct AStar
  {
    // [회] 최대 반복 횟수. 이를 초과하면 탐색 실패로 간주.
    // 10000 = costmap 해상도 0.15m에서 ~7000셀이므로 충분.
    int max_iterations = 10000;

    // [m] 골 도달 허용 오차. 현재 셀과 goal 사이 거리가 이 이내면 성공.
    // 0.3m = 2셀(0.15m) 정도의 여유.
    double goal_tolerance = 0.3;

    // [무차원] costmap 비용에 곱하는 가중치.
    // g(n) += costmap_cost × cost_weight.
    // 0.05 = costmap 비용 100인 셀을 지나면 5.0의 추가 비용.
    // 높이면 장애물 회피 강화, 낮추면 최단거리 선호.
    double cost_weight = 0.05;

    // [무차원] 통과 불가 비용 임계값. costmap 비용이 이 이상이면 벽으로 처리.
    // 80.0 = cone_cost_max(100)보다 낮아서 콘 중심 근처는 통과 불가.
    // lane_cost_max(50)보다 높아서 차선은 통과 가능.
    double obstacle_cost = 80.0;

    // [무차원] local goal 허용 최대 비용.
    // 좌/우 backbone 끝점 선분 위에서 goal 후보를 선택할 때,
    // 이 값 미만인 픽셀만 goal 후보로 허용한다.
    // obstacle_cost와 동일하게 설정하면 벽이 아닌 모든 셀을 허용.
    double goal_max_cost = 80.0;
  } astar;

  // ============================================================
  // Vehicle — T870 전동 카트 제원
  //
  // 경진대회에서 사용하는 T870 전동 카트의 물리적 제원.
  // Planner와 Postprocess에서 차량 크기를 고려한 충돌 검사,
  // 최소 회전 반경 계산 등에 사용된다.
  // ============================================================
  struct Vehicle
  {
    // [m] 차량 폭 (좌우 바퀴 사이 거리).
    // 충돌 검사 시 경로 양옆으로 width/2 만큼 여유를 확인.
    // 0.50m = 50cm.
    double width = 0.50;

    // [m] 축거(앞바퀴 중심 ~ 뒷바퀴 중심 거리).
    // Ackermann 기구학에서 최소 회전 반경 계산에 사용.
    // 0.87m = T870 카트의 실측값.
    double wheelbase = 0.73;

    // [rad] 앞바퀴 최대 조향각.
    // 0.314 rad ≈ 18° (π/10).
    // 이 값이 작을수록 최소 회전 반경이 커짐 → 좁은 곳에서 회전 불가.
    double delta_max = 0.314;

    // [m] 최소 회전 반경 계산 함수.
    // Ackermann 기구학: R_min = wheelbase / tan(delta_max).
    // 0.87 / tan(0.314) ≈ 0.87 / 0.325 ≈ 2.68m.
    // Postprocess에서 생성된 경로의 곡률이 이 반경보다 급하면
    // 실제 차량이 따라갈 수 없으므로 경로를 수정해야 한다.
    double r_min() const { return wheelbase / std::tan(delta_max); }
  } vehicle;

  // ============================================================
  // Postprocess — 경로 후처리
  //
  // AStarPlanner가 생성한 "raw 경로"를 제어기(Pure Pursuit 등)가
  // 사용할 수 있도록 정리하는 5단계 처리:
  //   1. Prune           : Douglas-Peucker 유사 단순화로 직선 구간 중간점 제거
  //   2. Resample        : 불균등한 간격의 경유점을 일정 간격(ds)으로 재배치
  //   3. Smooth          : 이동 평균(Moving Average)으로 지그재그 완화
  //   4. Curvature Clamp : 최대 곡률 제한 (차량 최소 회전 반경 보장)
  //   5. Yaw             : 접선 벡터 → atan2 헤딩 각도 계산
  // ============================================================
  struct Postprocess
  {
    // [m] 리샘플링 간격. 경유점 사이의 목표 거리.
    // 0.10m = 10cm 간격. 작을수록 경로가 촘촘 → 제어기 추종 정밀.
    // 크게 하면 경유점 수 감소 → 연산량 감소, 하지만 곡선 표현력 저하.
    double resample_ds = 0.10;

    // [개] 이동 평균 윈도우 크기 (양쪽 합산).
    // 5 = 전후 2개씩 + 자기 자신 = 5개 평균.
    // 키우면 더 부드러운 경로, 줄이면 원본에 가까운 경로.
    // 너무 크면 곡선이 안쪽으로 잘려 장애물 충돌 가능.
    int smooth_window = 5;

    // [m] 가지치기(pruning) 최대 편차.
    // 스무딩된 점이 원본 경로에서 이 거리 이상 벗어나면 제거.
    // 0.15m = 15cm. 스무딩이 과도하게 경로를 밀어낸 경우를 보정.
    double prune_max_dev = 0.15;

    // [회] curvature_clamp 반복 횟수 상한.
    // 곡률 초과 지점을 반복적으로 완화하는 최대 횟수.
    // 30 = 대부분의 경우 10회 내로 수렴하지만, 급커브가 많으면 더 필요.
    int curvature_clamp_max_iter = 30;
  } postprocess;

  // ============================================================
  // SensorTf — velodyne → base_link 좌표 오프셋
  //
  // LiDAR(velodyne) 센서의 물리적 장착 위치를 base_link 기준으로 표현.
  // 인식 결과가 velodyne 프레임으로 들어오면, 이 오프셋을 빼서
  // base_link(차량 뒷축 중심) 좌표계로 변환한다.
  //
  // 양수 방향: tf_x=전방, tf_y=좌측, tf_z=위쪽.
  // ============================================================
  struct SensorTf
  {
    // [m] LiDAR가 base_link 기준 전방으로 얼마나 떨어져 있는지.
    // yaml에서 0.0 → LiDAR가 차량 뒷축 바로 위에 장착된 경우.
    // 코드 기본값 0.7 → 70cm 전방 (실제 장착 위치에 맞게 yaml에서 오버라이드).
    double tf_x = 0.7;

    // [m] LiDAR가 base_link 기준 좌측으로 얼마나 떨어져 있는지.
    // 보통 차량 중앙에 장착하므로 0.0.
    double tf_y = 0.0;

    // [m] LiDAR가 base_link 기준 위로 얼마나 떨어져 있는지.
    // 0.7m = 지면에서 70cm 높이에 LiDAR 장착.
    // 지면 분리(Patchwork++) 결과와는 무관하고, costmap 좌표 변환에만 사용.
    double tf_z = 0.7;
  } sensor_tf;

  // ============================================================
  // Timeouts — 인식 데이터 타임아웃
  //
  // 인식(Perception) 모듈에서 받은 데이터가 얼마나 오래되면
  // "stale(유효하지 않음)"로 판단할지 결정.
  // 타임아웃 초과 시 Planner는 이전 경로를 유지하거나 정지한다.
  // ============================================================
  struct Timeouts
  {
    // [ms] 인식 데이터 유효 시간.
    // 300ms = 0.3초. LiDAR가 ~10Hz(100ms 주기)이므로
    // 300ms = 약 3 프레임 지연까지 허용.
    // 줄이면 더 신선한 데이터만 사용 (안전하지만 데이터 드랍 증가),
    // 키우면 오래된 데이터도 사용 (연속성 좋지만 지연 위험).
    int perception_ms = 300;
  } timeouts;

  // ============================================================
  // Chainer — DirectionChainer v2 파라미터
  //
  // Component → Backbone → Branch 기반 좌/우 차선 경계 체이닝.
  //
  // ── 체이닝 개요 ──
  // 인식 결과(콘 BBox, 차선 점)를 "좌측 경계"와 "우측 경계"로
  // 분류하고, 각 측면에서 일렬로 연결(chain)하는 알고리즘.
  //
  // 동작 순서:
  //   1. Seed 선택 : 차량에 가장 가까운 좌/우 포인트를 시작점으로 선택
  //   2. Backbone  : seed에서 greedy kNN으로 전방 포인트를 하나씩 연결
  //   3. Branch    : backbone에서 갈라지는 가지(분기)를 추적
  //   4. 결과      : 좌/우 경계선 → Costmap에 전달
  //
  // 비용함수: cost = alpha*d + beta*theta + gamma*lateral + delta*size_diff
  //   d       = 유클리드 거리
  //   theta   = 현재 heading과 후보 방향 사이의 각도 차이
  //   lateral = heading에 수직인 횡방향 오프셋
  //   size_diff = 콘 크기 변화 (차선 점에는 적용 안 됨)
  // ============================================================
  struct Chainer
  {
    // ── Seed 선택 ──
    // [m] |y| < side_seed_y 인 포인트는 seed 후보에서 제외.
    // 차량 바로 앞(중앙)에 있는 점은 좌/우 판별이 애매하므로 제외.
    // 0.3m = 차량 중심에서 좌우 30cm 이내는 무시.
    // 줄이면 중앙 가까운 점도 seed 가능, 키우면 확실히 좌/우인 점만 사용.
    double side_seed_y = 0.3;           ///< [m] |y| < 이 값이면 seed 후보 제외

    // ── kNN + 게이트 ──
    // [개] k-최근접 이웃(kNN) 탐색 시 후보 수.
    // 현재 chain 끝 포인트에서 가장 가까운 k개의 centroid를 후보로 선정.
    // 큰 값 = 더 많은 후보 탐색 → 정확하지만 느림.
    // 작은 값 = 빠르지만 최적 연결을 놓칠 수 있음.
    int k = 8;                          ///< centroid kNN 후보 수

    // [m] kNN 후보의 최대 허용 거리.
    // 이 거리보다 먼 포인트는 후보에서 제거.
    // 1.5m = 콘 간격이 보통 1~2m이므로 적절한 값.
    // 줄이면 가까운 것만 연결 (조밀한 콘), 키우면 듬성듬성한 콘도 연결.
    double d_max = 1.5;                 ///< [m] neighbor 최대 거리

    // [deg] 전방 탐색 원뿔의 전체 각도.
    // 120° = 좌우 ±60° 범위만 "전방"으로 인정.
    // 현재 heading 기준으로 이 각도 밖에 있는 후보는 무시.
    // 줄이면 직진 성향 강화, 키우면 급커브 대응력 향상.
    double forward_cone_deg = 120.0;    ///< [deg] 전방 cone 전체 각도 (±60°)

    // [m] 횡방향(heading에 수직) 오차 한계.
    // 후보 포인트가 현재 heading 방향에서 좌우로 이 거리 이상 벗어나면 제거.
    // 0.75m = 차로 폭(1.5m)의 절반. 같은 차선 경계선 위의 점만 연결.
    // 줄이면 엄격한 직선 연결, 키우면 곡선에서도 유연하게 연결.
    double lateral_gate = 0.75;         ///< [m] 횡방향 오차 한계

    // ── 비용함수 가중치 ──
    // 후보 중 비용이 가장 낮은 것을 다음 chain 포인트로 선택.
    // cost = alpha*d + beta*theta + gamma*lateral + delta*size_diff

    // [무차원] 유클리드 거리(d) 가중치.
    // 높이면 가까운 점을 강하게 선호. 너무 높으면 방향 무시하고 가까운 것만 연결.
    double alpha = 1.0;                 ///< 거리 비용

    // [무차원] 방향 오차(theta) 가중치.
    // 높이면 현재 진행 방향과 일치하는 점을 강하게 선호 → 직선 유지.
    // 1.2 = 거리(1.0)보다 약간 높아서 방향 일관성을 우선시.
    double beta = 1.2;                  ///< 방향 오차 비용

    // [무차원] 횡방향 오프셋 가중치.
    // 높이면 heading 직선 위에 있는 점을 선호.
    // 0.6 = 보조적 역할. 곡선 구간에서는 약간의 횡이동을 허용.
    double gamma = 0.6;                 ///< 횡오차 비용

    // [무차원] 크기 변화 비용 가중치 (콘 전용).
    // 연속된 콘의 BBox 크기가 급변하면 다른 물체일 가능성 → 패널티.
    // 차선 점(LaneBoundary)에는 크기 정보가 없으므로 적용 안 됨.
    // 0.2 = 약한 보조 역할.
    double delta = 0.2;                 ///< 크기 변화 비용 (콘 전용)

    // ── Backbone (greedy chaining) ──
    // [무차원] 좌/우측 선호도 가중치.
    // backbone 생성 시, 같은 side(좌 or 우)에 있는 후보에 보너스를 줌.
    // 0.5 = 같은 side면 비용에서 0.5만큼 감소 → 같은 편 포인트 선호.
    // 키우면 side 전환 억제, 줄이면 side 구분 약해짐.
    double lambda_side = 0.5;           ///< side preference 가중치

    // ── Branch ──
    // [개] 하나의 branch에서 최대로 연결할 포인트 수.
    // branch가 너무 길어지면 잘못된 연결일 가능성 → 제한.
    // 40 = 리샘플 간격 0.1m 기준 최대 4m 길이의 branch.
    int max_branch_len = 40;            ///< branch 최대 길이 (포인트 수)

    // ── 종료/제한 ──
    // [개] backbone의 최대 포인트 수.
    // 100 = 리샘플 간격 0.1m 기준 최대 10m의 backbone.
    // costmap size에 맞춰 조절. 맵이 16m이면 8m 전방까지 = 80개면 충분.
    int max_chain_len = 100;            ///< backbone 최대 길이

    // ── 리샘플링 ──
    // [m] 체이닝 결과를 일정 간격으로 리샘플링하는 거리.
    // 0.1m = 10cm 간격. costmap에 그릴 때 균일한 밀도를 보장.
    double resample_ds = 0.1;           ///< [m] component 전체 리샘플 간격

    // ── 디버그 ──
    // true = RViz2에 디버그용 Marker(체인 시각화 등)를 퍼블리시.
    // 개발/튜닝 중에는 true, 본 대회에서는 false로 연산량 절약.
    // chain 통계 로그(L_comp/R_comp 등)도 이 플래그에 의해 제어됨.
    bool publish_debug = true;          ///< 디버그 마커 + 체인 통계 로그 발행 여부
  } chainer;

  // ============================================================
  // load() — ROS 2 파라미터 서버에서 값 읽기
  //
  // ── 호출 시점 ──
  //   lc_planner_node의 생성자에서 한 번 호출된다.
  //
  // ── 동작 원리 ──
  //   ROS 2의 파라미터 시스템은 2단계로 동작한다:
  //     1단계. declare_parameter(name, default_val)
  //       → 노드에 "이 파라미터가 존재한다"고 등록.
  //       → yaml 파일에 해당 이름의 값이 있으면 yaml 값을 사용.
  //       → yaml에 없으면 default_val(= 구조체 멤버 기본값)을 사용.
  //     2단계. get_parameter(name).get_value<T>()
  //       → 1단계에서 결정된 값을 가져와 구조체에 저장.
  //
  // ── 파라미터 이름 규칙 ──
  //   yaml에서 "costmap:" 아래의 "size_x:" 는
  //   코드에서 "costmap.size_x" 라는 '.'으로 구분된 이름이 된다.
  //   이는 ROS 2의 '.' 네이밍 컨벤션(parameter namespace hierarchy).
  //
  // ── yaml 파일 경로 ──
  //   planning_lc.yaml 파일이 launch 파일에서 로드됨.
  //   config/planning_lc.yaml → lc_planner_node.ros__parameters 아래에 값 정의.
  //
  // ── 런타임 변경 ──
  //   현재 load()는 생성 시 1회만 호출하므로 런타임 변경 불가.
  //   동적 파라미터 변경이 필요하면 add_on_set_parameters_callback()을 추가해야 함.
  // ============================================================
  void load(rclcpp::Node * node)
  {
    // 헬퍼 람다: declare + get 을 한 줄로 처리.
    //   - name       : yaml의 파라미터 경로 (예: "costmap.size_x")
    //   - default_val: yaml에 값이 없을 때 사용할 기본값 (= 구조체 멤버 초기값)
    //   - 반환값     : yaml 또는 기본값에서 읽어온 실제 값
    //
    // decltype(default_val) 을 통해 타입을 자동 추론하므로,
    // double/int/bool/string 등을 동일한 패턴으로 처리할 수 있다.
    auto p = [&](const std::string & name, auto default_val) {
      node->declare_parameter(name, rclcpp::ParameterValue(default_val));
      return node->get_parameter(name).get_value<decltype(default_val)>();
    };

    // ── Costmap 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.costmap.*
    costmap.size_x         = p("costmap.size_x",         costmap.size_x);
    costmap.size_y         = p("costmap.size_y",         costmap.size_y);
    costmap.resolution     = p("costmap.resolution",     costmap.resolution);
    costmap.cone_cost_max  = p("costmap.cone_cost_max",  costmap.cone_cost_max);
    costmap.lane_cost_max  = p("costmap.lane_cost_max",  costmap.lane_cost_max);
    costmap.cone_radius    = p("costmap.cone_radius",    costmap.cone_radius);
    costmap.sigma          = p("costmap.sigma",          costmap.sigma);
    costmap.cost_threshold = p("costmap.cost_threshold", costmap.cost_threshold);
    costmap.entry_wall_ego_y  = p("costmap.entry_wall_ego_y",  costmap.entry_wall_ego_y);

    // ── AStar 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.astar.*
    astar.max_iterations  = p("astar.max_iterations",  astar.max_iterations);
    astar.goal_tolerance  = p("astar.goal_tolerance",  astar.goal_tolerance);
    astar.cost_weight     = p("astar.cost_weight",     astar.cost_weight);
    astar.obstacle_cost   = p("astar.obstacle_cost",   astar.obstacle_cost);
    astar.goal_max_cost   = p("astar.goal_max_cost",   astar.goal_max_cost);

    // ── Vehicle 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.vehicle.*
    // 주의: r_min()은 함수이므로 파라미터로 로드하지 않고 width, wheelbase, delta_max에서 계산.
    vehicle.width     = p("vehicle.width",     vehicle.width);
    vehicle.wheelbase = p("vehicle.wheelbase", vehicle.wheelbase);
    vehicle.delta_max = p("vehicle.delta_max", vehicle.delta_max);

    // ── Postprocess 파라미터 로드 ──
    postprocess.resample_ds   = p("postprocess.resample_ds",   postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);
    postprocess.curvature_clamp_max_iter = p("postprocess.curvature_clamp_max_iter", postprocess.curvature_clamp_max_iter);

    // ── SensorTf 파라미터 로드 ──
    // 주의: 코드 기본값(tf_x=0.7)과 yaml 값(tf_x=0.0)이 다를 수 있음.
    // yaml이 로드되면 yaml 값이 우선.
    sensor_tf.tf_x = p("sensor_tf.tf_x", sensor_tf.tf_x);
    sensor_tf.tf_y = p("sensor_tf.tf_y", sensor_tf.tf_y);
    sensor_tf.tf_z = p("sensor_tf.tf_z", sensor_tf.tf_z);

    // ── Timeouts 파라미터 로드 ──
    timeouts.perception_ms = p("timeouts.perception_ms", timeouts.perception_ms);

    // ── Chainer (v2 DirectionChainer) 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.chainer.*
    chainer.side_seed_y       = p("chainer.side_seed_y",       chainer.side_seed_y);
    chainer.k                 = p("chainer.k",                 chainer.k);
    chainer.d_max             = p("chainer.d_max",             chainer.d_max);
    chainer.forward_cone_deg  = p("chainer.forward_cone_deg",  chainer.forward_cone_deg);
    chainer.lateral_gate      = p("chainer.lateral_gate",      chainer.lateral_gate);
    chainer.alpha             = p("chainer.alpha",             chainer.alpha);
    chainer.beta              = p("chainer.beta",              chainer.beta);
    chainer.gamma             = p("chainer.gamma",             chainer.gamma);
    chainer.delta             = p("chainer.delta",             chainer.delta);
    chainer.lambda_side       = p("chainer.lambda_side",       chainer.lambda_side);
    chainer.max_branch_len    = p("chainer.max_branch_len",    chainer.max_branch_len);
    chainer.max_chain_len     = p("chainer.max_chain_len",     chainer.max_chain_len);
    chainer.resample_ds       = p("chainer.resample_ds",       chainer.resample_ds);
    chainer.publish_debug     = p("chainer.publish_debug",     chainer.publish_debug);

    // 로드 완료 시 주요 파라미터를 로그에 출력.
    // 디버깅 시 "yaml 값이 제대로 반영됐는지" 확인하는 데 유용.
    RCLCPP_INFO(
      node->get_logger(),
      "LC PlanningParams loaded: Costmap(%.0fx%.0f res=%.2f) AStar(iter=%d tol=%.1f) d_max=%.1f lat_gate=%.2f",
      costmap.size_x, costmap.size_y, costmap.resolution,
      astar.max_iterations, astar.goal_tolerance,
      chainer.d_max, chainer.lateral_gate);
  }
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__PARAMS_HPP_
