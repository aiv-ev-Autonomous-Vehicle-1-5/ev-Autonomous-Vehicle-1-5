/**
 * @file params.hpp
 * @brief LC 플래너 파이프라인의 모든 파라미터를 관리하는 구조체
 *
 * planning_mr_ver 기반 + Chainer 섹션 추가.
 * planning_lc.yaml에서 로드되는 파라미터들을 구조체로 관리한다.
 *
 * ── 전체 구조 ──
 *   PlanningParams (최상위 구조체)
 *     ├── Costmap      : 2D 그리드 맵 위에 장애물/차선 비용을 그리는 설정
 *     ├── Planner      : Costmap 위에서 Greedy 전진 탐색으로 경로를 생성하는 설정
 *     ├── Vehicle       : T870 전동 카트의 물리적 제원 (폭, 축거, 최대 조향각)
 *     ├── Safety        : 장애물과의 안전 마진
 *     ├── Speed         : 최대 속도 및 횡가속도 제한
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
#ifndef PLANNING_LC_VER__COMMON__PARAMS_HPP_
#define PLANNING_LC_VER__COMMON__PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace planning_lc_ver
{

struct PlanningParams
{
  // ============================================================
  // Costmap — Magnetic Resistance costmap 파라미터
  //
  // 차량 주변에 2D 그리드(격자) 맵을 만들고, 장애물(콘)과 차선에
  // "자기 반발력(Magnetic Resistance)" 형태의 비용을 부여한다.
  // Planner는 이 비용이 낮은 곳을 따라 경로를 생성한다.
  //
  // 좌표계: base_link 기준. x가 전방, y가 좌측.
  // 맵 원점은 차량 중심이며, 맵은 [-size_x/2, +size_x/2] x [-size_y/2, +size_y/2] 범위.
  // ============================================================
  struct Costmap
  {
    // --- 맵 크기 ---
    // [m] costmap의 x방향(전후) 전체 길이.
    // 값을 키우면 더 먼 거리까지 비용을 계산하지만, 셀 수가 늘어나 연산량 증가.
    // yaml에서는 16.0 으로 설정 → 전방 8m + 후방 8m 범위.
    double size_x = 10.0;

    // [m] costmap의 y방향(좌우) 전체 길이.
    // 차로 폭이 1.5m이므로 양쪽 여유를 두고 충분히 넓게 설정해야 한다.
    double size_y = 10.0;

    // [m/cell] 그리드 한 칸의 물리적 크기.
    // 0.05 = 5cm 해상도. 작을수록 정밀하지만 메모리 & 연산량 급증.
    // 그리드 총 셀 수 = (size_x / resolution) * (size_y / resolution).
    // 예: 16m / 0.05 = 320칸 → 320 x 320 = 102,400 셀.
    double resolution = 0.05;

    // --- 비용 크기 ---
    // [무차원] 콘(장애물)이 중심에서 가지는 최대 비용값.
    // 높일수록 Planner가 콘을 더 강하게 회피한다.
    double cone_cost_max = 100.0;

    // [무차원] 차선 경계가 중심에서 가지는 최대 비용값.
    // cone_cost_max보다 작으면 → 콘 회피 > 차선 회피 우선순위.
    // 즉, 콘을 피하기 위해 차선을 약간 침범하는 것을 허용.
    double lane_cost_max = 50.0;

    // --- 비용 확산 ---
    // [m] 콘 주변에 비용이 퍼지는 물리적 반경.
    // PE 드럼 직경이 500mm(반지름 250mm)이므로, 0.65m = 드럼 가장자리 + 40cm 마진.
    double cone_radius = 0.65;

    // [m] 가우시안 확산의 표준편차. 비용이 중심에서 얼마나 부드럽게 감소하는지 결정.
    // 작을수록 비용이 장애물 가까이에만 집중, 클수록 넓게 퍼짐.
    double sigma = 1.0;

    // [무차원] 이 값 이하의 비용은 0으로 잘라냄(clamp).
    // 너무 작은 비용은 무시하여 연산량 감소 + 노이즈 제거.
    double cost_threshold = 2.0;
  } costmap;

  // ============================================================
  // Planner — Greedy 전진 탐색 파라미터
  //
  // Costmap 위에서 "탐욕적 전진 탐색(Greedy Forward Search)"으로 경로를 생성.
  // 현재 위치에서 search_radius 만큼 전방을 탐색하여
  // 비용이 가장 낮은 다음 지점을 선택하는 것을 max_steps번 반복한다.
  //
  // 동작 원리:
  //   1. 현재 heading(진행 방향) 기준으로 forward_cone 안의 후보 셀을 탐색
  //   2. 비용이 최소인 셀을 다음 경유점으로 선택
  //   3. heading을 갱신 (heading_damping으로 급격한 방향 전환 억제)
  //   4. max_steps번 반복하거나 맵 밖으로 나가면 종료
  // ============================================================
  struct Planner
  {
    // [m] 매 스텝에서 다음 경유점을 탐색하는 반경.
    // 작을수록 세밀한 경로, 클수록 대략적이지만 빠른 경로.
    // resolution(0.05m)보다 충분히 커야 의미 있는 탐색이 됨.
    double search_radius = 0.30;

    // [회] 최대 탐색 반복 횟수. 경로의 최대 길이를 결정.
    // 최대 경로 길이 ≈ search_radius * max_steps = 0.3 * 200 = 60m.
    // 코스가 긴 경우 늘려야 하고, 너무 크면 연산 시간 증가.
    int max_steps = 200;

    // [무차원] 초기 heading(진행 방향) 벡터의 x, y 성분.
    // (1.0, 0.0) = 차량 전방(+x)을 향해 출발.
    // 좌회전 코스라면 (0.7, 0.7) 등으로 초기 방향을 틀 수 있음.
    double heading_init_x = 1.0;
    double heading_init_y = 0.0;

    // [deg] 전방 탐색 원뿔(cone)의 전체 각도.
    // 60° → 좌우 ±30° 범위만 탐색. 좁을수록 직진 성향 강화.
    // yaml에서 90°로 설정 → 좌우 ±45°.
    double forward_cone_deg = 60.0;

    // [deg] 한 스텝에서 heading이 바뀔 수 있는 최대 각도.
    // 실제 차량의 조향 한계를 반영. 작을수록 부드러운 경로.
    // yaml에서 10°로 설정 → 한 스텝당 최대 10° 꺾임.
    double max_steer_per_step_deg = 30.0;

    // [0.0 ~ 1.0] heading 갱신 시 이전 heading을 얼마나 유지할지.
    // 0.0 = 완전히 새 방향, 1.0 = 이전 방향 유지(직진만 함).
    // 0.5 = 이전 heading 50% + 새 heading 50% 혼합 → 적당한 부드러움.
    // 줄이면 장애물 반응 빠름, 높이면 직진 안정성 향상.
    double heading_damping = 0.5;
  } planner;

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
    double wheelbase = 0.87;

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
  // Safety — 안전 마진
  //
  // 차량 외곽에서 추가로 확보하는 여유 거리.
  // 실제 충돌 판정 범위 = vehicle.width/2 + safety.margin.
  // ============================================================
  struct Safety
  {
    // [m] 장애물과 차량 사이 최소 여유 거리.
    // 0.10m = 10cm. 차체 폭(50cm)에 양쪽 10cm씩 추가하면
    // 총 70cm 통로가 있어야 통과 가능.
    // 좁은 콘 배치 구간에서는 줄여야 할 수 있지만,
    // 너무 작으면 센서 오차 시 충돌 위험.
    double margin = 0.10;
  } safety;

  // ============================================================
  // Speed — 속도 제한
  //
  // 경로의 곡률에 따라 속도를 조절하기 위한 파라미터.
  // 커브 구간에서는 v = sqrt(a_lat_max / curvature) 로 감속.
  // ============================================================
  struct Speed
  {
    // [m/s] 직선 구간 최대 속도.
    // 1.60 m/s ≈ 5.76 km/h. 경진대회 안전 규정에 맞춰 설정.
    // 높이면 빨라지지만, 제어 지연으로 장애물 회피 실패 가능.
    double v_max = 1.60;

    // [m/s^2] 허용 최대 횡가속도(centripetal acceleration).
    // 커브에서 v^2 / R ≤ a_lat_max 이 되도록 속도를 제한.
    // 2.0 m/s^2 = 약 0.2G. 전동 카트 타이어 그립 한계 고려.
    // 높이면 커브에서 빠르지만 미끄러질 위험.
    double a_lat_max = 2.0;
  } speed;

  // ============================================================
  // Postprocess — 경로 후처리
  //
  // Planner가 생성한 "raw 경로"를 제어기(Pure Pursuit 등)가
  // 사용할 수 있도록 정리하는 3단계 처리:
  //   1. Resample  : 불균등한 간격의 경유점을 일정 간격(ds)으로 재배치
  //   2. Smooth    : 이동 평균(Moving Average)으로 지그재그 제거
  //   3. Prune     : 원래 경로에서 너무 벗어난 점을 제거(과도한 스무딩 보정)
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
    // branch 모드 설정.
    // "backbone_and_branches" = backbone + 갈라지는 branch 둘 다 추적.
    // "backbone_only"         = backbone만 (분기 무시).
    std::string branch_mode = "backbone_and_branches";

    // [개] 하나의 branch에서 최대로 연결할 포인트 수.
    // branch가 너무 길어지면 잘못된 연결일 가능성 → 제한.
    // 40 = 리샘플 간격 0.1m 기준 최대 4m 길이의 branch.
    int max_branch_len = 40;            ///< branch 최대 길이 (포인트 수)

    // ── 종료/제한 ──
    // [개] backbone의 최대 포인트 수.
    // 100 = 리샘플 간격 0.1m 기준 최대 10m의 backbone.
    // costmap size에 맞춰 조절. 맵이 16m이면 8m 전방까지 = 80개면 충분.
    int max_chain_len = 100;            ///< backbone 최대 길이

    // ── 콘 우선순위 ──
    // 동일 영역에 콘(장애물)과 차선 점이 공존할 때의 처리.
    // true  = 콘이 있으면 그 근처의 차선 점을 제거 (콘이 더 신뢰성 높음).
    // false = 둘 다 유지 (차선 점도 살림).
    // 경진대회에서는 콘이 물리적 장애물이므로 우선시하는 것이 안전.
    bool cone_priority = true;          ///< 동일 영역에 콘+차선 공존 시 차선 제거

    // ── 신뢰도 필터 ──
    // [0.0 ~ 1.0] 인식 결과의 최소 신뢰도(confidence) 컷오프.
    // 이 값 미만의 BBox/LaneBoundary는 체이닝에서 제외.
    // 0.1 = 10% 이상이면 사용. 매우 느슨한 필터.
    // 높이면 확실한 탐지만 사용 (안전하지만 데이터 부족 가능),
    // 낮추면 노이즈가 많은 탐지도 포함.
    double min_confidence = 0.1;        ///< BBox/LaneBoundary 최소 신뢰도 컷오프

    // ── 리샘플링 ──
    // [m] 체이닝 결과를 일정 간격으로 리샘플링하는 거리.
    // 0.1m = 10cm 간격. costmap에 그릴 때 균일한 밀도를 보장.
    double resample_ds = 0.1;           ///< [m] component 전체 리샘플 간격

    // ── 디버그 ──
    // true = RViz2에 디버그용 Marker(체인 시각화 등)를 퍼블리시.
    // 개발/튜닝 중에는 true, 본 대회에서는 false로 연산량 절약.
    bool publish_debug = true;          ///< 디버그 마커 발행 여부
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
    costmap.sigma          = p("costmap.sigma",           costmap.sigma);
    costmap.cost_threshold = p("costmap.cost_threshold", costmap.cost_threshold);

    // ── Planner 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.planner.*
    planner.search_radius  = p("planner.search_radius",  planner.search_radius);
    planner.max_steps      = p("planner.max_steps",      planner.max_steps);
    planner.heading_init_x = p("planner.heading_init_x", planner.heading_init_x);
    planner.heading_init_y = p("planner.heading_init_y", planner.heading_init_y);
    planner.forward_cone_deg       = p("planner.forward_cone_deg",       planner.forward_cone_deg);
    planner.max_steer_per_step_deg = p("planner.max_steer_per_step_deg", planner.max_steer_per_step_deg);
    planner.heading_damping        = p("planner.heading_damping",        planner.heading_damping);

    // ── Vehicle 파라미터 로드 ──
    // yaml 경로: lc_planner_node.ros__parameters.vehicle.*
    // 주의: r_min()은 함수이므로 파라미터로 로드하지 않고 width, wheelbase, delta_max에서 계산.
    vehicle.width     = p("vehicle.width",     vehicle.width);
    vehicle.wheelbase = p("vehicle.wheelbase", vehicle.wheelbase);
    vehicle.delta_max = p("vehicle.delta_max", vehicle.delta_max);

    // ── Safety 파라미터 로드 ──
    safety.margin = p("safety.margin", safety.margin);

    // ── Speed 파라미터 로드 ──
    speed.v_max     = p("speed.v_max",     speed.v_max);
    speed.a_lat_max = p("speed.a_lat_max", speed.a_lat_max);

    // ── Postprocess 파라미터 로드 ──
    postprocess.resample_ds   = p("postprocess.resample_ds",   postprocess.resample_ds);
    postprocess.smooth_window = p("postprocess.smooth_window", postprocess.smooth_window);
    postprocess.prune_max_dev = p("postprocess.prune_max_dev", postprocess.prune_max_dev);

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
    chainer.branch_mode       = p("chainer.branch_mode",       chainer.branch_mode);
    chainer.max_branch_len    = p("chainer.max_branch_len",    chainer.max_branch_len);
    chainer.max_chain_len     = p("chainer.max_chain_len",     chainer.max_chain_len);
    chainer.cone_priority     = p("chainer.cone_priority",     chainer.cone_priority);
    chainer.min_confidence    = p("chainer.min_confidence",    chainer.min_confidence);
    chainer.resample_ds       = p("chainer.resample_ds",       chainer.resample_ds);
    chainer.publish_debug     = p("chainer.publish_debug",     chainer.publish_debug);

    // 로드 완료 시 주요 파라미터를 로그에 출력.
    // 디버깅 시 "yaml 값이 제대로 반영됐는지" 확인하는 데 유용.
    RCLCPP_INFO(
      node->get_logger(),
      "LC PlanningParams loaded: grid=%.0fx%.0f res=%.3f d_max=%.1f lat_gate=%.2f lambda=%.2f",
      costmap.size_x, costmap.size_y, costmap.resolution,
      chainer.d_max, chainer.lateral_gate, chainer.lambda_side);
  }
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COMMON__PARAMS_HPP_
