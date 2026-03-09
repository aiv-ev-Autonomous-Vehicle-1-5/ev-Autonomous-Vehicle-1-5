/**
 * @file lc_planner_node.cpp
 * @brief LC Planner 메인 노드 구현 — DirectionChainer v2 7단계 파이프라인
 *
 * ══════════════════════════════════════════════════════════════
 *  이 파일은 lc_planner_node.hpp에 선언된 LCPlannerNode를 구현한다.
 *
 *  핵심 구조:
 *    생성자 — 파라미터 로드, QoS 설정, 구독/발행 생성, 타이머 시작
 *    on_timer() — 10Hz로 호출되는 메인 루프 (7단계 파이프라인)
 *
 *  7단계 파이프라인 (on_timer 콜백):
 *    Stage 0: Stale Gate — perception 데이터 타임아웃 검사
 *                          (오래된 데이터로 경로 생성 방지)
 *    Stage 1: Input Parse — 콘/차선 ROS 메시지 → 단일 ChainPoint 벡터
 *                           (LiDAR bbox는 sensor_tf 오프셋 보정)
 *    Stage 2: DirectionChainer — Component→Backbone→Branch + 리샘플
 *                                (포인트를 좌/우 체인으로 분류·연결)
 *    Stage 3: Costmap + A* — 가우시안 코스트맵 생성 + A* 경로 탐색
 *                            (좌/우 체인 → 코스트맵 → A* 경로 계획)
 *    Stage 5: Postprocess — prune → smooth → curvature_clamp → resample → curvature_clamp → yaw
 *                           (경로 정제: 이상치 제거, 스무딩, 곡률 제한, 등간격화, 방향각)
 *    Stage 6: Safety Check — 곡률 검사
 *                            (Menger 곡률 → 경로 실현 가능성 판정)
 *    Stage 7: Publish — 경로, 상태, 디버그 토픽 발행
 *                       (Core: 항상 발행, Debug: 구독자 있을 때만)
 * ══════════════════════════════════════════════════════════════
 */
// ── 프로젝트 내부 헤더 ──
#include "chaining_costmap_ver/nodes/chaining_costmap_ver_node.hpp"   // LCPlannerNode 클래스 선언
#include "chaining_costmap_ver/common/geometry.hpp"          // dist(), cross2(), to_path_msg() 등 기하 유틸
#include "chaining_costmap_ver/common/debug_publish.hpp"     // 디버그 시각화 헬퍼
#include "chaining_costmap_ver/safety/safety_checker.hpp"    // safety_checker::check() — 곡률 검사

// ── ROS 2 컴포넌트 등록 매크로 ──
// 이 매크로를 통해 이 노드를 shared library(.so)로 빌드하고,
// rclcpp::ComponentManager가 런타임에 동적 로딩할 수 있게 한다.
#include <rclcpp_components/register_node_macro.hpp>

// ── 표준 라이브러리 ──
#include <chrono>     // std::chrono::milliseconds (타이머 주기용)
#include <cmath>      // std::sqrt, std::abs 등
#include <algorithm>  // std::clamp, std::min, std::max 등

namespace chaining_costmap_ver
{

// ══════════════════════════════════════════════════════════════
//  생성자 — 파라미터 로드, QoS 설정, 구독/발행/타이머 초기화
// ══════════════════════════════════════════════════════════════
LCPlannerNode::LCPlannerNode(const rclcpp::NodeOptions & options)
: Node("lc_planner_node", options),  // 노드 이름: "lc_planner_node"
  // 타임스탬프 초기값을 (0초, 0나노초)로 설정
  // → 데이터를 한 번도 받기 전에는 check_stale()에서 stale로 판정됨
  stamp_lanes_(0, 0, RCL_ROS_TIME),
  stamp_bboxes_(0, 0, RCL_ROS_TIME)
{
  // yaml 파라미터 파일에서 모든 설정값을 로드한다
  // (vehicle, sensor_tf, chainer, costmap, postprocess, timeouts 등)
  params_.load(this);

  // ── QoS 설정: Best Effort, depth=1 ──
  // Best Effort: 메시지 손실을 허용하되 최신 데이터를 빠르게 수신
  // depth=1: 큐에 최대 1개만 보관 → 항상 최신 데이터만 처리
  // 자율주행에서는 오래된 데이터를 재전송 받는 것보다 최신 데이터가 중요하다
  rclcpp::QoS qos_be(1);
  qos_be.best_effort();

  // ── 구독 설정 ──
  // 콜백에서 UniquePtr(소유권 이동)을 사용하여 복사 비용을 제거한다.
  // now()로 수신 시각을 기록 → check_stale()에서 신선도 판별에 사용.

  // 차선 경계 구독: 카메라 인지 노드가 발행하는 차선 점들
  sub_lanes_ = create_subscription<ev_msgs::msg::LaneBoundaryArray>(
    "/perception/lane_boundaries", qos_be,
    [this](ev_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
      stamp_lanes_ = now();              // 수신 시각 기록
      last_lanes_ = std::move(msg);      // 소유권 이동 (zero-copy)
    });

  // 바운딩 박스 구독: LiDAR 인지 노드가 발행하는 장애물(콘/드럼) 위치
  sub_bboxes_ = create_subscription<ev_msgs::msg::BBoxArray>(
    "/perception/bboxes", qos_be,
    [this](ev_msgs::msg::BBoxArray::UniquePtr msg) {
      stamp_bboxes_ = now();             // 수신 시각 기록
      last_bboxes_ = std::move(msg);     // 소유권 이동 (zero-copy)
    });

  // ── Core 퍼블리셔: 항상 발행하는 핵심 토픽 ──
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/path", qos_be);           // 최종 후처리 경로 → 제어기가 구독
  pub_status_ = create_publisher<std_msgs::msg::String>(
    "/planning/status", qos_be);         // 플래너 상태 문자열 (OK/STALE/INFEASIBLE 등)

  // ── Debug 퍼블리셔: RViz2 시각화용 (lazy publishing) ──
  // RViz2는 Reliable QoS로 구독하므로, 디버그 토픽도 Reliable로 발행한다.
  // → on_timer()에서 get_subscription_count() > 0 체크 후 발행.
  rclcpp::QoS qos_dbg(1);
  qos_dbg.reliable();

  pub_dbg_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/debug/costmap", qos_dbg);
  pub_dbg_raw_path_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/raw_path", qos_dbg);
  pub_dbg_left_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/left_chain", qos_dbg);
  pub_dbg_right_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/right_chain", qos_dbg);
  pub_dbg_left_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/left_branches", qos_dbg);
  pub_dbg_right_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/right_branches", qos_dbg);
  pub_dbg_seeds_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/seeds", qos_dbg);
  pub_dbg_local_goal_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/local_goal", qos_dbg);
  pub_dbg_obstacle_wall_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/obstacle_wall", qos_dbg);
  pub_dbg_curvature_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/curvature", qos_dbg);

  // ── 10Hz 타이머 ──
  // wall timer: 시뮬레이션 시간이 아닌 실제 시계(wall clock) 기준
  // 100ms 주기 → 초당 10회 파이프라인 실행
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LCPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "LCPlannerNode initialized (10 Hz, DirectionChainer v2 + Costmap + A*)");
}

// ══════════════════════════════════════════════════════════════
//  Stage 1: Input Parse — ROS 메시지 → ChainPoint 벡터 변환
// ══════════════════════════════════════════════════════════════
/**
 * @brief 콘/차선 ROS 메시지를 단일 ChainPoint 벡터로 변환
 *
 * [센서 TF 오프셋 보정]
 *   LiDAR(velodyne)는 차량의 base_link와 물리적으로 다른 위치에 장착되어 있다.
 *   예를 들어, LiDAR가 base_link보다 앞으로 0.5m, 위로 0.3m 떨어져 있으면:
 *     sensor_tf.tf_x = 0.5,  sensor_tf.tf_y = 0.0
 *   bbox의 좌표(velodyne 프레임 기준)에 이 오프셋을 더해서
 *   base_link 프레임 기준으로 통일한다.
 *
 *   ※ 카메라 차선 점은 이미 base_link 기준으로 발행된다고 가정하므로
 *     별도의 TF 보정을 하지 않는다.
 *
 * [좌/우 구분]
 *   이 함수에서는 모든 점을 하나의 벡터에 섞어 넣는다.
 *   좌/우 분류는 Stage 2(DirectionChainer)에서 seed 기반으로 수행한다.
 *   → 센서 종류(CONE/LANE)에 관계없이 동일한 체이닝 로직 적용 가능
 */
void LCPlannerNode::parse_input(
  std::vector<ChainPoint> & all_pts) const
{
  // ── BBox 수집 (LiDAR 장애물: PE 드럼, 교통 콘 등) ──
  // velodyne 좌표계 → base_link 좌표계 변환을 위해 sensor_tf 오프셋 적용
  if (last_bboxes_) {
    const double ox = params_.sensor_tf.tf_x;  // x축 오프셋 (전후 방향)
    const double oy = params_.sensor_tf.tf_y;  // y축 오프셋 (좌우 방향)
    for (const auto & b : last_bboxes_->bboxes) {
      ChainPoint cp;
      cp.x = b.position.x + ox;   // velodyne_x + 오프셋 → base_link_x
      cp.y = b.position.y + oy;   // velodyne_y + 오프셋 → base_link_y
      cp.type = PointType::CONE;   // 장애물 타입
      cp.confidence = b.confidence; // 인지 신뢰도 (0.0~1.0)
      cp.label = b.label;           // DBSCAN 클러스터 라벨
      cp.size_x = b.size_x;         // 바운딩 박스 x 크기 [m]
      cp.size_y = b.size_y;         // 바운딩 박스 y 크기 [m]
      all_pts.push_back(cp);
    }
  }

  // ── 차선 점 수집 (카메라 차선 인식 결과) ──
  // 카메라 점은 이미 base_link 기준이므로 TF 보정 없이 그대로 사용
  if (last_lanes_) {
    for (const auto & bd : last_lanes_->boundaries) {
      for (const auto & p : bd.points) {
        ChainPoint cp;
        cp.x = p.x;                   // base_link 기준 x 좌표
        cp.y = p.y;                   // base_link 기준 y 좌표
        cp.type = PointType::LANE;     // 차선 타입
        cp.confidence = bd.confidence; // 차선 인식 신뢰도
        cp.label = -1;                 // 차선에는 클러스터 라벨 없음 → -1
        all_pts.push_back(cp);
      }
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  Stage 0: Stale Gate — 인지 데이터 신선도 검사
// ══════════════════════════════════════════════════════════════
/**
 * @brief 인지 데이터가 신선(fresh)한지 검사
 *
 * [로직 상세]
 *   현재 시각(t)과 각 데이터의 마지막 수신 시각(stamp_*)의 차이를 밀리초로 계산.
 *   dt = (t - stamp_*).nanoseconds() * 1e-6  →  나노초를 밀리초로 변환
 *
 *   dt <= perception_ms 이면 해당 데이터는 "fresh"
 *   dt >  perception_ms 이면 해당 데이터는 "stale"
 *
 * [OR 조건]
 *   차선 OR bbox 중 하나라도 fresh하면 → have_perception = true
 *   이유: 주행 상황에 따라 하나의 센서만 유효할 수 있다.
 *     - 직선 구간: 차선만 보이고 콘이 없을 수 있음
 *     - 장애물 구간: 콘만 있고 차선이 가려질 수 있음
 *   따라서 어느 쪽이든 데이터가 있으면 경로 계획을 시도한다.
 *
 * [한 번도 받지 못한 경우]
 *   stamp_*이 (0,0)으로 초기화되어 있으므로 dt가 매우 크게 나온다.
 *   → last_lanes_/last_bboxes_가 nullptr이면 if 블록 자체를 건너뜀
 *   → have_perception은 false 유지 → stale 판정
 *
 * @return true: 모든 인지 데이터가 stale → on_timer에서 "STALE" 발행 후 중단
 *         false: 하나 이상의 데이터가 fresh → 파이프라인 계속 진행
 */
bool LCPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  // 차선 데이터 신선도 검사
  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;  // ns → ms 변환
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  // bbox 데이터 신선도 검사
  if (last_bboxes_) {
    const double dt = (t - stamp_bboxes_).nanoseconds() * 1e-6;  // ns → ms 변환
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  // have_perception == false → 둘 다 stale → true 반환 (파이프라인 중단)
  return !have_perception;
}

// ══════════════════════════════════════════════════════════════
//  on_timer() — 10Hz 메인 루프: 7단계 파이프라인
// ══════════════════════════════════════════════════════════════
void LCPlannerNode::on_timer()
{
  // 발행할 메시지의 타임스탬프 — 모든 토픽이 동일한 시각을 사용
  const auto stamp = now();
  // 좌표 프레임: 모든 데이터는 base_link 기준
  const std::string frame_id = "base_link";

  // ======== Stage 0: Stale Gate ========
  // 인지 데이터가 오래되었으면 "STALE" 상태만 발행하고 즉시 반환.
  // 오래된 데이터로 경로를 생성하면 이미 지나간 장애물을 회피하거나,
  // 사라진 차선을 따라가는 등 위험한 동작을 할 수 있다.
  if (check_stale()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[Stage0] STALE — perception timeout");
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = "STALE";
    pub_status_->publish(std::move(status_msg));
    return;  // 파이프라인 중단 — 경로를 발행하지 않음
  }

  // ======== Stage 1: Input Parse ========
  // ROS 메시지(BBoxArray, LaneBoundaryArray) → 단일 ChainPoint 벡터
  // LiDAR bbox에는 sensor_tf 오프셋 보정이 적용됨
  std::vector<ChainPoint> all_pts;
  parse_input(all_pts);

  // ======== Stage 2: DirectionChainer ========
  // 모든 점을 좌/우로 분류하고, 각 방향에 대해:
  //   Component(연결된 점 그룹) → Backbone(주 경로) → Branch(갈래)
  // 를 생성한다. 결과는 dc_result.left / dc_result.right에 저장.
  auto dc_result = direction_chainer_.chain(all_pts, params_);

  // ======== Stage 3: Costmap Generation + A* Path Planning ========
  // 3a. ChainPoint → ChainedPoint 변환 (left, right, unchained)
  std::vector<ChainedPoint> left_chained, right_chained, unchained_chained;
  for (const auto & p : dc_result.left.component)
    left_chained.push_back(p.to_chained_point());
  for (const auto & p : dc_result.right.component)
    right_chained.push_back(p.to_chained_point());
  for (const auto & p : dc_result.unchained)
    unchained_chained.push_back(p.to_chained_point());

  // 3b. Costmap 생성 (가우시안 비용 지도)
  auto costmap = costmap_generator_.generate(
    left_chained, right_chained, unchained_chained, params_);

  // 3b-2. Entry walls: 시드→ego 양옆까지 벽을 그려서 입구 유도
  // 양쪽 backbone이 있을 때만 적용 (한쪽만 있으면 벽을 만들 수 없음)
  if (costmap.valid &&
      !dc_result.left.backbone.empty() && !dc_result.right.backbone.empty()) {
    Point2D left_seed = {
      dc_result.left.backbone.front().x,
      dc_result.left.backbone.front().y};
    Point2D right_seed = {
      dc_result.right.backbone.front().x,
      dc_result.right.backbone.front().y};
    CostmapGenerator::apply_entry_walls(costmap, left_seed, right_seed, params_);
  }

  // ── 3c. Goal 계산 ──
  // A* 탐색의 목표점(local_goal)을 결정한다.
  // 좌/우 backbone의 끝점(.back())을 기준으로 트랙 중앙을 추정한다.
  //
  // Case 1: 양쪽 backbone 모두 존재
  //   goal = 좌/우 끝점의 중점 ((lx+rx)/2, (ly+ry)/2)
  //
  // Case 2/3: 한쪽만 존재
  //   goal.x = 해당 끝점.x
  //   goal.y = 해당 끝점.y × 0.5       ← y를 절반으로 줄여 중앙 쪽으로 보정
  //   (좌측 y>0, 우측 y<0 이므로 ×0.5는 항상 중심선 방향)
  Point2D goal = {0.0, 0.0};
  bool have_goal = false;
  if (!dc_result.left.backbone.empty() && !dc_result.right.backbone.empty()) {
    double lx = dc_result.left.backbone.back().x;
    double rx = dc_result.right.backbone.back().x;
    double ly = dc_result.left.backbone.back().y;
    double ry = dc_result.right.backbone.back().y;
    goal.x = (lx + rx) / 2.0;
    goal.y = (ly + ry) / 2.0; // 양쪽 끝점의 중점
    have_goal = true;
  } else if (!dc_result.left.backbone.empty()) {
    goal.x = dc_result.left.backbone.back().x;
    goal.y = dc_result.left.backbone.back().y * 0.5;
    have_goal = true;
  } else if (!dc_result.right.backbone.empty()) {
    goal.x = dc_result.right.backbone.back().x;
    goal.y = dc_result.right.backbone.back().y * 0.5;
    have_goal = true;
  }

  // 3d. Goal을 costmap 경계 안쪽으로 clamp (방향 유지, 거리만 축소)
  //   backbone이 costmap보다 멀리 뻗어있으면 goal이 격자 밖에 놓여
  //   A*가 즉시 빈 경로를 반환한다. 이를 방지하기 위해
  //   costmap 유효 범위 안쪽 1셀 마진으로 clamp한다.
  if (have_goal && costmap.valid) {
    const double margin = costmap.resolution;  // 1셀 마진
    const double x_min = costmap.origin_x + margin;
    const double x_max = costmap.origin_x + costmap.cols * costmap.resolution - margin;
    const double y_min = costmap.origin_y + margin;
    const double y_max = costmap.origin_y + costmap.rows * costmap.resolution - margin;
    goal.x = std::clamp(goal.x, x_min, x_max);
    goal.y = std::clamp(goal.y, y_min, y_max);
  }

  // 3e. A* 경로 탐색
  std::vector<Point2D> raw_path;
  if (have_goal && costmap.valid) {
    raw_path = astar_planner_.plan(costmap, {0.0, 0.0}, goal, params_);
  }

  // ======== Stage 5: Postprocess ========
  // 원시 경로를 5단계로 정제한다:
  //   1) prune:           이상치 제거 — prune_max_dev [m] 기준
  //   2) smooth:          이동 평균 스무딩 — smooth_window 크기
  //   2.5) curvature_clamp: 최대 곡률 제한 — kappa_max = 1/R_min
  //   3) resample:        등간격 리샘플링 — resample_ds [m] 간격
  //   4) yaw:             방향각(heading) 계산
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds,
    1.0 / params_.vehicle.r_min(),  // kappa_max = 1/R_min
    params_.postprocess.curvature_clamp_max_iter);

  // ======== Stage 6: Safety Check ========
  // Menger 곡률 공식으로 후처리된 경로의 최대 곡률(κ_max)을 계산하고:
  //   - κ_max > 1/r_min  → INFEASIBLE (물리적으로 추종 불가)
  //   - 그 외             → OK
  auto safety = safety_checker::check(pp_result, params_);

  // ── 상태 로그 (INFO 레벨) ──
  const double r_min = params_.vehicle.r_min();
  const double kappa_limit = 1.0 / r_min;
  const double r_actual = (safety.max_curvature > 1e-6) ? 1.0 / safety.max_curvature : 999.0;
  if (safety.reason == "ok") {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Planner] OK — path:%zu pts, kappa=%.3f (r=%.2fm), limit=%.3f (r_min=%.2fm, delta_max=%.1f°)",
      pp_result.path.size(), safety.max_curvature, r_actual,
      kappa_limit, r_min, params_.vehicle.delta_max * 180.0 / M_PI);
  } else if (safety.reason == "curvature_exceeds_r_min") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Planner] FAIL — curvature_exceeds_r_min: "
      "kappa=%.3f (r=%.2fm) > limit=%.3f (r_min=%.2fm, delta_max=%.1f°)",
      safety.max_curvature, r_actual,
      kappa_limit, r_min,
      params_.vehicle.delta_max * 180.0 / M_PI);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Planner] FAIL — %s", safety.reason.c_str());
  }

  // ======== Stage 7: Publish (발행) ========
  // Core 토픽은 항상 발행하고, Debug 토픽은 구독자가 있을 때만 발행한다.

  // ── Core: 최종 경로 발행 ──
  // to_path_msg(): Point2D 벡터를 nav_msgs::msg::Path로 변환하는 유틸 함수
  // 제어기(controller)가 이 토픽을 구독하여 조향/속도를 결정한다.
  auto path_msg = std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(pp_result.path, frame_id, stamp));
  pub_path_->publish(std::move(path_msg));

  // ── Core: 플래너 상태 발행 ──
  // safety.reason: "ok", "no_valid_path", "curvature_exceeds_r_min" 등
  // 상위 시스템(state machine)이 이 상태를 보고 정지/서행 등을 결정할 수 있다.
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = safety.reason;
  pub_status_->publish(std::move(status_msg));

  // ── Debug: costmap (OccupancyGrid로 시각화) ──
  if (pub_dbg_costmap_->get_subscription_count() > 0 && costmap.valid) {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.stamp = stamp;
    grid_msg->header.frame_id = frame_id;
    grid_msg->info.resolution = static_cast<float>(costmap.resolution);
    grid_msg->info.width = costmap.cols;
    grid_msg->info.height = costmap.rows;
    grid_msg->info.origin.position.x = costmap.origin_x;
    grid_msg->info.origin.position.y = costmap.origin_y;
    grid_msg->info.origin.orientation.w = 1.0;
    grid_msg->data.resize(costmap.rows * costmap.cols);
    for (int i = 0; i < costmap.rows * costmap.cols; ++i) {
      // costmap 비용(0~100+)을 OccupancyGrid 값(0~100)으로 매핑
      grid_msg->data[i] = static_cast<int8_t>(
        std::min(100.0, costmap.data[i]));
    }
    pub_dbg_costmap_->publish(std::move(grid_msg));
  }

  // ── Debug: obstacle_wall (obstacle_cost 이상인 셀을 빨간색 CUBE로 표시) ──
  if (pub_dbg_obstacle_wall_->get_subscription_count() > 0 && costmap.valid) {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "obstacle_wall";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = costmap.resolution;
    m.scale.y = costmap.resolution;
    m.scale.z = 0.005; // 극히 얇은 바닥면
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.6f;
    m.pose.orientation.w = 1.0;

    const double thresh = params_.astar.obstacle_cost;
    for (int r = 0; r < costmap.rows; ++r) {
      for (int c = 0; c < costmap.cols; ++c) {
        if (costmap.data[r * costmap.cols + c] >= thresh) {
          geometry_msgs::msg::Point pt;
          pt.x = costmap.origin_x + (c + 0.5) * costmap.resolution;
          pt.y = costmap.origin_y + (r + 0.5) * costmap.resolution;
          pt.z = -0.01;  // chain(z=0)보다 아래에 렌더링
          m.points.push_back(pt);
        }
      }
    }
    ma.markers.push_back(m);
    pub_dbg_obstacle_wall_->publish(
      std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
  }

  // ── Debug: curvature (곡률 초과 지점 — 노란색 구) ──
  if (pub_dbg_curvature_->get_subscription_count() > 0 && pp_result.path.size() >= 3) {
    const double r_min = params_.vehicle.r_min();
    const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

    visualization_msgs::msg::MarkerArray ma;

    // DELETEALL로 이전 프레임 마커 제거
    visualization_msgs::msg::Marker del;
    del.header.stamp = stamp;
    del.header.frame_id = frame_id;
    del.ns = "curvature_exceed";
    del.id = -1;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    int marker_id = 0;
    for (size_t i = 0; i + 2 < pp_result.path.size(); ++i) {
      const auto & a = pp_result.path[i];
      const auto & b = pp_result.path[i + 1];
      const auto & c = pp_result.path[i + 2];

      const double ab = dist(a, b);
      const double bc = dist(b, c);
      const double ac = dist(a, c);
      const double denom = ab * bc * ac;
      if (denom < 1e-12) continue;

      const Point2D ba = b - a;
      const Point2D cb = c - b;
      const double kappa = 2.0 * std::abs(cross2(ba, cb)) / denom;

      if (kappa > kappa_limit) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = frame_id;
        m.ns = "curvature_exceed";
        m.id = marker_id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = b.x;
        m.pose.position.y = b.y;
        m.pose.position.z = 0.15;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.15;
        m.scale.y = 0.15;
        m.scale.z = 0.15;
        // 초과량에 따라 노란색→빨간색 그라데이션
        const double ratio = std::min((kappa / kappa_limit - 1.0) * 2.0, 1.0);
        m.color.r = 1.0f;
        m.color.g = static_cast<float>(1.0 - ratio);  // 초과 많을수록 빨강
        m.color.b = 0.0f;
        m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration::from_seconds(0.2);
        ma.markers.push_back(m);
      }
    }
    pub_dbg_curvature_->publish(
      std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
  }

  // ── Debug: raw_path (A* 원시 경로, 후처리 전) ──
  if (pub_dbg_raw_path_->get_subscription_count() > 0) {
    pub_dbg_raw_path_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(raw_path, frame_id, stamp)));
  }

  // ── Debug: chainer 시각화 (backbone, branches, seeds) ──
  // publish_debug 파라미터가 false이면 chainer 관련 시각화를 완전히 건너뛴다.
  if (params_.chainer.publish_debug) {

  // ── Debug: left/right backbone (Path) ──
  // backbone: DirectionChainer가 생성한 주 경로(체인)의 핵심 점들.
  // RViz2에서 Path 타입으로 시각화하여 좌/우 체인이 올바르게 생성되었는지 확인.
  // 왼쪽 체인: 보통 좌측 차선/콘을 따라가는 연결 선
  if (pub_dbg_left_chain_->get_subscription_count() > 0) {
    std::vector<Point2D> left_pts;
    left_pts.reserve(dc_result.left.backbone.size());
    for (const auto & p : dc_result.left.backbone) {
      left_pts.push_back(p.to_point2d());  // ChainPoint → Point2D 변환
    }
    pub_dbg_left_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(left_pts, frame_id, stamp)));
  }

  // 오른쪽 체인: 보통 우측 차선/콘을 따라가는 연결 선
  if (pub_dbg_right_chain_->get_subscription_count() > 0) {
    std::vector<Point2D> right_pts;
    right_pts.reserve(dc_result.right.backbone.size());
    for (const auto & p : dc_result.right.backbone) {
      right_pts.push_back(p.to_point2d());  // ChainPoint → Point2D 변환
    }
    pub_dbg_right_chain_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(right_pts, frame_id, stamp)));
  }

  // ── Debug: branches (MarkerArray — LINE_STRIP) ──
  // branch: backbone에서 갈라져 나온 부가 경로.
  // 각 branch는 backbone의 한 점(parent)에서 시작하여 여러 점으로 이어진다.
  // RViz2에서 LINE_STRIP 마커로 시각화 → 체이닝 알고리즘의 분기 구조 확인.
  //
  // 색상 규칙:
  //   왼쪽 branch: 연두색 (r=0.5, g=1.0, b=0.5) — ns="left_branches"
  //   오른쪽 branch: 연분홍 (r=1.0, g=0.5, b=0.5) — ns="right_branches"
    // branch 마커 생성 람다 — 좌/우 공통 로직을 재사용하기 위해 람다로 추출
    auto make_branch_markers = [&](
      const std::vector<BranchInfo> & branches,      // branch 정보 벡터
      const std::vector<ChainPoint> & backbone,      // 해당 방향의 backbone 점들
      float r, float g, float b_color,               // 마커 색상 (RGB)
      const std::string & ns) -> visualization_msgs::msg::MarkerArray  // 네임스페이스
    {
      visualization_msgs::msg::MarkerArray ma;

      // 이전 프레임의 잔여 마커를 모두 삭제 (잔상 방지)
      visualization_msgs::msg::Marker del;
      del.header.stamp = stamp;
      del.header.frame_id = frame_id;
      del.ns = ns;
      del.id = -1;  // DELETEALL용 고유 ID (branch id=0과 충돌 방지)
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(del);

      int id = 0;  // 각 마커의 고유 ID (RViz2에서 구분용)
      for (const auto & br : branches) {
        // 빈 branch이거나 parent 인덱스가 범위 밖이면 건너뛴다
        if (br.points.empty() || br.parent_backbone_idx < 0 ||
            br.parent_backbone_idx >= static_cast<int>(backbone.size())) {
          continue;
        }

        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = frame_id;
        m.ns = ns;           // 네임스페이스: "left_branches" 또는 "right_branches"
        m.id = id++;         // 고유 ID (같은 ns 내에서 중복 불가)
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 연결선 마커
        m.action = visualization_msgs::msg::Marker::ADD;        // 추가/갱신
        m.scale.x = 0.03;   // 선 두께 3cm
        m.color.r = r;
        m.color.g = g;
        m.color.b = b_color;
        m.color.a = 0.8f;   // 약간 투명 (80% 불투명)

        // LINE_STRIP의 첫 번째 점: backbone 위의 parent 점 (분기 시작점)
        geometry_msgs::msg::Point pt;
        pt.x = backbone[br.parent_backbone_idx].x;
        pt.y = backbone[br.parent_backbone_idx].y;
        pt.z = 0.0;
        m.points.push_back(pt);
        // 이후 점들: branch를 따라가는 점들 (parent → branch[0] → branch[1] → ...)
        for (const auto & bp : br.points) {
          pt.x = bp.x;
          pt.y = bp.y;
          m.points.push_back(pt);
        }

        ma.markers.push_back(m);
      }
      return ma;
    };

    // 왼쪽 branch 시각화 (연두색)
    if (pub_dbg_left_branches_->get_subscription_count() > 0) {
      auto ma = make_branch_markers(
        dc_result.left.branches, dc_result.left.backbone,
        0.5f, 1.0f, 0.5f, "left_branches");
      pub_dbg_left_branches_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // 오른쪽 branch 시각화 (연분홍)
    if (pub_dbg_right_branches_->get_subscription_count() > 0) {
      auto ma = make_branch_markers(
        dc_result.right.branches, dc_result.right.backbone,
        1.0f, 0.5f, 0.5f, "right_branches");
      pub_dbg_right_branches_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // ── Debug: seeds & goals (MarkerArray — SPHERE) ──
    // seed: 체이닝 알고리즘이 시작한 점 (backbone의 첫 번째 점)
    // goal: 체이닝 알고리즘이 도달한 끝점 (backbone의 마지막 점)
    //
    // 시각화 색상:
    //   왼쪽 seed:  초록색 (r=0, g=1, b=0) — backbone.front()
    //   오른쪽 seed: 빨간색 (r=1, g=0, b=0) — backbone.front()
    //   모든 goal:   파란색 (r=0, g=0, b=1) — backbone.back()
    //
    // SPHERE 마커: 직경 0.15m 구체 → RViz2에서 눈에 잘 보이는 점으로 표시됨
    if (pub_dbg_seeds_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray ma;
      int id = 0;

      // 한 방향(좌 또는 우)의 seed/goal 마커를 추가하는 람다
      auto add_seed = [&](const SideResult & side, float r, float g, float b_color) {
        if (side.backbone.empty()) return;  // backbone이 비었으면 건너뛰기

        // ── seed 마커: backbone의 시작점 (체이닝 출발점) ──
        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = frame_id;
        m.ns = "seeds";        // 네임스페이스: seed/goal 마커 그룹
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;   // 구체 마커
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = m.scale.y = m.scale.z = 0.15;           // 직경 15cm
        m.color.r = r;        // 왼쪽=초록, 오른쪽=빨강
        m.color.g = g;
        m.color.b = b_color;
        m.color.a = 1.0f;     // 완전 불투명
        m.pose.position.x = side.backbone.front().x;  // seed = 체인 시작점
        m.pose.position.y = side.backbone.front().y;
        m.pose.orientation.w = 1.0;  // 단위 쿼터니언 (회전 없음)
        ma.markers.push_back(m);

        // ── goal 마커: backbone의 끝점 (파란색으로 통일) ──
        m.id = id++;
        m.color.r = 0.0f;
        m.color.g = 0.0f;
        m.color.b = 1.0f;     // 파란색 = goal 마커 (좌/우 구분 없이 동일)
        m.pose.position.x = side.backbone.back().x;   // goal = 체인 끝점
        m.pose.position.y = side.backbone.back().y;
        ma.markers.push_back(m);
      };

      add_seed(dc_result.left, 0.0f, 1.0f, 0.0f);    // 왼쪽: 초록 seed + 파란 goal
      add_seed(dc_result.right, 1.0f, 0.0f, 0.0f);   // 오른쪽: 빨간 seed + 파란 goal

      pub_dbg_seeds_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
    }

    // ── local_goal 마커: A* 탐색 목표점 (노란색 구체) ──
    if (pub_dbg_local_goal_->get_subscription_count() > 0 && have_goal) {
      visualization_msgs::msg::MarkerArray goal_ma;
      visualization_msgs::msg::Marker gm;
      gm.header.frame_id = "base_link";
      gm.header.stamp = now();
      gm.ns = "local_goal";
      gm.id = 0;
      gm.type = visualization_msgs::msg::Marker::SPHERE;
      gm.action = visualization_msgs::msg::Marker::ADD;
      gm.scale.x = gm.scale.y = gm.scale.z = 0.3;
      gm.color.r = 1.0f;
      gm.color.g = 1.0f;
      gm.color.b = 0.0f;
      gm.color.a = 1.0f;
      gm.pose.position.x = goal.x;
      gm.pose.position.y = goal.y;
      gm.pose.position.z = 0.0;
      goal_ma.markers.push_back(gm);
      pub_dbg_local_goal_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(goal_ma));
    }
  // ── 디버그 로그 (2초마다 출력) ──
  // RCLCPP_INFO_THROTTLE: 지정된 주기(2000ms)마다 한 번만 로그 출력
  // → 10Hz 콜백에서 매번 출력하면 로그가 범람하므로 throttle로 제한
  // L_comp/R_comp: 좌/우 component 점 수
  // L_bb/R_bb: 좌/우 backbone 점 수
  // L_br/R_br: 좌/우 branch 개수
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "chain: L_comp=%zu L_bb=%zu L_br=%zu  R_comp=%zu R_bb=%zu R_br=%zu",
      dc_result.left.component.size(), dc_result.left.backbone.size(),
      dc_result.left.branches.size(),
      dc_result.right.component.size(), dc_result.right.backbone.size(),
      dc_result.right.branches.size());
  }
}

}  // namespace chaining_costmap_ver

// ══════════════════════════════════════════════════════════════
//  ROS 2 컴포넌트 등록
// ══════════════════════════════════════════════════════════════
// 이 매크로는 LCPlannerNode를 ROS 2 컴포넌트로 등록한다.
// 등록하면 다음과 같은 장점이 있다:
//
// 1) 동적 로딩: rclcpp::ComponentManager가 런타임에 .so 파일을 로드
//    → launch file에서 ComposableNode로 사용 가능
//
// 2) Intra-process Communication:
//    같은 프로세스에 여러 컴포넌트를 로딩하면,
//    토픽 발행/구독 시 직렬화/역직렬화 없이 포인터만 전달 (Zero-copy)
//    → perception 노드와 같은 프로세스에 로딩하면 지연 시간 대폭 감소
//
// 3) 독립 실행도 가능:
//    CMakeLists.txt에 별도 executable(lc_planner_node_exe)이 있으면
//    ros2 run으로 단독 실행도 가능
//
// [요구 사항]
//    생성자가 (const rclcpp::NodeOptions &) 시그니처를 가져야 한다.
//    → 위의 LCPlannerNode(const rclcpp::NodeOptions & options)가 이를 만족.
RCLCPP_COMPONENTS_REGISTER_NODE(chaining_costmap_ver::LCPlannerNode)
