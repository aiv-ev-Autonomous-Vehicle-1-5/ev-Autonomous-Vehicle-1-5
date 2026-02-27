/**
 * @file lc_planner_node.cpp
 * @brief LC Planner 메인 노드 구현 — DirectionChainer v2 8단계 파이프라인
 *
 * ══════════════════════════════════════════════════════════════
 *  이 파일은 lc_planner_node.hpp에 선언된 LCPlannerNode를 구현한다.
 *
 *  핵심 구조:
 *    생성자 — 파라미터 로드, QoS 설정, 구독/발행 생성, 타이머 시작
 *    on_timer() — 10Hz로 호출되는 메인 루프 (8단계 파이프라인)
 *
 *  8단계 파이프라인 (on_timer 콜백):
 *    Stage 0: Stale Gate — perception 데이터 타임아웃 검사
 *                          (오래된 데이터로 경로 생성 방지)
 *    Stage 1: Input Parse — 콘/차선 ROS 메시지 → 단일 ChainPoint 벡터
 *                           (LiDAR bbox는 sensor_tf 오프셋 보정)
 *    Stage 2: DirectionChainer — Component→Backbone→Branch + 리샘플
 *                                (포인트를 좌/우 체인으로 분류·연결)
 *    Stage 3: CDT Centerline — CDT 기반 외심(circumcenter) centerline 추출
 *                              (좌/우 체인에 CDT → 외심 필터링 → greedy 연결)
 *    Stage 5: Postprocess — prune → smooth → resample → yaw
 *                           (경로 정제: 이상치 제거, 스무딩, 등간격화, 방향각)
 *    Stage 6: Safety Check — 곡률/속도 검사
 *                            (Menger 곡률 → 실현 가능성 + 안전 속도)
 *    Stage 7: Publish — 경로, 상태, 디버그 토픽 발행
 *                       (Core: 항상 발행, Debug: 구독자 있을 때만)
 * ══════════════════════════════════════════════════════════════
 */
// ── 프로젝트 내부 헤더 ──
#include "chaining_CDT/nodes/chaining_CDT_node.hpp"   // LCPlannerNode 클래스 선언
#include "chaining_CDT/common/geometry.hpp"          // dist(), cross2(), to_path_msg() 등 기하 유틸
#include "chaining_CDT/common/debug_publish.hpp"     // 디버그 시각화 헬퍼
#include "chaining_CDT/safety/safety_checker.hpp"    // safety_checker::check() — 곡률/속도 검사

// ── ROS 2 컴포넌트 등록 매크로 ──
// 이 매크로를 통해 이 노드를 shared library(.so)로 빌드하고,
// rclcpp::ComponentManager가 런타임에 동적 로딩할 수 있게 한다.
#include <rclcpp_components/register_node_macro.hpp>

// ── 표준 라이브러리 ──
#include <chrono>     // std::chrono::milliseconds (타이머 주기용)
#include <cmath>      // std::sqrt, std::abs 등
#include <algorithm>  // std::clamp, std::min, std::max 등

namespace chaining_CDT
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
  // (vehicle, speed, sensor_tf, chainer, costmap, postprocess, timeouts 등)
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
  // 구독자가 없으면 메시지 생성을 건너뛰어 CPU/메모리를 절약한다.
  // → on_timer()에서 get_subscription_count() > 0 체크 후 발행.

  pub_dbg_centerline_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/centerline", qos_be);        // CDT 외심 연결 centerline
  pub_dbg_circumcenters_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/circumcenters", qos_be);     // 필터 통과한 외심 점
  pub_dbg_left_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/left_chain", qos_be);    // 왼쪽 backbone 체인 (Path)
  pub_dbg_right_chain_ = create_publisher<nav_msgs::msg::Path>(
    "/planning/debug/right_chain", qos_be);   // 오른쪽 backbone 체인 (Path)
  pub_dbg_left_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/left_branches", qos_be);  // 왼쪽 branch (LINE_STRIP 마커)
  pub_dbg_right_branches_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/right_branches", qos_be); // 오른쪽 branch (LINE_STRIP 마커)
  pub_dbg_seeds_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chaining/debug/seeds", qos_be);          // 체이닝 시드/골 (SPHERE 마커)

  // ── 10Hz 타이머 ──
  // wall timer: 시뮬레이션 시간이 아닌 실제 시계(wall clock) 기준
  // 100ms 주기 → 초당 10회 파이프라인 실행
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LCPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "LCPlannerNode initialized (10 Hz, DirectionChainer v2 + CDT centerline)");
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
//  on_timer() — 10Hz 메인 루프: 8단계 파이프라인
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

  // ======== Stage 3: CDT Centerline Extraction ========
  // 좌/우 경계 체인에 Constrained Delaunay Triangulation(CDT)을 수행하고,
  // 삼각형의 외심(circumcenter)을 기하학적으로 필터링하여 centerline을 추출한다.
  // 결과: raw_path (후처리 전 원시 경로, Point2D 벡터)
  auto raw_path = cdt_extractor_.extract(
    dc_result.left.component, dc_result.right.component, params_);

  // ======== Stage 5: Postprocess ========
  // 원시 경로를 4단계로 정제한다:
  //   1) prune:    이상치(크게 벗어난 점) 제거 — prune_max_dev [m] 기준
  //   2) smooth:   이동 평균(moving average) 스무딩 — smooth_window 크기
  //   3) resample: 등간격 리샘플링 — resample_ds [m] 간격
  //   4) yaw:      각 웨이포인트의 방향각(heading) 계산
  auto pp_result = postprocessor_.process(
    raw_path,
    params_.postprocess.prune_max_dev,
    params_.postprocess.smooth_window,
    params_.postprocess.resample_ds);

  // ======== Stage 6: Safety Check ========
  // Menger 곡률 공식으로 후처리된 경로의 최대 곡률(κ_max)을 계산하고:
  //   - κ_max > 1/r_min  → INFEASIBLE (물리적으로 추종 불가)
  //   - 그 외             → OK, 안전 속도 = min(v_max, sqrt(a_lat_max / κ_max))
  auto safety = safety_checker::check(pp_result, params_);

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

  // ── Debug: centerline (CDT 외심 연결 결과) ──
  // RViz2에서 CDT 기반 centerline을 Path로 시각화.
  if (pub_dbg_centerline_->get_subscription_count() > 0) {
    pub_dbg_centerline_->publish(std::make_unique<nav_msgs::msg::Path>(
      to_path_msg(raw_path, frame_id, stamp)));
  }

  // ── Debug: circumcenters (필터 통과한 외심 점들) ──
  // raw_path의 각 점을 SPHERE 마커로 시각화하여 외심 분포를 확인.
  if (pub_dbg_circumcenters_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray ma;
    int cc_id = 0;
    for (const auto & pt : raw_path) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = "circumcenters";
      m.id = cc_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = m.scale.y = m.scale.z = 0.08;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.9f;
      m.pose.position.x = pt.x;
      m.pose.position.y = pt.y;
      m.pose.orientation.w = 1.0;
      ma.markers.push_back(m);
    }
    pub_dbg_circumcenters_->publish(
      std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
  }

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
  //
  // publish_debug 파라미터가 false이면 branch/seed 시각화를 완전히 건너뛴다.
  if (params_.chainer.publish_debug) {
    // branch 마커 생성 람다 — 좌/우 공통 로직을 재사용하기 위해 람다로 추출
    auto make_branch_markers = [&](
      const std::vector<BranchInfo> & branches,      // branch 정보 벡터
      const std::vector<ChainPoint> & backbone,      // 해당 방향의 backbone 점들
      float r, float g, float b_color,               // 마커 색상 (RGB)
      const std::string & ns) -> visualization_msgs::msg::MarkerArray  // 네임스페이스
    {
      visualization_msgs::msg::MarkerArray ma;
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

}  // namespace chaining_CDT

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
RCLCPP_COMPONENTS_REGISTER_NODE(chaining_CDT::LCPlannerNode)
