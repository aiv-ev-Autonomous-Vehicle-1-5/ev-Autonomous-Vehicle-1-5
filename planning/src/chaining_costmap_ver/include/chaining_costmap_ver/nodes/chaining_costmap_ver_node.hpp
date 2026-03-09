/**
 * @file lc_planner_node.hpp
 * @brief LC Planner 메인 ROS 2 노드 — DirectionChainer v2 7단계 파이프라인
 *
 * ══════════════════════════════════════════════════════════════
 *  LC Planner 아키텍처 개요
 * ══════════════════════════════════════════════════════════════
 *
 * [역할]
 *   이 노드는 자율주행 경로 계획의 "오케스트레이터(orchestrator)"이다.
 *   perception(인지) 결과를 입력받아 → 경로를 생성 → 제어기에 전달한다.
 *
 * [실행 주기]
 *   10Hz wall timer 콜백 (100ms 주기)으로 파이프라인을 실행한다.
 *   → wall timer를 사용하므로 시뮬레이션 시간과 무관하게 실시간 동작.
 *
 * [입력 토픽] (Best Effort QoS, depth=1)
 *   - /perception/lane_boundaries : 카메라 차선 인식 결과 (LaneBoundaryArray)
 *   - /perception/bboxes          : LiDAR 장애물(콘) 바운딩 박스 (BBoxArray)
 *
 *   ※ Best Effort QoS를 사용하는 이유:
 *     인지 데이터는 실시간성이 중요하며, 오래된 데이터를 재전송 받는 것보다
 *     최신 데이터만 빠르게 받는 것이 자율주행에 적합하다.
 *
 * [출력 토픽]
 *   Core:
 *     - /planning/path   : 최종 후처리된 경로 (nav_msgs/Path)
 *     - /planning/status : 플래너 상태 문자열 (std_msgs/String)
 *   Debug (lazy publishing — 구독자가 있을 때만 발행):
 *     - /planning/debug/left_chain   : 왼쪽 backbone 체인 (Path)
 *     - /planning/debug/right_chain  : 오른쪽 backbone 체인 (Path)
 *     - /chaining/debug/left_branches  : 왼쪽 branch 시각화 (MarkerArray)
 *     - /chaining/debug/right_branches : 오른쪽 branch 시각화 (MarkerArray)
 *     - /chaining/debug/seeds          : 체이닝 시드/골 마커 (MarkerArray)
 *
 * ──────────────────────────────────────────────────────────────
 *  7단계 파이프라인 (on_timer 콜백에서 순차 실행)
 * ──────────────────────────────────────────────────────────────
 *
 *   Stage 0: Stale Gate (신선도 검사)
 *     → 마지막으로 받은 인지 데이터가 timeout_ms 이내인지 확인.
 *       오래된 데이터로 경로를 생성하면 위험하므로 "STALE" 상태를 발행하고 중단.
 *
 *   Stage 1: Input Parse (입력 파싱)
 *     → BBox/LaneBoundary ROS 메시지를 내부 ChainPoint 벡터로 변환.
 *       - LiDAR bbox: sensor_tf 오프셋(velodyne→base_link) 보정 적용
 *       - 차선 점: 그대로 사용 (카메라는 base_link 기준이라 오프셋 불필요)
 *       - 좌/우 구분은 여기서 하지 않는다 (Stage 2에서 seed 기반으로 결정)
 *
 *   Stage 2: DirectionChainer
 *     → Component 분리 → Backbone 생성 → Branch 분기
 *       연결된 포인트 그룹(component)을 찾고, 주 경로(backbone)와
 *       갈래(branch)로 분리한다.
 *
 *   Stage 3: Costmap Generation + A* Path Planning (코스트맵 생성 + A* 경로 탐색)
 *     → 좌/우 체인 포인트로 가우시안 코스트맵을 생성하고,
 *       A* 알고리즘으로 시작점(ego)에서 목표점(local_goal)까지 경로를 탐색한다.
 *
 *   Stage 5: Postprocess (후처리)
 *     → prune(이상치 제거) → smooth(스무딩) → curvature_clamp(곡률 제한)
 *       → resample(등간격 리샘플링) → curvature_clamp(곡률 제한)
 *       → yaw(방향각 계산) 순서로 경로를 정제한다.
 *
 *   Stage 6: Safety Check (안전 검사)
 *     → Menger 곡률 공식으로 최대 곡률을 계산하고,
 *       차량의 최소 회전 반경 제한에 따라 경로 실현 가능성(곡률 검사)을 판정한다.
 *
 *   Stage 7: Publish (발행)
 *     → Core: 최종 경로 + 플래너 상태
 *     → Debug: costmap, raw_path, chains, branches, seeds
 *       (디버그 토픽은 구독자가 있을 때만 발행 → 연산 절약)
 *
 * ──────────────────────────────────────────────────────────────
 *  ROS 2 컴포넌트 등록
 * ──────────────────────────────────────────────────────────────
 *   RCLCPP_COMPONENTS_REGISTER_NODE 매크로로 등록되어 있어서,
 *   rclcpp::ComponentManager (혹은 launch file의 ComposableNode)를 통해
 *   같은 프로세스에 동적 로딩 가능하다.
 *   → Intra-process Communication(프로세스 내 통신)을 활용하면
 *     토픽 발행/구독 시 Zero-copy로 데이터를 전달할 수 있어 성능 향상.
 * ──────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_COSTMAP_VER__NODES__LC_PLANNER_NODE_HPP_
#define CHAINING_COSTMAP_VER__NODES__LC_PLANNER_NODE_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"
#include "chaining_costmap_ver/costmap/costmap_generator.hpp"
#include "chaining_costmap_ver/planner/astar_planner.hpp"
#include "chaining_costmap_ver/postprocess/path_postprocessor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ev_msgs/msg/lane_boundary_array.hpp>
#include <ev_msgs/msg/b_box_array.hpp>

#include <vector>

namespace chaining_costmap_ver
{

/**
 * @class LCPlannerNode
 * @brief LC Planner 메인 ROS 2 노드
 *
 * rclcpp::Node을 상속하여 7단계 파이프라인을 10Hz로 실행한다.
 * 컴포넌트(composable node)로 등록되어 있어, 같은 프로세스에서
 * 다른 노드와 함께 로딩하여 Intra-process 통신이 가능하다.
 */
class LCPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief 생성자 — 파라미터 로드, 구독/발행 설정, 타이머 시작
   * @param options  rclcpp::NodeOptions (컴포넌트 로딩 시 전달됨)
   *
   * NodeOptions를 받는 이유:
   *   RCLCPP_COMPONENTS_REGISTER_NODE 매크로가 이 시그니처를 요구한다.
   *   ComposableNode로 사용할 때 use_intra_process_comms 등의 옵션을 전달.
   */
  explicit LCPlannerNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 10Hz 타이머 콜백 — 7단계 파이프라인을 순차 실행
   *
   * Stage 0~7을 순서대로 호출하며, Stage 0에서 데이터가 stale이면
   * "STALE" 상태만 발행하고 즉시 반환(early return)한다.
   */
  void on_timer();

  /**
   * @brief [Stage 1] 콘/차선 ROS 메시지를 단일 ChainPoint 벡터로 변환
   *
   * ── 변환 규칙 ──
   *
   * [BBox → ChainPoint] (LiDAR 장애물, 주로 PE 드럼/교통 콘)
   *   - 좌표: b.position.x + sensor_tf.tf_x,  b.position.y + sensor_tf.tf_y
   *     → velodyne 좌표계를 base_link 좌표계로 변환하기 위해 오프셋 적용
   *     → sensor_tf는 yaml 파라미터에서 로드 (tf_x, tf_y: LiDAR→base_link 변위)
   *   - type: CONE
   *   - confidence, label, size_x, size_y: BBox 메시지에서 그대로 복사
   *
   * [LaneBoundary → ChainPoint] (카메라 차선 인식 점)
   *   - 좌표: 그대로 사용 (카메라는 base_link 기준으로 점을 발행한다고 가정)
   *   - type: LANE
   *   - confidence: boundary의 confidence
   *   - label: -1 (차선에는 클러스터 라벨이 없으므로)
   *
   * ── 주의 ──
   *   좌/우 구분은 이 함수에서 하지 않는다.
   *   DirectionChainer(Stage 2)가 seed 선택 알고리즘으로 좌/우를 결정한다.
   *
   * @param[out] all_pts  변환된 ChainPoint들이 추가될 벡터
   */
  void parse_input(std::vector<ChainPoint> & all_pts) const;

  /**
   * @brief [Stage 0] 인지 데이터 신선도(stale) 검사
   *
   * ── 로직 ──
   *   현재 시각과 마지막 수신 타임스탬프의 차이(dt)를 계산한다.
   *   dt > perception_ms 이면 해당 데이터는 "stale"(오래됨)으로 판정.
   *
   *   차선 OR bbox 중 하나라도 fresh하면 → false (stale 아님)
   *   둘 다 stale이거나 한 번도 받지 못했으면 → true (stale)
   *
   *   ※ OR 조건인 이유:
   *     차선만 보이는 구간, 콘만 보이는 구간이 있으므로
   *     하나만 있어도 경로 계획이 가능하다.
   *
   * @return true: 데이터가 stale → 파이프라인 중단 필요
   */
  bool check_stale() const;

  // ══════════════════════════════════════════════════════════════
  //  멤버 변수
  // ══════════════════════════════════════════════════════════════

  // ── 파라미터 ──
  // yaml에서 로드한 모든 플래너 파라미터를 담는 구조체
  // (sensor_tf, timeouts, chainer, costmap, astar, postprocess, vehicle)
  PlanningParams params_;

  // ── 파이프라인 모듈들 ──
  // 각 모듈은 stateless에 가깝게 설계되어, 매 콜백마다 params와 입력을 받아 처리
  DirectionChainer        direction_chainer_;   ///< Stage 2: 방향 기반 체이닝
  CostmapGenerator        costmap_generator_;   ///< Stage 3a: 가우시안 비용 지도 생성
  AStarPlanner            astar_planner_;       ///< Stage 3b: A* 경로 탐색
  PathPostprocessor       postprocessor_;       ///< Stage 5: prune→smooth→curvature_clamp→resample→yaw

  // ── 최신 입력 데이터 (콜백에서 갱신) ──
  // UniquePtr을 사용하여 소유권 이동(move)으로 복사 비용을 없앤다
  ev_msgs::msg::LaneBoundaryArray::UniquePtr last_lanes_;   ///< 마지막으로 받은 차선 데이터
  ev_msgs::msg::BBoxArray::UniquePtr          last_bboxes_; ///< 마지막으로 받은 bbox 데이터

  // ── 입력 타임스탬프 (stale 검사용) ──
  // 각 메시지를 마지막으로 수신한 시각 (ROS 시간 기준)
  rclcpp::Time stamp_lanes_;    ///< 차선 데이터 수신 시각
  rclcpp::Time stamp_bboxes_;   ///< bbox 데이터 수신 시각

  // ── 구독자(Subscription) ──
  rclcpp::Subscription<ev_msgs::msg::LaneBoundaryArray>::SharedPtr sub_lanes_;   ///< /perception/lane_boundaries 구독
  rclcpp::Subscription<ev_msgs::msg::BBoxArray>::SharedPtr sub_bboxes_;          ///< /perception/bboxes 구독

  // ── Core 퍼블리셔 (항상 발행) ──
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;       ///< /planning/path — 최종 경로
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;   ///< /planning/status — 플래너 상태

  // ── Debug 퍼블리셔 (구독자가 있을 때만 발행 = lazy publishing) ──
  // lazy publishing: get_subscription_count() > 0 일 때만 메시지를 생성/발행
  // → RViz2에서 해당 토픽을 구독하지 않으면 CPU/메모리 낭비를 방지
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dbg_costmap_;       ///< costmap 시각화 (OccupancyGrid)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_raw_path_;             ///< A* 원시 경로 (후처리 전)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_left_chain_;           ///< 왼쪽 backbone 체인
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_right_chain_;          ///< 오른쪽 backbone 체인
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_left_branches_;   ///< 왼쪽 branch 시각화
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_right_branches_;  ///< 오른쪽 branch 시각화
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_seeds_;            ///< 시드/골 마커 시각화
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_local_goal_;      ///< A* goal 시각화
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_obstacle_wall_;  ///< obstacle_cost 이상 셀 (빨간색)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_curvature_;     ///< 곡률 초과 지점 (노란색 구)

  // ── 타이머 ──
  // 100ms(10Hz) 주기의 wall timer — on_timer() 콜백을 호출
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__NODES__LC_PLANNER_NODE_HPP_
