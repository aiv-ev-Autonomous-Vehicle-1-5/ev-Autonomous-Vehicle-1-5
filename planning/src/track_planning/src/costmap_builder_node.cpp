#include "track_planning/costmap_builder_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

// ROS 2 노드를 컴포넌트로 등록하기 위한 매크로 헤더
#include <rclcpp_components/register_node_macro.hpp>
// PointCloud2 메시지 내 데이터를 반복자(iterator) 패턴으로 읽기 위한 헤더
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace track_planning
{

// 생성자: NodeOptions를 매개변수로 받아 rclcpp::Node를 초기화함
CostmapBuilderNode::CostmapBuilderNode(const rclcpp::NodeOptions & options)
: Node("costmap_builder", options) // 부모 클래스인 rclcpp::Node의 생성자 호출, 노드 이름을 "costmap_builder"로 지정
{
  // 파라미터 선언 및 초기화 (declare_parameter)
  // 토픽 및 프레임 관련 파라미터
  input_topic_ = declare_parameter<std::string>("input_topic", "/pointcloud/clustered_split");
  output_topic_ = declare_parameter<std::string>("output_topic", "/planning/costmap");
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

  // 그리드(코스트맵) 기하학적 파라미터
  origin_x_ = declare_parameter<double>("origin_x", -1.0); // 코스트맵 원점 x (m)
  origin_y_ = declare_parameter<double>("origin_y", -4.0); // 코스트맵 원점 y (m)
  grid_width_ = declare_parameter<double>("grid_width", 11.0); // 코스트맵 전체 너비 (m)
  grid_height_ = declare_parameter<double>("grid_height", 8.0); // 코스트맵 전체 높이 (m)
  resolution_ = declare_parameter<double>("resolution", 0.10); // 셀 1개의 크기 (m/cell)

  // 장애물 생성 관련 파라미터
  obstacle_radius_ = declare_parameter<double>("obstacle_radius", 0.3035); // 장애물로 간주할 반경 (m)
  occupied_value_ = declare_parameter<int>("occupied_value", 100); // 장애물 위치의 코스트 값 (100 = 완전 점유)

  // 인플레이션(안전 거리 확장) 파라미터
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.45); // 장애물 주변으로 비용을 퍼뜨릴 반경
  inflation_cost_max_ = declare_parameter<int>("inflation_cost_max", 99); // 인플레이션 시작점(장애물 경계)의 최대 비용
  inflation_cost_min_ = declare_parameter<int>("inflation_cost_min", 1); // 인플레이션 끝점의 최소 비용

  // 데이터 최신성 (Staleness) 파라미터
  stale_threshold_sec_ = declare_parameter<double>("stale_threshold_sec", 0.5); // 입력 데이터가 이 시간(초)을 초과하면 무시

  // 그리드의 가로(cols) 및 세로(rows) 셀 개수 계산
  // std::ceil: 소수점 이하 올림 처리 (안전한 공간 확보를 위해)
  // static_cast<int>: 계산된 double 값을 int형으로 명시적 형변환
  grid_cols_ = static_cast<int>(std::ceil(grid_width_ / resolution_));
  grid_rows_ = static_cast<int>(std::ceil(grid_height_ / resolution_));
  
  // 1차원 배열(std::vector 추정)을 사용해 2차원 그리드 데이터 크기 할당 및 0으로 초기화
  grid_data_.resize(static_cast<size_t>(grid_cols_ * grid_rows_), 0);

  // ROS 2 통신 구독자 및 발행자 설정
  auto sensor_qos = rclcpp::SensorDataQoS(); // 센서 데이터에 적합한 Best-Effort 기반 QoS 설정
  
  // PointCloud2 토픽 구독 설정. 콜백 함수로 pointcloud_callback 바인딩
  sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, sensor_qos,
    std::bind(&CostmapBuilderNode::pointcloud_callback, this, std::placeholders::_1));

  // OccupancyGrid 발행 설정. 신뢰성(reliable) 및 후발 구독자(transient_local)를 위한 QoS
  auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
  pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, costmap_qos);

  // 노드 초기화 완료 로그 출력
  RCLCPP_INFO(get_logger(),
    "CostmapBuilder started | in: %s | out: %s | frame: %s | grid: %dx%d | res: %.2fm",
    input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
    grid_cols_, grid_rows_, resolution_);
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void CostmapBuilderNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 1. 데이터 최신성 검사
  // 현재 시간에서 메시지 헤더의 타임스탬프를 빼서 데이터의 생성 나이(age)를 초 단위로 계산
  const double age = (this->now() - msg->header.stamp).seconds();
  if (age > stale_threshold_sec_) {
    // 임계값을 초과하면 로그를 2초에 한 번씩만 출력하고 처리를 중단(return)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Input stale: %.3f s > %.3f threshold", age, stale_threshold_sec_);
    return;
  }

  // 2. 코스트맵 초기화
  reset_grid();
  // 3. 포인트 클라우드 클러스터 중심을 계산하여 원형 장애물로 맵에 그림
  rasterize_obstacles(*msg);
  // 4. 장애물 주변으로 비용(Cost)을 점진적으로 감소시키며 확장
  inflate_obstacles();
  // 5. 계산된 코스트맵을 ROS 메시지로 발행
  publish_costmap(msg->header);
}

// ---------------------------------------------------------------------------
// Step 1: Reset
// ---------------------------------------------------------------------------

void CostmapBuilderNode::reset_grid()
{
  // std::fill 알고리즘을 사용하여 grid_data_의 모든 원소를 0으로 초기화
  std::fill(grid_data_.begin(), grid_data_.end(), static_cast<int8_t>(0));
}

// ---------------------------------------------------------------------------
// Step 2: Rasterize cluster centroids as circular obstacles
// ---------------------------------------------------------------------------

void CostmapBuilderNode::rasterize_obstacles(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  // PointCloud2 메시지의 2D 배열 구조(width * height)를 통해 전체 포인트 개수 산출
  const size_t n_points = static_cast<size_t>(cloud.width) * cloud.height;
  if (n_points == 0U) { // 포인트가 없으면 함수 종료 (0U는 부호 없는 정수 0)
    return;
  }

  // PointCloud2의 바이너리 데이터 중 'x', 'y', 'cluster_id' 필드만 추출하는 반복자 생성
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<int32_t> it_cid(cloud, "cluster_id");

  // 각 클러스터의 중심점을 구하기 위해 합계와 개수를 저장할 구조체 선언
  struct Accum
  {
    double sx = 0.0; // x 좌표의 합
    double sy = 0.0; // y 좌표의 합
    int count = 0;   // 포인트 개수
  };
  // 클러스터 ID(int32_t)를 키(Key)로, Accum 구조체를 값(Value)으로 갖는 해시 맵
  std::unordered_map<int32_t, Accum> clusters;

  // 전체 포인트를 순회하며 클러스터별 데이터 축적
  for (size_t i = 0; i < n_points; ++i, ++it_x, ++it_y, ++it_cid) {
    const int32_t cid = *it_cid;
    if (cid < 0) {
      continue;  // 클러스터 ID가 음수면 노이즈로 간주하고 다음 포인트로 건너뜀(continue)
    }
    auto & c = clusters[cid]; // 맵에서 해당 클러스터 ID의 Accum 객체 참조를 가져옴
    c.sx += static_cast<double>(*it_x); // x좌표 누적
    c.sy += static_cast<double>(*it_y); // y좌표 누적
    c.count += 1; // 포인트 수 증가
  }

  // 장애물을 그릴 반경을 셀 단위 개수로 변환
  const int obs_cells = static_cast<int>(std::ceil(obstacle_radius_ / resolution_));
  const double obs_r2 = obstacle_radius_ * obstacle_radius_; // 거리 계산을 단순화하기 위한 반경의 제곱값

  // 해시 맵(clusters)에 저장된 모든 클러스터에 대해 반복
  for (const auto & [cid, acc] : clusters) {
   

    // 누적된 좌표를 개수로 나누어 중심점(Centroid) 계산
    const double cx = acc.sx / acc.count;
    const double cy = acc.sy / acc.count;

    int center_col = 0;
    int center_row = 0;
    // 중심점의 실제 좌표(cx, cy)를 그리드 배열의 인덱스(col, row)로 변환
    if (!world_to_grid(cx, cy, center_col, center_row)) {
      continue;  // 중심점이 그리드 영역 밖이면 무시
    }

    // 중심점을 기준으로 반경(obs_cells) 내의 사각형 영역을 순회
    for (int dr = -obs_cells; dr <= obs_cells; ++dr) {
      for (int dc = -obs_cells; dc <= obs_cells; ++dc) {
        const int c = center_col + dc;
        const int r = center_row + dr;
        
        // 탐색하는 셀이 그리드 범위를 벗어나면 건너뜀
        if (c < 0 || c >= grid_cols_ || r < 0 || r >= grid_rows_) {
          continue;
        }
        
        // 중심 셀로부터의 실제 물리적 거리(m) 계산
        const double dx = dc * resolution_;
        const double dy = dr * resolution_;
        
        // 피타고라스 정리 (dx^2 + dy^2 <= r^2)를 통해 원형 영역 내부인지 확인
        if (dx * dx + dy * dy <= obs_r2) {
          // 원형 영역 내부에 있으면 해당 셀을 점유 상태(occupied_value_)로 마킹
          grid_data_[static_cast<size_t>(grid_index(c, r))] =
            static_cast<int8_t>(occupied_value_);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Step 3: Inflate obstacles (linear cost decay)
// ---------------------------------------------------------------------------

void CostmapBuilderNode::inflate_obstacles()
{
  // 현재 점유된(장애물로 마킹된) 셀들의 [열, 행] 위치를 저장할 벡터
  std::vector<std::pair<int, int>> occupied_cells;
  for (int r = 0; r < grid_rows_; ++r) {
    for (int c = 0; c < grid_cols_; ++c) {
      if (grid_data_[static_cast<size_t>(grid_index(c, r))] ==
        static_cast<int8_t>(occupied_value_))
      {
        occupied_cells.emplace_back(c, r); // 점유된 셀 좌표 추가
      }
    }
  }

  // 인플레이션 반경을 셀 단위 개수로 변환
  const int inf_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const double inv_inf = 1.0 / inflation_radius_; // 나눗셈 연산을 최적화하기 위해 역수 사용

  // 찾은 모든 장애물 셀을 중심으로 인플레이션(확장) 연산 수행
  for (const auto & [oc, or_] : occupied_cells) {
    for (int dr = -inf_cells; dr <= inf_cells; ++dr) {
      for (int dc = -inf_cells; dc <= inf_cells; ++dc) {
        const int c = oc + dc;
        const int r = or_ + dr;
        
        if (c < 0 || c >= grid_cols_ || r < 0 || r >= grid_rows_) {
          continue;
        }

        const size_t idx = static_cast<size_t>(grid_index(c, r));

        // 이미 완전 점유된(장애물인) 셀은 비용을 수정할 필요가 없으므로 건너뜀
        if (grid_data_[idx] == static_cast<int8_t>(occupied_value_)) {
          continue;
        }

        // 장애물 셀로부터 현재 탐색 중인 셀까지의 유클리드 거리 (std::hypot)
        const double dist = std::hypot(dc * resolution_, dr * resolution_);
        if (dist > inflation_radius_) {
          continue; // 인플레이션 반경을 벗어나면 무시
        }

        // 선형 감소 로직 (Linear decay)
        // ratio: 0.0 (장애물 경계) ~ 1.0 (인플레이션 끝단) 사이의 비율
        const double ratio = dist * inv_inf;  
        // 거리가 멀어질수록 inflation_cost_max_ 에서 inflation_cost_min_ 으로 점진적 감소
        const int cost = inflation_cost_max_ -
          static_cast<int>((inflation_cost_max_ - inflation_cost_min_) * ratio);

        // 여러 장애물의 인플레이션 영역이 겹칠 경우, 가장 높은 코스트를 유지
        if (static_cast<int8_t>(cost) > grid_data_[idx]) {
          grid_data_[idx] = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Step 4: Publish OccupancyGrid
// ---------------------------------------------------------------------------

void CostmapBuilderNode::publish_costmap(const std_msgs::msg::Header & header)
{
  // OccupancyGrid 메시지 객체를 동적 할당 (스마트 포인터 std::unique_ptr 사용)
  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  // 메시지 메타데이터(타임스탬프, 프레임 ID) 설정
  msg->header.stamp = header.stamp;
  msg->header.frame_id = frame_id_;

  // 코스트맵의 메타데이터(해상도, 크기, 원점) 설정
  msg->info.resolution = static_cast<float>(resolution_);
  msg->info.width = static_cast<uint32_t>(grid_cols_);
  msg->info.height = static_cast<uint32_t>(grid_rows_);
  msg->info.origin.position.x = origin_x_;
  msg->info.origin.position.y = origin_y_;
  msg->info.origin.position.z = 0.0;
  // 쿼터니언을 사용한 회전값 (x,y,z,w = 0,0,0,1 은 회전이 없음을 의미)
  msg->info.origin.orientation.w = 1.0; 

  // 계산된 1차원 배열(grid_data_)의 데이터를 ROS 메시지의 data 배열로 복사
  msg->data.assign(grid_data_.begin(), grid_data_.end());

  // 메시지 발행 (std::move를 통해 소유권을 퍼블리셔로 이전하여 불필요한 복사 방지)
  pub_costmap_->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// 실제 세계의 좌표(wx, wy)를 코스트맵 배열의 인덱스(col, row)로 변환하는 헬퍼 함수
bool CostmapBuilderNode::world_to_grid(
  double wx, double wy, int & col, int & row) const
{
  // (현재좌표 - 원점) / 해상도 수식을 통해 인덱스 산출 후 버림(floor)
  col = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  row = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  // 변환된 인덱스가 그리드 배열 범위 내에 있는지 확인하여 boolean 반환
  return (col >= 0 && col < grid_cols_ && row >= 0 && row < grid_rows_);
}

// 2차원(col, row) 인덱스를 1차원 배열(std::vector)의 인덱스로 변환하는 헬퍼 함수
int CostmapBuilderNode::grid_index(int col, int row) const
{
  // 행(row) * 가로길이(grid_cols_) + 열(col)
  return row * grid_cols_ + col;
}

}  // namespace track_planning

// 플러그인 형태로 이 노드를 동적으로 로드할 수 있도록 컴포넌트로 등록하는 매크로
RCLCPP_COMPONENTS_REGISTER_NODE(track_planning::CostmapBuilderNode)