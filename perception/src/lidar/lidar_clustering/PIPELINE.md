# lidar_clustering 파이프라인 작동 단계

## 1단계: 노드 초기화

**파일**: `clustering_component.cpp:11-40`

```cpp
eps_ = this->declare_parameter("eps", 0.5);
min_pts_ = this->declare_parameter("min_pts", 5);
use_gpu_ = this->declare_parameter("use_gpu", true);
```

- YAML에서 파라미터 3개 로드
- GPU 모드면 `DBSCANGpu(50000)` 생성 → GPU 메모리 사전 할당

**GPU 메모리 할당** — `dbscan_gpu.cu:232-255`:
```cpp
cudaMalloc(&r->d_px, max_points * sizeof(float));
cudaMalloc(&r->d_py, max_points * sizeof(float));
cudaMalloc(&r->d_pz, max_points * sizeof(float));
cudaMalloc(&r->d_cell_indices, ...);
cudaMalloc(&r->d_sorted_indices, ...);
// ... 등 총 12개
```

- GPU 초기화 실패 시 `RCLCPP_FATAL` 출력 후 `rclcpp::shutdown()`
- `input` 토픽 구독, `output` 토픽 퍼블리시 설정

---

## 2단계: 포인트 수신 & x,y,z 추출

**파일**: `clustering_component.cpp:42-61`

```cpp
const uint32_t num_points = msg->width * msg->height;
if (num_points == 0) {
  publisher_->publish(*msg);
  return;
}

sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
  points.push_back({*iter_x, *iter_y, *iter_z});
}
```

- `/voxel_grid/output`에서 PointCloud2 수신
- 바이트 배열에서 `x, y, z`만 추출 → `std::vector<Point3D>`

---

## 3단계: DBSCAN 분기

**파일**: `clustering_component.cpp:64-69`

```cpp
std::vector<int32_t> labels;
if (use_gpu_ && dbscan_gpu_) {
  labels = dbscan_gpu_->cluster(points, eps_, min_pts_);  // GPU 경로
} else {
  labels = DBSCAN::cluster(points, eps_, min_pts_);        // CPU 경로
}
```

---

## [GPU 경로]

### 3-A. SoA 변환 + AABB 계산 (CPU)

**파일**: `dbscan_gpu.cpp:84-100`

```cpp
std::vector<float> hx(n), hy(n), hz(n);
float min_x = points[0].x, max_x = points[0].x;

for (int i = 0; i < n; i++) {
  hx[i] = points[i].x;  hy[i] = points[i].y;  hz[i] = points[i].z;
  if (hx[i] < min_x) min_x = hx[i];
  if (hx[i] > max_x) max_x = hx[i];
  // y, z도 동일
}
```

- AoS → SoA 변환: GPU coalesced access에 최적화
- AABB: 전체 포인트의 min/max 좌표 → 3D 그리드 범위 결정

### 3-B. 3D 그리드 계산 + 용량 확인 (CPU)

**파일**: `dbscan_gpu.cpp:102-114`

```cpp
const float cell_size = static_cast<float>(eps);
const int grid_x = (int)((max_x - min_x) / eps) + 2;
const int grid_y = (int)((max_y - min_y) / eps) + 2;
const int grid_z = (int)((max_z - min_z) / eps) + 2;
const size_t num_cells = grid_x * grid_y * grid_z;

if (num_cells > 10000000) return labels;  // 너무 크면 포기
impl_->ensure_capacity(n, num_cells);     // GPU 메모리 부족하면 재할당
```

### 3-C. 포인트 업로드 (CPU → GPU)

**파일**: `dbscan_gpu.cu:279-288`

```cpp
cudaMemcpy(h->d_px, hx, n * sizeof(float), cudaMemcpyHostToDevice);
cudaMemcpy(h->d_py, hy, n * sizeof(float), cudaMemcpyHostToDevice);
cudaMemcpy(h->d_pz, hz, n * sizeof(float), cudaMemcpyHostToDevice);
```

### 3-D. 셀 인덱스 계산 (GPU 커널 1)

**파일**: `dbscan_gpu.cu:74-96`

```cuda
__global__ void kernel_compute_cell_indices(...) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n) return;

  int cx = (int)((px[idx] - origin_x) * inv_cell_size);
  int cy = (int)((py[idx] - origin_y) * inv_cell_size);
  int cz = (int)((pz[idx] - origin_z) * inv_cell_size);

  cell_indices[idx] = cx + cy * grid_x + cz * grid_x * grid_y;  // 3D → 1D
}
```

- N개 스레드가 동시에 각 포인트의 소속 셀을 계산

**런치**: `dbscan_gpu.cu:299-306`
```cpp
int block_size = 256;
int grid_dim = (n + block_size - 1) / block_size;
kernel_compute_cell_indices<<<grid_dim, block_size>>>(...);
```

**결과 다운로드** (GPU → CPU): `dbscan_gpu.cu:311-312`
```cpp
cudaMemcpy(h_cell_indices_out, h->d_cell_indices, n * sizeof(int), cudaMemcpyDeviceToHost);
```

### 3-E. 셀 기준 정렬 (CPU)

**파일**: `dbscan_gpu.cpp:131-142`

```cpp
std::vector<int> sorted_indices(n);
std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
std::sort(sorted_indices.begin(), sorted_indices.end(),
  [&h_cell_indices](int a, int b) {
    return h_cell_indices[a] < h_cell_indices[b];
  });

for (int i = 0; i < n; i++) {
  sorted_cells[i] = h_cell_indices[sorted_indices[i]];
}
```

- 같은 셀의 포인트가 메모리에서 연속 → GPU 캐시 효율 극대화

### 3-F. 정렬 데이터 업로드 + 셀 바운드 계산 (GPU 커널 2)

**업로드** — `dbscan_gpu.cu:324-330`:
```cpp
cudaMemcpy(h->d_sorted_indices, h_sorted_indices, ...);
cudaMemcpy(h->d_sorted_cells, h_sorted_cells, ...);
cudaMemset(h->d_cell_start, -1, num_cells * sizeof(int));
cudaMemset(h->d_cell_end, 0, num_cells * sizeof(int));
```

**커널** — `dbscan_gpu.cu:98-111`:
```cuda
__global__ void kernel_find_cell_bounds(const int* cell_indices,
  int* cell_start, int* cell_end, int n)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n) return;

  int cell = cell_indices[idx];
  if (idx == 0 || cell != cell_indices[idx - 1])
    cell_start[cell] = idx;
  if (idx == n - 1 || cell != cell_indices[idx + 1])
    cell_end[cell] = idx + 1;
}
```

결과 예시:
```
sorted_cells: [0, 0, 0, 3, 3, 7, 7, 7, 7]
cell_start[0]=0, cell_end[0]=3   → 셀0에 포인트 3개
cell_start[3]=3, cell_end[3]=5   → 셀3에 포인트 2개
cell_start[7]=5, cell_end[7]=9   → 셀7에 포인트 4개
```

### 3-G. 이웃 카운트 + 코어 판별 (GPU 커널 3)

**파일**: `dbscan_gpu.cu:113-165`

```cuda
__global__ void kernel_count_neighbors(...) {
  int sorted_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int pid = sorted_indices[sorted_idx];

  int count = 0;
  // 27개 인접 셀 순회
  for (int dz = -1; dz <= 1; dz++)
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        int ncell = nx + ny * grid_x + nz * grid_x * grid_y;
        int start = cell_start[ncell];
        if (start < 0) continue;

        for (int j = start; j < end_val; j++) {
          float dist_sq = ddx*ddx + ddy*ddy + ddz*ddz;
          if (dist_sq <= eps_sq) count++;
        }
      }

  neighbor_counts[pid] = count;
  is_core[pid] = (count >= min_pts) ? 1 : 0;
}
```

- **가장 연산량이 큰 커널** — N개 스레드가 동시에 이웃 탐색

### 3-H. DSU Union (GPU 커널 4)

**라벨 초기화** — `dbscan_gpu.cu:423-426`:
```cuda
__global__ void kernel_init_labels(int* labels, int n) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n) labels[idx] = idx;  // 자기 자신이 루트
}
```

**Union 커널** — `dbscan_gpu.cu:428-484`:
```cuda
__global__ void kernel_dsu_union(...) {
  int pid = sorted_indices[sorted_idx];
  if (!is_core[pid]) return;  // 코어 포인트만 union 시작

  // 27개 인접 셀 순회
  for (dz, dy, dx) {
    for (int j = start; j < end_val; j++) {
      if (dist_sq <= eps_sq) {
        device_union(labels, pid, qid);  // 같은 집합으로 병합
      }
    }
  }
}
```

**device_union** — `dbscan_gpu.cu:49-72`:
```cuda
__device__ void device_union(int* labels, int i, int j) {
  while (true) {
    while (labels[root_i] != root_i) root_i = labels[root_i];
    while (labels[root_j] != root_j) root_j = labels[root_j];

    if (root_i == root_j) return;

    int small = min(root_i, root_j);
    int large = max(root_i, root_j);

    if (atomicCAS(&labels[large], large, small) == large)
      return;  // 성공
    // 실패 → 재시도
  }
}
```

### 3-I. DSU 라벨 다운로드 (GPU → CPU)

**파일**: `dbscan_gpu.cu:519-520`
```cpp
cudaMemcpy(h_labels_out, d_labels, n * sizeof(int), cudaMemcpyDeviceToHost);
```

### 3-J. CPU 후처리: 경로 압축 + 클러스터 ID 부여

**파일**: `dbscan_gpu.cpp:178-246`

```cpp
// 1. 경로 압축
for (int i = 0; i < n; i++) {
  int root = i;
  while (h_dsu_labels[root] != root) root = h_dsu_labels[root];
  int curr = i;
  while (curr != root) {
    int next = h_dsu_labels[curr];
    h_dsu_labels[curr] = root;
    curr = next;
  }
}

// 2. 코어 포인트 포함 여부 확인
std::vector<bool> tree_has_core(n, false);
for (int i = 0; i < n; i++) {
  if (h_is_core[i]) tree_has_core[h_dsu_labels[i]] = true;
}

// 3. 클러스터 ID 할당
for (int i = 0; i < n; i++) {
  int root = h_dsu_labels[i];
  if (root_to_cluster_id[root] == -2) {
    if (tree_has_core[root])
      root_to_cluster_id[root] = cluster_counter++;  // 유효 클러스터
    else
      root_to_cluster_id[root] = -1;                 // 노이즈
  }
  labels[i] = root_to_cluster_id[root];
}
```

---

## [CPU 경로]

**파일**: `dbscan.cpp:36-118`

```cpp
// 1. KD-Tree 구축
KDTree index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
index.buildIndex();

// 2. 각 포인트 순회
for (size_t i = 0; i < n; ++i) {
  if (labels[i] != -1) continue;

  // 3. 반경 탐색
  index.radiusSearch(query_pt, eps_sq, neighbors, search_params);
  if (neighbors.size() < min_pts) continue;  // 노이즈

  // 4. 새 클러스터 시작 + BFS 확장
  labels[i] = cluster_id;
  while (!expand_queue.empty()) {
    size_t q = expand_queue.front(); expand_queue.pop();
    labels[q] = cluster_id;

    index.radiusSearch(q_pt, eps_sq, sub_neighbors, search_params);
    if (sub_neighbors.size() >= min_pts) {
      for (auto & nb : sub_neighbors)
        if (labels[nb.first] == -1) expand_queue.push(nb.first);
    }
  }
  ++cluster_id;
}
```

---

## 4단계: 출력 PointCloud2 빌드

**파일**: `clustering_component.cpp:71-114`

```cpp
output_msg.point_step = 16;  // x(4) + y(4) + z(4) + label(4)
output_msg.row_step = 16 * num_points;
output_msg.data.resize(output_msg.row_step);

for (uint32_t i = 0; i < num_points; ++i) {
  const size_t offset = i * 16;
  std::memcpy(&output_msg.data[offset + 0],  &points[i].x, 4);
  std::memcpy(&output_msg.data[offset + 4],  &points[i].y, 4);
  std::memcpy(&output_msg.data[offset + 8],  &points[i].z, 4);
  std::memcpy(&output_msg.data[offset + 12], &labels[i], 4);
}
publisher_->publish(output_msg);
```

---

## 5단계: 로깅

**파일**: `clustering_component.cpp:116-130`

```cpp
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
  "DBSCAN[%s]: %u pts -> %d clusters, %d noise",
  use_gpu_ ? "GPU" : "CPU", num_points, max_label + 1, noise_count);
```

---

## 전체 데이터 흐름 (GPU 경로)

```
PointCloud2 수신
    ↓
x,y,z 추출 → vector<Point3D>           [CPU]
    ↓
AoS→SoA 변환 + AABB                    [CPU]
    ↓
cudaMemcpy (hx,hy,hz → d_px,d_py,d_pz) [CPU→GPU]
    ↓
커널1: 셀 인덱스 계산                    [GPU] ←── 결과 다운로드
    ↓
셀 기준 정렬                             [CPU]
    ↓
cudaMemcpy (정렬 데이터 업로드)           [CPU→GPU]
    ↓
커널2: 셀 바운드 계산                     [GPU]
    ↓
커널3: 이웃 카운트 + 코어 판별            [GPU] ←── 가장 무거운 연산
    ↓
커널4-a: 라벨 초기화 (labels[i]=i)       [GPU]
    ↓
커널4-b: DSU Union (atomicCAS)           [GPU]
    ↓
cudaMemcpy (라벨 다운로드)                [GPU→CPU]
    ↓
경로 압축 + 클러스터 ID 부여              [CPU]
    ↓
PointCloud2 빌드 (x,y,z,label)          [CPU]
    ↓
publish
```
