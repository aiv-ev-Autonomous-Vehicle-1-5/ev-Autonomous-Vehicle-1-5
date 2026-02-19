---
# 2.12
## 작업 요약
- `lidar_clustering` 패키지 신규 생성 (DBSCAN 클러스터링)
- PatchWork++ 비지면 출력을 구독하여 DBSCAN으로 장애물 그룹화
- nanoflann(header-only KD-tree) 사용한 CPU DBSCAN 구현 + CPU fallback 유지
- **GPU DBSCAN 구현**: 순수 CUDA C++로 3D Grid 기반 이웃 탐색 가속
- `component_container` → `component_container_mt` 변경 (SingleThread 블로킹 문제 해결)
- launch 파일에 5번째 ComposableNode로 클러스터링 노드 추가
- 최종 파이프라인: `Driver → Transform → CropBox → PatchWork++ → DBSCAN Clustering(GPU)`

## GPU 성능 최적화 (이슈 해결)
**증상**: 초기 GPU 구현 적용 시 10Hz 입력이 0.27Hz로 급락.
**원인**:
1. **PCIe 대역폭 포화**: GPU에서 찾은 모든 이웃 리스트(수백만 개 정수)를 CPU로 복사.
2. **CPU 병목**: BFS 클러스터링을 CPU에서 수행.

**해결**: **GPU-based Disjoint Set Union (DSU)** 알고리즘 도입.
- 이웃 리스트를 CPU로 전송하지 않고 GPU 메모리 내에서 즉시 `Union` 연산 수행.
- 최종 `labels` 배열(점 개수만큼)만 다운로드하도록 변경.
- `dbscan_gpu.cu`: `kernel_dsu_union` 및 `device_union` (atomicCAS) 구현.
- `dbscan_gpu.cpp`: 이웃 리스트 다운로드 및 CPU BFS 로직 제거.

**결과**: 데이터 전송량 수십 배 감소, 전처리 속도 실시간(10Hz) 복구.

## 파이프라인 토픽 흐름
```
VelodyneDriver → Transform → CropBox → PatchWork++ → DBSCAN Clustering
                                            |               |
                                  /patchworkpp/nonground  /clustering/nonground
                                    (x,y,z FLOAT32)      (x,y,z FLOAT32 + label INT32)
```

## 파일 출처 구분
| 파일 | 출처 |
|------|------|
| `nanoflann.hpp` | GitHub에서 다운로드 (header-only KD-tree 라이브러리) |
| `dbscan.hpp` / `dbscan.cpp` | 직접 작성 (nanoflann을 활용한 CPU DBSCAN 알고리즘 구현) |
| `dbscan_gpu.hpp` | 직접 작성 (PIMPL 패턴 GPU DBSCAN C++ 인터페이스) |
| `dbscan_gpu_kernels.h` | 직접 작성 (C 스타일 GPU 커널 API 헤더, opaque handle) |
| `dbscan_gpu.cu` | 직접 작성 (CUDA 커널 4개 + C API 래퍼, nvcc 컴파일) |
| `dbscan_gpu.cpp` | 직접 작성 (GPU 호스트 로직, g++ 컴파일) |
| `clustering_component.hpp` / `clustering_component.cpp` | 직접 작성 (velodyne_cropbox의 composable node 패턴 참고) |
| `CMakeLists.txt` / `package.xml` | 직접 작성 (velodyne_cropbox 빌드 설정 패턴 참고, CUDA 지원 추가) |
| `clustering_params.yaml` | 직접 작성 |

### nanoflann.hpp 다운로드 정보
- 출처: https://raw.githubusercontent.com/jlblancoc/nanoflann/master/include/nanoflann.hpp
- 저장 위치: `src/lidar/lidar_clustering/include/lidar_clustering/nanoflann.hpp`
- 용도: DBSCAN의 radiusSearch를 위한 KD-tree 구현 (외부 빌드 의존성 없이 헤더만 포함)
- header-only 라이브러리이므로 .hpp 파일 하나가 전체 소스코드임 (별도 .cpp 없음)

## 파일 구조
```
src/lidar/lidar_clustering/
├── CMakeLists.txt                              ← 직접 작성 (CUDA + ROS2 빌드)
├── package.xml                                 ← 직접 작성
├── config/
│   └── clustering_params.yaml                  ← 직접 작성 (eps, min_pts, use_gpu)
├── include/lidar_clustering/
│   ├── nanoflann.hpp                           ← GitHub에서 다운로드
│   ├── dbscan.hpp                              ← 직접 작성 (CPU DBSCAN)
│   ├── dbscan_gpu.hpp                          ← 직접 작성 (GPU DBSCAN C++ 인터페이스)
│   ├── dbscan_gpu_kernels.h                    ← 직접 작성 (GPU C API 헤더)
│   └── clustering_component.hpp                ← 직접 작성 (ROS2 노드)
└── src/
    ├── dbscan.cpp                              ← 직접 작성 (CPU DBSCAN, g++)
    ├── dbscan_gpu.cu                           ← 직접 작성 (CUDA 커널, nvcc)
    ├── dbscan_gpu.cpp                          ← 직접 작성 (GPU 호스트 로직, g++)
    └── clustering_component.cpp                ← 직접 작성 (ROS2 노드, g++)
```

## 각 파일 상세 설명

### dbscan.hpp
- `Point3D` 구조체 정의 (float x, y, z)
- `DBSCAN::cluster(points, eps, min_pts)` 정적 메서드 선언 → `vector<int32_t>` labels 반환

### dbscan.cpp
- `PointCloudAdaptor`: nanoflann 인터페이스 구조체 (vector<Point3D>를 zero-copy 참조)
- `KDTree` 타입 정의: nanoflann::KDTreeSingleIndexAdaptor (3차원, L2 거리)
- `DBSCAN::cluster()` 구현:
  1. KD-tree 빌드 (leaf_size=10)
  2. 각 포인트에서 `radiusSearch(eps²)`로 반경 내 이웃 탐색
  3. 이웃 수 >= min_pts이면 core point → BFS 큐로 클러스터 확장
  4. 결과: label >= 0 (클러스터 ID), label == -1 (노이즈)

### dbscan_gpu.hpp
- `DBSCANGpu` 클래스 (PIMPL 패턴)
- CUDA 타입이 헤더에 노출되지 않음 → g++만으로 include 가능
- 인터페이스: `std::vector<int32_t> cluster(const std::vector<Point3D>&, double eps, int min_pts)`
- 생성자에서 GPU 메모리 사전 할당 (기본 max_points=50000)
- move semantics 지원, copy 금지

### dbscan_gpu_kernels.h
- C 스타일 헤더 (`extern "C"`)
- `GpuResourcesHandle`: opaque 포인터 (`struct GpuResources*`)
- GPU 자원 생명주기: `gpu_resources_create()` / `gpu_resources_destroy()`
- 커널 래퍼 함수 5개: `gpu_upload_points`, `gpu_compute_cell_indices`, `gpu_upload_sorted_and_build_bounds`, `gpu_count_neighbors`, `gpu_write_neighbors`
- `gpu_resize_neighbor_list()`: 이웃 리스트 버퍼 동적 확장

### dbscan_gpu.cu
- **nvcc로 컴파일** (STL 헤더 0개, CUB/Thrust 미사용)
- `GpuResources` 구조체 정의: GPU 디바이스 포인터 12개 + 용량 정보
- CUDA 커널 4개:
  - `kernel_compute_cell_indices`: 포인트 → 3D 격자 셀 인덱스 (1 thread/point)
  - `kernel_find_cell_bounds`: 정렬된 배열에서 각 셀의 시작/끝 인덱스 탐색
  - `kernel_count_neighbors`: 27개 인접 셀 탐색, eps 내 이웃 카운트 + core point 판별
  - `kernel_write_neighbors`: 프리컴퓨티드 offset으로 CSR 이웃 리스트 작성
- C API 래퍼: 각 커널을 호출하고 H2D/D2H 메모리 복사 수행
- `calloc`으로 초기화하여 부분 실패 시에도 `gpu_resources_destroy` 안전

### dbscan_gpu.cpp
- **g++로 컴파일** (STL 자유롭게 사용)
- `DBSCANGpu::Impl` 구조체: `GpuResourcesHandle` 래핑, 용량 관리
- `DBSCANGpu::cluster()` 구현 흐름:
  1. SoA 변환 + AABB 계산 (CPU)
  2. GPU로 포인트 업로드
  3. GPU에서 셀 인덱스 계산 → CPU로 다운로드
  4. CPU에서 `std::sort`로 셀 기준 정렬
  5. 정렬된 데이터 GPU 업로드 + 셀 경계 생성
  6. GPU에서 이웃 카운트 + core point 판별 → CPU로 다운로드
  7. CPU에서 prefix sum으로 CSR offset 계산
  8. GPU에서 이웃 리스트 작성 → CPU로 다운로드
  9. CPU에서 BFS 클러스터 확장
- `ensure_capacity()`: 포인트/셀 수 초과 시 GPU 자원 재생성
- grid 크기 > 10,000,000이면 all -1 반환 (CPU fallback 유도)

### clustering_component.hpp
- `ClusteringComponent` 클래스 (rclcpp::Node 상속)
- 멤버: subscription_, publisher_, eps_, min_pts_, **use_gpu_**, **dbscan_gpu_**

### clustering_component.cpp
- 생성자: `eps`(기본 0.5), `min_pts`(기본 5), **`use_gpu`(기본 true)** 파라미터 선언
- GPU 초기화: `DBSCANGpu(50000)` 생성, 실패 시 `use_gpu_ = false`로 CPU fallback
- `pointCloudCallback()`:
  1. 입력 PointCloud2에서 x, y, z 추출 → `vector<Point3D>` 변환
  2. **GPU/CPU 분기**: `use_gpu_ && dbscan_gpu_` → GPU, 아니면 CPU
  3. 새 PointCloud2 생성: x(offset 0), y(offset 4), z(offset 8), label(offset 12) → point_step=16
  4. header 타임스탬프 그대로 보존하여 `/clustering/nonground`로 발행
  5. 1초 간격 throttle 로그: **[GPU]/[CPU]** 태그, 포인트 수, 클러스터 수, 노이즈 수 출력
- `RCLCPP_COMPONENTS_REGISTER_NODE(lidar_clustering::ClusteringComponent)`로 composable node 등록

### clustering_params.yaml
```yaml
clustering_component:
  ros__parameters:
    eps: 0.5        # 이웃 탐색 반경 (미터)
    min_pts: 5      # 코어 포인트 최소 이웃 수
    use_gpu: true   # GPU 사용 여부 (false면 CPU fallback)
```

### CMakeLists.txt
- `project(lidar_clustering LANGUAGES CXX CUDA)`: CUDA 언어 활성화
- `cmake_minimum_required(VERSION 3.18)`: CUDAToolkit 지원
- `CMAKE_CUDA_ARCHITECTURES 86`: sm_86 PTX 빌드 (sm_89 JIT)
- `add_library(dbscan_gpu_kernels STATIC src/dbscan_gpu.cu)`: nvcc STATIC lib
- `add_library(clustering_component SHARED src/dbscan.cpp src/dbscan_gpu.cpp src/clustering_component.cpp)`: g++ SHARED lib
- `target_link_libraries(clustering_component dbscan_gpu_kernels)`: STATIC → SHARED 링크
- `POSITION_INDEPENDENT_CODE ON`, `CUDA_SEPARABLE_COMPILATION ON`

### package.xml
- 패키지명: lidar_clustering, 빌드타입: ament_cmake
- 의존: rclcpp, rclcpp_components, sensor_msgs

## 기존 파일 수정 내역

### lidar_launch/launch/my_velodyne_VLP16-composed-launch.py
- clustering 파라미터 로드 코드 추가 (lidar_clustering 패키지의 clustering_params.yaml 읽기)
- ComposableNodeContainer에 5번째 노드 추가:
  ```python
  ComposableNode(
      package='lidar_clustering',
      plugin='lidar_clustering::ClusteringComponent',
      name='clustering_component',
      parameters=[clustering_params],
      remappings=[
          ('input', '/patchworkpp/nonground'),
          ('output', '/clustering/nonground')
      ]),
  ```

### lidar_launch/package.xml
- `<exec_depend>lidar_clustering</exec_depend>` 한 줄 추가

## 검증 방법
```bash
colcon build --packages-select lidar_clustering lidar_launch
ros2 topic hz /clustering/nonground          # ~10Hz 확인
ros2 topic echo /clustering/nonground --once  # x,y,z,label 필드 확인
```

## 문제점: clustering 패키지 추가 후 velodyne_points 토픽 발행 불안정

### 증상
- `lidar_clustering` 패키지를 추가한 뒤 `velodyne_points` 토픽이 10Hz로 안정적으로 나오지 않음
- 전처리 패키지를 빼고 velodyne 기본 패키지만 실행하면 정상 10Hz 출력

### 원인: SingleThreadedExecutor에서 DBSCAN 콜백 블로킹

launch 파일에서 `component_container`를 사용하고 있었음:
```python
executable='component_container',  # ← SingleThreadedExecutor (1스레드)
```

`component_container`는 내부적으로 **SingleThreadedExecutor**를 사용한다.
5개 노드(driver, transform, cropbox, patchwork++, DBSCAN)가 **1프로세스 1스레드**에서 모든 콜백이 순차(serial) 실행된다.

#### ROS2 Component Container 스레딩 모델

| 실행 방식 | 프로세스 | 스레드(Executor) |
|---|---|---|
| `component_container` | 1개 프로세스 | **SingleThreadedExecutor** (1스레드) |
| `component_container_mt` | 1개 프로세스 | **MultiThreadedExecutor** (멀티스레드) |
| 별도 노드 실행 | 노드당 1프로세스 | 각자 스레드 |

#### 메시지 수신부터 콜백 실행까지의 흐름
```
[네트워크]  →  [DDS]  →  [Subscription 큐]  →  [Executor]  →  [콜백 실행]
   ①           ②            ③                   ④              ⑤
```

1. **네트워크**: Velodyne LiDAR가 100ms마다(10Hz) UDP 패킷 전송
2. **DDS 레이어**: DDS 미들웨어가 자체 스레드에서 UDP를 받아 역직렬화 (executor와 무관하게 독립 동작)
3. **Subscription 큐에 적재**: DDS가 메시지를 해당 subscription의 내부 큐에 넣음. **QoS는 이 큐에 영향**을 줌
4. **Executor가 큐에서 콜백을 꺼냄**: SingleThreadedExecutor는 콜백을 하나 꺼내서 끝날 때까지 실행한 후 다음 콜백을 꺼냄
5. **콜백 실행**: 해당 콜백이 리턴될 때까지 executor는 다음 콜백을 처리하지 못함

#### SingleThreadedExecutor 내부 동작 (의사코드)
```cpp
while (running) {
  callback = wait_for_ready_callback();  // 큐에서 하나 꺼냄
  callback.execute();                     // ← 이게 끝날 때까지 여기서 멈춤
  // 끝나야 다음 줄로 넘어감
}
```

#### DBSCAN 콜백이 오래 걸리는 이유

`clustering_component.cpp`의 `pointCloudCallback()`이 매 프레임마다 동기적으로 실행하는 작업:
1. **KD-tree 빌드** - 모든 포인트로 공간 인덱스 구조 생성
2. **모든 포인트마다 radiusSearch** - N개 포인트 각각에 대해 반경 내 이웃 탐색
3. **BFS로 클러스터 확장** - core point마다 큐 기반 확장

비지면(non-ground) 포인트가 10,000개 이상이면 이 연산이 수십~수백 ms 소요될 수 있다.

#### 타임라인으로 보는 블로킹 문제 (DBSCAN 80ms 가정)

```
시간(ms)  0    10    20    30    40    50    60    70    80    90   100
          |     |     |     |     |     |     |     |     |     |     |
Executor: [driver 2ms][transform 5ms][cropbox 1ms][patchwork 8ms][====== DBSCAN 80ms ======]
                                                                  ↑                        ↑
                                                              다른 콜백                  여기서야
                                                              실행 불가                 다음 콜백 가능

DDS 큐:   ← 이 동안 driver 큐에 메시지가 계속 쌓임 →
          t=10ms: velodyne 패킷 도착 → 큐에 적재 (처리 못함)
          t=20ms: velodyne 패킷 도착 → 큐에 적재 (처리 못함)
          ...
          t=80ms: 큐 depth 초과 → Best Effort이므로 메시지 DROP
```

### QoS와 Executor는 별개의 레이어

| 개념 | 역할 | 결정하는 것 |
|---|---|---|
| **QoS** | DDS 통신 정책 | 메시지 유실 시 재전송 여부, 큐 크기 |
| **Executor** | 콜백 실행 모델 | 콜백을 순차로 돌릴지, 병렬로 돌릴지 |

- **Best Effort**: 메시지 유실 시 재전송 안 함 → "콜백을 비동기로 실행하겠다"는 뜻이 **아님**
- **Reliable**: 메시지 유실 시 재전송 → publisher에게 backpressure 신호
- Best Effort이기 때문에 오히려 문제가 악화됨: DBSCAN 블로킹 동안 큐가 넘치면 메시지가 **그냥 버려짐**

#### QoS 큐 동작 (SensorDataQoS 기본: Best Effort + Keep Last + depth=5)

Subscription 큐는 **FIFO(선입선출)**이다. DBSCAN이 끝나면 최신 메시지가 아니라 **큐에 쌓인 순서대로** 처리한다:

```
DBSCAN이 느려서 큐가 쌓이는 경우:
t=0ms:   nonground_0 → DBSCAN 시작
t=100ms: nonground_1 → 큐 [1]
t=200ms: nonground_2 → 큐 [1, 2]
t=300ms: nonground_3 → 큐 [1, 2, 3]
t=400ms: nonground_4 → 큐 [1, 2, 3, 4]
t=500ms: nonground_5 → 큐 [1, 2, 3, 4, 5]  ← depth=5 꽉 참
t=600ms: nonground_6 → 큐 [2, 3, 4, 5, 6]  ← nonground_1 버려짐
t=700ms: DBSCAN 끝   → nonground_2를 꺼냄   ← 700ms 전의 오래된 데이터 처리
```

최신 메시지만 받으려면 QoS depth를 1로 설정해야 한다:
```cpp
auto qos = rclcpp::SensorDataQoS();
qos.keep_last(1);  // 큐에 최신 1개만 유지
```

### 해결: `component_container` → `component_container_mt` 변경

MultiThreadedExecutor는 스레드 풀에서 콜백을 병렬 실행한다:
```python
executable='component_container_mt',  # ← MultiThreadedExecutor
```

```
시간(ms)  0    10    20    30    40    50    60    70    80    90   100
          |     |     |     |     |     |     |     |     |     |     |
스레드1:  [driver][driver][driver][driver][driver]...     ← 10Hz 안정
스레드2:  [transform][transform][transform]...
스레드3:  [cropbox][patchwork][cropbox][patchwork]...
스레드4:  [========= DBSCAN =========][===== DBSCAN =====]  ← 느려도 독립적
```

DBSCAN이 오래 걸려도 driver/transform은 자기 스레드에서 독립적으로 10Hz를 유지한다.

### GPU 가속 검토

| 노드 | CPU 부하 | GPU 가능? | 현실적? |
|---|---|---|---|
| Velodyne Driver | 낮음 (UDP I/O) | 의미없음 | X |
| Velodyne Transform | 낮음 | 가능하나 불필요 | X |
| CropBox | 매우 낮음 (단순 범위 비교) | 가능하나 불필요 | X |
| Patchwork++ | 중간 | 공식 GPU 버전 없음 | 어려움 |
| DBSCAN Clustering | **높음** (KD-tree + radius search) | cuML, CUDA 등 존재 | O |

GPU 가속이 현실적인 건 DBSCAN뿐이며, 우선 `component_container_mt`로 전환하여 토픽 블로킹 문제를 해결한 뒤, 필요시 GPU DBSCAN을 검토한다.

## GPU DBSCAN 구현

### 개요
CPU DBSCAN(nanoflann KD-tree)이 포인트 수가 많을 때 수십~수백 ms 걸리는 문제를 해결하기 위해 CUDA GPU 가속을 구현.
CPU fallback을 유지하여 GPU 초기화 실패 시 자동으로 CPU 경로를 사용한다.

### 환경
- GPU: NVIDIA RTX 4060 (8GB, sm_89 Ada Lovelace)
- CUDA Toolkit: 11.5 (`/usr/local/cuda-11.5/`)
- GCC: 11.4
- sm_86(Ampere) PTX 빌드 → sm_89(Ada) JIT 컴파일로 동작

### 알고리즘: 3D Grid 기반 GPU DBSCAN (Hybrid)

KD-tree는 GPU에 부적합(재귀적, warp divergence). 대신 **3D 균일 격자(Voxel Grid)** 로 공간 인덱싱한다.

**Phase 1: GPU - 이웃 탐색**
1. CPU에서 AABB(바운딩박스) 계산
2. GPU에서 셀 인덱스 계산 (1 thread/point), CPU로 다운로드
3. CPU에서 `std::sort`로 셀 기준 정렬 → GPU로 업로드
4. GPU에서 셀 경계(start/end) 배열 생성
5. GPU에서 27개 인접 셀 탐색, eps 내 이웃 수 카운트 → core point 판별
6. CPU에서 prefix sum으로 CSR offset 계산
7. GPU에서 CSR 이웃 리스트 작성, CPU로 다운로드

**Phase 2: CPU - BFS 라벨 할당**
- GPU에서 받은 CSR 이웃 리스트 + is_core 배열로 BFS 확장
- 10k-30k 포인트 기준 < 1ms

### cuML vs 순수 CUDA C++ 선택

cuML은 GPU DBSCAN을 제공하지만:
- RAPIDS/cuML 설치가 무겁고 복잡 (conda 환경, 수 GB)
- YOLO(카메라 차선인식)는 PyTorch/TensorRT를 사용하므로 cuML과 무관
- 라이다 포인트 수(10k-30k)에서는 순수 CUDA로 충분한 성능

따라서 외부 의존성 없는 **순수 CUDA C++** 구현을 선택하였다.

### 빌드 문제: nvcc 11.5 + GCC 11 비호환

#### 문제
nvcc 11.5가 GCC 11의 `<functional>` 헤더를 파싱하지 못함:
```
/usr/include/c++/11/bits/std_function.h:435:145:
error: parameter packs not expanded with '...'
```

Thrust와 CUB 모두 `<functional>`을 간접 포함(transitive include)하므로, `.cu` 파일에서 이들을 사용하면 같은 에러 발생.

#### 시도한 해결 방법
1. `CMAKE_CUDA_STANDARD`를 17에서 14로 변경 → 동일 에러 (GCC 헤더 자체가 문제)
2. Thrust를 CUB(저수준 API)로 교체 → 동일 에러 (CUB도 `<functional>` 포함)

#### 최종 해결: 아키텍처 분리 (C API 경계)

`.cu` 파일에서 STL 헤더를 완전히 제거하고, C 스타일 API로 분리:

```
dbscan_gpu.cu (nvcc)          dbscan_gpu.cpp (g++)
┌─────────────────────┐       ┌─────────────────────┐
│ CUDA 커널 4개       │       │ DBSCANGpu 클래스    │
│ - compute_cell_idx  │       │ - SoA 변환          │
│ - find_cell_bounds  │  C    │ - std::sort (정렬)  │
│ - count_neighbors   │ ←API→ │ - prefix sum        │
│ - write_neighbors   │       │ - BFS 클러스터링    │
│                     │       │ - std::vector 등    │
│ C API 래퍼 함수     │       │                     │
│ (opaque handle)     │       │                     │
└─────────────────────┘       └─────────────────────┘
     ↓ nvcc 컴파일                 ↓ g++ 컴파일
  STATIC lib (.a)              SHARED lib (.so)
  (dbscan_gpu_kernels)         (clustering_component)
```

- **dbscan_gpu_kernels.h**: C 헤더, `GpuResourcesHandle` opaque 포인터 패턴
- **dbscan_gpu.cu**: CUDA 커널 + C API 래퍼만 포함 (STL 헤더 0개)
- **dbscan_gpu.cpp**: 호스트 로직, STL 자유롭게 사용 (g++ 컴파일)

이 방식으로 nvcc는 GCC 11의 STL 헤더를 전혀 보지 않게 된다.

### 파일 구조 (GPU 추가 후)
```
src/lidar/lidar_clustering/
├── CMakeLists.txt                              ← 수정 (CUDA 지원, STATIC+SHARED 구조)
├── config/
│   └── clustering_params.yaml                  ← 수정 (use_gpu: true 추가)
├── include/lidar_clustering/
│   ├── nanoflann.hpp                           ← 기존 유지
│   ├── dbscan.hpp                              ← 기존 유지 (CPU fallback)
│   ├── clustering_component.hpp                ← 수정 (DBSCANGpu 멤버 추가)
│   ├── dbscan_gpu.hpp                          ← 신규 (PIMPL 패턴, C++ 인터페이스)
│   └── dbscan_gpu_kernels.h                    ← 신규 (C API, opaque handle)
└── src/
    ├── dbscan.cpp                              ← 기존 유지 (CPU fallback)
    ├── dbscan_gpu.cu                           ← 신규 (CUDA 커널 + C 래퍼, nvcc)
    ├── dbscan_gpu.cpp                          ← 신규 (호스트 로직, g++)
    └── clustering_component.cpp                ← 수정 (GPU/CPU 분기)
```

### 주요 설계 원칙
1. **PIMPL 패턴**: `dbscan_gpu.hpp`에 CUDA 타입 노출 안 함 → `clustering_component.cpp`는 g++만으로 컴파일
2. **사전 할당**: `cudaMalloc`은 생성자에서 1회 실행 (프레임마다 할당하면 1-5ms 낭비)
3. **CPU fallback**: GPU 초기화 실패 또는 CUDA 에러 시 자동으로 CPU 경로
4. **STATIC + SHARED 분리**: `.cu` → nvcc로 STATIC lib, `.cpp` → g++로 SHARED lib (ROS2 헤더와 nvcc 충돌 방지)

### CMakeLists.txt 구조
```cmake
# CUDA 커널 (nvcc 컴파일, STL 미포함)
add_library(dbscan_gpu_kernels STATIC src/dbscan_gpu.cu)
set_target_properties(dbscan_gpu_kernels PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON)
target_link_libraries(dbscan_gpu_kernels CUDA::cudart)

# ROS2 컴포넌트 (g++ 컴파일, STL 사용)
add_library(clustering_component SHARED
  src/dbscan.cpp src/dbscan_gpu.cpp src/clustering_component.cpp)
target_link_libraries(clustering_component dbscan_gpu_kernels)
```

### GPU/CPU 분기 로직 (`clustering_component.cpp`)
```cpp
// 생성자: GPU 초기화 (실패 시 CPU fallback)
if (use_gpu_) {
  try {
    dbscan_gpu_ = std::make_unique<DBSCANGpu>(50000);
  } catch (...) {
    use_gpu_ = false;  // CPU fallback
  }
}

// 콜백: GPU/CPU 분기
if (use_gpu_ && dbscan_gpu_) {
  labels = dbscan_gpu_->cluster(points, eps_, min_pts_);
} else {
  labels = DBSCAN::cluster(points, eps_, min_pts_);
}
```

### 검증 방법
```bash
# 빌드
colcon build --packages-select lidar_clustering

# 실행 후 확인
ros2 topic hz /clustering/nonground          # 10Hz 유지 확인
ros2 topic echo /clustering/nonground --once  # label 필드 확인

# GPU 로그 확인: "GPU DBSCAN initialized (max 50000 points)" 출력 확인
# CPU fallback 테스트: clustering_params.yaml에서 use_gpu: false로 변경
```

---
# 2.10
- 컴포넌트 아키텍처 전환 완료
- velodyne_point_cloud 패키지에서 전방 화각(180도) crop (pcl library 이용)
- velodyne_cropbx 패키지에서 velodyne_point_cloud에서 받은 토픽을 구독하고 roi 영역을 crop-box함.
