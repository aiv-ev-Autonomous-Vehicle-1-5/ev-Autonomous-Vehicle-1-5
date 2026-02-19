# 작업 일지: LiDAR Clustering 개발 및 성능 최적화
**날짜:** 2026년 2월 12일  
**대상:** `lidar_clustering` 패키지 및 `lidar_launch` 구성

## 1. 개요
Patchwork++(지면 제거) 이후 단계인 **비지면(Non-ground) 포인트 클라우드에 대한 객체 클러스터링(DBSCAN)**을 구현하고, 실시간성(10Hz) 확보를 위해 GPU 가속 및 스레딩 모델을 최적화함.

---

## 2. 작업 상세 내용

### 2.1. `lidar_clustering` 패키지 신규 개발
- **목표:** Patchwork++에서 분류된 `/patchworkpp/nonground` 토픽을 구독하여 장애물 단위로 그룹화(Labeling).
- **초기 구현:**
    - **알고리즘:** DBSCAN (Density-Based Spatial Clustering of Applications with Noise).
    - **CPU 버전:** `nanoflann` (KD-Tree) 기반 구현.
    - **GPU 버전(초기):** CUDA를 이용한 이웃 탐색(Neighbor Search) 가속.

### 2.2. 성능 이슈 발생 (0.27 Hz)
- **증상:**
    - CPU 버전은 포인트가 많을 때 느림.
    - 초기 GPU 버전을 적용했으나 **Topic Hz가 10Hz → 0.27Hz로 급락**하는 현상 발생.
- **원인 분석:**
    1.  **PCIe 대역폭 병목 (Critical):** GPU에서 계산한 **모든 이웃 리스트(Neighbor List)**를 CPU로 복사함. 포인트당 수백 개의 이웃이 존재하므로 매 프레임 수백 MB의 데이터 전송 발생.
    2.  **CPU 병목:** 이웃 리스트를 받은 후, 실제 클러스터링(BFS) 로직을 CPU에서 수행하여 GPU 이점을 살리지 못함.
- **해결: GPU DSU (Disjoint Set Union) 도입**
    - **알고리즘 변경:** 이웃 리스트를 전송하지 않고, **GPU 메모리 내부에서 Union-Find 알고리즘**으로 점들을 연결.
    - **커널 구현:** `dbscan_gpu.cu`에 `device_union` (AtomicCAS 사용) 및 `kernel_dsu_union` 구현.
    - **최적화:** 최종 결과인 `labels` 배열(Point 개수만큼의 Int)만 CPU로 다운로드.
- **결과:** 데이터 전송량 수십 배 감소, **10Hz 실시간 성능 복구**.

### 2.3. 파이프라인 블로킹 및 스레딩 모델 최적화
- **이슈 제기:** `component_container` 사용 시 DBSCAN과 같은 무거운 노드가 전체 파이프라인을 블로킹하여 **입력 데이터(LiDAR Packet) 유실** 가능성 존재.
- **조치:** Launch 파일 확인 결과 **`component_container_mt` (MultiThreadedExecutor)**가 이미 적용되어 있음을 확인.
- **심화 분석 (Single vs Multi):**
    - **`_mt` (Multi-Thread):** Driver가 DBSCAN 처리 중에도 별도 스레드에서 데이터를 수신하므로 **입력 유실 방지**에 탁월. 단, 직렬 파이프라인에서는 컨텍스트 스위칭 오버헤드로 인해 순수 연산 Hz는 소폭 하락할 수 있음.
    - **결론:** 현재 시스템(직렬 구조)에서는 **싱글 스레드(`component_container`)**가 오버헤드가 적어 유리하나, 데이터 안정성을 위해 멀티스레드 컨테이너 사용을 유지하거나 필요시 분리.

### 2.4. 아키텍처 및 통신 효율성 검토
- **Python vs C++:**
    - Python 노드 간 Shared Memory가 가능하더라도, **객체 변환(Deserialization) 및 연산 속도(GIL)** 한계로 인해 대용량 PointCloud 처리에는 **C++ Component**가 필수적임을 재확인.
- **컨테이너 분리 전략:**
    - 향후 병렬화가 필요할 경우:
        - **전처리 컨테이너 (Single Thread):** Driver → Transform → CropBox (고속 순차 처리)
        - **분석 컨테이너 (Multi Thread):** Patchwork++ → DBSCAN, Deep Learning (병렬 처리)
        - 두 컨테이너 간 통신은 **Shared Memory (Iceoryx/CycloneDDS)** 활용.

### 2.5. 동적 환경 성능 저하 (9.9Hz → 9.1Hz)
- **증상:** 라이다를 들고 움직일 때 처리 속도가 소폭 저하됨.
- **원인:** 정지 상태보다 관측되는 유효 포인트(Non-ground) 수가 증가하여 DBSCAN 연산량 증가.
- **향후 계획:** **Voxel Grid Downsampling**을 적용하여 포인트 밀도를 일정하게 유지함으로써 처리 속도를 안정화할 예정.

---

## 3. 최종 시스템 구성
- **Pipeline:** `Velodyne Driver` → `Transform` → `CropBox` → `Patchwork++` → `DBSCAN (GPU DSU)`
- **Executor:** `component_container_mt` (Multi-Threaded)
- **Status:** 10Hz 실시간 처리 달성 (GPU 가속 적용 완료)
