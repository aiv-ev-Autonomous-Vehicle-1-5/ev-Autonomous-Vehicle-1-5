# ev_msgs

## Package 정보
- **패키지명**: ev_msgs
- **설명**: EV 자율주행 프로젝트 공유 메시지 정의 패키지
- **의존성**: `geometry_msgs`, `std_msgs`

## 메시지 정의

### 1. BBox.msg
단일 바운딩 박스 메시지

| 필드 | 타입 | 설명 |
|---|---|---|
| `position` | `geometry_msgs/Point` | 바운딩 박스 중심 위치 |
| `size_x` | `float32` | X축 크기 |
| `size_y` | `float32` | Y축 크기 |
| `size_z` | `float32` | Z축 크기 |
| `label` | `int32` | 객체 레이블 |

### 2. BBoxArray.msg
바운딩 박스 배열 메시지

| 필드 | 타입 | 설명 |
|---|---|---|
| `header` | `std_msgs/Header` | 메시지 헤더 (타임스탬프, 프레임 ID) |
| `bboxes` | `BBox[]` | 바운딩 박스 배열 |

### 3. LaneBoundary.msg
단일 차선 경계 메시지

| 필드 | 타입 | 설명 |
|---|---|---|
| `header` | `std_msgs/Header` | 메시지 헤더 (타임스탬프, 프레임 ID) |
| `points` | `geometry_msgs/Point[]` | 차선 경계 포인트 배열 |
| `confidence` | `float32` | 검출 신뢰도 |
| `lane_id` | `int32` | 차선 클러스터 ID (YOLO 인스턴스 세그멘테이션 기준) |

### 4. LaneBoundaryArray.msg
차선 경계 배열 메시지

| 필드 | 타입 | 설명 |
|---|---|---|
| `header` | `std_msgs/Header` | 메시지 헤더 (타임스탬프, 프레임 ID) |
| `boundaries` | `LaneBoundary[]` | 차선 경계 배열 |
