# velodyne_cropbox — 3D CropBox 포인트클라우드 필터

## 개요

3D 축 정렬 바운딩 박스 기반 포인트클라우드 필터링 컴포넌트. 지정된 3D 박스 내부 또는 외부의 포인트만 통과시킨다.

- Node: `CropBoxComponent` (ComposableNode)
- 모든 원본 포인트 필드 보존 (zero-copy 효율)

## 기능

```
입력 PointCloud2
    │
    ├─ negative = false → 박스 내부 포인트만 통과
    ├─ negative = true  → 박스 외부 포인트만 통과
    │
    ▼
출력 PointCloud2 (원본 필드 보존)
```

## 파라미터

| 파라미터 | 타입 | 설명 |
|---------|------|------|
| `x_min` | double | X 최소값 (m) |
| `x_max` | double | X 최대값 (m) |
| `y_min` | double | Y 최소값 (m) |
| `y_max` | double | Y 최대값 (m) |
| `z_min` | double | Z 최소값 (m) |
| `z_max` | double | Z 최대값 (m) |
| `negative` | bool | true: 박스 외부 통과, false: 박스 내부 통과 |

## 소스 파일

| 파일 | 역할 |
|------|------|
| `include/velodyne_cropbox/cropbox_component.hpp` | 컴포넌트 클래스 선언 |
| `src/cropbox_component.cpp` | CropBox 필터링 로직 구현 |
| `config/cropbox.yaml` | 로컬 파라미터 (참고용) |
