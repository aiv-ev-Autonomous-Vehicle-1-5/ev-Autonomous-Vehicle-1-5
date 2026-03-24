# t870_cmake 패키지 파이프라인

## 패키지 정보
- **Package**: t870_cmake
- **Description**: t870_ros 패키지용 CMake 빌드 매크로 패키지
- ROS 토픽/노드 없음 (빌드 도구 전용)

---

## 제공 매크로

### t870_package()

`ament_auto_find_build_dependencies()` + 컴파일러 설정 래퍼 매크로.

t870_ros 하위 패키지들의 CMakeLists.txt에서 공통 빌드 설정을 간소화하기 위해 사용된다.
