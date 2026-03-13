cmake_minimum_required(VERSION 3.8)
project(pp_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(t870_msgs REQUIRED)
find_package(erp42_msgs REQUIRED)

include_directories(include)

# 라이브러리: Pure Pursuit 알고리즘 (순수 계산, 노드 비의존)
add_library(pp_pursuit SHARED
  src/pursuit/pursuit_algorithm.cpp
)
ament_target_dependencies(pp_pursuit
  visualization_msgs
)

# 라이브러리: 디버그 시각화 (RViz2 마커)
add_library(pp_debug_viz SHARED
  src/debug/debug_visualizer.cpp
)
ament_target_dependencies(pp_debug_viz
  rclcpp
  visualization_msgs
)

# 컴포넌트: 메인 노드 (ComposableNode)
add_library(pp_controller_component SHARED
  src/nodes/pure_pursuit_relative_node.cpp
)
target_link_libraries(pp_controller_component
  pp_pursuit
  pp_debug_viz
)
ament_target_dependencies(pp_controller_component
  rclcpp rclcpp_components
  std_msgs visualization_msgs t870_msgs erp42_msgs
)

# 컴포넌트 등록 + standalone 실행 파일 자동 생성
rclcpp_components_register_node(pp_controller_component
  PLUGIN "pp_controller_cpp::PurePursuitRelativeNode"
  EXECUTABLE pure_pursuit_relative_node
)

install(TARGETS
  pp_pursuit
  pp_debug_viz
  pp_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
