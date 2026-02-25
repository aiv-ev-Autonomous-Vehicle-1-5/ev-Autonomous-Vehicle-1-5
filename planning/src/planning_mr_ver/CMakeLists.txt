cmake_minimum_required(VERSION 3.8)
project(planning_mr_ver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(track_msgs REQUIRED)

include_directories(include)

# ============================================================================
# Library 1: planning_mr_costmap (costmap generation)
# ============================================================================
add_library(planning_mr_costmap SHARED
  src/costmap/costmap_generator.cpp
)
ament_target_dependencies(planning_mr_costmap
  rclcpp nav_msgs geometry_msgs track_msgs
)
install(TARGETS planning_mr_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 2: planning_mr_planner (magnetic planner)
# ============================================================================
add_library(planning_mr_planner SHARED
  src/planner/magnetic_planner.cpp
)
ament_target_dependencies(planning_mr_planner
  rclcpp geometry_msgs track_msgs
)
install(TARGETS planning_mr_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 3: planning_mr_postprocess (reused postprocessor)
# ============================================================================
add_library(planning_mr_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)
ament_target_dependencies(planning_mr_postprocess
  rclcpp geometry_msgs track_msgs
)
install(TARGETS planning_mr_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Component: mr_planner_component (ComposableNode)
# ============================================================================
add_library(mr_planner_component SHARED
  src/nodes/mr_planner_node.cpp
)
target_link_libraries(mr_planner_component
  planning_mr_costmap
  planning_mr_planner
  planning_mr_postprocess
)
ament_target_dependencies(mr_planner_component
  rclcpp rclcpp_components
  nav_msgs geometry_msgs std_msgs track_msgs
)

rclcpp_components_register_node(mr_planner_component
  PLUGIN "planning_mr_ver::MRPlannerNode"
  EXECUTABLE mr_planner_node_exe
)

install(TARGETS mr_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers, config, launch
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
