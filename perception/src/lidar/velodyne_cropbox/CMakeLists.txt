cmake_minimum_required(VERSION 3.8)
project(velodyne_cropbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create shared library component
add_library(cropbox_component SHARED
  src/cropbox_component.cpp
)

target_include_directories(cropbox_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(cropbox_component PUBLIC cxx_std_17)

ament_target_dependencies(cropbox_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

# Register the component with rclcpp_components
rclcpp_components_register_nodes(cropbox_component "velodyne_cropbox::CropBoxComponent")

# Install the library
install(TARGETS cropbox_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cropbox_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
