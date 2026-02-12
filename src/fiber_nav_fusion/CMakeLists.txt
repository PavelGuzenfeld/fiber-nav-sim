cmake_minimum_required(VERSION 3.8)
project(fiber_nav_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fiber_nav_sensors REQUIRED)
find_package(Eigen3 REQUIRED)

# Position EKF library (pure math, no ROS)
add_library(position_ekf src/position_ekf.cpp)
target_include_directories(position_ekf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(position_ekf PUBLIC Eigen3::Eigen)
target_compile_features(position_ekf PUBLIC cxx_std_23)

# Fusion node
add_executable(fiber_vision_fusion src/fiber_vision_fusion.cpp)
ament_target_dependencies(fiber_vision_fusion
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
  fiber_nav_sensors
)

# Position EKF node
add_executable(position_ekf_node src/position_ekf_node.cpp)
target_include_directories(position_ekf_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(position_ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  px4_msgs
  fiber_nav_sensors
  tf2
)
target_link_libraries(position_ekf_node position_ekf Eigen3::Eigen)

# Landmark correction node
add_executable(landmark_correction src/landmark_correction.cpp)
ament_target_dependencies(landmark_correction
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  fiber_vision_fusion
  position_ekf_node
  landmark_correction
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Fetch doctest using FetchContent (CMake 3.14+)
  include(FetchContent)
  FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
    SYSTEM
  )
  FetchContent_MakeAvailable(doctest)

  # Unit tests
  add_executable(test_fusion test/test_fusion.cpp)
  target_link_libraries(test_fusion PRIVATE doctest::doctest)
  target_compile_features(test_fusion PRIVATE cxx_std_23)

  # Position EKF tests
  add_executable(test_position_ekf test/test_position_ekf.cpp)
  target_link_libraries(test_position_ekf PRIVATE doctest::doctest position_ekf)

  install(TARGETS test_fusion test_position_ekf
    DESTINATION lib/${PROJECT_NAME}
  )

  # Register tests with ctest directly (ament_add_test has XML parsing issues with doctest)
  enable_testing()
  add_test(NAME test_fusion COMMAND test_fusion)
  add_test(NAME test_position_ekf COMMAND test_position_ekf)
endif()

ament_package()
