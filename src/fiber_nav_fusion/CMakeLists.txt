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

  install(TARGETS test_fusion
    DESTINATION lib/${PROJECT_NAME}
  )

  # Register test with ctest directly (ament_add_test has XML parsing issues with doctest)
  enable_testing()
  add_test(NAME test_fusion COMMAND test_fusion)
endif()

ament_package()
