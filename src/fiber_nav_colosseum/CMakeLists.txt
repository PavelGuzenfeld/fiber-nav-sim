cmake_minimum_required(VERSION 3.14)
project(fiber_nav_colosseum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(fiber_nav_physics REQUIRED)

# ──────────────────────────────────────────────
# FetchContent: msgpack-cxx (header-only C++ msgpack)
# ──────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  msgpack-cxx
  GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
  GIT_TAG        cpp-6.1.1
  GIT_SHALLOW    TRUE
  SYSTEM
)
set(MSGPACK_USE_BOOST OFF CACHE BOOL "" FORCE)
set(MSGPACK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MSGPACK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(msgpack-cxx)

# ──────────────────────────────────────────────
# AirSim RPC client library (raw TCP + msgpack-rpc)
# ──────────────────────────────────────────────
add_library(airsim_rpc_client
  src/airsim_rpc_client.cpp
)
target_include_directories(airsim_rpc_client PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(airsim_rpc_client PUBLIC msgpack-cxx)

# ──────────────────────────────────────────────
# Colosseum bridge node (AirSim API → ROS 2 topics)
# ──────────────────────────────────────────────
add_executable(colosseum_bridge_node
  src/colosseum_bridge_node.cpp
)
target_include_directories(colosseum_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(colosseum_bridge_node airsim_rpc_client)
ament_target_dependencies(colosseum_bridge_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs px4_msgs
  cv_bridge image_transport tf2_ros rosgraph_msgs
)

# ──────────────────────────────────────────────
# Colosseum physics bridge node (VTOL physics + MAVLink HIL + AirSim rendering)
# ──────────────────────────────────────────────
add_executable(colosseum_physics_bridge_node
  src/colosseum_physics_bridge_node.cpp
)
target_include_directories(colosseum_physics_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(colosseum_physics_bridge_node
  airsim_rpc_client
  fiber_nav_physics::fiber_nav_physics
)
ament_target_dependencies(colosseum_physics_bridge_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs px4_msgs
  tf2_ros rosgraph_msgs
)

install(TARGETS
  colosseum_bridge_node
  colosseum_physics_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ──────────────────────────────────────────────
# Tests (doctest)
# ──────────────────────────────────────────────
if(BUILD_TESTING)
  FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
    SYSTEM
  )
  FetchContent_MakeAvailable(doctest)

  add_executable(test_airsim_rpc_client tests/airsim_rpc_client_test.cpp)
  target_link_libraries(test_airsim_rpc_client PRIVATE
    airsim_rpc_client doctest::doctest)
  target_include_directories(test_airsim_rpc_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_airsim_rpc_client PRIVATE cxx_std_23)

  enable_testing()
  add_test(NAME test_airsim_rpc_client COMMAND test_airsim_rpc_client)
endif()

ament_package()
