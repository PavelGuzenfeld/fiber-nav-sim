cmake_minimum_required(VERSION 3.8)
project(fiber_nav_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Hold mode node
add_executable(hold_mode_node src/hold_mode_node.cpp)
target_include_directories(hold_mode_node PRIVATE include)
ament_target_dependencies(hold_mode_node rclcpp px4_msgs px4_ros2_cpp)
target_link_libraries(hold_mode_node Eigen3::Eigen)

# Canyon mission node
add_executable(canyon_mission_node src/canyon_mission_node.cpp)
target_include_directories(canyon_mission_node PRIVATE include)
ament_target_dependencies(canyon_mission_node rclcpp px4_msgs px4_ros2_cpp)
target_link_libraries(canyon_mission_node Eigen3::Eigen)

# VTOL navigation node
add_executable(vtol_navigation_node src/vtol_navigation_node.cpp)
target_include_directories(vtol_navigation_node PRIVATE include)
ament_target_dependencies(vtol_navigation_node rclcpp px4_msgs px4_ros2_cpp)
target_link_libraries(vtol_navigation_node Eigen3::Eigen)

install(TARGETS
  hold_mode_node
  canyon_mission_node
  vtol_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_canyon_waypoints test/test_canyon_waypoints.cpp)
  target_include_directories(test_canyon_waypoints PRIVATE include)
  target_link_libraries(test_canyon_waypoints Eigen3::Eigen)

  ament_add_gtest(test_vtol_navigation test/test_vtol_navigation.cpp)
  target_include_directories(test_vtol_navigation PRIVATE include)
  target_link_libraries(test_vtol_navigation Eigen3::Eigen)
endif()

ament_package()
