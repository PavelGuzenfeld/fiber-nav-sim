cmake_minimum_required(VERSION 3.8)
project(fiber_nav_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpoolVelocity.msg"
  DEPENDENCIES std_msgs
)

# Spool simulator driver
add_executable(spool_sim_driver src/spool_sim_driver.cpp)
ament_target_dependencies(spool_sim_driver
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(spool_sim_driver "${cpp_typesupport_target}")

# Vision direction simulator
add_executable(vision_direction_sim src/vision_direction_sim.cpp)
ament_target_dependencies(vision_direction_sim
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
)

install(TARGETS
  spool_sim_driver
  vision_direction_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
