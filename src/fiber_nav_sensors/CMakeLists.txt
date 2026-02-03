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
find_package(nav_msgs REQUIRED)
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
  nav_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(spool_sim_driver "${cpp_typesupport_target}")

# Vision direction simulator
add_executable(vision_direction_sim src/vision_direction_sim.cpp)
ament_target_dependencies(vision_direction_sim
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
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
  add_executable(test_spool_sensor test/test_spool_sensor.cpp)
  target_link_libraries(test_spool_sensor PRIVATE doctest::doctest)
  target_compile_features(test_spool_sensor PRIVATE cxx_std_23)

  add_executable(test_vision_sensor test/test_vision_sensor.cpp)
  target_link_libraries(test_vision_sensor PRIVATE doctest::doctest)
  target_compile_features(test_vision_sensor PRIVATE cxx_std_23)

  install(TARGETS test_spool_sensor test_vision_sensor
    DESTINATION lib/${PROJECT_NAME}
  )

  # Register tests with ament
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_spool_sensor
    COMMAND $<TARGET_FILE:test_spool_sensor>
    TIMEOUT 60
  )
  ament_add_test(test_vision_sensor
    COMMAND $<TARGET_FILE:test_vision_sensor>
    TIMEOUT 60
  )
endif()

ament_package()
