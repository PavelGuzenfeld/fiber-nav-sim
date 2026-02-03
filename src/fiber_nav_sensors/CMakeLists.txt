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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
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

# Mock attitude publisher (for testing without PX4)
add_executable(mock_attitude_publisher src/mock_attitude_publisher.cpp)
ament_target_dependencies(mock_attitude_publisher
  rclcpp
  px4_msgs
)

# Plane controller (for applying forces in Gazebo)
add_executable(plane_controller src/plane_controller.cpp)
ament_target_dependencies(plane_controller
  rclcpp
  geometry_msgs
  std_srvs
)

# Image streamer (for web-based camera visualization)
add_executable(image_streamer src/image_streamer.cpp)
ament_target_dependencies(image_streamer
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(image_streamer ${OpenCV_LIBS})

install(TARGETS
  spool_sim_driver
  vision_direction_sim
  mock_attitude_publisher
  plane_controller
  image_streamer
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
