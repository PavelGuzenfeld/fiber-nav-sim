cmake_minimum_required(VERSION 3.22)
project(fiber_nav_physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── MAVLink c_library_v2 ──
include(FetchContent)
FetchContent_Declare(
    mavlink
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(mavlink)

# MAVLink is header-only: create interface target
add_library(mavlink_headers INTERFACE)
target_include_directories(mavlink_headers SYSTEM INTERFACE
    $<BUILD_INTERFACE:${mavlink_SOURCE_DIR}>
    $<BUILD_INTERFACE:${mavlink_SOURCE_DIR}/common>
    $<INSTALL_INTERFACE:include/mavlink>
    $<INSTALL_INTERFACE:include/mavlink/common>
)

# ── Shared physics library ──
add_library(fiber_nav_physics
    src/mavlink_connection.cpp
    src/hil_sensor_publisher.cpp
    src/aerodynamics.cpp
    src/motor_dynamics.cpp
    src/vtol_transition.cpp
    src/sensor_synthesis.cpp
    src/vehicle_dynamics.cpp
)

target_include_directories(fiber_nav_physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(fiber_nav_physics PUBLIC mavlink_headers)

# ── Tests (doctest) ──
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(doctest)

    add_executable(fiber_nav_physics_test
        tests/physics_test.cpp
    )
    target_link_libraries(fiber_nav_physics_test PRIVATE
        fiber_nav_physics
        doctest::doctest
    )

    add_test(NAME fiber_nav_physics_test COMMAND fiber_nav_physics_test)
endif()

# ── ament integration (optional, for ROS 2 workspace builds) ──
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    ament_export_targets(fiber_nav_physics_targets HAS_LIBRARY_TARGET)

    install(
        DIRECTORY include/
        DESTINATION include
    )

    # Install MAVLink headers so consumers get them transitively
    install(
        DIRECTORY ${mavlink_SOURCE_DIR}/
        DESTINATION include/mavlink
        FILES_MATCHING PATTERN "*.h"
    )

    install(
        TARGETS fiber_nav_physics mavlink_headers
        EXPORT fiber_nav_physics_targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
    )

    ament_package()
endif()
