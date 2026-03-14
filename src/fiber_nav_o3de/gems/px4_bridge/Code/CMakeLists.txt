ly_add_target(
    NAME PX4Bridge.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        px4bridge_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
)

ly_add_target(
    NAME PX4Bridge ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        px4bridge_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::PX4Bridge.Static
)

# Register as a runtime gem for GameLauncher and other targets
ly_create_alias(NAME PX4Bridge.Clients NAMESPACE Gem TARGETS Gem::PX4Bridge)
ly_create_alias(NAME PX4Bridge.Servers NAMESPACE Gem TARGETS Gem::PX4Bridge)
ly_create_alias(NAME PX4Bridge.Unified NAMESPACE Gem TARGETS Gem::PX4Bridge)

# Fetch MAVLink C headers (header-only, no build step)
include(FetchContent)
FetchContent_Declare(
    mavlink
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG master
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(mavlink)

target_include_directories(PX4Bridge.Static
    PUBLIC
        ${mavlink_SOURCE_DIR}
        ${mavlink_SOURCE_DIR}/common
)

# Unit tests
if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    ly_add_target(
        NAME PX4Bridge.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
        NAMESPACE Gem
        FILES_CMAKE
            px4bridge_test_files.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
                Include
        BUILD_DEPENDENCIES
            PRIVATE
                AZ::AzTest
                Gem::PX4Bridge.Static
    )
    ly_add_googletest(
        NAME Gem::PX4Bridge.Tests
    )
endif()
