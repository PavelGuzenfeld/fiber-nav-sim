ly_add_target(
    NAME VTOLDynamics.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        vtoldynamics_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            Gem::PhysX.Static
            Gem::PX4Bridge.Static
)

ly_add_target(
    NAME VTOLDynamics ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        vtoldynamics_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::VTOLDynamics.Static
)

# Register for all O3DE contexts (GameLauncher, AssetBuilder, Editor, etc.)
ly_create_alias(NAME VTOLDynamics.Clients NAMESPACE Gem TARGETS Gem::VTOLDynamics)
ly_create_alias(NAME VTOLDynamics.Servers NAMESPACE Gem TARGETS Gem::VTOLDynamics)
ly_create_alias(NAME VTOLDynamics.Unified NAMESPACE Gem TARGETS Gem::VTOLDynamics)
ly_create_alias(NAME VTOLDynamics.Builders NAMESPACE Gem TARGETS Gem::VTOLDynamics)
ly_create_alias(NAME VTOLDynamics.Tools NAMESPACE Gem TARGETS Gem::VTOLDynamics)

if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    ly_add_target(
        NAME VTOLDynamics.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
        NAMESPACE Gem
        FILES_CMAKE
            vtoldynamics_test_files.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
                Include
        BUILD_DEPENDENCIES
            PRIVATE
                AZ::AzTest
                Gem::VTOLDynamics.Static
    )
    ly_add_googletest(
        NAME Gem::VTOLDynamics.Tests
    )
endif()
