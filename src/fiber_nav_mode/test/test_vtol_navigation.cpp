#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include <Eigen/Core>

// Test pure geometry and config logic without px4_ros2 dependencies.
// Re-define the structs and free functions locally (same pattern as test_canyon_waypoints.cpp).

namespace fiber_nav_mode {

struct VtolNavConfig
{
    float cruise_alt_m = 50.f;
    float climb_rate = 2.f;
    float fw_accept_radius = 50.f;
    float mc_transition_dist = 200.f;
    float mc_approach_speed = 5.f;
    float fw_transition_timeout = 30.f;
    float mc_transition_timeout = 60.f;
};

struct VtolWaypoint
{
    float x;
    float y;
    float heading;
    float acceptance_radius;
};

// Replicate the static helper functions from VtolNavigationMode
float courseToTarget(const Eigen::Vector3f& pos, float target_x, float target_y)
{
    return std::atan2(target_y - pos.y(), target_x - pos.x());
}

float horizontalDistTo(const Eigen::Vector3f& pos, float target_x, float target_y)
{
    const float dx = target_x - pos.x();
    const float dy = target_y - pos.y();
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace fiber_nav_mode

using fiber_nav_mode::VtolNavConfig;
using fiber_nav_mode::VtolWaypoint;
using fiber_nav_mode::courseToTarget;
using fiber_nav_mode::horizontalDistTo;

// --- Course angle tests ---

TEST(VtolCourse, NorthIsZero)
{
    // Target directly north (positive x in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 100.f, 0.f);
    EXPECT_NEAR(course, 0.f, 0.01f);
}

TEST(VtolCourse, EastIsHalfPi)
{
    // Target directly east (positive y in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 0.f, 100.f);
    EXPECT_NEAR(course, static_cast<float>(M_PI / 2.0), 0.01f);
}

TEST(VtolCourse, SouthIsPi)
{
    // Target directly south (negative x in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, -100.f, 0.f);
    // atan2(0, -100) = pi
    EXPECT_NEAR(std::abs(course), static_cast<float>(M_PI), 0.01f);
}

TEST(VtolCourse, WestIsNegativeHalfPi)
{
    // Target directly west (negative y in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 0.f, -100.f);
    EXPECT_NEAR(course, static_cast<float>(-M_PI / 2.0), 0.01f);
}

TEST(VtolCourse, DiagonalNorthEast)
{
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 100.f, 100.f);
    EXPECT_NEAR(course, static_cast<float>(M_PI / 4.0), 0.01f);
}

TEST(VtolCourse, FromNonOriginPosition)
{
    // Vehicle at (500, 300), target at (500, 400) = due east
    Eigen::Vector3f pos{500.f, 300.f, -50.f};
    float course = courseToTarget(pos, 500.f, 400.f);
    EXPECT_NEAR(course, static_cast<float>(M_PI / 2.0), 0.01f);
}

TEST(VtolCourse, CourseToHomeFromFar)
{
    // Vehicle far away, course back to (0,0)
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float course = courseToTarget(pos, 0.f, 0.f);
    // Should be roughly south-west (negative y, positive x)
    EXPECT_LT(course, 0.f);           // west component → negative
    EXPECT_GT(course, -static_cast<float>(M_PI));
}

// --- Horizontal distance tests ---

TEST(VtolDistance, ZeroDistance)
{
    Eigen::Vector3f pos{100.f, 200.f, -50.f};
    EXPECT_NEAR(horizontalDistTo(pos, 100.f, 200.f), 0.f, 0.001f);
}

TEST(VtolDistance, SimpleDistance)
{
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    EXPECT_NEAR(horizontalDistTo(pos, 300.f, 400.f), 500.f, 0.1f);
}

TEST(VtolDistance, IgnoresAltitude)
{
    // Same XY, different altitude → distance should be 0
    Eigen::Vector3f pos{100.f, 200.f, -10.f};
    EXPECT_NEAR(horizontalDistTo(pos, 100.f, 200.f), 0.f, 0.001f);
}

TEST(VtolDistance, CanyonEndToHome)
{
    // From the last canyon waypoint back to home
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    EXPECT_NEAR(dist, 1400.6f, 1.f);  // sqrt(40^2 + 1400^2) ≈ 1400.6
}

// --- Waypoint acceptance tests ---

TEST(VtolAcceptance, WithinRadius)
{
    VtolWaypoint wp{100.f, 200.f, 0.f, 50.f};
    Eigen::Vector3f pos{80.f, 190.f, -50.f};
    float dist = horizontalDistTo(pos, wp.x, wp.y);
    EXPECT_LT(dist, wp.acceptance_radius);
}

TEST(VtolAcceptance, OutsideRadius)
{
    VtolWaypoint wp{100.f, 200.f, 0.f, 50.f};
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float dist = horizontalDistTo(pos, wp.x, wp.y);
    EXPECT_GT(dist, wp.acceptance_radius);
}

TEST(VtolAcceptance, DefaultRadiusFromConfig)
{
    VtolNavConfig config;
    VtolWaypoint wp{100.f, 0.f, NAN, 0.f};  // accept_radius = 0 → use config default
    float accept = wp.acceptance_radius > 0.f ? wp.acceptance_radius : config.fw_accept_radius;
    EXPECT_FLOAT_EQ(accept, 50.f);
}

TEST(VtolAcceptance, OverrideRadiusPerWaypoint)
{
    VtolNavConfig config;
    VtolWaypoint wp{100.f, 0.f, NAN, 30.f};  // explicit per-WP radius
    float accept = wp.acceptance_radius > 0.f ? wp.acceptance_radius : config.fw_accept_radius;
    EXPECT_FLOAT_EQ(accept, 30.f);
}

// --- Altitude AMSL conversion tests ---

TEST(VtolAltitude, AmslConversion)
{
    // Ground AMSL ref = 100m, cruise alt = 50m → target AMSL = 150m
    float alt_amsl_ref = 100.f;
    float cruise_alt_m = 50.f;
    float target = alt_amsl_ref + cruise_alt_m;
    EXPECT_FLOAT_EQ(target, 150.f);
}

TEST(VtolAltitude, AmslRefComputation)
{
    // Current AMSL = 145m, current AGL = 45m → ground AMSL = 100m
    float current_amsl = 145.f;
    float current_agl = 45.f;
    float alt_amsl_ref = current_amsl - current_agl;
    EXPECT_FLOAT_EQ(alt_amsl_ref, 100.f);
}

// --- MC transition distance tests ---

TEST(VtolTransition, WithinMcTransitionDist)
{
    VtolNavConfig config;
    config.mc_transition_dist = 200.f;
    Eigen::Vector3f pos{150.f, 0.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    EXPECT_LT(dist, config.mc_transition_dist);
}

TEST(VtolTransition, OutsideMcTransitionDist)
{
    VtolNavConfig config;
    config.mc_transition_dist = 200.f;
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    EXPECT_GT(dist, config.mc_transition_dist);
}

// --- State transition logic tests ---

TEST(VtolStateMachine, ClimbAltitudeCheck)
{
    VtolNavConfig config;
    config.cruise_alt_m = 50.f;
    float tolerance = 2.f;

    // At 47m AGL → not yet at cruise
    float agl_47 = 47.f;
    EXPECT_FALSE(agl_47 >= config.cruise_alt_m - tolerance);

    // At 48m AGL → within tolerance
    float agl_48 = 48.f;
    EXPECT_TRUE(agl_48 >= config.cruise_alt_m - tolerance);

    // At 50m AGL → exactly at cruise
    float agl_50 = 50.f;
    EXPECT_TRUE(agl_50 >= config.cruise_alt_m - tolerance);
}

TEST(VtolStateMachine, HomeApproachCheck)
{
    float home_approach_dist = 5.f;

    // At 4m from home → done
    Eigen::Vector3f pos_close{3.f, 2.f, -50.f};
    EXPECT_LT(horizontalDistTo(pos_close, 0.f, 0.f), home_approach_dist);

    // At 10m from home → not done
    Eigen::Vector3f pos_far{8.f, 6.f, -50.f};
    EXPECT_GT(horizontalDistTo(pos_far, 0.f, 0.f), home_approach_dist);
}

// --- Config default tests ---

TEST(VtolConfig, DefaultValues)
{
    VtolNavConfig config;
    EXPECT_FLOAT_EQ(config.cruise_alt_m, 50.f);
    EXPECT_FLOAT_EQ(config.climb_rate, 2.f);
    EXPECT_FLOAT_EQ(config.fw_accept_radius, 50.f);
    EXPECT_FLOAT_EQ(config.mc_transition_dist, 200.f);
    EXPECT_FLOAT_EQ(config.mc_approach_speed, 5.f);
    EXPECT_FLOAT_EQ(config.fw_transition_timeout, 30.f);
    EXPECT_FLOAT_EQ(config.mc_transition_timeout, 60.f);
}

// --- Waypoint sequence tests ---

TEST(VtolWaypoints, CanyonSequenceProgression)
{
    // Canyon waypoints should progress monotonically in Y (east)
    std::vector<VtolWaypoint> wps = {
        {0.f, 200.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
        {0.f, 600.f, NAN, 0.f},
        {0.f, 800.f, NAN, 0.f},
        {0.f, 1000.f, NAN, 0.f},
        {-20.f, 1200.f, NAN, 0.f},
        {-40.f, 1400.f, NAN, 0.f},
    };

    for (std::size_t i = 1; i < wps.size(); ++i) {
        EXPECT_GT(wps[i].y, wps[i - 1].y)
            << "Waypoint " << i << " Y should be greater than waypoint " << i - 1;
    }
}

TEST(VtolWaypoints, ConsecutiveDistanceReasonable)
{
    std::vector<VtolWaypoint> wps = {
        {0.f, 200.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
        {0.f, 600.f, NAN, 0.f},
    };

    for (std::size_t i = 1; i < wps.size(); ++i) {
        float dx = wps[i].x - wps[i - 1].x;
        float dy = wps[i].y - wps[i - 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        EXPECT_GT(dist, 10.f) << "Waypoints too close together";
        EXPECT_LT(dist, 500.f) << "Waypoints too far apart";
    }
}
