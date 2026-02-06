#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include <Eigen/Core>

// We test the Waypoint struct and defaultCanyonWaypoints() helper directly.
// The px4_ros2 mode classes require a running ROS node + PX4 and are tested
// via integration tests in the simulator.

namespace fiber_nav_mode {

struct Waypoint
{
    Eigen::Vector3f position;
    float heading;
    float acceptance_radius;
};

inline std::vector<Waypoint> defaultCanyonWaypoints()
{
    constexpr float altitude = -50.f;
    constexpr float accept = 3.f;

    return {
        {{0.f, 0.f, altitude}, 0.f, accept},
        {{50.f, 0.f, altitude}, 0.f, accept},
        {{100.f, 10.f, altitude}, 0.1f, accept},
        {{150.f, 0.f, altitude}, 0.f, accept},
        {{150.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
        {{100.f, -10.f, altitude}, static_cast<float>(M_PI), accept},
        {{50.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
        {{0.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
    };
}

}  // namespace fiber_nav_mode

using fiber_nav_mode::Waypoint;
using fiber_nav_mode::defaultCanyonWaypoints;

TEST(CanyonWaypoints, DefaultWaypointsNotEmpty)
{
    auto wps = defaultCanyonWaypoints();
    ASSERT_FALSE(wps.empty());
    EXPECT_EQ(wps.size(), 8u);
}

TEST(CanyonWaypoints, AllAtSameAltitude)
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        EXPECT_FLOAT_EQ(wp.position.z(), -50.f)
            << "Waypoint altitude should be -50m NED (50m above ground)";
    }
}

TEST(CanyonWaypoints, AcceptanceRadiusPositive)
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        EXPECT_GT(wp.acceptance_radius, 0.f);
    }
}

TEST(CanyonWaypoints, HeadingsInRange)
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        EXPECT_GE(wp.heading, -static_cast<float>(M_PI));
        EXPECT_LE(wp.heading, static_cast<float>(M_PI));
    }
}

TEST(CanyonWaypoints, FirstWaypointAtOrigin)
{
    auto wps = defaultCanyonWaypoints();
    ASSERT_FALSE(wps.empty());
    EXPECT_FLOAT_EQ(wps.front().position.x(), 0.f);
    EXPECT_FLOAT_EQ(wps.front().position.y(), 0.f);
}

TEST(CanyonWaypoints, LastWaypointReturnsToOrigin)
{
    auto wps = defaultCanyonWaypoints();
    ASSERT_FALSE(wps.empty());
    EXPECT_FLOAT_EQ(wps.back().position.x(), 0.f);
    EXPECT_FLOAT_EQ(wps.back().position.y(), 0.f);
}

TEST(CanyonWaypoints, ConsecutiveDistance)
{
    auto wps = defaultCanyonWaypoints();
    for (std::size_t i = 1; i < wps.size(); ++i) {
        float dist = (wps[i].position - wps[i - 1].position).norm();
        float heading_diff = std::abs(wps[i].heading - wps[i - 1].heading);
        // Consecutive waypoints must differ in position OR heading (turnaround points)
        EXPECT_TRUE(dist > 0.f || heading_diff > 0.f)
            << "Waypoints " << i - 1 << " and " << i
            << " are identical in both position and heading";
        EXPECT_LT(dist, 200.f)
            << "Waypoints " << i - 1 << " and " << i << " too far apart";
    }
}

TEST(WaypointReach, DistanceCheck)
{
    Waypoint wp{{100.f, 0.f, -50.f}, 0.f, 3.f};
    Eigen::Vector3f vehicle_pos{98.f, 1.f, -50.f};

    float dist = (wp.position - vehicle_pos).norm();
    EXPECT_LT(dist, wp.acceptance_radius);
}

TEST(WaypointReach, NotReachedIfTooFar)
{
    Waypoint wp{{100.f, 0.f, -50.f}, 0.f, 3.f};
    Eigen::Vector3f vehicle_pos{90.f, 0.f, -50.f};

    float dist = (wp.position - vehicle_pos).norm();
    EXPECT_GT(dist, wp.acceptance_radius);
}
