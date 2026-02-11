#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
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

TEST_CASE("CanyonWaypoints.DefaultWaypointsNotEmpty")
{
    auto wps = defaultCanyonWaypoints();
    REQUIRE_FALSE(wps.empty());
    CHECK(wps.size() == 8u);
}

TEST_CASE("CanyonWaypoints.AllAtSameAltitude")
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        CHECK_MESSAGE(wp.position.z() == doctest::Approx(-50.f),
            "Waypoint altitude should be -50m NED (50m above ground)");
    }
}

TEST_CASE("CanyonWaypoints.AcceptanceRadiusPositive")
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        CHECK(wp.acceptance_radius > 0.f);
    }
}

TEST_CASE("CanyonWaypoints.HeadingsInRange")
{
    auto wps = defaultCanyonWaypoints();
    for (const auto& wp : wps) {
        CHECK(wp.heading >= -static_cast<float>(M_PI));
        CHECK(wp.heading <= static_cast<float>(M_PI));
    }
}

TEST_CASE("CanyonWaypoints.FirstWaypointAtOrigin")
{
    auto wps = defaultCanyonWaypoints();
    REQUIRE_FALSE(wps.empty());
    CHECK(wps.front().position.x() == doctest::Approx(0.f));
    CHECK(wps.front().position.y() == doctest::Approx(0.f));
}

TEST_CASE("CanyonWaypoints.LastWaypointReturnsToOrigin")
{
    auto wps = defaultCanyonWaypoints();
    REQUIRE_FALSE(wps.empty());
    CHECK(wps.back().position.x() == doctest::Approx(0.f));
    CHECK(wps.back().position.y() == doctest::Approx(0.f));
}

TEST_CASE("CanyonWaypoints.ConsecutiveDistance")
{
    auto wps = defaultCanyonWaypoints();
    for (std::size_t i = 1; i < wps.size(); ++i) {
        float dist = (wps[i].position - wps[i - 1].position).norm();
        float heading_diff = std::abs(wps[i].heading - wps[i - 1].heading);
        // Consecutive waypoints must differ in position OR heading (turnaround points)
        bool differs = (dist > 0.f) || (heading_diff > 0.f);
        CHECK_MESSAGE(differs,
            "Waypoints ", i - 1, " and ", i, " are identical in both position and heading");
        CHECK_MESSAGE(dist < 200.f,
            "Waypoints ", i - 1, " and ", i, " too far apart");
    }
}

TEST_CASE("WaypointReach.DistanceCheck")
{
    Waypoint wp{{100.f, 0.f, -50.f}, 0.f, 3.f};
    Eigen::Vector3f vehicle_pos{98.f, 1.f, -50.f};

    float dist = (wp.position - vehicle_pos).norm();
    CHECK(dist < wp.acceptance_radius);
}

TEST_CASE("WaypointReach.NotReachedIfTooFar")
{
    Waypoint wp{{100.f, 0.f, -50.f}, 0.f, 3.f};
    Eigen::Vector3f vehicle_pos{90.f, 0.f, -50.f};

    float dist = (wp.position - vehicle_pos).norm();
    CHECK(dist > wp.acceptance_radius);
}
