#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <fiber_nav_sensors/cable_dynamics.hpp>

using namespace fiber_nav_sensors;

TEST_CASE("CableProperties defaults are sensible") {
    CableProperties props;
    CHECK(props.mass_per_meter == doctest::Approx(0.003));
    CHECK(props.diameter == doctest::Approx(0.0009));
    CHECK(props.breaking_strength == doctest::Approx(50.0));
    CHECK(props.gravity == doctest::Approx(9.81));
}

TEST_CASE("compute_airborne_length") {
    SUBCASE("zero deployed returns zero") {
        CHECK(compute_airborne_length(0.0, 100.0) == doctest::Approx(0.0));
    }

    SUBCASE("zero altitude returns zero") {
        CHECK(compute_airborne_length(1000.0, 0.0) == doctest::Approx(0.0));
    }

    SUBCASE("negative values return zero") {
        CHECK(compute_airborne_length(-10.0, 50.0) == doctest::Approx(0.0));
        CHECK(compute_airborne_length(100.0, -5.0) == doctest::Approx(0.0));
    }

    SUBCASE("short deploy at high altitude is limited by deployed") {
        // Only 10m deployed at 100m altitude — all 10m is airborne
        double result = compute_airborne_length(10.0, 100.0);
        CHECK(result == doctest::Approx(10.0));
    }

    SUBCASE("long deploy at low altitude is limited by catenary") {
        // 5000m deployed at 80m altitude — bounded by pi/2 * altitude
        double result = compute_airborne_length(5000.0, 80.0);
        double max_expected = 80.0 * (std::numbers::pi / 2.0);
        CHECK(result == doctest::Approx(max_expected));
    }

    SUBCASE("moderate deploy equals catenary limit") {
        // 200m deployed at 100m — catenary limit is ~157m
        double result = compute_airborne_length(200.0, 100.0);
        double catenary = 100.0 * (std::numbers::pi / 2.0);
        CHECK(result == doctest::Approx(std::min(200.0, std::max(100.0, catenary))));
    }
}

TEST_CASE("compute_cable_drag") {
    CableProperties props;

    SUBCASE("zero velocity gives zero drag") {
        auto drag = compute_cable_drag(props, 100.0, 0.0, 0.0);
        CHECK(drag.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("zero airborne length gives zero drag") {
        auto drag = compute_cable_drag(props, 0.0, 20.0, 0.0);
        CHECK(drag.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("drag opposes velocity direction") {
        auto drag = compute_cable_drag(props, 100.0, 20.0, 0.0);
        CHECK(drag.x < 0.0);  // Opposes +X velocity
        CHECK(drag.y == doctest::Approx(0.0));
        CHECK(drag.z == doctest::Approx(0.0));
    }

    SUBCASE("drag opposes Y velocity") {
        auto drag = compute_cable_drag(props, 100.0, 0.0, -15.0);
        CHECK(drag.x == doctest::Approx(0.0));
        CHECK(drag.y > 0.0);  // Opposes -Y velocity
    }

    SUBCASE("drag increases with velocity squared") {
        auto drag1 = compute_cable_drag(props, 100.0, 10.0, 0.0);
        auto drag2 = compute_cable_drag(props, 100.0, 20.0, 0.0);
        // 2x velocity → 4x drag
        CHECK(drag2.magnitude() == doctest::Approx(drag1.magnitude() * 4.0).epsilon(0.01));
    }

    SUBCASE("drag increases linearly with length") {
        auto drag1 = compute_cable_drag(props, 100.0, 20.0, 0.0);
        auto drag2 = compute_cable_drag(props, 200.0, 20.0, 0.0);
        CHECK(drag2.magnitude() == doctest::Approx(drag1.magnitude() * 2.0).epsilon(0.01));
    }

    SUBCASE("drag magnitude is physically reasonable") {
        // 500m airborne, 20 m/s → F = 0.5 * 1.225 * 1.1 * 0.0009 * (500*0.4) * 400
        // = 0.5 * 1.225 * 1.1 * 0.0009 * 200 * 400 = 48.5 N
        auto drag = compute_cable_drag(props, 500.0, 20.0, 0.0);
        CHECK(drag.magnitude() > 10.0);   // Significant at range
        CHECK(drag.magnitude() < 100.0);  // But not insane
    }
}

TEST_CASE("compute_cable_weight") {
    CableProperties props;

    SUBCASE("zero length gives zero weight") {
        auto w = compute_cable_weight(props, 0.0);
        CHECK(w.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("weight is downward (negative Z)") {
        auto w = compute_cable_weight(props, 100.0);
        CHECK(w.x == doctest::Approx(0.0));
        CHECK(w.y == doctest::Approx(0.0));
        CHECK(w.z < 0.0);
    }

    SUBCASE("100m cable weight is ~2.9 N") {
        auto w = compute_cable_weight(props, 100.0);
        // 0.003 kg/m * 9.81 * 100m = 2.943 N
        CHECK(w.magnitude() == doctest::Approx(2.943).epsilon(0.01));
    }

    SUBCASE("weight scales linearly") {
        auto w1 = compute_cable_weight(props, 50.0);
        auto w2 = compute_cable_weight(props, 100.0);
        CHECK(w2.magnitude() == doctest::Approx(w1.magnitude() * 2.0).epsilon(0.01));
    }
}

TEST_CASE("compute_spool_friction") {
    CableProperties props;

    SUBCASE("stationary gives zero friction") {
        auto f = compute_spool_friction(props, 0.0, 0.0, 0.0, 0.0);
        CHECK(f.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("no payout gives zero friction") {
        auto f = compute_spool_friction(props, 10.0, 0.0, 0.0, 0.0);
        CHECK(f.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("friction opposes velocity") {
        auto f = compute_spool_friction(props, 20.0, 0.0, 0.0, 5.0);
        CHECK(f.x < 0.0);  // Opposes +X
    }

    SUBCASE("friction = static + kinetic * payout") {
        // speed=20, payout=5: friction = 0.5 + 0.02*5 = 0.6 N
        auto f = compute_spool_friction(props, 20.0, 0.0, 0.0, 5.0);
        CHECK(f.magnitude() == doctest::Approx(0.6).epsilon(0.01));
    }
}

TEST_CASE("compute_cable_forces integration") {
    CableProperties props;

    SUBCASE("no deployment gives zero forces") {
        auto r = compute_cable_forces(props, 0.0, 80.0, 20.0, 0.0, 0.0, 5.0, false);
        CHECK(r.tension == doctest::Approx(0.0));
        CHECK(r.total_force.magnitude() == doctest::Approx(0.0));
        CHECK_FALSE(r.is_broken);
    }

    SUBCASE("already broken gives zero forces") {
        auto r = compute_cable_forces(props, 500.0, 80.0, 20.0, 0.0, 0.0, 5.0, true);
        CHECK(r.total_force.magnitude() == doctest::Approx(0.0));
        CHECK(r.is_broken);
    }

    SUBCASE("moderate flight produces reasonable forces") {
        // 500m deployed, 80m altitude, 20 m/s forward
        auto r = compute_cable_forces(props, 500.0, 80.0, 20.0, 0.0, 0.0, 15.0, false);
        CHECK(r.tension > 0.0);
        CHECK(r.drag_magnitude > 0.0);
        CHECK(r.weight_magnitude > 0.0);
        CHECK(r.friction_magnitude > 0.0);
        CHECK(r.airborne_length > 0.0);
        CHECK(r.airborne_length <= 500.0);
        CHECK_FALSE(r.is_broken);
    }

    SUBCASE("cable breaks at high tension") {
        // Use very low breaking strength
        CableProperties weak = props;
        weak.breaking_strength = 1.0;  // 1 N
        auto r = compute_cable_forces(weak, 500.0, 80.0, 20.0, 0.0, 0.0, 15.0, false);
        CHECK(r.is_broken);
        CHECK(r.total_force.magnitude() == doctest::Approx(0.0));
    }

    SUBCASE("total force has drag + weight + friction components") {
        auto r = compute_cable_forces(props, 200.0, 80.0, 15.0, 0.0, -1.0, 10.0, false);
        // Force X should be negative (drag + friction oppose +X velocity)
        CHECK(r.total_force.x < 0.0);
        // Force Z should be negative (weight pulls down)
        CHECK(r.total_force.z < 0.0);
    }
}

TEST_CASE("ForceVec magnitude") {
    CHECK(ForceVec{3.0, 4.0, 0.0}.magnitude() == doctest::Approx(5.0));
    CHECK(ForceVec{0.0, 0.0, 0.0}.magnitude() == doctest::Approx(0.0));
    CHECK(ForceVec{1.0, 1.0, 1.0}.magnitude() == doctest::Approx(std::sqrt(3.0)));
}

TEST_CASE("max_safe_range") {
    CableProperties props;

    SUBCASE("zero altitude returns zero") {
        CHECK(max_safe_range(props, 0.0, 20.0, 40.0) == doctest::Approx(0.0));
    }

    SUBCASE("zero tension limit returns zero") {
        CHECK(max_safe_range(props, 80.0, 20.0, 0.0) == doctest::Approx(0.0));
    }

    SUBCASE("high limit at moderate speed returns unbounded") {
        // 80m altitude, 15 m/s, 40N limit — tension is well within limit
        double result = max_safe_range(props, 80.0, 15.0, 40.0);
        CHECK(result == doctest::Approx(1e6));
    }

    SUBCASE("very low limit returns -1 (can't even fly)") {
        // 80m altitude, 20 m/s, 0.1N limit — impossibly low
        double result = max_safe_range(props, 80.0, 20.0, 0.1);
        CHECK(result == doctest::Approx(-1.0));
    }

    SUBCASE("moderate limit at high speed finds valid range") {
        // 80m altitude, 25 m/s, 5N limit
        double result = max_safe_range(props, 80.0, 25.0, 5.0);
        CHECK(result > 0.0);
        CHECK(result < 1e6);
    }

    SUBCASE("higher speed reduces safe range") {
        double r1 = max_safe_range(props, 80.0, 15.0, 10.0);
        double r2 = max_safe_range(props, 80.0, 25.0, 10.0);
        // Both may be 1e6 (unbounded) or r2 < r1
        if (r1 < 1e6 && r2 < 1e6) {
            CHECK(r2 <= r1);
        }
    }

    SUBCASE("zero speed — only weight matters") {
        // At zero speed, drag=0, friction=static only
        // Tension = sqrt(friction^2 + weight^2)
        double result = max_safe_range(props, 80.0, 0.0, 40.0);
        CHECK(result == doctest::Approx(1e6));
    }
}
