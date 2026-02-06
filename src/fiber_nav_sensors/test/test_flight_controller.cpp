#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

// Include the flight math header under test
#include <fiber_nav_sensors/flight_math.hpp>

using namespace fiber_nav_sensors;

// ---- QuaternionToEuler ----

TEST_CASE("QuaternionToEuler.Identity") {
    Quat q{1.0, 0.0, 0.0, 0.0};
    auto [roll, pitch, yaw] = quaternion_to_euler(q);
    CHECK(roll == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(pitch == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(yaw == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("QuaternionToEuler.Yaw90") {
    // 90° yaw = rotation about Z
    double angle = std::numbers::pi / 2.0;
    Quat q{std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
    auto [roll, pitch, yaw] = quaternion_to_euler(q);
    CHECK(roll == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(pitch == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(yaw == doctest::Approx(std::numbers::pi / 2.0).epsilon(1e-6));
}

TEST_CASE("QuaternionToEuler.Roll90") {
    // 90° roll = rotation about X
    double angle = std::numbers::pi / 2.0;
    Quat q{std::cos(angle / 2.0), std::sin(angle / 2.0), 0.0, 0.0};
    auto [roll, pitch, yaw] = quaternion_to_euler(q);
    CHECK(roll == doctest::Approx(std::numbers::pi / 2.0).epsilon(1e-6));
    CHECK(pitch == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(yaw == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("QuaternionToEuler.Pitch45") {
    // 45° pitch = rotation about Y
    double angle = std::numbers::pi / 4.0;
    Quat q{std::cos(angle / 2.0), 0.0, std::sin(angle / 2.0), 0.0};
    auto [roll, pitch, yaw] = quaternion_to_euler(q);
    CHECK(roll == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(pitch == doctest::Approx(std::numbers::pi / 4.0).epsilon(1e-6));
    CHECK(yaw == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("QuaternionToEuler.GimbalLock") {
    // 90° pitch (gimbal lock) — sinp >= 1.0 triggers copysign branch
    Quat q{std::cos(std::numbers::pi / 4.0), 0.0,
            std::sin(std::numbers::pi / 4.0), 0.0};
    auto [roll, pitch, yaw] = quaternion_to_euler(q);
    CHECK(pitch == doctest::Approx(std::numbers::pi / 2.0).epsilon(1e-6));
}

// ---- RotateVector ----

TEST_CASE("RotateVector.Identity") {
    Quat q{1.0, 0.0, 0.0, 0.0};
    Vec3 v{1.0, 2.0, 3.0};
    auto r = rotate_vector(q, v);
    CHECK(r.x == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(r.y == doctest::Approx(2.0).epsilon(1e-10));
    CHECK(r.z == doctest::Approx(3.0).epsilon(1e-10));
}

TEST_CASE("RotateVector.Yaw90Z") {
    // 90° about Z: (1,0,0) -> (0,1,0)
    double angle = std::numbers::pi / 2.0;
    Quat q{std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
    Vec3 v{1.0, 0.0, 0.0};
    auto r = rotate_vector(q, v);
    CHECK(r.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(r.y == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(r.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("RotateVector.MagnitudePreservation") {
    // Arbitrary rotation should preserve vector magnitude
    Quat q{0.7071, 0.3536, 0.3536, 0.5};
    // Normalize the quaternion
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;

    Vec3 v{3.0, 4.0, 5.0};
    double mag_before = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    auto r = rotate_vector(q, v);
    double mag_after = std::sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
    CHECK(mag_after == doctest::Approx(mag_before).epsilon(1e-6));
}

TEST_CASE("RotateVector.CompoundRotation") {
    // 180° about Z: (1,0,0) -> (-1,0,0)
    double angle = std::numbers::pi;
    Quat q{std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
    Vec3 v{1.0, 0.0, 0.0};
    auto r = rotate_vector(q, v);
    CHECK(r.x == doctest::Approx(-1.0).epsilon(1e-6));
    CHECK(r.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(r.z == doctest::Approx(0.0).epsilon(1e-6));
}

// ---- WrapAngle ----

TEST_CASE("WrapAngle.InRange") {
    CHECK(wrap_angle(0.5) == doctest::Approx(0.5).epsilon(1e-10));
    CHECK(wrap_angle(-0.5) == doctest::Approx(-0.5).epsilon(1e-10));
    CHECK(wrap_angle(3.0) == doctest::Approx(3.0).epsilon(1e-10));
}

TEST_CASE("WrapAngle.OverPi") {
    double result = wrap_angle(std::numbers::pi + 0.1);
    CHECK(result == doctest::Approx(-std::numbers::pi + 0.1).epsilon(1e-10));
}

TEST_CASE("WrapAngle.UnderNegPi") {
    double result = wrap_angle(-std::numbers::pi - 0.1);
    CHECK(result == doctest::Approx(std::numbers::pi - 0.1).epsilon(1e-10));
}

TEST_CASE("WrapAngle.LargeValues") {
    // 4*pi + 0.5 should wrap to 0.5
    double result = wrap_angle(4.0 * std::numbers::pi + 0.5);
    CHECK(result == doctest::Approx(0.5).epsilon(1e-10));

    // -4*pi - 0.5 should wrap to -0.5
    result = wrap_angle(-4.0 * std::numbers::pi - 0.5);
    CHECK(result == doctest::Approx(-0.5).epsilon(1e-10));
}

// ---- Clamp ----

TEST_CASE("Clamp.InRange") {
    CHECK(clamp(5.0, 0.0, 10.0) == doctest::Approx(5.0));
}

TEST_CASE("Clamp.BelowMin") {
    CHECK(clamp(-5.0, 0.0, 10.0) == doctest::Approx(0.0));
}

TEST_CASE("Clamp.AboveMax") {
    CHECK(clamp(15.0, 0.0, 10.0) == doctest::Approx(10.0));
}

// ---- WaypointLogic ----

TEST_CASE("WaypointLogic.HeadingComputation") {
    // Waypoint at (100, 0), position at (0, 0) -> heading = 0 (east)
    double dx = 100.0 - 0.0;
    double dy = 0.0 - 0.0;
    double heading = std::atan2(dy, dx);
    CHECK(heading == doctest::Approx(0.0).epsilon(1e-10));

    // Waypoint at (0, 100) -> heading = pi/2 (north)
    dx = 0.0;
    dy = 100.0;
    heading = std::atan2(dy, dx);
    CHECK(heading == doctest::Approx(std::numbers::pi / 2.0).epsilon(1e-10));
}

TEST_CASE("WaypointLogic.YawErrorWrapping") {
    // Desired yaw = -170°, current yaw = 170° -> error should be ~+20° not -340°
    double desired = -170.0 * std::numbers::pi / 180.0;
    double current = 170.0 * std::numbers::pi / 180.0;
    double error = wrap_angle(desired - current);
    CHECK(std::abs(error) < std::numbers::pi);
    CHECK(error == doctest::Approx(20.0 * std::numbers::pi / 180.0).epsilon(1e-6));
}

TEST_CASE("WaypointLogic.AcceptanceRadius") {
    // Within acceptance radius of 20m
    double dx = 10.0;
    double dy = 10.0;
    double dist = std::sqrt(dx * dx + dy * dy);
    CHECK(dist < 20.0);

    // Outside acceptance radius
    dx = 15.0;
    dy = 15.0;
    dist = std::sqrt(dx * dx + dy * dy);
    CHECK(dist > 20.0);
}

// ---- PDControl ----

TEST_CASE("PDControl.ZeroError") {
    double kp = 8.0;
    double kd = 2.0;
    double error = 0.0;
    double rate = 0.0;
    double output = kp * error + kd * (-rate);
    CHECK(output == doctest::Approx(0.0));
}

TEST_CASE("PDControl.ProportionalResponse") {
    double kp = 8.0;
    double kd = 2.0;
    double error = 1.0;
    double rate = 0.0;
    double output = kp * error + kd * (-rate);
    CHECK(output == doctest::Approx(8.0));
}

TEST_CASE("PDControl.DerivativeDamping") {
    double kp = 8.0;
    double kd = 2.0;
    double error = 0.0;
    double rate = 3.0;  // Moving toward target
    double output = kp * error + kd * (-rate);
    CHECK(output == doctest::Approx(-6.0));
}
