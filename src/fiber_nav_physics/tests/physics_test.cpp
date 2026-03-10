#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "fiber_nav_physics/aerodynamics.hpp"
#include "fiber_nav_physics/motor_dynamics.hpp"
#include "fiber_nav_physics/sensor_synthesis.hpp"
#include "fiber_nav_physics/vehicle_dynamics.hpp"
#include "fiber_nav_physics/vtol_transition.hpp"

using namespace fiber_nav_physics;

// ── Vec3 tests ──

TEST_CASE("Vec3 basic operations")
{
    Vec3 a{1.0f, 2.0f, 3.0f};
    Vec3 b{4.0f, 5.0f, 6.0f};

    auto sum = a + b;
    CHECK(sum.x == doctest::Approx(5.0f));
    CHECK(sum.y == doctest::Approx(7.0f));
    CHECK(sum.z == doctest::Approx(9.0f));

    CHECK(a.dot(b) == doctest::Approx(32.0f));

    auto cross = a.cross(b);
    CHECK(cross.x == doctest::Approx(-3.0f));
    CHECK(cross.y == doctest::Approx(6.0f));
    CHECK(cross.z == doctest::Approx(-3.0f));

    Vec3 unit{1.0f, 0.0f, 0.0f};
    CHECK(unit.length() == doctest::Approx(1.0f));
}

// ── Quaternion tests ──

TEST_CASE("Quat identity rotation")
{
    Quat q = Quat::identity();
    Vec3 v{1.0f, 2.0f, 3.0f};
    auto rotated = q.rotate(v);
    CHECK(rotated.x == doctest::Approx(v.x));
    CHECK(rotated.y == doctest::Approx(v.y));
    CHECK(rotated.z == doctest::Approx(v.z));
}

TEST_CASE("Quat 90-degree rotation around Z")
{
    // 90 deg around Z: (1,0,0) → (0,1,0)
    Quat q = Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, 3.14159265f / 2.0f);
    Vec3 v{1.0f, 0.0f, 0.0f};
    auto rotated = q.rotate(v);
    CHECK(rotated.x == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(rotated.y == doctest::Approx(1.0f).epsilon(1e-4));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("Quat inverse rotation roundtrip")
{
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.0f);
    Vec3 v{1.0f, 2.0f, 3.0f};
    auto rotated = q.rotate(v);
    auto back = q.inverse_rotate(rotated);
    CHECK(back.x == doctest::Approx(v.x).epsilon(1e-4));
    CHECK(back.y == doctest::Approx(v.y).epsilon(1e-4));
    CHECK(back.z == doctest::Approx(v.z).epsilon(1e-4));
}

// ── Aerodynamics tests ──

TEST_CASE("Aerodynamics: zero velocity produces no forces")
{
    AerodynamicsModel aero{};
    aero.min_airspeed = 2.0f;

    auto state = aero.compute(Vec3::zero(), Vec3::zero());
    CHECK(state.airspeed == doctest::Approx(0.0f));
    CHECK(state.lift_force.length() == doctest::Approx(0.0f));
    CHECK(state.drag_force.length() == doctest::Approx(0.0f));
}

TEST_CASE("Aerodynamics: lift increases with airspeed")
{
    AerodynamicsModel aero{};
    aero.coeffs.CLa = 5.0f;
    aero.coeffs.CD0 = 0.02f;
    aero.coeffs.alpha_stall = 0.3f;
    aero.wing.area = 0.5f;
    aero.wing.aspect_ratio = 6.0f;
    aero.wing.air_density = 1.225f;
    aero.wing.forward = {0.0f, 0.0f, 1.0f};
    aero.wing.upward = {-1.0f, 0.0f, 0.0f};
    aero.min_airspeed = 2.0f;

    // Forward flight at 10 m/s with slight AoA
    Vec3 vel_slow{0.0f, 0.0f, 5.0f};  // 5 m/s forward
    Vec3 vel_fast{0.0f, 0.0f, 20.0f}; // 20 m/s forward

    auto state_slow = aero.compute(vel_slow, Vec3::zero());
    auto state_fast = aero.compute(vel_fast, Vec3::zero());

    // Drag increases with speed^2
    CHECK(state_fast.drag_force.length() > state_slow.drag_force.length());
}

TEST_CASE("Aerodynamics: CL post-stall model")
{
    AerodynamicsModel aero{};
    aero.coeffs.CL0 = 0.0f;
    aero.coeffs.CLa = 5.0f;
    aero.coeffs.CLa_stall = -2.0f;
    aero.coeffs.alpha_stall = 0.25f;

    // Pre-stall: linear region
    float cl_pre = aero.compute_cl(0.1f);
    CHECK(cl_pre == doctest::Approx(0.5f));

    // At stall: CL = CLa * alpha_stall = 5.0 * 0.25 = 1.25
    float cl_stall = aero.compute_cl(0.25f);
    CHECK(cl_stall == doctest::Approx(1.25f));

    // Post-stall: CL drops
    float cl_post = aero.compute_cl(0.35f);
    CHECK(cl_post < cl_stall);
}

// ── Motor dynamics tests ──

TEST_CASE("Motor dynamics: step converges to commanded velocity")
{
    MotorConfig config{};
    config.max_rot_velocity = 1000.0f;
    config.time_constant_up = 0.01f;
    config.time_constant_down = 0.02f;

    MotorState state{};
    state.commanded_velocity = 500.0f;

    std::array<MotorConfig, 1> configs{config};
    std::array<MotorState, 1> states{state};

    // Step many times to converge
    for (int i = 0; i < 1000; ++i)
    {
        step_motor_dynamics(configs, states, 0.005f);
    }

    CHECK(states[0].current_velocity == doctest::Approx(500.0f).epsilon(0.01));
}

TEST_CASE("Motor forces: single motor thrust along +Z")
{
    MotorConfig config{};
    config.position = {0.1f, 0.1f, 0.0f};
    config.motor_constant = 8.5e-6f;
    config.moment_constant = 0.016f;
    config.direction = 1;

    MotorState state{};
    state.current_velocity = 800.0f;

    std::array<MotorConfig, 1> configs{config};
    std::array<MotorState, 1> states{state};

    auto ft = compute_motor_forces(configs, states);
    // Thrust should be along +Z
    CHECK(ft.force.z > 0.0f);
    CHECK(ft.force.x == doctest::Approx(0.0f));
    CHECK(ft.force.y == doctest::Approx(0.0f));
    // Torque should be non-zero (arm + reaction)
    CHECK(ft.torque.length() > 0.0f);
}

// ── VTOL transition tests ──

TEST_CASE("VTOL transition: MC → FW → MC")
{
    VtolTransition vtol{};
    vtol.fw_transition_speed = 15.0f;
    vtol.mc_transition_speed = 8.0f;
    vtol.back_transition_time = 2.0f;

    CHECK(vtol.flight_mode == VTOLFlightMode::Multicopter);

    // Accelerate past FW threshold
    vtol.update(16.0f, 0.1f); // → TransitionToFW
    CHECK(vtol.flight_mode == VTOLFlightMode::TransitionToFW);

    vtol.update(16.0f, 0.1f); // → FixedWing
    CHECK(vtol.flight_mode == VTOLFlightMode::FixedWing);

    // Decelerate below MC threshold
    vtol.update(7.0f, 0.1f); // → TransitionToMC
    CHECK(vtol.flight_mode == VTOLFlightMode::TransitionToMC);

    // Wait for back transition
    for (int i = 0; i < 25; ++i)
    {
        vtol.update(5.0f, 0.1f);
    }
    CHECK(vtol.flight_mode == VTOLFlightMode::Multicopter);
}

TEST_CASE("VTOL transition: timeout reverts to MC")
{
    VtolTransition vtol{};
    vtol.fw_transition_speed = 15.0f;
    vtol.transition_timeout = 1.0f;

    vtol.update(16.0f, 0.1f); // → TransitionToFW
    CHECK(vtol.flight_mode == VTOLFlightMode::TransitionToFW);

    // Speed drops below threshold during transition, wait for timeout
    for (int i = 0; i < 15; ++i)
    {
        vtol.update(10.0f, 0.1f);
    }
    CHECK(vtol.flight_mode == VTOLFlightMode::Multicopter);
}

// ── Sensor synthesis tests ──

TEST_CASE("Sensor synthesis: barometric altitude")
{
    VehicleState state{};
    state.position = {0.0f, 0.0f, -100.0f}; // 100m above ground (NED: -z = up)

    SensorNoiseConfig noise{};
    noise.baro_noise_stddev = 0.0f; // no noise for test

    GPSConfig gps{};
    gps.origin_alt = 141.0f; // MSL

    auto sensor = synthesize_sensors(state, Vec3::zero(), 0.005f, noise, gps);

    // Altitude should be ~241m MSL
    CHECK(sensor.pressure_alt == doctest::Approx(241.0f));

    // Pressure at 241m should be less than sea level
    CHECK(sensor.abs_pressure < 1013.25f);
}

TEST_CASE("GPS synthesis: position conversion")
{
    VehicleState state{};
    state.position = {0.0f, 0.0f, 0.0f}; // at origin

    GPSConfig gps{};
    gps.origin_lat = 31.164093;
    gps.origin_lon = 34.532227;
    gps.origin_alt = 141.0f;

    auto gps_data = synthesize_gps(state, gps);
    CHECK(gps_data.lat == doctest::Approx(static_cast<int32_t>(31.164093 * 1e7)).epsilon(1));
    CHECK(gps_data.lon == doctest::Approx(static_cast<int32_t>(34.532227 * 1e7)).epsilon(1));
    CHECK(gps_data.alt == doctest::Approx(141000).epsilon(10)); // 141m in mm
}

// ── Vehicle dynamics integration test ──

TEST_CASE("Vehicle dynamics: free fall")
{
    VehicleConfig config{};
    config.mass = 2.0f;
    config.inertia = {0.03f, 0.03f, 0.01f};

    VehicleState initial{};
    initial.position = {0.0f, 0.0f, -100.0f}; // 100m up (NED)
    initial.orientation = Quat::identity();

    VehicleDynamics dyn;
    dyn.init(config, initial);

    // Step for 1 second at 200 Hz
    for (int i = 0; i < 200; ++i)
    {
        dyn.step(0.005f);
    }

    // Should have fallen: velocity.z > 0 (downward in NED)
    CHECK(dyn.state().velocity.z > 0.0f);
    // Approximately v = g*t = 9.81 * 1.0 ≈ 9.81 m/s
    CHECK(dyn.state().velocity.z == doctest::Approx(9.81f).epsilon(0.1));
    // Should have moved down: position.z > -100
    CHECK(dyn.state().position.z > -100.0f);
}

TEST_CASE("Vehicle dynamics: hover with thrust")
{
    VehicleConfig config{};
    config.mass = 2.0f;
    config.inertia = {0.03f, 0.03f, 0.01f};

    // Single motor producing exactly mg thrust upward
    MotorConfig motor{};
    motor.motor_constant = 1.0f; // simplified: thrust = omega^2
    motor.max_rot_velocity = 100.0f;
    motor.time_constant_up = 0.001f;
    config.motors.push_back(motor);

    VehicleState initial{};
    initial.position = {0.0f, 0.0f, -50.0f}; // 50m up
    // Tailsitter hover: body +Z (thrust) must point world -Z (up in NED)
    // 180° pitch rotation around Y axis
    initial.orientation = Quat::from_axis_angle({0.0f, 1.0f, 0.0f}, 3.14159265f);

    VehicleDynamics dyn;
    dyn.init(config, initial);

    // Command motor to produce mg = 2 * 9.81 = 19.62 N
    // thrust = motor_constant * omega^2 = 1.0 * omega^2
    // omega = sqrt(19.62) ≈ 4.43
    // normalized = 4.43 / 100 = 0.0443
    MotorOutputs outputs{};
    outputs.controls[0] = 4.43f / 100.0f;
    dyn.set_motor_outputs(outputs);

    // Step for 2 seconds
    for (int i = 0; i < 400; ++i)
    {
        dyn.step(0.005f);
    }

    // Vehicle should be approximately stationary (velocity near zero)
    CHECK(std::abs(dyn.state().velocity.z) < 1.0f);
}
