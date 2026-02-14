#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_sensors/gimbal_math.hpp>

#include <cmath>
#include <numbers>

using fiber_nav_sensors::AxisFilter;
using fiber_nav_sensors::ActivationGate;
using fiber_nav_sensors::GravityVector;
using fiber_nav_sensors::GimbalTargets;
using fiber_nav_sensors::gravityInBody;
using fiber_nav_sensors::gimbalTargets;
using fiber_nav_sensors::saturationRatio;

namespace {

/// Build a quaternion from axis-angle (axis must be unit length).
struct Quat { float w, x, y, z; };

Quat quatFromAxisAngle(float ax, float ay, float az, float angle_rad)
{
    float ha = angle_rad * 0.5f;
    float s = std::sin(ha);
    return {std::cos(ha), ax * s, ay * s, az * s};
}

/// Quaternion multiply q1 * q2.
Quat quatMul(const Quat& q1, const Quat& q2)
{
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    };
}

constexpr float kDeg2Rad = std::numbers::pi_v<float> / 180.f;

}  // namespace

// ============================================================================
// AxisFilter tests
// ============================================================================

TEST_CASE("AxisFilter.StepResponse.ConvergesToTarget")
{
    AxisFilter f;
    float target = 0.5f;
    // Run for 5*tau = 1.5s at 50 Hz (75 steps) with generous rate limit
    for (int i = 0; i < 75; ++i) {
        f.update(target, 0.02f, 0.3f, 10.f);
    }
    CHECK(f.filtered == doctest::Approx(target).epsilon(0.01));
}

TEST_CASE("AxisFilter.StepResponse.ZeroFromNonZero")
{
    AxisFilter f;
    // Drive to 0.5 first
    for (int i = 0; i < 100; ++i) {
        f.update(0.5f, 0.02f, 0.3f, 10.f);
    }
    CHECK(f.filtered == doctest::Approx(0.5f).epsilon(0.01));

    // Now drive to zero
    for (int i = 0; i < 100; ++i) {
        f.update(0.f, 0.02f, 0.3f, 10.f);
    }
    CHECK(f.filtered == doctest::Approx(0.f).epsilon(0.01));
}

TEST_CASE("AxisFilter.StepResponse.NegativeTarget")
{
    AxisFilter f;
    for (int i = 0; i < 100; ++i) {
        f.update(-0.7f, 0.02f, 0.3f, 10.f);
    }
    CHECK(f.filtered == doctest::Approx(-0.7f).epsilon(0.01));
}

TEST_CASE("AxisFilter.RateLimit.LargeStepClamped")
{
    AxisFilter f;
    // max_rate = 1.0 rad/s, dt = 0.02s → max delta = 0.02 rad
    float result = f.update(1.0f, 0.02f, 0.001f, 1.0f);  // tiny tau → alpha≈1
    // From 0 toward 1.0, clamped to 0.02
    CHECK(result == doctest::Approx(0.02f).epsilon(0.005));
}

TEST_CASE("AxisFilter.RateLimit.SmallStepNotClamped")
{
    AxisFilter f;
    // Target 0.01, max_rate=1.0, dt=0.02 → max_delta=0.02 > 0.01
    // With tiny tau, alpha≈1, so filtered≈target
    float result = f.update(0.01f, 0.02f, 0.001f, 1.0f);
    CHECK(result == doctest::Approx(0.01f).epsilon(0.005));
}

TEST_CASE("AxisFilter.RateLimit.NegativeDirection")
{
    AxisFilter f;
    // Drive to 0.5 first
    for (int i = 0; i < 200; ++i) {
        f.update(0.5f, 0.02f, 0.01f, 10.f);
    }
    // Now step to -0.5, rate limit = 1.0 rad/s → max_delta = 0.02
    float before = f.filtered;
    float result = f.update(-0.5f, 0.02f, 0.001f, 1.0f);
    CHECK(result < before);
    CHECK((before - result) == doctest::Approx(0.02f).epsilon(0.005));
}

TEST_CASE("AxisFilter.LowPass.AlphaComputation")
{
    AxisFilter f;
    // tau=1.0, dt=1.0 → alpha = 1/(1+1) = 0.5
    // Step from 0 to 1.0, no rate limit (large max_rate)
    float result = f.update(1.0f, 1.0f, 1.0f, 100.f);
    CHECK(result == doctest::Approx(0.5f).epsilon(0.01));
}

TEST_CASE("AxisFilter.LowPass.NoiseRejection")
{
    AxisFilter f;
    // Alternating ±0.5 noise around 0, with moderate tau
    for (int i = 0; i < 100; ++i) {
        float noise = (i % 2 == 0) ? 0.5f : -0.5f;
        f.update(noise, 0.02f, 0.5f, 10.f);
    }
    // After alternating noise, filter output should be near 0
    CHECK(std::abs(f.filtered) < 0.1f);
}

TEST_CASE("AxisFilter.EdgeCase.ZeroDt")
{
    AxisFilter f;
    // dt=0 → jump to target immediately
    float result = f.update(0.7f, 0.f, 0.3f, 1.0f);
    CHECK(result == doctest::Approx(0.7f));
}

TEST_CASE("AxisFilter.EdgeCase.LargeDt")
{
    AxisFilter f;
    // dt=100s, tau=0.3 → alpha ≈ 1.0, essentially instant tracking
    // But rate limit at 1.0 rad/s * 100s = 100 rad — no clamp
    float result = f.update(0.5f, 100.f, 0.3f, 1.0f);
    CHECK(result == doctest::Approx(0.5f).epsilon(0.01));
}

// ============================================================================
// GravityVector tests
// ============================================================================

TEST_CASE("GravityVector.Hover.IdentityQuat")
{
    // Identity quaternion: no rotation. In ENU, gravity = [0,0,-1].
    // For a tailsitter, identity = nose up. Body X is horizontal.
    auto g = gravityInBody(1.f, 0.f, 0.f, 0.f);
    // g_body = R^T * [0,0,-1] → for identity R, g_body = [0,0,-1]
    // gx = 2*(w*y - x*z) = 0
    // gy = -2*(y*z + w*x) = 0
    // gz = 2*(x*x + y*y) - 1 = -1
    CHECK(g.gx == doctest::Approx(0.f).epsilon(0.01));
    CHECK(g.gy == doctest::Approx(0.f).epsilon(0.01));
    CHECK(g.gz == doctest::Approx(-1.f).epsilon(0.01));
}

TEST_CASE("GravityVector.FwFlight.NoseForward")
{
    // Tailsitter pitched 90° forward: nose (Z) points forward, belly (X) points down.
    // Rotation: 90° around Y axis (pitch forward).
    auto q = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    // In FW flight, gravity should be mostly along body X (belly = down).
    CHECK(g.gx == doctest::Approx(1.f).epsilon(0.05));
    CHECK(std::abs(g.gy) < 0.1f);
    CHECK(std::abs(g.gz) < 0.1f);
}

TEST_CASE("GravityVector.BankedTurn.30deg")
{
    // FW flight (90° pitch) + 30° bank.
    // After 90° pitch around Y, the flight direction is along world X.
    // Banking = roll around world X (the flight direction).
    auto pitch = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto bank = quatFromAxisAngle(1.f, 0.f, 0.f, 30.f * kDeg2Rad);
    auto q = quatMul(bank, pitch);  // body→world: first pitch, then bank
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    // gx = cos(30°) ≈ 0.866 (belly mostly down)
    CHECK(g.gx == doctest::Approx(std::cos(30.f * kDeg2Rad)).epsilon(0.05));
    // gy should be nonzero (bank creates lateral gravity component)
    CHECK(std::abs(g.gy) > 0.3f);
}

TEST_CASE("GravityVector.BankedTurn.45deg")
{
    auto pitch = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto bank = quatFromAxisAngle(1.f, 0.f, 0.f, 45.f * kDeg2Rad);
    auto q = quatMul(bank, pitch);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    // At 45° bank, gx ≈ cos(45°) ≈ 0.707
    CHECK(g.gx == doctest::Approx(std::cos(45.f * kDeg2Rad)).epsilon(0.05));
}

TEST_CASE("GravityVector.PitchUp.15deg")
{
    // FW flight with 15° nose-up pitch: 90° - 15° = 75° pitch from hover.
    auto q = quatFromAxisAngle(0.f, 1.f, 0.f, 75.f * kDeg2Rad);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    // gx = cos(15°) ≈ 0.966 (mostly down)
    CHECK(g.gx > 0.9f);
    // gz should have a small component from the pitch-up
    CHECK(std::abs(g.gz) > 0.1f);
}

// ============================================================================
// GimbalTargets tests
// ============================================================================

TEST_CASE("GimbalTargets.Inactive.ReturnsZero")
{
    GravityVector g{1.0f, 0.5f, -0.8f};
    auto t = gimbalTargets(g, false, 1.f, 0.7f, 1.f, 0.7f);
    CHECK(t.yaw == 0.f);
    CHECK(t.pitch == 0.f);
}

TEST_CASE("GimbalTargets.Active.CorrectYaw")
{
    // gx=1.0, gy=0.2 → yaw = -atan2(0.2, 1.0) ≈ -0.197
    GravityVector g{1.0f, 0.2f, 0.0f};
    auto t = gimbalTargets(g, true, 1.f, 0.7f, 1.f, 0.7f);
    CHECK(t.yaw == doctest::Approx(-std::atan2(0.2f, 1.0f)).epsilon(0.01));
}

TEST_CASE("GimbalTargets.Active.CorrectPitch")
{
    // gx=1.0, gz=0.3 → pitch = -atan2(0.3, 1.0) ≈ -0.291
    GravityVector g{1.0f, 0.0f, 0.3f};
    auto t = gimbalTargets(g, true, 1.f, 0.7f, 1.f, 0.7f);
    CHECK(t.pitch == doctest::Approx(-std::atan2(0.3f, 1.0f)).epsilon(0.01));
}

TEST_CASE("GimbalTargets.GainScaling")
{
    GravityVector g{1.0f, 0.2f, 0.0f};
    auto t1 = gimbalTargets(g, true, 1.f, 0.7f, 1.f, 0.7f);
    auto t2 = gimbalTargets(g, true, 2.f, 0.7f, 1.f, 0.7f);  // 2x yaw gain
    // Gain 2x should roughly double the output (before clamping)
    CHECK(std::abs(t2.yaw) > std::abs(t1.yaw));
    CHECK(std::abs(t2.yaw) == doctest::Approx(2.f * std::abs(t1.yaw)).epsilon(0.01));
}

TEST_CASE("GimbalTargets.MaxAngleClamping")
{
    // gx=0.5, gy=0.5 → atan2(0.5, 0.5) = π/4 ≈ 0.785
    // max_angle = 0.3 → should clamp
    GravityVector g{0.5f, 0.5f, 0.5f};
    auto t = gimbalTargets(g, true, 1.f, 0.3f, 1.f, 0.3f);
    CHECK(std::abs(t.yaw) <= 0.3f + 0.001f);
    CHECK(std::abs(t.pitch) <= 0.3f + 0.001f);
}

// ============================================================================
// ActivationGate tests
// ============================================================================

TEST_CASE("ActivationGate.StartsInactive")
{
    ActivationGate gate;
    CHECK_FALSE(gate.active);
    CHECK(gate.consecutive_count == 0);
}

TEST_CASE("ActivationGate.NeedsConsecutiveSamples")
{
    ActivationGate gate;
    // Feed 4 samples above enable threshold (need 5 to activate)
    for (int i = 0; i < 4; ++i) {
        CHECK_FALSE(gate.check(0.9f, 0.85f, 0.5f, 5));
    }
    // 5th sample activates
    CHECK(gate.check(0.9f, 0.85f, 0.5f, 5));
    CHECK(gate.active);
}

TEST_CASE("ActivationGate.ResetOnDipBelowEnable")
{
    ActivationGate gate;
    // 3 above enable
    for (int i = 0; i < 3; ++i) {
        gate.check(0.9f, 0.85f, 0.5f, 5);
    }
    CHECK(gate.consecutive_count == 3);
    // One dip below enable (but above disable) resets counter
    gate.check(0.7f, 0.85f, 0.5f, 5);
    CHECK(gate.consecutive_count == 0);
    CHECK_FALSE(gate.active);
}

TEST_CASE("ActivationGate.Hysteresis.StaysActiveAboveDisable")
{
    ActivationGate gate;
    // Activate
    for (int i = 0; i < 5; ++i) {
        gate.check(0.9f, 0.85f, 0.5f, 5);
    }
    CHECK(gate.active);
    // gx drops to 0.6 — below enable (0.85) but above disable (0.5) → stay active
    CHECK(gate.check(0.6f, 0.85f, 0.5f, 5));
    CHECK(gate.active);
}

TEST_CASE("ActivationGate.Hysteresis.DeactivatesBelowDisable")
{
    ActivationGate gate;
    // Activate
    for (int i = 0; i < 5; ++i) {
        gate.check(0.9f, 0.85f, 0.5f, 5);
    }
    CHECK(gate.active);
    // gx drops below disable threshold → deactivate
    CHECK_FALSE(gate.check(0.4f, 0.85f, 0.5f, 5));
    CHECK_FALSE(gate.active);
}

TEST_CASE("ActivationGate.ReactivationRequiresFullCount")
{
    ActivationGate gate;
    // Activate then deactivate
    for (int i = 0; i < 5; ++i) gate.check(0.9f, 0.85f, 0.5f, 5);
    gate.check(0.3f, 0.85f, 0.5f, 5);
    CHECK_FALSE(gate.active);
    // Need full 5 consecutive to reactivate
    for (int i = 0; i < 4; ++i) {
        CHECK_FALSE(gate.check(0.9f, 0.85f, 0.5f, 5));
    }
    CHECK(gate.check(0.9f, 0.85f, 0.5f, 5));
}

TEST_CASE("ActivationGate.TransitionSimulation")
{
    // Simulate a realistic FW transition: gx ramps 0→1 over ~2s at 50Hz
    ActivationGate gate;
    for (int i = 0; i < 100; ++i) {
        float gx = static_cast<float>(i) / 100.f;  // 0 to 0.99
        gate.check(gx, 0.85f, 0.5f, 25);
    }
    // Should not activate until gx > 0.85 for 25 consecutive samples
    // gx crosses 0.85 at sample 86, so activation at sample 86+25=cannot happen
    // since gx only reaches 0.99 at sample 99. With 14 samples above 0.85,
    // the gate should NOT activate during a linear ramp
    CHECK_FALSE(gate.active);
}

// ============================================================================
// Saturation tests
// ============================================================================

TEST_CASE("Saturation.ZeroCommand")
{
    CHECK(saturationRatio(0.f, 0.7f) == doctest::Approx(0.f));
}

TEST_CASE("Saturation.HalfCommand")
{
    CHECK(saturationRatio(0.35f, 0.7f) == doctest::Approx(0.5f));
}

TEST_CASE("Saturation.FullCommand")
{
    CHECK(saturationRatio(0.7f, 0.7f) == doctest::Approx(1.f));
}

TEST_CASE("Saturation.OverCommand.Clamped")
{
    CHECK(saturationRatio(1.0f, 0.7f) == doctest::Approx(1.f));
}

TEST_CASE("Saturation.NegativeCommand")
{
    // Saturation uses abs(cmd)
    CHECK(saturationRatio(-0.35f, 0.7f) == doctest::Approx(0.5f));
}

TEST_CASE("Saturation.ZeroMaxAngle")
{
    // Edge case: max_angle=0 should return 0 (not crash)
    CHECK(saturationRatio(0.5f, 0.f) == doctest::Approx(0.f));
}
