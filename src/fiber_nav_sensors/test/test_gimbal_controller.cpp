#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_sensors/gimbal_math.hpp>

#include <cmath>
#include <numbers>

using fiber_nav_sensors::AxisFilter;
using fiber_nav_sensors::GravityVector;
using fiber_nav_sensors::GimbalTargets;
using fiber_nav_sensors::gravityInBody;
using fiber_nav_sensors::gimbalTargetsNadir;
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
constexpr float kPiHalf = std::numbers::pi_v<float> / 2.f;
constexpr float kMaxAngle = 2.0f;

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
    CHECK(g.gx == doctest::Approx(0.f).epsilon(0.01));
    CHECK(g.gy == doctest::Approx(0.f).epsilon(0.01));
    CHECK(g.gz == doctest::Approx(-1.f).epsilon(0.01));
}

TEST_CASE("GravityVector.FwFlight.NoseForward")
{
    // Tailsitter pitched 90° forward: nose (Z) points forward, belly (X) points down.
    auto q = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    CHECK(g.gx == doctest::Approx(1.f).epsilon(0.05));
    CHECK(std::abs(g.gy) < 0.1f);
    CHECK(std::abs(g.gz) < 0.1f);
}

TEST_CASE("GravityVector.BankedTurn.30deg")
{
    auto pitch = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto bank = quatFromAxisAngle(1.f, 0.f, 0.f, 30.f * kDeg2Rad);
    auto q = quatMul(bank, pitch);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    CHECK(g.gx == doctest::Approx(std::cos(30.f * kDeg2Rad)).epsilon(0.05));
    CHECK(std::abs(g.gy) > 0.3f);
}

TEST_CASE("GravityVector.BankedTurn.45deg")
{
    auto pitch = quatFromAxisAngle(0.f, 1.f, 0.f, std::numbers::pi_v<float> / 2.f);
    auto bank = quatFromAxisAngle(1.f, 0.f, 0.f, 45.f * kDeg2Rad);
    auto q = quatMul(bank, pitch);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    CHECK(g.gx == doctest::Approx(std::cos(45.f * kDeg2Rad)).epsilon(0.05));
}

TEST_CASE("GravityVector.PitchUp.15deg")
{
    auto q = quatFromAxisAngle(0.f, 1.f, 0.f, 75.f * kDeg2Rad);
    auto g = gravityInBody(q.w, q.x, q.y, q.z);
    CHECK(g.gx > 0.9f);
    CHECK(std::abs(g.gz) > 0.1f);
}

// ============================================================================
// GimbalTargetsNadir tests — tail-mount nadir tracking
// ============================================================================

TEST_CASE("GimbalNadir.Hover.ZeroCorrection")
{
    // g = [0, 0, -1]: sensor already at nadir → both targets ≈ 0
    GravityVector g{0.f, 0.f, -1.f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    CHECK(t.yaw == doctest::Approx(0.f).epsilon(0.01));
    CHECK(t.pitch == doctest::Approx(0.f).epsilon(0.01));
}

TEST_CASE("GimbalNadir.FwCruise.PitchMinusPiHalf")
{
    // g = [1, 0, 0]: need -π/2 pitch, 0 roll
    GravityVector g{1.f, 0.f, 0.f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    CHECK(t.yaw == doctest::Approx(0.f).epsilon(0.01));
    CHECK(t.pitch == doctest::Approx(-kPiHalf).epsilon(0.01));
}

TEST_CASE("GimbalNadir.BankedFw.RollAndPitch")
{
    // 30° bank in FW: g ≈ [0.866, 0.5, 0]
    GravityVector g{0.866f, 0.5f, 0.f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    CHECK(t.yaw == doctest::Approx(std::atan2(0.5f, 0.866f)).epsilon(0.02));
    CHECK(t.pitch == doctest::Approx(-kPiHalf).epsilon(0.05));
}

TEST_CASE("GimbalNadir.FwClimb15.ReducedPitch")
{
    // 15° climb: g ≈ [0.966, 0, -0.259]
    GravityVector g{0.966f, 0.f, -0.259f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    CHECK(t.yaw == doctest::Approx(0.f).epsilon(0.01));
    // pitch = -atan2(0.966, 0.259) ≈ -1.31
    CHECK(t.pitch == doctest::Approx(-std::atan2(0.966f, 0.259f)).epsilon(0.02));
    CHECK(std::abs(t.pitch) < kPiHalf);  // less than 90° — climbing reduces correction
}

TEST_CASE("GimbalNadir.FwDive15.IncreasedPitch")
{
    // 15° dive: g ≈ [0.966, 0, 0.259]
    GravityVector g{0.966f, 0.f, 0.259f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    // pitch = -atan2(0.966, -0.259) ≈ -1.83
    CHECK(t.pitch == doctest::Approx(-std::atan2(0.966f, -0.259f)).epsilon(0.02));
    CHECK(std::abs(t.pitch) > kPiHalf);  // beyond 90° — diving increases correction
}

TEST_CASE("GimbalNadir.NearHoverGuard.RollIsZero")
{
    // gx, gy near zero → roll guard returns 0
    GravityVector g{0.05f, 0.03f, -0.998f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    CHECK(t.yaw == doctest::Approx(0.f));  // guard: gxy² = 0.0034 < 0.01
    CHECK(std::abs(t.pitch) < 0.1f);       // near-zero pitch
}

TEST_CASE("GimbalNadir.MaxAngleClamping")
{
    // pitch_max = 1.0 → should clamp even though FW needs -π/2
    GravityVector g{1.f, 0.f, 0.f};
    auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, 1.0f);
    CHECK(t.pitch == doctest::Approx(-1.0f).epsilon(0.001));
}

TEST_CASE("GimbalNadir.GainScaling")
{
    GravityVector g{0.866f, 0.5f, 0.f};
    auto t1 = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
    auto t2 = gimbalTargetsNadir(g, 2.f, 4.0f, 1.f, kMaxAngle);  // 2x roll gain
    CHECK(std::abs(t2.yaw) == doctest::Approx(2.f * std::abs(t1.yaw)).epsilon(0.01));
}

TEST_CASE("GimbalNadir.Continuity.SmoothTransition")
{
    // Simulate gradual transition from hover to FW: gz goes -1→0, gx goes 0→1
    float prev_pitch = 0.f;
    for (int i = 0; i <= 90; ++i) {
        float angle = static_cast<float>(i) * kDeg2Rad;
        GravityVector g{std::sin(angle), 0.f, -std::cos(angle)};
        auto t = gimbalTargetsNadir(g, 1.f, kMaxAngle, 1.f, kMaxAngle);
        // Pitch should monotonically decrease (become more negative)
        CHECK(t.pitch <= prev_pitch + 0.01f);
        prev_pitch = t.pitch;
    }
    // Final pitch should be approximately -π/2
    CHECK(prev_pitch == doctest::Approx(-kPiHalf).epsilon(0.02));
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
    CHECK(saturationRatio(-0.35f, 0.7f) == doctest::Approx(0.5f));
}

TEST_CASE("Saturation.ZeroMaxAngle")
{
    CHECK(saturationRatio(0.5f, 0.f) == doctest::Approx(0.f));
}
