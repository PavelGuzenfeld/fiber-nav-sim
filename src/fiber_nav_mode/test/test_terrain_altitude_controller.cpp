#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <algorithm>
#include <cmath>

// Test pure math without ROS dependencies.
// Re-define config struct and inline the controller formulas
// (same pattern as test_vtol_navigation.cpp).

namespace terrain_ctrl {

struct TerrainFollowConfig
{
    bool enabled = false;
    float target_agl = 30.f;
    float kp = 0.5f;
    float max_rate = 3.f;
    float filter_tau = 0.5f;
    float fallback_timeout = 5.f;
    float min_agl = 10.f;
    float max_agl = 200.f;
};

float clampedTargetAgl(const TerrainFollowConfig& cfg)
{
    return std::clamp(cfg.target_agl, cfg.min_agl, cfg.max_agl);
}

/// P-controller: NED vz (positive = descend)
float computeVzCommand(float filtered_dist_bottom, const TerrainFollowConfig& cfg)
{
    const float error = filtered_dist_bottom - clampedTargetAgl(cfg);
    return std::clamp(error * cfg.kp, -cfg.max_rate, cfg.max_rate);
}

/// FW height rate (positive = climb) = -vz
float computeHeightRate(float filtered_dist_bottom, const TerrainFollowConfig& cfg)
{
    return -computeVzCommand(filtered_dist_bottom, cfg);
}

/// Dynamic AMSL target
float computeTargetAmsl(float current_amsl, float filtered_dist_bottom,
                        float fallback_amsl, bool active, const TerrainFollowConfig& cfg)
{
    if (!active) {
        return fallback_amsl;
    }
    const float terrain_amsl = current_amsl - filtered_dist_bottom;
    return terrain_amsl + clampedTargetAgl(cfg);
}

/// Low-pass filter step
float filterStep(float filtered, float raw, float dt, float tau)
{
    const float alpha = dt / (tau + dt);
    return filtered + alpha * (raw - filtered);
}

}  // namespace terrain_ctrl

using terrain_ctrl::TerrainFollowConfig;
using terrain_ctrl::clampedTargetAgl;
using terrain_ctrl::computeVzCommand;
using terrain_ctrl::computeHeightRate;
using terrain_ctrl::computeTargetAmsl;
using terrain_ctrl::filterStep;

// --- Rate computation tests ---

TEST_CASE("TerrainRate.AboveTarget")
{
    // dist=40, target=30, kp=0.5 -> error=10, rate=5.0, clamped to 3.0
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    cfg.max_rate = 3.f;
    float vz = computeVzCommand(40.f, cfg);
    CHECK(vz == doctest::Approx(3.f));
}

TEST_CASE("TerrainRate.BelowTarget")
{
    // dist=20, target=30, kp=0.5 -> error=-10, rate=-5.0, clamped to -3.0
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    cfg.max_rate = 3.f;
    float vz = computeVzCommand(20.f, cfg);
    CHECK(vz == doctest::Approx(-3.f));
}

TEST_CASE("TerrainRate.AtTarget")
{
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    float vz = computeVzCommand(30.f, cfg);
    CHECK(vz == doctest::Approx(0.f));
}

TEST_CASE("TerrainRate.SmallError")
{
    // dist=32, target=30, kp=0.5 -> error=2, rate=1.0 (not clamped)
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    cfg.max_rate = 3.f;
    float vz = computeVzCommand(32.f, cfg);
    CHECK(vz == doctest::Approx(1.f));
}

// --- Filter tests ---

TEST_CASE("TerrainFilter.Converges")
{
    // Step input 30 -> 40, tau=0.5, dt=0.02
    // Should converge to within 1% of target in ~4 * tau = 2s (100 steps)
    const float tau = 0.5f;
    const float dt = 0.02f;
    float filtered = 30.f;  // initial

    // Run for 4 seconds (200 steps)
    for (int i = 0; i < 200; ++i) {
        filtered = filterStep(filtered, 40.f, dt, tau);
    }
    CHECK(filtered == doctest::Approx(40.f).epsilon(0.01));
}

TEST_CASE("TerrainFilter.SmoothsNoise")
{
    // Alternating 29/31 around target 30, tau=0.5, dt=0.02
    const float tau = 0.5f;
    const float dt = 0.02f;
    float filtered = 30.f;

    for (int i = 0; i < 200; ++i) {
        float raw = (i % 2 == 0) ? 29.f : 31.f;
        filtered = filterStep(filtered, raw, dt, tau);
    }
    // Should stay close to 30 with small oscillation
    CHECK(filtered == doctest::Approx(30.f).epsilon(0.02));
}

// --- Dynamic AMSL computation ---

TEST_CASE("TerrainAmsl.DynamicComputation")
{
    // AMSL=180, dist=40, target=30 -> terrain_amsl=140, target_amsl=170
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    float result = computeTargetAmsl(180.f, 40.f, 200.f, true, cfg);
    CHECK(result == doctest::Approx(170.f));
}

// --- AGL clamping ---

TEST_CASE("TerrainClamp.MinAgl")
{
    TerrainFollowConfig cfg;
    cfg.target_agl = 5.f;
    cfg.min_agl = 10.f;
    cfg.max_agl = 200.f;
    CHECK(clampedTargetAgl(cfg) == doctest::Approx(10.f));
}

TEST_CASE("TerrainClamp.MaxAgl")
{
    TerrainFollowConfig cfg;
    cfg.target_agl = 250.f;
    cfg.min_agl = 10.f;
    cfg.max_agl = 200.f;
    CHECK(clampedTargetAgl(cfg) == doctest::Approx(200.f));
}

// --- Fallback ---

TEST_CASE("TerrainFallback.WhenInactive")
{
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    float result = computeTargetAmsl(180.f, 40.f, 200.f, false, cfg);
    CHECK(result == doctest::Approx(200.f));  // returns fallback
}

// --- Sign conventions ---

TEST_CASE("TerrainSign.HeightRate")
{
    // dist > target -> vz positive (descend in NED) -> height_rate negative (FW descend)
    // dist=40, target=30 -> vz = clamp(5, -3, 3) = 3 -> height_rate = -3
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    cfg.max_rate = 3.f;
    float hr = computeHeightRate(40.f, cfg);
    CHECK(hr == doctest::Approx(-3.f));
}

TEST_CASE("TerrainSign.VzCommandDescend")
{
    // dist > target -> positive vz (NED descend)
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;
    cfg.kp = 0.5f;
    float vz = computeVzCommand(35.f, cfg);
    CHECK(vz > 0.f);
}

// --- Config defaults ---

TEST_CASE("TerrainConfig.Defaults")
{
    TerrainFollowConfig cfg;
    CHECK_FALSE(cfg.enabled);
    CHECK(cfg.target_agl == doctest::Approx(30.f));
    CHECK(cfg.kp == doctest::Approx(0.5f));
    CHECK(cfg.max_rate == doctest::Approx(3.f));
    CHECK(cfg.filter_tau == doctest::Approx(0.5f));
    CHECK(cfg.fallback_timeout == doctest::Approx(5.f));
    CHECK(cfg.min_agl == doctest::Approx(10.f));
    CHECK(cfg.max_agl == doctest::Approx(200.f));
}
