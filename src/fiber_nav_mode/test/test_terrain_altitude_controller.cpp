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
    // Look-ahead parameters
    float lookahead_time = 3.f;
    float lookahead_max = 100.f;
    float feedforward_gain = 0.8f;
    float max_slope = 0.5f;
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

/// Feed-forward correction from terrain slope [m]
float feedforwardCorrection(float terrain_slope, float lookahead_dist,
                            const TerrainFollowConfig& cfg)
{
    if (lookahead_dist < 1.f) {
        return 0.f;
    }
    const float slope = std::clamp(terrain_slope, -cfg.max_slope, cfg.max_slope);
    return slope * lookahead_dist * cfg.feedforward_gain;
}

/// Dynamic AMSL target with feed-forward
float computeTargetAmsl(float current_amsl, float filtered_dist_bottom,
                        float fallback_amsl, bool active, const TerrainFollowConfig& cfg,
                        float ff_correction = 0.f)
{
    if (!active) {
        return fallback_amsl;
    }
    const float terrain_amsl = current_amsl - filtered_dist_bottom;
    return terrain_amsl + clampedTargetAgl(cfg) + ff_correction;
}

/// Low-pass filter step
float filterStep(float filtered, float raw, float dt, float tau)
{
    const float alpha = dt / (tau + dt);
    return filtered + alpha * (raw - filtered);
}

/// Compute look-ahead position in NED
struct LookaheadPos { float x; float y; float dist; };

LookaheadPos computeLookaheadPos(float cur_x, float cur_y,
                                  float vx_ned, float vy_ned,
                                  float ground_speed,
                                  const TerrainFollowConfig& cfg)
{
    if (ground_speed < 1.f) {
        return {cur_x, cur_y, 0.f};
    }
    float dist = std::min(ground_speed * cfg.lookahead_time, cfg.lookahead_max);
    float scale = dist / ground_speed;
    return {cur_x + vx_ned * scale, cur_y + vy_ned * scale, dist};
}

/// Compute terrain slope from two height queries
float computeTerrainSlope(float height_here, float height_ahead, float dist)
{
    if (dist < 1.f) return 0.f;
    return (height_ahead - height_here) / dist;
}

}  // namespace terrain_ctrl

using terrain_ctrl::TerrainFollowConfig;
using terrain_ctrl::clampedTargetAgl;
using terrain_ctrl::computeVzCommand;
using terrain_ctrl::computeHeightRate;
using terrain_ctrl::computeTargetAmsl;
using terrain_ctrl::filterStep;
using terrain_ctrl::feedforwardCorrection;
using terrain_ctrl::computeLookaheadPos;
using terrain_ctrl::computeTerrainSlope;

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
    // Look-ahead defaults
    CHECK(cfg.lookahead_time == doctest::Approx(3.f));
    CHECK(cfg.lookahead_max == doctest::Approx(100.f));
    CHECK(cfg.feedforward_gain == doctest::Approx(0.8f));
    CHECK(cfg.max_slope == doctest::Approx(0.5f));
}

// --- Look-ahead position computation ---

TEST_CASE("Lookahead.PositionComputation")
{
    TerrainFollowConfig cfg;
    cfg.lookahead_time = 3.f;
    cfg.lookahead_max = 100.f;

    // Flying north at 20 m/s from (0,0)
    auto la = computeLookaheadPos(0.f, 0.f, 20.f, 0.f, 20.f, cfg);
    CHECK(la.dist == doctest::Approx(60.f));     // 20 * 3 = 60m
    CHECK(la.x == doctest::Approx(60.f));         // 60m north
    CHECK(la.y == doctest::Approx(0.f));           // no east
}

TEST_CASE("Lookahead.ClampedToMax")
{
    TerrainFollowConfig cfg;
    cfg.lookahead_time = 3.f;
    cfg.lookahead_max = 100.f;

    // Flying east at 50 m/s → 150m unclamped, clamped to 100m
    auto la = computeLookaheadPos(100.f, 200.f, 0.f, 50.f, 50.f, cfg);
    CHECK(la.dist == doctest::Approx(100.f));     // clamped to max
    CHECK(la.x == doctest::Approx(100.f));         // no north change
    CHECK(la.y == doctest::Approx(300.f));          // 200 + 50 * (100/50)
}

TEST_CASE("Lookahead.SlowSpeedNoQuery")
{
    TerrainFollowConfig cfg;
    auto la = computeLookaheadPos(0.f, 0.f, 0.5f, 0.f, 0.5f, cfg);
    CHECK(la.dist == doctest::Approx(0.f));  // too slow, no look-ahead
}

// --- Feed-forward computation ---

TEST_CASE("Feedforward.RisingTerrain")
{
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.8f;
    cfg.max_slope = 0.5f;

    // Terrain rises 10m over 60m → slope = 0.167
    float slope = computeTerrainSlope(100.f, 110.f, 60.f);
    CHECK(slope == doctest::Approx(10.f / 60.f));

    // Feed-forward: 0.167 * 60 * 0.8 = 8.0m
    float ff = feedforwardCorrection(slope, 60.f, cfg);
    CHECK(ff == doctest::Approx(8.f).epsilon(0.1));
    CHECK(ff > 0.f);  // Rising terrain → positive correction (climb more)
}

TEST_CASE("Feedforward.FlatTerrain")
{
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.8f;

    float slope = computeTerrainSlope(100.f, 100.f, 60.f);
    CHECK(slope == doctest::Approx(0.f));

    float ff = feedforwardCorrection(slope, 60.f, cfg);
    CHECK(ff == doctest::Approx(0.f));
}

TEST_CASE("Feedforward.DescendingTerrain")
{
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.8f;
    cfg.max_slope = 0.5f;

    // Terrain drops 15m over 60m → slope = -0.25
    float slope = computeTerrainSlope(100.f, 85.f, 60.f);
    CHECK(slope < 0.f);

    float ff = feedforwardCorrection(slope, 60.f, cfg);
    CHECK(ff < 0.f);  // Descending terrain → negative correction (descend)
}

TEST_CASE("Feedforward.SlopeClamped")
{
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.8f;
    cfg.max_slope = 0.5f;

    // Extreme slope: 50m rise over 60m → slope = 0.833, clamped to 0.5
    float slope = computeTerrainSlope(100.f, 150.f, 60.f);
    CHECK(slope > cfg.max_slope);

    float ff = feedforwardCorrection(slope, 60.f, cfg);
    // Clamped: 0.5 * 60 * 0.8 = 24.0
    CHECK(ff == doctest::Approx(24.f));
}

TEST_CASE("Feedforward.ClampedToMaxRate")
{
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.8f;
    cfg.max_slope = 0.5f;
    cfg.max_rate = 3.f;

    // Even with large feed-forward, the P-controller output + ff is applied to
    // computeTargetAmsl (AMSL offset), not directly to rate. So no rate clamping here.
    // The rate clamping is in computeVzCommand which doesn't include ff.
    float slope = 0.3f;
    float ff = feedforwardCorrection(slope, 80.f, cfg);
    // 0.3 * 80 * 0.8 = 19.2m altitude offset — this is fine, TECS handles the rate
    CHECK(ff == doctest::Approx(19.2f));
}

TEST_CASE("Feedforward.ZeroDistReturnsZero")
{
    TerrainFollowConfig cfg;
    float ff = feedforwardCorrection(0.3f, 0.5f, cfg);  // dist < 1m
    CHECK(ff == doctest::Approx(0.f));
}

// --- AMSL with feed-forward ---

TEST_CASE("TerrainAmsl.WithFeedforward")
{
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;

    // Rising terrain: ff = +10m
    float result = computeTargetAmsl(180.f, 40.f, 200.f, true, cfg, 10.f);
    // terrain_amsl=140, target=140+30+10=180
    CHECK(result == doctest::Approx(180.f));
}

TEST_CASE("TerrainAmsl.FallbackIgnoresFeedforward")
{
    TerrainFollowConfig cfg;
    cfg.enabled = true;
    cfg.target_agl = 30.f;

    // When inactive, returns fallback regardless of ff
    float result = computeTargetAmsl(180.f, 40.f, 200.f, false, cfg, 10.f);
    CHECK(result == doctest::Approx(200.f));
}
