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
    float slope_tau = 1.0f;
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

/// Rate-limited smoothed AMSL target with two-layer safety floor.
/// Simulates the logic from TerrainAltitudeController::computeSmoothedTargetAmsl().
struct SmoothedTargetState {
    bool primed = false;
    float target = 0.f;
    bool emergency = false;
};

float computeSmoothedTargetAmsl(
    SmoothedTargetState& state,
    float raw_target, float fallback_amsl, float dt,
    float rate_slew, float terrain_amsl_filtered, bool terrain_primed,
    float min_agl, float raw_agl, float current_amsl)
{
    if (!state.primed) {
        state.target = fallback_amsl;
        state.primed = true;
    } else {
        const float max_delta = rate_slew * dt;
        const float delta = std::clamp(raw_target - state.target, -max_delta, max_delta);
        state.target += delta;
    }

    // Layer 1: Terrain floor
    if (terrain_primed) {
        const float terrain_floor = terrain_amsl_filtered + min_agl;
        if (state.target < terrain_floor) {
            state.target = terrain_floor;
        }
    }

    // Layer 2: Emergency recovery
    if (raw_agl > 0.f && raw_agl < min_agl) {
        const float deficit = min_agl - raw_agl;
        const float emergency_target = current_amsl + deficit;
        state.target = std::max(state.target, emergency_target);
        state.emergency = true;
    } else {
        state.emergency = false;
    }

    return state.target;
}

}  // namespace terrain_ctrl

using terrain_ctrl::TerrainFollowConfig;
using terrain_ctrl::SmoothedTargetState;
using terrain_ctrl::computeSmoothedTargetAmsl;
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
    CHECK(cfg.slope_tau == doctest::Approx(1.0f));
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

// --- Safety floor tests ---

TEST_CASE("SafetyFloor.TerrainFloorPreventsDescentBelowMinAgl")
{
    // Scenario: rate-limited target wants to descend below terrain + min_agl.
    // The terrain floor should clamp it.
    SmoothedTargetState state;
    const float dt = 0.1f;
    const float rate_slew = 1.0f;
    const float min_agl = 10.f;
    const float terrain_amsl = 150.f;  // terrain at 150m AMSL
    const float current_amsl = 165.f;  // aircraft at 165m AMSL (15m AGL)
    const float raw_agl = 15.f;

    // Raw target wants to descend to 155m (only 5m above terrain — below min_agl!)
    const float raw_target = 155.f;
    const float fallback = 200.f;

    // First call primes from fallback (200m)
    float result = computeSmoothedTargetAmsl(state, raw_target, fallback, dt,
        rate_slew, terrain_amsl, true, min_agl, raw_agl, current_amsl);
    CHECK(result == doctest::Approx(200.f));  // Primed from fallback

    // Subsequent calls descend toward raw_target but get clamped by terrain floor
    for (int i = 0; i < 1000; ++i) {
        result = computeSmoothedTargetAmsl(state, raw_target, fallback, dt,
            rate_slew, terrain_amsl, true, min_agl, raw_agl, current_amsl);
    }
    // Floor = terrain(150) + min_agl(10) = 160. Should never go below.
    CHECK(result >= terrain_amsl + min_agl - 0.01f);
    CHECK(result == doctest::Approx(160.f).epsilon(0.01));
}

TEST_CASE("SafetyFloor.NoClampWhenAboveFloor")
{
    // When target is well above terrain + min_agl, floor has no effect
    SmoothedTargetState state;
    const float dt = 0.1f;
    const float rate_slew = 1.0f;
    const float min_agl = 10.f;
    const float terrain_amsl = 150.f;
    const float current_amsl = 190.f;  // 40m AGL
    const float raw_agl = 40.f;
    const float raw_target = 180.f;    // 30m above terrain — well above floor
    const float fallback = 200.f;

    computeSmoothedTargetAmsl(state, raw_target, fallback, dt,
        rate_slew, terrain_amsl, true, min_agl, raw_agl, current_amsl);

    // Let it converge
    float result = 0.f;
    for (int i = 0; i < 500; ++i) {
        result = computeSmoothedTargetAmsl(state, raw_target, fallback, dt,
            rate_slew, terrain_amsl, true, min_agl, raw_agl, current_amsl);
    }
    // Should converge to raw_target (180), not the floor (160)
    CHECK(result == doctest::Approx(180.f).epsilon(0.1));
}

TEST_CASE("SafetyFloor.EmergencyClimbWhenBelowMinAgl")
{
    // Scenario: aircraft was flying normally, terrain suddenly rose but the
    // filtered terrain AMSL hasn't caught up yet (filter lag). Layer 2
    // (emergency) detects low raw AGL and forces immediate climb, even though
    // Layer 1's floor (based on filtered terrain) is still too low.
    SmoothedTargetState state;
    const float dt = 0.1f;
    const float rate_slew = 1.0f;
    const float min_agl = 10.f;

    // Phase 1: Normal flight — target converges to 180m (terrain=150)
    for (int i = 0; i < 500; ++i) {
        computeSmoothedTargetAmsl(state, 180.f, 200.f, dt,
            rate_slew, 150.f, true, min_agl, 30.f, 180.f);
    }
    CHECK(state.target == doctest::Approx(180.f).epsilon(0.1));

    // Phase 2: Terrain suddenly rises to 170m, but terrain_amsl_filtered is
    // still at 155m (filter hasn't caught up). Aircraft at 175m → AGL = 5m!
    const float terrain_filtered_lagging = 155.f;  // filter lag!
    const float actual_terrain = 170.f;  // real terrain
    const float current_amsl = 175.f;
    const float raw_agl = current_amsl - actual_terrain;  // 5m

    float result = computeSmoothedTargetAmsl(state, 180.f, 200.f, dt,
        rate_slew, terrain_filtered_lagging, true, min_agl, raw_agl, current_amsl);

    // Layer 1 floor = terrain_filtered(155) + min_agl(10) = 165 → doesn't help (target was 180)
    // Layer 2 emergency = current_amsl(175) + deficit(10-5=5) = 180 > target(~179.9)
    CHECK(result >= 180.f - 0.1f);
    CHECK(state.emergency == true);
}

TEST_CASE("SafetyFloor.EmergencyRecovery")
{
    SmoothedTargetState state;
    const float dt = 0.1f;
    const float rate_slew = 1.0f;
    const float min_agl = 10.f;

    // Phase 1: Normal flight, converge to ~180m
    for (int i = 0; i < 500; ++i) {
        computeSmoothedTargetAmsl(state, 180.f, 200.f, dt,
            rate_slew, 150.f, true, min_agl, 30.f, 180.f);
    }

    // Phase 2: Terrain rises (filter lagging), AGL = 5m → emergency
    computeSmoothedTargetAmsl(state, 170.f, 200.f, dt,
        rate_slew, 155.f, true, min_agl, 5.f, 175.f);
    CHECK(state.emergency == true);

    // Phase 3: Aircraft climbed, now at 12m AGL (above min_agl)
    computeSmoothedTargetAmsl(state, 200.f, 200.f, dt,
        rate_slew, 170.f, true, min_agl, 12.f, 182.f);
    CHECK(state.emergency == false);  // Recovered
}

// --- Sensor-derived terrain slope helper ---

namespace terrain_ctrl {

/// Compute sensor-derived terrain slope from consecutive terrain AMSL readings.
/// Returns spatial slope (m/m): positive = terrain rising ahead.
float computeSensorSlope(float terrain_now, float terrain_prev, float ground_speed, float dt)
{
    if (ground_speed < 1.f) return 0.f;
    return (terrain_now - terrain_prev) / (ground_speed * dt);
}

}  // namespace terrain_ctrl

using terrain_ctrl::computeSensorSlope;

// --- Sensor slope tests ---

TEST_CASE("SensorSlope.RisingTerrain")
{
    // Terrain rises 1m over one cycle at 18 m/s, dt=0.02s → distance = 0.36m
    // slope = 1.0 / 0.36 = 2.78 m/m
    const float ground_speed = 18.f;
    const float dt = 0.02f;
    const float terrain_prev = 150.f;
    const float terrain_now = 151.f;

    float slope = computeSensorSlope(terrain_now, terrain_prev, ground_speed, dt);
    CHECK(slope > 0.f);
    CHECK(slope == doctest::Approx(1.f / (18.f * 0.02f)));
}

TEST_CASE("SensorSlope.FlatTerrain")
{
    float slope = computeSensorSlope(150.f, 150.f, 18.f, 0.02f);
    CHECK(slope == doctest::Approx(0.f));
}

TEST_CASE("SensorSlope.DescendingTerrain")
{
    // Terrain drops 0.5m
    float slope = computeSensorSlope(149.5f, 150.f, 18.f, 0.02f);
    CHECK(slope < 0.f);
    CHECK(slope == doctest::Approx(-0.5f / (18.f * 0.02f)));
}

TEST_CASE("SensorSlope.LowSpeedReturnsZero")
{
    // Below 1 m/s threshold — guard against division by near-zero distance
    float slope = computeSensorSlope(151.f, 150.f, 0.5f, 0.02f);
    CHECK(slope == doctest::Approx(0.f));
}

TEST_CASE("SensorSlope.FilterConvergence")
{
    // EMA filter with slope_tau=1.0s should converge to true slope after ~4*tau.
    // True terrain slope = 0.1 (10% grade), ground_speed=18, dt=0.02s
    // Each cycle: terrain rises by slope * ground_speed * dt = 0.1 * 18 * 0.02 = 0.036m
    const float tau = 1.0f;
    const float dt = 0.02f;
    const float ground_speed = 18.f;
    const float true_slope = 0.1f;
    const float d_terrain = true_slope * ground_speed * dt;  // 0.036m per cycle

    float filtered_slope = 0.f;
    float terrain = 150.f;

    // Run for 4*tau = 4s = 200 steps
    for (int i = 0; i < 200; ++i) {
        terrain += d_terrain;
        float raw_slope = computeSensorSlope(terrain, terrain - d_terrain, ground_speed, dt);
        const float alpha = dt / (tau + dt);
        filtered_slope += alpha * (raw_slope - filtered_slope);
    }
    // Should converge to within 2% of true slope
    CHECK(filtered_slope == doctest::Approx(true_slope).epsilon(0.02));
}

TEST_CASE("SensorSlope.NoiseSuppression")
{
    // Alternating +0.5m / -0.5m noise on flat terrain.
    // Raw slope alternates ±1.39 m/m but filtered slope should stay near 0.
    const float tau = 1.0f;
    const float dt = 0.02f;
    const float ground_speed = 18.f;
    const float base_terrain = 150.f;

    float filtered_slope = 0.f;
    float prev_terrain = base_terrain;

    for (int i = 0; i < 200; ++i) {
        float noise = (i % 2 == 0) ? 0.5f : -0.5f;
        float terrain_now = base_terrain + noise;
        float raw_slope = computeSensorSlope(terrain_now, prev_terrain, ground_speed, dt);
        const float alpha = dt / (tau + dt);
        filtered_slope += alpha * (raw_slope - filtered_slope);
        prev_terrain = terrain_now;
    }
    // Should stay near zero despite large per-sample noise
    CHECK(std::abs(filtered_slope) < 0.1f);
}

TEST_CASE("SensorSlope.FeedforwardIntegration")
{
    // Sensor slope = 0.1, lookahead_dist = 54m (18 m/s * 3s), gain = 0.5
    // ff = 0.1 * 54 * 0.5 = 2.7m
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.5f;
    cfg.max_slope = 0.5f;

    float ff = feedforwardCorrection(0.1f, 54.f, cfg);
    CHECK(ff == doctest::Approx(2.7f));
}

TEST_CASE("SensorSlope.ClampedBeforeFeedforward")
{
    // Sensor slope = 1.0, but max_slope = 0.5 → clamped to 0.5
    // lookahead_dist = 54m, gain = 0.5
    // ff = 0.5 * 54 * 0.5 = 13.5m (not 1.0 * 54 * 0.5 = 27.0)
    TerrainFollowConfig cfg;
    cfg.feedforward_gain = 0.5f;
    cfg.max_slope = 0.5f;

    float ff = feedforwardCorrection(1.0f, 54.f, cfg);
    CHECK(ff == doctest::Approx(13.5f));
}
