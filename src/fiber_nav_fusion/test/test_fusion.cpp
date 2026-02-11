#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>

namespace {

// Simplified quaternion operations for testing
struct Quaternion {
    double w, x, y, z;

    static Quaternion from_euler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll / 2);
        double sr = std::sin(roll / 2);
        double cp = std::cos(pitch / 2);
        double sp = std::sin(pitch / 2);
        double cy = std::cos(yaw / 2);
        double sy = std::sin(yaw / 2);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }
};

struct Vector3 {
    double x, y, z;

    double norm() const { return std::sqrt(x*x + y*y + z*z); }

    Vector3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vector3 operator+(const Vector3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vector3 operator-(const Vector3& v) const { return {x-v.x, y-v.y, z-v.z}; }
};

// Rotate vector by quaternion: q * v * q^(-1)
Vector3 rotate(const Quaternion& q, const Vector3& v) {
    // Convert vector to quaternion
    Quaternion p = {0, v.x, v.y, v.z};

    // q * p
    Quaternion qp = {
        q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
        q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
        q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
        q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
    };

    // (q * p) * q^(-1)
    Quaternion qc = q.conjugate();
    Quaternion result = {
        qp.w*qc.w - qp.x*qc.x - qp.y*qc.y - qp.z*qc.z,
        qp.w*qc.x + qp.x*qc.w + qp.y*qc.z - qp.z*qc.y,
        qp.w*qc.y - qp.x*qc.z + qp.y*qc.w + qp.z*qc.x,
        qp.w*qc.z + qp.x*qc.y - qp.y*qc.x + qp.z*qc.w
    };

    return {result.x, result.y, result.z};
}

// Replicate fusion math
struct FusionModel {
    double slack_factor = 1.05;

    Vector3 fuse(double spool_velocity, Vector3 direction, Quaternion attitude) {
        // Reconstruct body velocity
        double corrected_speed = spool_velocity / slack_factor;
        Vector3 v_body = direction * corrected_speed;

        // Rotate to NED
        Vector3 v_ned = rotate(attitude, v_body);

        return v_ned;
    }
};

// --- Flight phase detection (mirrors fiber_vision_fusion.cpp) ---

enum class FlightPhase : uint8_t {
    MC,
    FW,
    TRANSITION_FW,
    TRANSITION_MC,
};

FlightPhase determine_flight_phase(uint8_t vehicle_type,
                                   bool in_transition_mode,
                                   bool in_transition_to_fw) {
    if (in_transition_mode) {
        return in_transition_to_fw ? FlightPhase::TRANSITION_FW
                                   : FlightPhase::TRANSITION_MC;
    }
    if (vehicle_type == 2) return FlightPhase::FW;
    return FlightPhase::MC;
}

float phase_velocity_variance(FlightPhase phase, float normal_var, float transition_var) {
    switch (phase) {
        case FlightPhase::MC:            return normal_var;
        case FlightPhase::FW:            return normal_var;
        case FlightPhase::TRANSITION_FW: return transition_var;
        case FlightPhase::TRANSITION_MC: return transition_var;
    }
    return normal_var;
}

// --- Sensor health tracker (mirrors fiber_vision_fusion.cpp) ---

struct SensorHealth {
    std::vector<bool> buffer;
    size_t window_size{50};
    size_t pos{0};
    size_t count{0};
    size_t hits{0};

    void init(size_t ws) {
        window_size = ws;
        buffer.assign(ws, false);
        pos = 0;
        count = 0;
        hits = 0;
    }

    void record(bool received) {
        if (buffer.empty()) return;
        if (count >= window_size && buffer[pos]) {
            --hits;
        }
        buffer[pos] = received;
        if (received) ++hits;
        pos = (pos + 1) % window_size;
        if (count < window_size) ++count;
    }

    float health_pct() const {
        if (count == 0) return 100.0f;
        return 100.0f * static_cast<float>(hits) / static_cast<float>(count);
    }
};

// --- Health-based variance scaling (mirrors fiber_vision_fusion.cpp) ---

float health_variance_scale(float health_pct) {
    float frac = std::max(0.01f, health_pct / 100.f);
    return 1.f / (frac * frac);
}

// --- Attitude staleness scaling (mirrors fiber_vision_fusion.cpp) ---

float attitude_staleness_scale(double age_seconds) {
    if (age_seconds < 0.2) return 1.f;
    if (age_seconds < 0.5) return 2.f;
    if (age_seconds < 1.0) return 4.f;
    return 0.f;  // skip publish
}

// --- Spool-EKF cross-validation (mirrors fiber_vision_fusion.cpp) ---

float cross_validation_scale(float spool_speed, float ekf_speed) {
    if (spool_speed < 0.5f && ekf_speed < 0.5f) return 1.f;
    float ref = std::max(spool_speed, ekf_speed);
    float innovation = std::abs(spool_speed - ekf_speed) / ref;
    if (innovation < 0.3f) return 1.f;
    float excess = (innovation - 0.3f) / 0.3f;
    return 1.f + 3.f * excess * excess;
}

// --- Online slack calibration (mirrors fiber_vision_fusion.cpp) ---

float slack_calibration_update(float current_slack, float spool_speed,
                               float ekf_speed, float ema_alpha) {
    if (spool_speed < 1.0f || ekf_speed < 1.0f) return current_slack;
    float ratio = spool_speed / ekf_speed;
    float updated = current_slack + ema_alpha * (ratio - current_slack);
    return std::clamp(updated, 0.8f, 1.2f);
}

// --- Heading cross-check (mirrors fiber_vision_fusion.cpp) ---

float heading_crosscheck_scale(float heading1_rad, float heading2_rad) {
    float diff = heading1_rad - heading2_rad;
    while (diff > static_cast<float>(M_PI)) diff -= 2.f * static_cast<float>(M_PI);
    while (diff < -static_cast<float>(M_PI)) diff += 2.f * static_cast<float>(M_PI);
    float abs_diff = std::abs(diff);
    if (abs_diff < 0.1f) return 1.f;
    float excess = (abs_diff - 0.1f) / 0.2f;
    return 1.f + 3.f * excess * excess;
}

// --- Adaptive ZUPT threshold (mirrors fiber_vision_fusion.cpp) ---

float adaptive_zupt_threshold(float noise_rms) {
    return std::max(0.01f, 3.f * noise_rms);
}

// --- Noise RMS helper for testing ---

float compute_noise_rms(const std::vector<float>& samples) {
    if (samples.empty()) return 0.f;
    float sum_sq = 0.f;
    for (float s : samples) {
        sum_sq += s * s;
    }
    return std::sqrt(sum_sq / static_cast<float>(samples.size()));
}

}  // namespace

TEST_CASE("Fusion.VelocityReconstruction") {
    FusionModel model;
    model.slack_factor = 1.0;  // No slack correction

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};  // Forward
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};  // Identity (level, north)

    Vector3 result = model.fuse(spool, direction, attitude);

    CHECK(result.x == doctest::Approx(10.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.SlackCorrection") {
    FusionModel model;
    model.slack_factor = 1.05;

    double spool = 10.5;  // Measured with slack
    Vector3 direction = {1.0, 0.0, 0.0};
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.fuse(spool, direction, attitude);

    // 10.5 / 1.05 = 10.0
    CHECK(result.x == doctest::Approx(10.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotation90Yaw") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};  // Forward in body

    // 90° yaw (heading East)
    Quaternion attitude = Quaternion::from_euler(0, 0, M_PI / 2);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Body X (forward) should become NED Y (East)
    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(10.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotation180Yaw") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};

    // 180° yaw (heading South)
    Quaternion attitude = Quaternion::from_euler(0, 0, M_PI);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Body X should become -NED X (South = -North)
    CHECK(result.x == doctest::Approx(-10.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotationPitchDown") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};

    // 45° pitch down (negative pitch in aerospace convention)
    Quaternion attitude = Quaternion::from_euler(0, -M_PI / 4, 0);

    Vector3 result = model.fuse(spool, direction, attitude);

    double expected = 10.0 / std::sqrt(2.0);

    // Forward velocity should split into North and Down
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));  // North
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));       // East
    CHECK(result.z == doctest::Approx(expected).epsilon(1e-6));  // Down (positive in NED)
}

TEST_CASE("Fusion.DiagonalDirection") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    // 45° to the right in body frame
    double sqrt2 = std::sqrt(2.0);
    Vector3 direction = {1.0/sqrt2, 1.0/sqrt2, 0.0};

    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};  // Identity

    Vector3 result = model.fuse(spool, direction, attitude);

    double expected = 10.0 / sqrt2;
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.ZeroVelocity") {
    FusionModel model;

    double spool = 0.0;
    Vector3 direction = {1.0, 0.0, 0.0};
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.fuse(spool, direction, attitude);

    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.VelocityMagnitudePreserved") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 15.0;
    Vector3 direction = {0.6, 0.8, 0.0};  // Unit vector

    // Random-ish attitude
    Quaternion attitude = Quaternion::from_euler(0.1, 0.2, 0.3);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Rotation should preserve magnitude
    CHECK(result.norm() == doctest::Approx(15.0).epsilon(1e-6));
}

// --- ZUPT model tests ---

struct ZuptModel {
    double threshold = 0.05;
    double zupt_variance = 0.001;
    double normal_variance = 0.01;
    double slack_factor = 1.05;

    bool is_zupt(double speed) const { return speed < threshold; }

    Vector3 velocity(double speed, Vector3 dir, Quaternion att) const {
        if (is_zupt(speed)) return {0, 0, 0};
        double corrected = speed / slack_factor;
        Vector3 v_body = dir * corrected;
        return rotate(att, v_body);
    }
};

TEST_CASE("ZUPT.ZeroWhenStopped") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.velocity(0.01, dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));
}

TEST_CASE("ZUPT.NormalWhenMoving") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.velocity(5.0, dir, att);
    double expected = 5.0 / 1.05;
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("ZUPT.ThresholdBoundary") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    // Just below threshold → ZUPT
    Vector3 below = model.velocity(0.04999, dir, att);
    CHECK(below.x == doctest::Approx(0.0));

    // Just above threshold → normal
    Vector3 above = model.velocity(0.05001, dir, att);
    CHECK(above.x > 0.0);
}

TEST_CASE("ZUPT.IndependentOfVisionDirection") {
    ZuptModel model;

    // ZUPT should produce zeros regardless of direction vector value
    // This models the case where vision direction is stale/unavailable
    Vector3 stale_dir = {0.5, 0.5, 0.707};  // Arbitrary stale direction
    Quaternion att = Quaternion::from_euler(0.1, 0.2, 0.3);  // Arbitrary attitude

    // Speed below threshold → ZUPT ignores direction entirely
    Vector3 result = model.velocity(0.01, stale_dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));

    // Even with zero direction (no vision), ZUPT still works
    Vector3 no_dir = {0.0, 0.0, 0.0};
    result = model.velocity(0.0, no_dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));
}

// --- DragBow model tests ---

struct DragBowModel {
    double k_drag = 0.0005;
    double tunnel_heading_deg = 90.0;
    double var_long = 1.0;
    double var_lat = 100.0;

    Vector3 position(double total_length, double speed) const {
        double x_est = total_length * (1.0 - k_drag * speed * speed);
        double h = tunnel_heading_deg * M_PI / 180.0;
        return {x_est * std::cos(h), x_est * std::sin(h), 0.0};
    }

    Vector3 variance() const {
        double h = tunnel_heading_deg * M_PI / 180.0;
        double cos_h = std::cos(h);
        double sin_h = std::sin(h);
        return {
            var_long * cos_h * cos_h + var_lat * sin_h * sin_h,
            var_long * sin_h * sin_h + var_lat * cos_h * cos_h,
            1e6
        };
    }
};

TEST_CASE("DragBow.StaticPosition") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;  // North

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(100.0).epsilon(1e-6));  // North
    CHECK(pos.y == doctest::Approx(0.0).epsilon(1e-6));     // East
}

TEST_CASE("DragBow.DragCorrection") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;

    // L=100, v=10, k=0.0005 → X_est = 100*(1 - 0.05) = 95
    Vector3 pos = model.position(100.0, 10.0);
    CHECK(pos.x == doctest::Approx(95.0).epsilon(1e-6));
}

TEST_CASE("DragBow.TunnelEast") {
    DragBowModel model;
    model.tunnel_heading_deg = 90.0;

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(0.0).epsilon(1e-6));     // North ≈ 0
    CHECK(pos.y == doctest::Approx(100.0).epsilon(1e-6));    // East = X_est
}

TEST_CASE("DragBow.TunnelNorth") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(100.0).epsilon(1e-6));   // North = X_est
    CHECK(pos.y == doctest::Approx(0.0).epsilon(1e-6));      // East ≈ 0
}

TEST_CASE("DragBow.VarianceEast") {
    DragBowModel model;
    model.tunnel_heading_deg = 90.0;

    Vector3 var = model.variance();
    // heading=90°: cos=0, sin=1
    // var_n = long*0 + lat*1 = 100 (lateral)
    // var_e = long*1 + lat*0 = 1 (longitudinal)
    CHECK(var.x == doctest::Approx(100.0).epsilon(1e-6));  // North = lateral
    CHECK(var.y == doctest::Approx(1.0).epsilon(1e-6));    // East = longitudinal
    CHECK(var.z == doctest::Approx(1e6).epsilon(1.0));
}

// --- Flight phase detection tests ---

TEST_CASE("FlightPhase.MCDefault") {
    // vehicle_type=1 (ROTARY_WING), not in transition
    auto phase = determine_flight_phase(1, false, false);
    CHECK(phase == FlightPhase::MC);
}

TEST_CASE("FlightPhase.FWFromVehicleType") {
    // vehicle_type=2 (FIXED_WING), not in transition
    auto phase = determine_flight_phase(2, false, false);
    CHECK(phase == FlightPhase::FW);
}

TEST_CASE("FlightPhase.TransitionToFW") {
    // in_transition_mode=true, in_transition_to_fw=true
    auto phase = determine_flight_phase(1, true, true);
    CHECK(phase == FlightPhase::TRANSITION_FW);
}

TEST_CASE("FlightPhase.TransitionToMC") {
    // in_transition_mode=true, in_transition_to_fw=false
    auto phase = determine_flight_phase(2, true, false);
    CHECK(phase == FlightPhase::TRANSITION_MC);
}

TEST_CASE("FlightPhase.TransitionOverridesVehicleType") {
    // Even if vehicle_type says FW, transition flag takes precedence
    auto phase = determine_flight_phase(2, true, true);
    CHECK(phase == FlightPhase::TRANSITION_FW);
}

TEST_CASE("FlightPhase.UnspecifiedVehicleTypeDefaultsMC") {
    // vehicle_type=0 (UNSPECIFIED), not in transition
    auto phase = determine_flight_phase(0, false, false);
    CHECK(phase == FlightPhase::MC);
}

// --- Adaptive variance tests ---

TEST_CASE("Variance.MCUsesNormalVariance") {
    float result = phase_velocity_variance(FlightPhase::MC, 0.01f, 0.04f);
    CHECK(result == doctest::Approx(0.01f));
}

TEST_CASE("Variance.FWUsesNormalVariance") {
    float result = phase_velocity_variance(FlightPhase::FW, 0.01f, 0.04f);
    CHECK(result == doctest::Approx(0.01f));
}

TEST_CASE("Variance.TransitionFWUsesTransitionVariance") {
    float result = phase_velocity_variance(FlightPhase::TRANSITION_FW, 0.01f, 0.04f);
    CHECK(result == doctest::Approx(0.04f));
}

TEST_CASE("Variance.TransitionMCUsesTransitionVariance") {
    float result = phase_velocity_variance(FlightPhase::TRANSITION_MC, 0.01f, 0.04f);
    CHECK(result == doctest::Approx(0.04f));
}

// --- Sensor health tests ---

TEST_CASE("Health.FullHealthWhenAllReceived") {
    SensorHealth h;
    h.init(10);
    for (int i = 0; i < 10; ++i) {
        h.record(true);
    }
    CHECK(h.health_pct() == doctest::Approx(100.0f));
}

TEST_CASE("Health.ZeroHealthWhenNoneReceived") {
    SensorHealth h;
    h.init(10);
    for (int i = 0; i < 10; ++i) {
        h.record(false);
    }
    CHECK(h.health_pct() == doctest::Approx(0.0f));
}

TEST_CASE("Health.HalfHealthWhenHalfReceived") {
    SensorHealth h;
    h.init(10);
    for (int i = 0; i < 10; ++i) {
        h.record(i % 2 == 0);
    }
    CHECK(h.health_pct() == doctest::Approx(50.0f));
}

TEST_CASE("Health.WindowSlidesToRecentSamples") {
    SensorHealth h;
    h.init(10);

    // First 10: no messages received (bad period)
    for (int i = 0; i < 10; ++i) {
        h.record(false);
    }
    CHECK(h.health_pct() == doctest::Approx(0.0f));

    // Next 10: all received (good period) — overwrites old bad samples
    for (int i = 0; i < 10; ++i) {
        h.record(true);
    }
    // Window now contains only the 10 good samples
    CHECK(h.health_pct() == doctest::Approx(100.0f));
}

TEST_CASE("Health.DefaultIs100WhenNoSamples") {
    SensorHealth h;
    h.init(50);
    CHECK(h.health_pct() == doctest::Approx(100.0f));
}

TEST_CASE("Health.PartialWindowBeforeFull") {
    SensorHealth h;
    h.init(50);
    // Only 5 samples, all received
    for (int i = 0; i < 5; ++i) {
        h.record(true);
    }
    CHECK(h.health_pct() == doctest::Approx(100.0f));
}

TEST_CASE("Health.ReinitClearsState") {
    SensorHealth h;
    h.init(10);
    for (int i = 0; i < 10; ++i) {
        h.record(true);
    }
    h.init(10);
    CHECK(h.count == 0);
    CHECK(h.hits == 0);
    CHECK(h.health_pct() == doctest::Approx(100.0f));
}

// --- Health-based variance scaling tests ---

TEST_CASE("HealthScale.At100Percent") {
    // 100% health → scale = 1 / (1.0 * 1.0) = 1.0
    float scale = health_variance_scale(100.0f);
    CHECK(scale == doctest::Approx(1.0f));
}

TEST_CASE("HealthScale.At50Percent") {
    // 50% health → scale = 1 / (0.5 * 0.5) = 4.0
    float scale = health_variance_scale(50.0f);
    CHECK(scale == doctest::Approx(4.0f));
}

TEST_CASE("HealthScale.At10Percent") {
    // 10% health → scale = 1 / (0.1 * 0.1) = 100.0
    float scale = health_variance_scale(10.0f);
    CHECK(scale == doctest::Approx(100.0f));
}

TEST_CASE("HealthScale.AtZeroPercent") {
    // 0% health → clamped to 0.01, scale = 1 / (0.01 * 0.01) = 10000.0
    float scale = health_variance_scale(0.0f);
    CHECK(scale == doctest::Approx(10000.0f));
}

TEST_CASE("HealthScale.MonotonicallyIncreasing") {
    // Lower health → higher scale (worse quality → more variance)
    float s100 = health_variance_scale(100.0f);
    float s50 = health_variance_scale(50.0f);
    float s10 = health_variance_scale(10.0f);
    CHECK(s100 < s50);
    CHECK(s50 < s10);
}

// --- Attitude staleness tests ---

TEST_CASE("AttStaleness.FreshAttitude") {
    // < 200ms → 1x (normal)
    CHECK(attitude_staleness_scale(0.0) == doctest::Approx(1.0f));
    CHECK(attitude_staleness_scale(0.1) == doctest::Approx(1.0f));
    CHECK(attitude_staleness_scale(0.199) == doctest::Approx(1.0f));
}

TEST_CASE("AttStaleness.SlightlyStale") {
    // 200-500ms → 2x
    CHECK(attitude_staleness_scale(0.2) == doctest::Approx(2.0f));
    CHECK(attitude_staleness_scale(0.3) == doctest::Approx(2.0f));
    CHECK(attitude_staleness_scale(0.499) == doctest::Approx(2.0f));
}

TEST_CASE("AttStaleness.Stale") {
    // 500ms-1s → 4x
    CHECK(attitude_staleness_scale(0.5) == doctest::Approx(4.0f));
    CHECK(attitude_staleness_scale(0.7) == doctest::Approx(4.0f));
    CHECK(attitude_staleness_scale(0.999) == doctest::Approx(4.0f));
}

TEST_CASE("AttStaleness.VeryStaleSkipsPublish") {
    // > 1s → 0 (signal to skip publish entirely)
    CHECK(attitude_staleness_scale(1.0) == doctest::Approx(0.0f));
    CHECK(attitude_staleness_scale(2.0) == doctest::Approx(0.0f));
    CHECK(attitude_staleness_scale(10.0) == doctest::Approx(0.0f));
}

// --- Cross-validation tests ---

TEST_CASE("CrossVal.BothSlowNoScale") {
    // Both < 0.5 m/s → no comparison meaningful, return 1.0
    float scale = cross_validation_scale(0.3f, 0.4f);
    CHECK(scale == doctest::Approx(1.0f));
}

TEST_CASE("CrossVal.GoodAgreement") {
    // Spool=10, EKF=10 → 0% innovation → 1.0
    float scale = cross_validation_scale(10.0f, 10.0f);
    CHECK(scale == doctest::Approx(1.0f));
}

TEST_CASE("CrossVal.SmallDisagreement") {
    // Spool=10, EKF=8 → 20% innovation → < 30% threshold → 1.0
    float scale = cross_validation_scale(10.0f, 8.0f);
    CHECK(scale == doctest::Approx(1.0f));
}

TEST_CASE("CrossVal.LargeDisagreement") {
    // Spool=10, EKF=5 → 50% innovation → above 30% threshold
    float scale = cross_validation_scale(10.0f, 5.0f);
    CHECK(scale > 1.0f);
}

TEST_CASE("CrossVal.SevereDisagreement") {
    // Spool=10, EKF=1 → 90% innovation → very high scale
    float scale = cross_validation_scale(10.0f, 1.0f);
    CHECK(scale > 4.0f);
}

TEST_CASE("CrossVal.SymmetricWhenSpoolLower") {
    // EKF > spool should behave similarly
    float scale1 = cross_validation_scale(10.0f, 5.0f);
    float scale2 = cross_validation_scale(5.0f, 10.0f);
    CHECK(scale1 == doctest::Approx(scale2));
}

// --- Slack calibration tests ---

TEST_CASE("SlackCal.NoUpdateBelowSpeedThreshold") {
    // Both speeds must be > 1 m/s for calibration
    float slack = 1.05f;
    CHECK(slack_calibration_update(slack, 0.5f, 10.0f, 0.1f) == doctest::Approx(1.05f));
    CHECK(slack_calibration_update(slack, 10.0f, 0.5f, 0.1f) == doctest::Approx(1.05f));
    CHECK(slack_calibration_update(slack, 0.3f, 0.3f, 0.1f) == doctest::Approx(1.05f));
}

TEST_CASE("SlackCal.ConvergesToRatio") {
    // Spool=10.5, EKF=10.0 → ratio=1.05 → should converge to 1.05
    float slack = 1.0f;  // start at 1.0
    for (int i = 0; i < 200; ++i) {
        slack = slack_calibration_update(slack, 10.5f, 10.0f, 0.05f);
    }
    CHECK(slack == doctest::Approx(1.05f).epsilon(0.01));
}

TEST_CASE("SlackCal.ConvergesToLowerRatio") {
    // Spool=10.0, EKF=10.0 → ratio=1.0 → should converge to 1.0
    float slack = 1.05f;  // start at 1.05
    for (int i = 0; i < 200; ++i) {
        slack = slack_calibration_update(slack, 10.0f, 10.0f, 0.05f);
    }
    CHECK(slack == doctest::Approx(1.0f).epsilon(0.01));
}

TEST_CASE("SlackCal.ClampedToMin") {
    // Very low ratio → clamped to 0.8
    float slack = 0.85f;
    slack = slack_calibration_update(slack, 5.0f, 10.0f, 1.0f);  // ratio=0.5, alpha=1.0
    CHECK(slack == doctest::Approx(0.8f));
}

TEST_CASE("SlackCal.ClampedToMax") {
    // Very high ratio → clamped to 1.2
    float slack = 1.1f;
    slack = slack_calibration_update(slack, 20.0f, 10.0f, 1.0f);  // ratio=2.0, alpha=1.0
    CHECK(slack == doctest::Approx(1.2f));
}

TEST_CASE("SlackCal.EMASmoothing") {
    // Small alpha → slow convergence
    float slack = 1.0f;
    float after_one = slack_calibration_update(slack, 10.5f, 10.0f, 0.01f);
    // EMA: 1.0 + 0.01 * (1.05 - 1.0) = 1.0005
    CHECK(after_one == doctest::Approx(1.0005f).epsilon(0.0001));
}

// --- Heading cross-check tests ---

TEST_CASE("HeadingCheck.GoodAgreement") {
    // Headings within 0.1 rad → scale = 1.0
    CHECK(heading_crosscheck_scale(0.0f, 0.05f) == doctest::Approx(1.0f));
    CHECK(heading_crosscheck_scale(1.0f, 1.09f) == doctest::Approx(1.0f));
}

TEST_CASE("HeadingCheck.SmallDivergence") {
    // 0.3 rad diff → excess = (0.3 - 0.1) / 0.2 = 1.0 → 1 + 3*1 = 4.0
    float scale = heading_crosscheck_scale(0.0f, 0.3f);
    CHECK(scale == doctest::Approx(4.0f));
}

TEST_CASE("HeadingCheck.LargeDivergence") {
    // 0.5 rad diff → excess = (0.5 - 0.1) / 0.2 = 2.0 → 1 + 3*4 = 13.0
    float scale = heading_crosscheck_scale(0.0f, 0.5f);
    CHECK(scale == doctest::Approx(13.0f));
}

TEST_CASE("HeadingCheck.WrapsAround") {
    // Headings near ±π should measure short way around
    // 3.0 rad and -3.0 rad → actual diff ≈ 0.28 rad (short way: 2π - 6.0 ≈ 0.28)
    float scale = heading_crosscheck_scale(3.0f, -3.0f);
    // diff = 6.0 → wrapped = 6.0 - 2π ≈ -0.283 → abs = 0.283
    // excess = (0.283 - 0.1) / 0.2 = 0.915 → 1 + 3*0.838 ≈ 3.51
    CHECK(scale > 1.0f);
    CHECK(scale < 5.0f);
}

TEST_CASE("HeadingCheck.OppositeHeadings") {
    // π rad diff → excess = (π - 0.1) / 0.2 ≈ 15.2 → very large scale
    float scale = heading_crosscheck_scale(0.0f, static_cast<float>(M_PI));
    CHECK(scale > 50.0f);
}

TEST_CASE("HeadingCheck.IdenticalHeadings") {
    CHECK(heading_crosscheck_scale(1.5f, 1.5f) == doctest::Approx(1.0f));
}

// --- Adaptive ZUPT tests ---

TEST_CASE("AdaptiveZUPT.MinimumThreshold") {
    // noise_rms = 0 → threshold = max(0.01, 0) = 0.01
    CHECK(adaptive_zupt_threshold(0.0f) == doctest::Approx(0.01f));
}

TEST_CASE("AdaptiveZUPT.ScalesWithNoise") {
    // noise_rms = 0.02 → threshold = max(0.01, 0.06) = 0.06
    CHECK(adaptive_zupt_threshold(0.02f) == doctest::Approx(0.06f));
}

TEST_CASE("AdaptiveZUPT.HighNoiseHighThreshold") {
    // noise_rms = 0.1 → threshold = max(0.01, 0.3) = 0.3
    CHECK(adaptive_zupt_threshold(0.1f) == doctest::Approx(0.3f));
}

TEST_CASE("AdaptiveZUPT.ThreeTimesNoise") {
    // Verify the 3× relationship
    float rms = 0.05f;
    float thresh = adaptive_zupt_threshold(rms);
    CHECK(thresh == doctest::Approx(3.0f * rms));
}

TEST_CASE("AdaptiveZUPT.MinFloorApplied") {
    // Very small noise should still give minimum threshold
    CHECK(adaptive_zupt_threshold(0.001f) == doctest::Approx(0.01f));
    CHECK(adaptive_zupt_threshold(0.003f) == doctest::Approx(0.01f));
}

// --- Noise RMS computation tests ---

TEST_CASE("NoiseRMS.EmptyReturnsZero") {
    std::vector<float> samples;
    CHECK(compute_noise_rms(samples) == doctest::Approx(0.0f));
}

TEST_CASE("NoiseRMS.ConstantValue") {
    // All same value → RMS = that value
    std::vector<float> samples(10, 0.03f);
    CHECK(compute_noise_rms(samples) == doctest::Approx(0.03f));
}

TEST_CASE("NoiseRMS.MixedValues") {
    // RMS of {0.01, 0.02, 0.03} = sqrt((0.0001 + 0.0004 + 0.0009) / 3)
    std::vector<float> samples = {0.01f, 0.02f, 0.03f};
    float expected = std::sqrt((0.0001f + 0.0004f + 0.0009f) / 3.f);
    CHECK(compute_noise_rms(samples) == doctest::Approx(expected));
}
