#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace fiber_nav_fusion {

/// 6-state 2D position EKF for GPS-denied dead reckoning.
/// State: [x, y, vx, vy, wind_x, wind_y] in NED frame.
/// Prediction: pos += (v_measured - wind) * dt
/// Measurements: velocity (adaptive R from OF quality), speed consistency,
///               cable length inequality constraint.
struct PositionEkfState {
    Eigen::Vector<float, 6> x = Eigen::Vector<float, 6>::Zero();
    Eigen::Matrix<float, 6, 6> P = Eigen::Matrix<float, 6, 6>::Identity();
    bool initialized = false;
};

struct PositionEkfConfig {
    // Process noise (per second)
    float q_pos = 0.01f;
    float q_vel = 0.5f;
    float q_wind = 0.01f;

    // Measurement noise base values
    float r_velocity = 0.5f;
    float r_speed = 0.1f;

    // OF quality → measurement noise scaling
    float of_quality_min = 0.3f;
    float of_quality_scale = 5.0f;

    // Cable constraint
    float cable_margin = 0.95f;

    // Initial uncertainty
    float p0_pos = 1.0f;
    float p0_vel = 1.0f;
    float p0_wind = 2.0f;
};

/// Initialize EKF state at (0,0) with zero velocity and zero wind.
PositionEkfState initialize(const PositionEkfConfig& config);

/// Initialize EKF at a known position (e.g. last GPS fix).
PositionEkfState initializeAt(const PositionEkfConfig& config, float x, float y);

/// Predict step: propagate state forward by dt.
/// v_measured = spool_speed * R(attitude) * OF_direction (NED, computed externally).
/// pos += (v_measured - wind) * dt; velocity set to measurement; wind = random walk.
PositionEkfState predict(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float dt,
    float vn_measured,
    float ve_measured);

/// Velocity measurement update with adaptive noise based on OF quality.
PositionEkfState updateVelocity(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float vn_measured,
    float ve_measured,
    float of_quality);

/// Speed consistency: scalar ||v_state|| should match spool_speed.
PositionEkfState updateSpeedConsistency(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float spool_speed);

/// Cable inequality constraint: ||pos|| <= cable_deployed_length.
/// Applied as pseudo-measurement when position exceeds margin * cable_length.
PositionEkfState applyCableConstraint(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float cable_deployed_length);

/// Position measurement update (from TERCOM fix or GPS).
/// Standard 2D position Kalman update: H = [1 0 0 0 0 0; 0 1 0 0 0 0]
PositionEkfState updatePosition(
    const PositionEkfState& state,
    float x_measured, float y_measured,
    float variance);

/// Position update with full 2x2 covariance matrix (anisotropic TERCOM).
/// Allows direction-dependent uncertainty: var_xx (North), var_yy (East), var_xy (cross).
PositionEkfState updatePosition(
    const PositionEkfState& state,
    float x_measured, float y_measured,
    float var_xx, float var_yy, float var_xy);

/// Reset position to known values (e.g. GPS re-acquired).
PositionEkfState resetPosition(
    const PositionEkfState& state,
    float x, float y, float variance);

// --- Inline accessors ---

inline std::array<float, 2> position(const PositionEkfState& s) {
    return {s.x(0), s.x(1)};
}

inline std::array<float, 2> velocity(const PositionEkfState& s) {
    return {s.x(2), s.x(3)};
}

inline std::array<float, 2> wind(const PositionEkfState& s) {
    return {s.x(4), s.x(5)};
}

inline std::array<float, 2> positionSigma(const PositionEkfState& s) {
    return {std::sqrt(s.P(0, 0)), std::sqrt(s.P(1, 1))};
}

inline float distanceFromHome(const PositionEkfState& s) {
    return std::sqrt(s.x(0) * s.x(0) + s.x(1) * s.x(1));
}

}  // namespace fiber_nav_fusion
