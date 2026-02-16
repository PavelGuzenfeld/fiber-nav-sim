#include <fiber_nav_fusion/position_ekf.hpp>

#include <algorithm>
#include <cmath>

namespace fiber_nav_fusion {

PositionEkfState initialize(const PositionEkfConfig& config) {
    PositionEkfState s;
    s.x.setZero();
    s.P.setZero();
    s.P(0, 0) = config.p0_pos;
    s.P(1, 1) = config.p0_pos;
    s.P(2, 2) = config.p0_vel;
    s.P(3, 3) = config.p0_vel;
    s.P(4, 4) = config.p0_wind;
    s.P(5, 5) = config.p0_wind;
    s.initialized = true;
    return s;
}

PositionEkfState initializeAt(const PositionEkfConfig& config, float x, float y) {
    auto s = initialize(config);
    s.x(0) = x;
    s.x(1) = y;
    return s;
}

PositionEkfState predict(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float dt,
    float vn_measured,
    float ve_measured)
{
    if (dt <= 0.f) return state;

    PositionEkfState out = state;

    // State transition: pos += (v_measured - wind) * dt
    // Velocity state tracks measured velocity
    // Wind state is random walk
    float wind_n = state.x(4);
    float wind_e = state.x(5);

    out.x(0) = state.x(0) + (vn_measured - wind_n) * dt;  // x += (vn - wn) * dt
    out.x(1) = state.x(1) + (ve_measured - wind_e) * dt;  // y += (ve - we) * dt
    out.x(2) = vn_measured;                                 // vx = measurement
    out.x(3) = ve_measured;                                 // vy = measurement
    // wind unchanged (random walk via Q)

    // State transition matrix F
    Eigen::Matrix<float, 6, 6> F = Eigen::Matrix<float, 6, 6>::Identity();
    // d(pos)/d(wind) = -dt
    F(0, 4) = -dt;
    F(1, 5) = -dt;

    // Process noise Q
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
    Q(0, 0) = config.q_pos * dt;
    Q(1, 1) = config.q_pos * dt;
    Q(2, 2) = config.q_vel * dt;
    Q(3, 3) = config.q_vel * dt;
    Q(4, 4) = config.q_wind * dt;
    Q(5, 5) = config.q_wind * dt;

    out.P = F * state.P * F.transpose() + Q;

    return out;
}

PositionEkfState updateVelocity(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float vn_measured,
    float ve_measured,
    float of_quality)
{
    // H = [0 0 1 0 0 0; 0 0 0 1 0 0] — observe vx, vy
    Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
    H(0, 2) = 1.f;
    H(1, 3) = 1.f;

    // Measurement
    Eigen::Vector2f z{vn_measured, ve_measured};

    // Adaptive R: scale by OF quality
    float quality_clamped = std::clamp(of_quality, config.of_quality_min, 1.0f);
    float quality_range = 1.0f - config.of_quality_min;
    float quality_factor = (quality_range > 1e-6f)
        ? config.of_quality_scale - (config.of_quality_scale - 1.0f) *
          (quality_clamped - config.of_quality_min) / quality_range
        : 1.0f;

    // Clamp quality below min to max scale
    if (of_quality < config.of_quality_min) {
        quality_factor = config.of_quality_scale;
    }

    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * config.r_velocity * quality_factor;

    // Innovation
    Eigen::Vector2f y = z - H * state.x;

    // Innovation covariance
    Eigen::Matrix2f S = H * state.P * H.transpose() + R;

    // Kalman gain
    Eigen::Matrix<float, 6, 2> K = state.P * H.transpose() * S.inverse();

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

PositionEkfState updateSpeedConsistency(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float spool_speed)
{
    float vx = state.x(2);
    float vy = state.x(3);
    float speed_state = std::sqrt(vx * vx + vy * vy);

    // Avoid division by zero
    if (speed_state < 0.1f) return state;

    // H = d(||v||)/d(state) = [0, 0, vx/||v||, vy/||v||, 0, 0]
    Eigen::Matrix<float, 1, 6> H = Eigen::Matrix<float, 1, 6>::Zero();
    H(0, 2) = vx / speed_state;
    H(0, 3) = vy / speed_state;

    float z = spool_speed;
    float y = z - speed_state;
    float R = config.r_speed;

    float S = (H * state.P * H.transpose())(0, 0) + R;
    Eigen::Vector<float, 6> K = state.P * H.transpose() / S;

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

PositionEkfState applyCableConstraint(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float cable_deployed_length)
{
    if (cable_deployed_length <= 0.f) return state;

    float px = state.x(0);
    float py = state.x(1);
    float dist = std::sqrt(px * px + py * py);

    // Only apply when position exceeds margin * cable_length
    if (dist <= config.cable_margin * cable_deployed_length) {
        return state;
    }

    // Avoid division by zero
    if (dist < 1e-6f) return state;

    // Pseudo-measurement: ||pos|| = cable_length
    // H = d(||pos||)/d(state) = [px/dist, py/dist, 0, 0, 0, 0]
    Eigen::Matrix<float, 1, 6> H = Eigen::Matrix<float, 1, 6>::Zero();
    H(0, 0) = px / dist;
    H(0, 1) = py / dist;

    float z = cable_deployed_length;
    float y = z - dist;
    float R = (cable_deployed_length * 0.05f) * (cable_deployed_length * 0.05f);

    float S = (H * state.P * H.transpose())(0, 0) + R;
    Eigen::Vector<float, 6> K = state.P * H.transpose() / S;

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

PositionEkfState updatePosition(
    const PositionEkfState& state,
    float x_measured, float y_measured,
    float variance)
{
    // H = [1 0 0 0 0 0; 0 1 0 0 0 0] — observe x, y
    Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
    H(0, 0) = 1.f;
    H(1, 1) = 1.f;

    // Measurement
    Eigen::Vector2f z{x_measured, y_measured};

    // Measurement noise
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * variance;

    // Innovation
    Eigen::Vector2f y = z - H * state.x;

    // Innovation covariance
    Eigen::Matrix2f S = H * state.P * H.transpose() + R;

    // Kalman gain
    Eigen::Matrix<float, 6, 2> K = state.P * H.transpose() * S.inverse();

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

PositionEkfState updatePosition(
    const PositionEkfState& state,
    float x_measured, float y_measured,
    float var_xx, float var_yy, float var_xy)
{
    // H = [1 0 0 0 0 0; 0 1 0 0 0 0] — observe x, y
    Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
    H(0, 0) = 1.f;
    H(1, 1) = 1.f;

    // Measurement
    Eigen::Vector2f z{x_measured, y_measured};

    // Anisotropic measurement noise
    Eigen::Matrix2f R;
    R << var_xx, var_xy,
         var_xy, var_yy;

    // Innovation
    Eigen::Vector2f y = z - H * state.x;

    // Innovation covariance
    Eigen::Matrix2f S = H * state.P * H.transpose() + R;

    // Kalman gain
    Eigen::Matrix<float, 6, 2> K = state.P * H.transpose() * S.inverse();

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

PositionEkfState resetPosition(
    const PositionEkfState& state,
    float x, float y, float variance)
{
    PositionEkfState out = state;
    out.x(0) = x;
    out.x(1) = y;
    out.P(0, 0) = variance;
    out.P(1, 1) = variance;
    // Clear position-velocity cross-correlations
    out.P(0, 2) = 0.f; out.P(0, 3) = 0.f;
    out.P(1, 2) = 0.f; out.P(1, 3) = 0.f;
    out.P(2, 0) = 0.f; out.P(3, 0) = 0.f;
    out.P(2, 1) = 0.f; out.P(3, 1) = 0.f;
    return out;
}

PositionEkfState updateCrossTrackPrior(
    const PositionEkfState& state,
    float cross_x, float cross_y,
    float cross_track_distance,
    float discriminability,
    float r_min, float r_max)
{
    // H = cross-track unit vector: observes the cross-track component of position
    // H = [cross_x, cross_y, 0, 0, 0, 0]
    Eigen::Matrix<float, 1, 6> H = Eigen::Matrix<float, 1, 6>::Zero();
    H(0, 0) = cross_x;
    H(0, 1) = cross_y;

    // Innovation: pull toward path (cross_track_distance = 0 on path)
    float z = 0.f;  // measurement: cross-track should be zero
    float y = z - cross_track_distance;  // = -cross_track_distance

    // R scales with discriminability: low disc → small R (strong constraint),
    // high disc → large R (let TERCOM handle it)
    float R = r_min + discriminability * (r_max - r_min);

    float S = (H * state.P * H.transpose())(0, 0) + R;
    Eigen::Vector<float, 6> K = state.P * H.transpose() / S;

    PositionEkfState out = state;
    out.x = state.x + K * y;
    out.P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * state.P;

    return out;
}

}  // namespace fiber_nav_fusion
