#pragma once

#include <algorithm>
#include <cmath>

namespace fiber_nav_sensors {

/// Single-axis gimbal filter: low-pass + rate limiter.
struct AxisFilter {
    float filtered = 0.f;
    float prev = 0.f;

    float update(float target, float dt, float tau, float max_rate) {
        if (dt > 0.f) {
            float alpha = dt / (tau + dt);
            filtered += alpha * (target - filtered);

            float max_delta = max_rate * dt;
            float delta = filtered - prev;
            if (std::abs(delta) > max_delta) {
                filtered = prev + std::copysign(max_delta, delta);
            }
        } else {
            filtered = target;
        }
        prev = filtered;
        return filtered;
    }
};

/// Hysteresis gate for gimbal activation.
/// Requires gx > enable_threshold for `required_count` consecutive samples
/// to activate. Deactivates when gx < disable_threshold.
struct ActivationGate {
    bool active = false;
    int consecutive_count = 0;

    bool check(float gx, float enable_thresh, float disable_thresh,
               int required_count) {
        if (active) {
            if (gx < disable_thresh) {
                active = false;
                consecutive_count = 0;
            }
        } else {
            if (gx > enable_thresh) {
                ++consecutive_count;
                if (consecutive_count >= required_count) {
                    active = true;
                }
            } else {
                consecutive_count = 0;
            }
        }
        return active;
    }
};

/// Gravity vector in SDF body frame.
struct GravityVector {
    float gx, gy, gz;
};

/// Gimbal yaw and pitch targets [rad].
struct GimbalTargets {
    float yaw, pitch;
};

/// Compute gravity vector in SDF body frame from body→world quaternion.
///
/// Gazebo ENU: gravity = [0, 0, -1] in world frame.
/// Odometry quaternion = body→world (R_bw).
/// g_body = R_bw^T * [0,0,-1] = -(row 3 of R_bw).
///
/// SDF tailsitter body: Z=nose, X=belly(down when sitting).
/// In hover (nose up): body Z≈up, body X≈horizontal → gx≈0
/// In FW flight: body Z≈forward, body X≈down → gx≈1
inline GravityVector gravityInBody(float qw, float qx, float qy, float qz)
{
    return {
        .gx = 2.f * (qw * qy - qx * qz),
        .gy = -2.f * (qy * qz + qw * qx),
        .gz = 2.f * (qx * qx + qy * qy) - 1.f,
    };
}

/// Compute gimbal yaw/pitch targets from gravity vector.
///
/// Yaw gimbal axis = SDF Z (nose). Camera points along SDF X.
/// To keep camera nadir, yaw correction = -atan2(gy, gx).
///
/// Pitch gimbal axis = SDF Y (lateral). Camera points along SDF X.
/// Pitch correction = -atan2(gz, gx): compensates nose-up/down tilt.
///
/// @param active  Whether the activation gate is open (from ActivationGate).
///                When false, returns zero commands.
inline GimbalTargets gimbalTargets(const GravityVector& g, bool active,
                                   float yaw_gain, float yaw_max_angle,
                                   float pitch_gain, float pitch_max_angle)
{
    if (!active) {
        return {0.f, 0.f};
    }
    return {
        .yaw = std::clamp(-yaw_gain * std::atan2(g.gy, g.gx),
                          -yaw_max_angle, yaw_max_angle),
        .pitch = std::clamp(-pitch_gain * std::atan2(g.gz, g.gx),
                            -pitch_max_angle, pitch_max_angle),
    };
}

/// Compute gimbal saturation ratio [0..1] from command and max angle.
inline float saturationRatio(float cmd, float max_angle)
{
    if (max_angle <= 0.f) {
        return 0.f;
    }
    return std::clamp(std::abs(cmd) / max_angle, 0.f, 1.f);
}

}  // namespace fiber_nav_sensors
