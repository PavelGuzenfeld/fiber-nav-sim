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

/// Compute gimbal targets for nadir tracking from tail-mount rest position.
///
/// Tail mount with 90deg static offset on gimbal_camera_link: at rest,
/// sensor looks along -Z_body (nadir in hover). Gimbal corrects as body tilts.
///
/// Derivation: sensor direction in body frame after gimbal actuation:
///   look = R_z(a) * R_y(b) * [0,0,-1] = [-sinb*cosa, -sinb*sina, -cosb]
/// Setting equal to gravity [gx, gy, gz]:
///   a (roll)  = atan2(gy, gx)
///   b (pitch) = -atan2(sqrt(gx^2+gy^2), -gz)
///
/// In hover (g=[0,0,-1]): roll=0, pitch=0 -> sensor already at nadir.
/// In FW    (g=[1,0,0]):  roll=0, pitch=-pi/2 -> sensor rotates to nadir.
///
/// Guard: when gx^2+gy^2 < 0.01 (near hover), roll is set to 0
/// to avoid atan2(0,0) noise.
inline GimbalTargets gimbalTargetsNadir(const GravityVector& g,
                                         float roll_gain, float roll_max,
                                         float pitch_gain, float pitch_max)
{
    float gxy_sq = g.gx * g.gx + g.gy * g.gy;
    float roll_target = (gxy_sq > 0.01f)
        ? std::atan2(g.gy, g.gx)
        : 0.f;
    float pitch_target = -std::atan2(std::sqrt(gxy_sq), -g.gz);

    return {
        .yaw = std::clamp(roll_gain * roll_target, -roll_max, roll_max),
        .pitch = std::clamp(pitch_gain * pitch_target, -pitch_max, pitch_max),
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
