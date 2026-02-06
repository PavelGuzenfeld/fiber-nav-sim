#pragma once

#include <cmath>
#include <numbers>

namespace fiber_nav_sensors {

struct Vec3 { double x, y, z; };
struct Quat { double w, x, y, z; };

inline Vec3 quaternion_to_euler(Quat const& q) {
    // ZYX convention (roll, pitch, yaw)
    double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr, cosr);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = (std::abs(sinp) >= 1.0)
        ? std::copysign(std::numbers::pi / 2.0, sinp)
        : std::asin(sinp);

    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny, cosy);

    return {roll, pitch, yaw};
}

inline Vec3 rotate_vector(Quat const& q, Vec3 const& v) {
    // q * v * q^-1 via cross-product form
    Vec3 u{q.x, q.y, q.z};
    double s = q.w;
    // t = 2 * (u x v)
    Vec3 t{2.0 * (u.y * v.z - u.z * v.y),
            2.0 * (u.z * v.x - u.x * v.z),
            2.0 * (u.x * v.y - u.y * v.x)};
    return {v.x + s * t.x + (u.y * t.z - u.z * t.y),
            v.y + s * t.y + (u.z * t.x - u.x * t.z),
            v.z + s * t.z + (u.x * t.y - u.y * t.x)};
}

inline double wrap_angle(double a) {
    while (a > std::numbers::pi) a -= 2.0 * std::numbers::pi;
    while (a < -std::numbers::pi) a += 2.0 * std::numbers::pi;
    return a;
}

inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

}  // namespace fiber_nav_sensors
