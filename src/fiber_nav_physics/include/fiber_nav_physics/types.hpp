#pragma once

#include <cmath>
#include <cstdint>

namespace fiber_nav_physics
{

/// Simple 3D vector — engine-agnostic replacement for AZ::Vector3
struct Vec3
{
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x, float y, float z) : x{x}, y{y}, z{z} {}

    static constexpr Vec3 zero() { return {0.0f, 0.0f, 0.0f}; }

    constexpr Vec3 operator+(Vec3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(Vec3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(Vec3 rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& operator-=(Vec3 rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    constexpr float dot(Vec3 rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    constexpr Vec3 cross(Vec3 rhs) const
    {
        return {y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float length_sq() const { return x * x + y * y + z * z; }

    Vec3 normalized() const
    {
        float len = length();
        return (len > 1e-9f) ? *this / len : zero();
    }
};

constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }

/// Quaternion (w, x, y, z) — Hamilton convention
struct Quat
{
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quat() = default;
    constexpr Quat(float w, float x, float y, float z) : w{w}, x{x}, y{y}, z{z} {}

    static constexpr Quat identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }

    /// Rotate a vector by this quaternion
    Vec3 rotate(Vec3 v) const
    {
        Vec3 u{x, y, z};
        float s = w;
        return 2.0f * u.dot(v) * u + (s * s - u.dot(u)) * v + 2.0f * s * u.cross(v);
    }

    /// Inverse rotation (conjugate, assumes unit quaternion)
    Vec3 inverse_rotate(Vec3 v) const
    {
        return conjugate().rotate(v);
    }

    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat normalized() const
    {
        float len = std::sqrt(w * w + x * x + y * y + z * z);
        return (len > 1e-9f) ? Quat{w / len, x / len, y / len, z / len} : identity();
    }

    /// Quaternion multiplication: this * rhs
    constexpr Quat operator*(Quat rhs) const
    {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
        };
    }

    /// Create from axis-angle (axis must be unit vector)
    static Quat from_axis_angle(Vec3 axis, float angle)
    {
        float half = angle * 0.5f;
        float s = std::sin(half);
        return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
    }

    /// Euler angles to quaternion (ZYX intrinsic = yaw-pitch-roll)
    static Quat from_euler(float roll, float pitch, float yaw)
    {
        float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        };
    }
};

// ── MAVLink-facing data structs (match O3DE px4_bridge types) ──

/// Sensor data for MAVLink HIL_SENSOR message
struct SensorData
{
    Vec3 accel{0.0f, 0.0f, -9.81f}; // m/s^2, body frame (NED)
    Vec3 gyro{};                      // rad/s, body frame
    Vec3 mag{};                       // gauss, body frame
    float abs_pressure{1013.25f};     // mbar
    float diff_pressure{0.0f};        // mbar (airspeed)
    float pressure_alt{0.0f};         // m
    float temperature{25.0f};         // degC
    uint32_t fields_updated{0x1FFF};
    uint8_t id{0};
};

/// GPS data for MAVLink HIL_GPS message
struct GPSData
{
    int32_t lat{0};  // degE7
    int32_t lon{0};  // degE7
    int32_t alt{0};  // mm MSL
    int16_t vn{0};   // cm/s NED
    int16_t ve{0};
    int16_t vd{0};
    uint16_t vel{0}; // cm/s ground speed
    uint16_t cog{0}; // cdeg course over ground
    uint8_t fix_type{3};
    uint8_t satellites_visible{12};
    uint16_t eph{20}; // cm
    uint16_t epv{40}; // cm
};

/// Motor outputs from PX4 via MAVLink HIL_ACTUATOR_CONTROLS
struct MotorOutputs
{
    static constexpr int max_motors = 16;
    float controls[max_motors]{};
    uint8_t mode{0};
    uint64_t flags{0};
};

// ── Aerodynamics configuration ──

/// Aerodynamic coefficient set — matches O3DE AeroCoefficients
struct AeroCoefficients
{
    float a0{0.0f};            // zero-lift AoA

    float CL0{0.0f};          // lift at zero AoA
    float CLa{0.0f};          // lift curve slope
    float CLa_stall{0.0f};    // post-stall lift slope

    float CD0{0.0f};          // parasitic drag
    float cda{0.0f};          // induced drag factor (unused, computed from AR)
    float CDa_stall{0.0f};    // post-stall drag slope

    float Cem0{0.0f};         // pitch moment at zero AoA
    float Cema{0.0f};         // pitch moment slope
    float Cema_stall{0.0f};   // post-stall pitch moment slope

    float CYb{0.0f};          // side force / sideslip
    float Cellb{0.0f};        // roll moment / sideslip
    float Cenb{0.0f};         // yaw moment / sideslip

    // Roll rate (p) derivatives
    float CDp{0.0f}, CYp{0.0f}, CLp{0.0f}, Cellp{0.0f}, Cemp{0.0f}, Cenp{0.0f};
    // Pitch rate (q) derivatives
    float CDq{0.0f}, CYq{0.0f}, CLq{0.0f}, Cellq{0.0f}, Cemq{0.0f}, Cenq{0.0f};
    // Yaw rate (r) derivatives
    float CDr{0.0f}, CYr{0.0f}, CLr{0.0f}, Cellr{0.0f}, Cemr{0.0f}, Cenr{0.0f};

    float alpha_stall{0.3f};  // rad
};

/// Wing geometry
struct WingGeometry
{
    float aspect_ratio{1.0f};
    float efficiency{0.97f};
    float area{0.1f};          // m^2
    float mac{0.1f};           // mean aerodynamic chord (m)
    float air_density{1.225f}; // kg/m^3
    Vec3 ref_pt{};             // reference point offset
    Vec3 forward{0.0f, 0.0f, 1.0f};  // body forward direction (tailsitter: +Z)
    Vec3 upward{-1.0f, 0.0f, 0.0f};  // body upward direction (tailsitter: -X)
};

// ── Motor configuration ──

/// Per-motor parameters
struct MotorConfig
{
    Vec3 position{};                   // relative to CoM (m)
    float motor_constant{8.54858e-06f}; // thrust = k * omega^2
    float moment_constant{0.016f};     // torque / thrust ratio
    float max_rot_velocity{1200.0f};   // rad/s
    float time_constant_up{0.0125f};   // s (spin-up)
    float time_constant_down{0.025f};  // s (spin-down)
    float rotor_drag_coeff{8.06428e-05f};
    float rolling_moment_coeff{1e-06f};
    int direction{1};                  // +1 = CCW, -1 = CW
};

/// Runtime motor state (not config)
struct MotorState
{
    float commanded_velocity{0.0f};
    float current_velocity{0.0f};
};

// ── VTOL state ──

enum class VTOLFlightMode : uint8_t
{
    Multicopter,
    TransitionToFW,
    FixedWing,
    TransitionToMC,
};

/// Aerodynamic state output (telemetry/debug)
struct AeroState
{
    float airspeed{0.0f};
    float angle_of_attack{0.0f};
    float sideslip_angle{0.0f};
    Vec3 lift_force{};
    Vec3 drag_force{};
    Vec3 aero_moment{};
};

// ── Vehicle state for dynamics integration ──

/// Full rigid body state in NED frame
struct VehicleState
{
    Vec3 position{};       // NED (m)
    Vec3 velocity{};       // NED (m/s)
    Quat orientation{};    // body → NED
    Vec3 angular_velocity{}; // body frame (rad/s)
};

/// Forces and torques acting on the vehicle (body frame)
struct ForcesTorques
{
    Vec3 force{};   // body frame (N)
    Vec3 torque{};  // body frame (N*m)
};

// ── Sensor synthesis configuration ──

struct SensorNoiseConfig
{
    float gyro_noise_stddev{0.0003394f};
    float accel_noise_stddev{0.004f};
    float baro_noise_stddev{0.01f};      // mbar
    float mag_noise_stddev{0.0001f};     // gauss
    float sea_level_pressure{1013.25f};  // mbar
    float sea_level_temp{288.15f};       // K
    Vec3 world_mag_field{0.21f, 0.05f, -0.42f}; // gauss
};

struct GPSConfig
{
    double origin_lat{31.164093};   // degrees
    double origin_lon{34.532227};   // degrees
    float origin_alt{141.0f};       // m MSL
    uint16_t eph{20};               // cm
    uint16_t epv{40};               // cm
    uint8_t sat_count{12};
    bool enabled{true};
};

} // namespace fiber_nav_physics
