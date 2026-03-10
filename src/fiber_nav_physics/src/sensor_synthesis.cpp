#include "fiber_nav_physics/sensor_synthesis.hpp"

#include <cmath>
#include <numbers>
#include <random>

namespace fiber_nav_physics
{

namespace
{
    thread_local std::mt19937 rng{std::random_device{}()};
} // namespace

float gaussian_noise(float stddev)
{
    if (stddev <= 0.0f)
    {
        return 0.0f;
    }
    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(rng);
}

SensorData synthesize_sensors(
    const VehicleState& state,
    Vec3 prev_vel,
    float dt,
    const SensorNoiseConfig& noise,
    const GPSConfig& gps_cfg)
{
    SensorData data{};
    Quat rot_inv = state.orientation.conjugate();

    // Gyroscope: body-frame angular velocity + noise
    Vec3 body_ang_vel = rot_inv.rotate(state.angular_velocity);
    data.gyro = {
        body_ang_vel.x + gaussian_noise(noise.gyro_noise_stddev),
        body_ang_vel.y + gaussian_noise(noise.gyro_noise_stddev),
        body_ang_vel.z + gaussian_noise(noise.gyro_noise_stddev),
    };

    // Accelerometer: body-frame specific force (accel + gravity) + noise
    if (dt > 0.0f)
    {
        Vec3 world_accel = (state.velocity - prev_vel) / dt;
        // Accelerometer measures gravity as upward when stationary
        // NED: gravity is (0, 0, +9.81)
        world_accel -= Vec3{0.0f, 0.0f, -9.81f};
        Vec3 body_accel = rot_inv.rotate(world_accel);
        data.accel = {
            body_accel.x + gaussian_noise(noise.accel_noise_stddev),
            body_accel.y + gaussian_noise(noise.accel_noise_stddev),
            body_accel.z + gaussian_noise(noise.accel_noise_stddev),
        };
    }

    // Magnetometer: world field rotated to body frame + noise
    Vec3 body_mag = rot_inv.rotate(noise.world_mag_field);
    data.mag = {
        body_mag.x + gaussian_noise(noise.mag_noise_stddev),
        body_mag.y + gaussian_noise(noise.mag_noise_stddev),
        body_mag.z + gaussian_noise(noise.mag_noise_stddev),
    };

    // Barometer: ISA atmosphere model
    // NED: position.z is down, altitude = -position.z + origin_alt
    float altitude = -state.position.z + gps_cfg.origin_alt;
    constexpr float L = 0.0065f;       // lapse rate K/m
    constexpr float g = 9.80665f;
    constexpr float M = 0.0289644f;    // molar mass dry air kg/mol
    constexpr float R = 8.31447f;      // gas constant
    float T0 = noise.sea_level_temp;
    float P0 = noise.sea_level_pressure;
    float exponent = g * M / (R * L);
    float pressure = P0 * std::pow(1.0f - L * altitude / T0, exponent);
    data.abs_pressure = pressure + gaussian_noise(noise.baro_noise_stddev);
    data.pressure_alt = altitude;
    data.temperature = T0 - L * altitude - 273.15f;
    data.fields_updated = 0x1FFF;

    return data;
}

GPSData synthesize_gps(
    const VehicleState& state,
    const GPSConfig& gps_cfg)
{
    GPSData gps{};

    // NED position to lat/lon
    constexpr double meters_per_deg_lat = 111320.0;
    double cos_lat = std::cos(gps_cfg.origin_lat * std::numbers::pi / 180.0);
    double meters_per_deg_lon = meters_per_deg_lat * cos_lat;

    // NED: x=North, y=East, z=Down
    double lat = gps_cfg.origin_lat + static_cast<double>(state.position.x) / meters_per_deg_lat;
    double lon = gps_cfg.origin_lon + static_cast<double>(state.position.y) / meters_per_deg_lon;
    double alt = gps_cfg.origin_alt - static_cast<double>(state.position.z); // -z = up

    gps.lat = static_cast<int32_t>(lat * 1e7);
    gps.lon = static_cast<int32_t>(lon * 1e7);
    gps.alt = static_cast<int32_t>(alt * 1000.0); // mm

    // Velocity NED (already NED)
    gps.vn = static_cast<int16_t>(state.velocity.x * 100.0f);
    gps.ve = static_cast<int16_t>(state.velocity.y * 100.0f);
    gps.vd = static_cast<int16_t>(state.velocity.z * 100.0f);

    float ground_speed = std::sqrt(
        state.velocity.x * state.velocity.x +
        state.velocity.y * state.velocity.y);
    gps.vel = static_cast<uint16_t>(ground_speed * 100.0f);

    // Course over ground (NED: atan2(east, north))
    float cog = std::atan2(state.velocity.y, state.velocity.x)
              * 180.0f / std::numbers::pi_v<float>;
    if (cog < 0.0f)
    {
        cog += 360.0f;
    }
    gps.cog = static_cast<uint16_t>(cog * 100.0f);

    gps.fix_type = 3;
    gps.satellites_visible = gps_cfg.sat_count;
    gps.eph = gps_cfg.eph;
    gps.epv = gps_cfg.epv;

    return gps;
}

} // namespace fiber_nav_physics
