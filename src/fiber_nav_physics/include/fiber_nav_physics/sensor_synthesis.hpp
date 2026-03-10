#pragma once

#include "fiber_nav_physics/types.hpp"

namespace fiber_nav_physics
{

/// Synthesize sensor data from vehicle state.
/// Pure math — no engine dependency.
/// Extracted from O3DE SensorCollectorComponent.

/// Synthesize IMU + barometer + magnetometer for HIL_SENSOR.
/// @param state     current vehicle state (NED frame)
/// @param prev_vel  previous NED velocity (for accel computation)
/// @param dt        time step (s)
/// @param noise     noise configuration
/// @param gps_cfg   GPS config (for origin altitude in baro computation)
/// @return SensorData ready for HIL_SENSOR message
SensorData synthesize_sensors(
    const VehicleState& state,
    Vec3 prev_vel,
    float dt,
    const SensorNoiseConfig& noise,
    const GPSConfig& gps_cfg);

/// Synthesize GPS data for HIL_GPS.
/// Converts NED position/velocity to lat/lon/alt in degE7/mm format.
/// @param state     current vehicle state (NED position relative to GPS origin)
/// @param gps_cfg   GPS origin and accuracy configuration
/// @return GPSData ready for HIL_GPS message
GPSData synthesize_gps(
    const VehicleState& state,
    const GPSConfig& gps_cfg);

/// Gaussian noise sample
float gaussian_noise(float stddev);

} // namespace fiber_nav_physics
