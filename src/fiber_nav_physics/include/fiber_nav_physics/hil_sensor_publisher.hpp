#pragma once

#include "fiber_nav_physics/mavlink_connection.hpp"
#include "fiber_nav_physics/types.hpp"

#include <memory>

namespace fiber_nav_physics
{

/// Packs sensor data into MAVLink HIL_SENSOR and HIL_GPS messages.
/// Engine-agnostic — extracted from O3DE px4_bridge.
class HilSensorPublisher
{
public:
    void init(std::shared_ptr<MavlinkConnection> connection);

    void send_hil_sensor(const SensorData& data, uint64_t time_usec);
    void send_hil_gps(const GPSData& data, uint64_t time_usec);
    void send_heartbeat();

    uint64_t sensor_count() const { return sensor_count_; }
    uint64_t gps_count() const { return gps_count_; }

private:
    std::shared_ptr<MavlinkConnection> connection_;
    uint64_t sensor_count_{0};
    uint64_t gps_count_{0};
};

} // namespace fiber_nav_physics
