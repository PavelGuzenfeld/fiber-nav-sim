#include "fiber_nav_physics/hil_sensor_publisher.hpp"

namespace fiber_nav_physics
{

void HilSensorPublisher::init(std::shared_ptr<MavlinkConnection> connection)
{
    connection_ = std::move(connection);
}

void HilSensorPublisher::send_hil_sensor(const SensorData& data, uint64_t time_usec)
{
    if (!connection_ || !connection_->is_connected())
    {
        return;
    }

    mavlink_message_t msg{};
    mavlink_msg_hil_sensor_pack(
        connection_->system_id(),
        connection_->component_id(),
        &msg,
        time_usec,
        data.accel.x, data.accel.y, data.accel.z,
        data.gyro.x, data.gyro.y, data.gyro.z,
        data.mag.x, data.mag.y, data.mag.z,
        data.abs_pressure,
        data.diff_pressure,
        data.pressure_alt,
        data.temperature,
        data.fields_updated,
        data.id);

    connection_->send(msg);
    ++sensor_count_;
}

void HilSensorPublisher::send_hil_gps(const GPSData& data, uint64_t time_usec)
{
    if (!connection_ || !connection_->is_connected())
    {
        return;
    }

    mavlink_message_t msg{};
    mavlink_msg_hil_gps_pack(
        connection_->system_id(),
        connection_->component_id(),
        &msg,
        time_usec,
        data.fix_type,
        data.lat, data.lon, data.alt,
        data.eph, data.epv,
        data.vel,
        data.vn, data.ve, data.vd,
        data.cog,
        data.satellites_visible,
        0,    // id (default GPS)
        0);   // yaw (cdeg, 0 = not available)

    connection_->send(msg);
    ++gps_count_;
}

void HilSensorPublisher::send_heartbeat()
{
    if (!connection_ || !connection_->is_connected())
    {
        return;
    }

    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(
        connection_->system_id(),
        connection_->component_id(),
        &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_INVALID,
        0, 0, 0);

    connection_->send(msg);
}

} // namespace fiber_nav_physics
