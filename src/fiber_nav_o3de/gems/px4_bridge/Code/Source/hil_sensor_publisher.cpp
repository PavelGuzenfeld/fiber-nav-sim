#include "hil_sensor_publisher.h"

namespace px4_bridge
{
    void HILSensorPublisher::Init(std::shared_ptr<MAVLinkConnection> connection)
    {
        m_connection = connection;
    }

    void HILSensorPublisher::SendHILSensor(const SensorData& data, uint64_t time_usec)
    {
        if (!m_connection || !m_connection->IsConnected())
        {
            return;
        }

        mavlink_message_t msg{};
        mavlink_msg_hil_sensor_pack(
            m_connection->GetSystemId(),
            m_connection->GetComponentId(),
            &msg,
            time_usec,
            data.accel.GetX(), data.accel.GetY(), data.accel.GetZ(),     // xacc, yacc, zacc
            data.gyro.GetX(), data.gyro.GetY(), data.gyro.GetZ(),        // xgyro, ygyro, zgyro
            data.mag.GetX(), data.mag.GetY(), data.mag.GetZ(),           // xmag, ymag, zmag
            data.abs_pressure,                                            // abs_pressure (mbar)
            data.diff_pressure,                                           // diff_pressure (mbar)
            data.pressure_alt,                                            // pressure_alt (m)
            data.temperature,                                             // temperature (degC)
            data.fields_updated,                                          // fields_updated bitmask
            data.id                                                       // sensor id
        );

        m_connection->Send(msg);
        ++m_sensorCount;
    }

    void HILSensorPublisher::SendHILGPS(const GPSData& data, uint64_t time_usec)
    {
        if (!m_connection || !m_connection->IsConnected())
        {
            return;
        }

        mavlink_message_t msg{};
        mavlink_msg_hil_gps_pack(
            m_connection->GetSystemId(),
            m_connection->GetComponentId(),
            &msg,
            time_usec,
            data.fix_type,
            data.lat, data.lon, data.alt,          // lat, lon, alt (degE7, degE7, mm MSL)
            data.eph, data.epv,                     // eph, epv (cm)
            data.vel,                               // vel (cm/s ground speed)
            data.vn, data.ve, data.vd,              // vn, ve, vd (cm/s NED)
            data.cog,                               // cog (cdeg)
            data.satellites_visible,                // satellites_visible
            0                                       // id (0 = default GPS)
        );

        m_connection->Send(msg);
        ++m_gpsCount;
    }

    void HILSensorPublisher::SendHeartbeat()
    {
        if (!m_connection || !m_connection->IsConnected())
        {
            return;
        }

        mavlink_message_t msg{};
        mavlink_msg_heartbeat_pack(
            m_connection->GetSystemId(),
            m_connection->GetComponentId(),
            &msg,
            MAV_TYPE_GENERIC,              // type
            MAV_AUTOPILOT_INVALID,         // autopilot (we're the simulator, not an AP)
            0,                              // base_mode
            0,                              // custom_mode
            0                               // system_status
        );

        m_connection->Send(msg);
    }

} // namespace px4_bridge
