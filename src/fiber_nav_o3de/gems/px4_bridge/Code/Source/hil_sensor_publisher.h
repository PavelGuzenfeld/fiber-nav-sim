#pragma once

#include <PX4Bridge/px4_bridge_bus.h>
#include "mavlink_connection.h"

#include <cstdint>

namespace px4_bridge
{
    /// Packs O3DE sensor data into MAVLink HIL_SENSOR and HIL_GPS messages
    /// and sends them to PX4 via the MAVLink connection.
    ///
    /// Rates:
    ///   HIL_SENSOR: 200 Hz (accel, gyro, mag, baro)
    ///   HIL_GPS:     10 Hz
    ///   HEARTBEAT:    1 Hz
    class HILSensorPublisher
    {
    public:
        HILSensorPublisher() = default;

        void Init(std::shared_ptr<MAVLinkConnection> connection);

        /// Pack and send HIL_SENSOR message
        void SendHILSensor(const SensorData& data, uint64_t time_usec);

        /// Pack and send HIL_GPS message
        void SendHILGPS(const GPSData& data, uint64_t time_usec);

        /// Send HEARTBEAT (call at 1Hz)
        void SendHeartbeat();

        /// Get count of sent messages (for diagnostics)
        uint64_t GetSensorCount() const { return m_sensorCount; }
        uint64_t GetGPSCount() const { return m_gpsCount; }

    private:
        std::shared_ptr<MAVLinkConnection> m_connection;
        uint64_t m_sensorCount{0};
        uint64_t m_gpsCount{0};
    };

} // namespace px4_bridge
