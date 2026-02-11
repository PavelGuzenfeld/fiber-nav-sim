#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>

namespace px4_bridge
{
    /// Sensor data from O3DE physics/sensors to be sent to PX4 via MAVLink HIL_SENSOR
    struct SensorData
    {
        // Accelerometer (m/s^2, body frame)
        AZ::Vector3 accel{0.0f, 0.0f, -9.81f};

        // Gyroscope (rad/s, body frame)
        AZ::Vector3 gyro{AZ::Vector3::CreateZero()};

        // Magnetometer (gauss, body frame)
        AZ::Vector3 mag{AZ::Vector3::CreateZero()};

        // Barometric pressure (mbar)
        float abs_pressure{1013.25f};

        // Differential pressure for airspeed (mbar)
        float diff_pressure{0.0f};

        // Barometric altitude (m)
        float pressure_alt{0.0f};

        // Temperature (degC)
        float temperature{25.0f};

        // Bitmask of updated fields (all = 0x1FFF)
        uint32_t fields_updated{0x1FFF};

        // Sensor ID (0 = default)
        uint8_t id{0};
    };

    /// GPS data to be sent to PX4 via MAVLink HIL_GPS
    struct GPSData
    {
        // Position (degE7)
        int32_t lat{0};
        int32_t lon{0};
        int32_t alt{0}; // mm MSL

        // Velocity (cm/s, NED)
        int16_t vn{0};
        int16_t ve{0};
        int16_t vd{0};

        // Ground speed (cm/s)
        uint16_t vel{0};

        // Course over ground (cdeg)
        uint16_t cog{0};

        // Fix type (0=no fix, 3=3D fix)
        uint8_t fix_type{3};

        // Number of satellites
        uint8_t satellites_visible{12};

        // Accuracy (cm)
        uint16_t eph{20}; // horizontal
        uint16_t epv{40}; // vertical
    };

    /// Motor output received from PX4 via MAVLink HIL_ACTUATOR_CONTROLS
    struct MotorOutputs
    {
        static constexpr int MAX_MOTORS = 16;
        float controls[MAX_MOTORS]{};
        uint8_t mode{0};
        uint64_t flags{0};
    };

    /// EBus for sending sensor data TO the bridge (O3DE sensors → MAVLink → PX4)
    class PX4BridgeRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual ~PX4BridgeRequests() = default;

        /// Send sensor data to PX4 (called at 200Hz from physics tick)
        virtual void SendSensorData(const SensorData& data) = 0;

        /// Send GPS data to PX4 (called at 10Hz)
        virtual void SendGPSData(const GPSData& data) = 0;

        /// Check if connected to PX4
        virtual bool IsConnected() const = 0;

        /// Get simulation time in microseconds
        virtual uint64_t GetSimTimeMicros() const = 0;
    };
    using PX4BridgeRequestBus = AZ::EBus<PX4BridgeRequests>;

    /// EBus for receiving motor outputs FROM PX4 (MAVLink → O3DE actuators)
    class PX4MotorOutputNotifications : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual ~PX4MotorOutputNotifications() = default;

        /// Called when new motor commands arrive from PX4
        virtual void OnMotorOutputsReceived(const MotorOutputs& outputs) = 0;
    };
    using PX4MotorOutputNotificationBus = AZ::EBus<PX4MotorOutputNotifications>;

} // namespace px4_bridge
