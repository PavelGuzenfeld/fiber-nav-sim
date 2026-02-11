#pragma once

#include <PX4Bridge/px4_bridge_bus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>

namespace vtol_dynamics
{
    /// Collects sensor data from O3DE PhysX rigid body and publishes
    /// to PX4 via the PX4BridgeRequestBus (MAVLink HIL_SENSOR + HIL_GPS).
    ///
    /// Mimics the Gazebo sensor plugins:
    ///   - IMU (accel + gyro from PhysX, with configurable noise)
    ///   - Barometer (from altitude, with configurable noise)
    ///   - Magnetometer (from world magnetic field, with configurable noise)
    ///   - GPS (from world position, configurable rate and accuracy)
    ///
    /// All sensor parameters configurable via Editor (no hardcoded values).
    class SensorCollectorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(SensorCollectorComponent, "{A4B5C6D7-E8F9-0123-4567-89ABCDEF0345}");

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        px4_bridge::SensorData CollectSensorData(float deltaTime);
        px4_bridge::GPSData CollectGPSData();

        // Sensor rates (Hz) — configurable
        float m_sensorRate{200.0f};   // HIL_SENSOR publish rate
        float m_gpsRate{10.0f};       // HIL_GPS publish rate

        // IMU noise (configurable, from model_px4.sdf sensor noise params)
        float m_gyroNoiseStddev{0.0003394f};
        float m_accelNoiseStddev{0.004f};

        // Barometer
        float m_baroNoiseStddev{0.01f};    // mbar
        float m_seaLevelPressure{1013.25f}; // mbar
        float m_seaLevelTemp{288.15f};      // K

        // Magnetometer (world field in gauss, configurable)
        AZ::Vector3 m_worldMagField{0.21f, 0.05f, -0.42f};
        float m_magNoiseStddev{0.0001f}; // gauss

        // GPS origin (configurable, default: Negev desert)
        double m_gpsOriginLat{31.164093};   // degrees
        double m_gpsOriginLon{34.532227};   // degrees
        float m_gpsOriginAlt{141.0f};       // m MSL
        uint16_t m_gpsEph{20};              // cm horizontal accuracy
        uint16_t m_gpsEpv{40};              // cm vertical accuracy
        uint8_t m_gpsSatCount{12};
        bool m_gpsEnabled{true};

        // Rate tracking
        float m_sensorAccum{0.0f};
        float m_gpsAccum{0.0f};

        // Previous velocity for accel computation
        AZ::Vector3 m_prevVelocity{AZ::Vector3::CreateZero()};
        bool m_hasPrevVelocity{false};
    };

} // namespace vtol_dynamics
