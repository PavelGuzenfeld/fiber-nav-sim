#pragma once

#include <PX4Bridge/px4_bridge_bus.h>
#include "mavlink_connection.h"
#include "hil_sensor_publisher.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

#include <memory>

namespace px4_bridge
{
    /// Main O3DE Component implementing the PX4 MAVLink Simulator Bridge.
    ///
    /// Architecture:
    ///   O3DE sensors → PX4BridgeRequestBus → this component → MAVLink TCP → PX4
    ///   PX4 → MAVLink TCP → this component → PX4MotorOutputNotificationBus → motor components
    ///
    /// Attach this to a single entity in the level. It manages the TCP connection
    /// to PX4 SITL and coordinates sensor/actuator data flow.
    class PX4MAVLinkBridge
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public PX4BridgeRequestBus::Handler
    {
    public:
        AZ_COMPONENT(PX4MAVLinkBridge, "{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}");

        static void Reflect(AZ::ReflectContext* context);

        // Component
        void Activate() override;
        void Deactivate() override;

        // TickBus — used for heartbeat timing only
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // PX4BridgeRequestBus
        void SendSensorData(const SensorData& data) override;
        void SendGPSData(const GPSData& data) override;
        bool IsConnected() const override;
        uint64_t GetSimTimeMicros() const override;

    private:
        void OnMAVLinkMessage(const mavlink_message_t& msg);

        // Configuration (editable in O3DE Editor)
        AZStd::string m_host{"127.0.0.1"};
        uint16_t m_port{4560};

        std::shared_ptr<MAVLinkConnection> m_connection;
        HILSensorPublisher m_hilPublisher;

        float m_heartbeatTimer{0.0f};
        static constexpr float HEARTBEAT_INTERVAL = 1.0f; // seconds

        // Simulation time tracking
        uint64_t m_simTimeMicros{0};
    };

} // namespace px4_bridge
