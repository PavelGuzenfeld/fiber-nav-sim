#include "px4_mavlink_bridge.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Console/ILogger.h>
#include <cinttypes>

namespace px4_bridge
{
    void PX4MAVLinkBridge::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PX4MAVLinkBridge, AZ::Component>()
                ->Version(1)
                ->Field("Host", &PX4MAVLinkBridge::m_host)
                ->Field("Port", &PX4MAVLinkBridge::m_port)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<PX4MAVLinkBridge>("PX4 MAVLink Bridge",
                    "Connects O3DE simulation to PX4 autopilot via MAVLink HIL protocol")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "PX4Bridge")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PX4MAVLinkBridge::m_host,
                        "PX4 Host", "PX4 SITL TCP address")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PX4MAVLinkBridge::m_port,
                        "PX4 Port", "PX4 SITL TCP port (default: 4560)")
                    ;
            }
        }
    }

    void PX4MAVLinkBridge::Activate()
    {
        m_connection = std::make_shared<MAVLinkConnection>();
        m_hilPublisher.Init(m_connection);

        // Set up message callback for receiving actuator commands from PX4
        m_connection->SetMessageCallback(
            [this](const mavlink_message_t& msg) { OnMAVLinkMessage(msg); });

        MAVLinkConnection::Config config{};
        config.port = m_port;

        if (!m_connection->Start(config))
        {
            AZLOG_ERROR("PX4Bridge: Failed to start TCP server on port %u", m_port);
            return;
        }

        AZ::TickBus::Handler::BusConnect();
        PX4BridgeRequestBus::Handler::BusConnect();

        AZLOG_INFO("PX4Bridge: Activated, TCP server on port %u waiting for PX4", m_port);
    }

    void PX4MAVLinkBridge::Deactivate()
    {
        PX4BridgeRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        m_connection->Stop();

        AZLOG_INFO("PX4Bridge: Deactivated (sent %" PRIu64 " sensor, %" PRIu64 " GPS messages)",
            m_hilPublisher.GetSensorCount(), m_hilPublisher.GetGPSCount());
    }

    void PX4MAVLinkBridge::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        // Update simulation time
        m_simTimeMicros += static_cast<uint64_t>(deltaTime * 1'000'000.0f);

        // Send heartbeat at 1Hz
        m_heartbeatTimer += deltaTime;
        if (m_heartbeatTimer >= HEARTBEAT_INTERVAL)
        {
            m_heartbeatTimer -= HEARTBEAT_INTERVAL;
            m_hilPublisher.SendHeartbeat();
        }

        // Send static sensor data at 200Hz until entity-bound SensorCollector is wired up.
        // This provides minimum viable data for PX4 EKF initialization:
        // - Accel: gravity only (stationary vehicle)
        // - Gyro: zero
        // - Baro: from GPS origin altitude
        // - Mag: Earth field for Negev desert
        // - GPS: stationary at origin
        m_sensorTimer += deltaTime;
        constexpr float SENSOR_INTERVAL = 0.005f; // 200 Hz
        if (m_sensorTimer >= SENSOR_INTERVAL)
        {
            m_sensorTimer -= SENSOR_INTERVAL;

            SensorData sensor{};
            sensor.accel = AZ::Vector3(0.0f, 0.0f, 9.81f); // gravity in body frame (stationary)
            sensor.gyro = AZ::Vector3::CreateZero();
            sensor.mag = AZ::Vector3(0.21f, 0.05f, -0.42f); // Negev desert mag field (gauss)
            sensor.abs_pressure = 996.0f;  // ~141m MSL pressure
            sensor.pressure_alt = 143.0f;  // 141m origin + 2m spawn height
            sensor.temperature = 20.0f;
            sensor.fields_updated = 0x1FFF;
            sensor.id = 0;
            m_hilPublisher.SendHILSensor(sensor, m_simTimeMicros);
        }

        m_gpsTimer += deltaTime;
        constexpr float GPS_INTERVAL = 0.1f; // 10 Hz
        if (m_gpsTimer >= GPS_INTERVAL)
        {
            m_gpsTimer -= GPS_INTERVAL;

            GPSData gps{};
            gps.fix_type = 3;
            gps.lat = static_cast<int32_t>(31.164093 * 1e7);
            gps.lon = static_cast<int32_t>(34.532227 * 1e7);
            gps.alt = 143000; // 143m MSL in mm
            gps.eph = 20;
            gps.epv = 40;
            gps.vel = 0;
            gps.vn = 0; gps.ve = 0; gps.vd = 0;
            gps.cog = 0;
            gps.satellites_visible = 12;
            m_hilPublisher.SendHILGPS(gps, m_simTimeMicros);
        }
    }

    void PX4MAVLinkBridge::SendSensorData(const SensorData& data)
    {
        m_hilPublisher.SendHILSensor(data, m_simTimeMicros);
    }

    void PX4MAVLinkBridge::SendGPSData(const GPSData& data)
    {
        m_hilPublisher.SendHILGPS(data, m_simTimeMicros);
    }

    bool PX4MAVLinkBridge::IsConnected() const
    {
        return m_connection && m_connection->IsConnected();
    }

    uint64_t PX4MAVLinkBridge::GetSimTimeMicros() const
    {
        return m_simTimeMicros;
    }

    void PX4MAVLinkBridge::OnMAVLinkMessage(const mavlink_message_t& msg)
    {
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
        {
            mavlink_hil_actuator_controls_t actuators{};
            mavlink_msg_hil_actuator_controls_decode(&msg, &actuators);

            MotorOutputs outputs{};
            for (int i = 0; i < MotorOutputs::MAX_MOTORS; ++i)
            {
                outputs.controls[i] = actuators.controls[i];
            }
            outputs.mode = actuators.mode;
            outputs.flags = actuators.flags;

            // Broadcast to all motor output components
            PX4MotorOutputNotificationBus::Broadcast(
                &PX4MotorOutputNotifications::OnMotorOutputsReceived, outputs);
            break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT:
            // PX4 heartbeat received — connection is alive
            break;

        default:
            // Ignore other messages
            break;
        }
    }

} // namespace px4_bridge
