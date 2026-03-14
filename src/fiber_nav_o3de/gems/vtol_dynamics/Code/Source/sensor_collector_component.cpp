#include "sensor_collector_component.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>

#include <random>
#include <cmath>

namespace vtol_dynamics
{
    // Thread-local RNG for sensor noise
    static thread_local std::mt19937 s_rng{std::random_device{}()};

    static float GaussianNoise(float stddev)
    {
        if (stddev <= 0.0f) return 0.0f;
        std::normal_distribution<float> dist(0.0f, stddev);
        return dist(s_rng);
    }

    void SensorCollectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SensorCollectorComponent, AZ::Component>()
                ->Version(1)
                ->Field("SensorRate", &SensorCollectorComponent::m_sensorRate)
                ->Field("GPSRate", &SensorCollectorComponent::m_gpsRate)
                ->Field("GyroNoise", &SensorCollectorComponent::m_gyroNoiseStddev)
                ->Field("AccelNoise", &SensorCollectorComponent::m_accelNoiseStddev)
                ->Field("BaroNoise", &SensorCollectorComponent::m_baroNoiseStddev)
                ->Field("SeaLevelPressure", &SensorCollectorComponent::m_seaLevelPressure)
                ->Field("SeaLevelTemp", &SensorCollectorComponent::m_seaLevelTemp)
                ->Field("WorldMagField", &SensorCollectorComponent::m_worldMagField)
                ->Field("MagNoise", &SensorCollectorComponent::m_magNoiseStddev)
                ->Field("GPSOriginLat", &SensorCollectorComponent::m_gpsOriginLat)
                ->Field("GPSOriginLon", &SensorCollectorComponent::m_gpsOriginLon)
                ->Field("GPSOriginAlt", &SensorCollectorComponent::m_gpsOriginAlt)
                ->Field("GPSEph", &SensorCollectorComponent::m_gpsEph)
                ->Field("GPSEpv", &SensorCollectorComponent::m_gpsEpv)
                ->Field("GPSSatCount", &SensorCollectorComponent::m_gpsSatCount)
                ->Field("GPSEnabled", &SensorCollectorComponent::m_gpsEnabled)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<SensorCollectorComponent>("Sensor Collector",
                    "Collects PhysX data and publishes to PX4 via MAVLink HIL")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "VTOLDynamics")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Rates")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_sensorRate,
                        "Sensor Rate", "HIL_SENSOR publish rate (Hz)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gpsRate,
                        "GPS Rate", "HIL_GPS publish rate (Hz)")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "IMU")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gyroNoiseStddev,
                        "Gyro Noise", "Gyroscope noise stddev (rad/s)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_accelNoiseStddev,
                        "Accel Noise", "Accelerometer noise stddev (m/s^2)")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Barometer")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_baroNoiseStddev,
                        "Baro Noise", "Barometric pressure noise (mbar)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_seaLevelPressure,
                        "Sea Level Pressure", "Reference pressure (mbar)")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Magnetometer")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_worldMagField,
                        "World Mag Field", "Earth magnetic field vector (gauss)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_magNoiseStddev,
                        "Mag Noise", "Magnetometer noise (gauss)")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "GPS")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gpsOriginLat,
                        "Origin Lat", "GPS origin latitude (degrees)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gpsOriginLon,
                        "Origin Lon", "GPS origin longitude (degrees)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gpsOriginAlt,
                        "Origin Alt", "GPS origin altitude MSL (m)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorCollectorComponent::m_gpsEnabled,
                        "GPS Enabled", "Enable/disable GPS output")
                    ;
            }
        }
    }

    void SensorCollectorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void SensorCollectorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void SensorCollectorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        // Sensor rate limiting
        m_sensorAccum += deltaTime;
        float sensorInterval = (m_sensorRate > 0.0f) ? 1.0f / m_sensorRate : 0.005f;

        if (m_sensorAccum >= sensorInterval)
        {
            m_sensorAccum -= sensorInterval;
            auto sensorData = CollectSensorData(sensorInterval);
            px4_bridge::PX4BridgeRequestBus::Broadcast(
                &px4_bridge::PX4BridgeRequests::SendSensorData, sensorData);
        }

        // GPS rate limiting
        m_gpsAccum += deltaTime;
        float gpsInterval = (m_gpsRate > 0.0f) ? 1.0f / m_gpsRate : 0.1f;

        if (m_gpsAccum >= gpsInterval && m_gpsEnabled)
        {
            m_gpsAccum -= gpsInterval;
            auto gpsData = CollectGPSData();
            px4_bridge::PX4BridgeRequestBus::Broadcast(
                &px4_bridge::PX4BridgeRequests::SendGPSData, gpsData);
        }
    }

    px4_bridge::SensorData SensorCollectorComponent::CollectSensorData(float deltaTime)
    {
        auto entityId = GetEntityId();
        px4_bridge::SensorData data{};

        // Get rigid body state
        AZ::Vector3 linearVel = AZ::Vector3::CreateZero();
        AZ::Vector3 angularVel = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(linearVel, entityId,
            &Physics::RigidBodyRequests::GetLinearVelocity);
        Physics::RigidBodyRequestBus::EventResult(angularVel, entityId,
            &Physics::RigidBodyRequests::GetAngularVelocity);

        AZ::Transform worldTm = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTm, entityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Quaternion rot = worldTm.GetRotation();
        AZ::Quaternion rotInv = rot.GetConjugate();

        // ── Gyroscope (body frame angular velocity + noise) ──
        AZ::Vector3 bodyAngVel = rotInv.TransformVector(angularVel);
        data.gyro = AZ::Vector3(
            bodyAngVel.GetX() + GaussianNoise(m_gyroNoiseStddev),
            bodyAngVel.GetY() + GaussianNoise(m_gyroNoiseStddev),
            bodyAngVel.GetZ() + GaussianNoise(m_gyroNoiseStddev));

        // ── Accelerometer (body frame, includes gravity + noise) ──
        if (m_hasPrevVelocity && deltaTime > 0.0f)
        {
            AZ::Vector3 worldAccel = (linearVel - m_prevVelocity) / deltaTime;
            // Add gravity (accelerometer measures gravity as upward force when stationary)
            worldAccel -= AZ::Vector3(0.0f, 0.0f, -9.81f);
            AZ::Vector3 bodyAccel = rotInv.TransformVector(worldAccel);
            data.accel = AZ::Vector3(
                bodyAccel.GetX() + GaussianNoise(m_accelNoiseStddev),
                bodyAccel.GetY() + GaussianNoise(m_accelNoiseStddev),
                bodyAccel.GetZ() + GaussianNoise(m_accelNoiseStddev));
        }
        m_prevVelocity = linearVel;
        m_hasPrevVelocity = true;

        // ── Magnetometer (world field rotated to body frame + noise) ──
        AZ::Vector3 bodyMag = rotInv.TransformVector(m_worldMagField);
        data.mag = AZ::Vector3(
            bodyMag.GetX() + GaussianNoise(m_magNoiseStddev),
            bodyMag.GetY() + GaussianNoise(m_magNoiseStddev),
            bodyMag.GetZ() + GaussianNoise(m_magNoiseStddev));

        // ── Barometer (ISA atmosphere model + noise) ──
        float altitude = worldTm.GetTranslation().GetZ() + m_gpsOriginAlt;
        // ISA: P = P0 * (1 - L*h/T0)^(g*M/(R*L))
        constexpr float L = 0.0065f;      // lapse rate K/m
        constexpr float g = 9.80665f;
        constexpr float M = 0.0289644f;   // molar mass dry air kg/mol
        constexpr float R = 8.31447f;     // gas constant
        float T0 = m_seaLevelTemp;
        float P0 = m_seaLevelPressure;
        float exponent = g * M / (R * L);
        float pressure = P0 * std::pow(1.0f - L * altitude / T0, exponent);
        data.abs_pressure = pressure + GaussianNoise(m_baroNoiseStddev);
        data.pressure_alt = altitude;
        data.temperature = T0 - L * altitude - 273.15f; // Convert to Celsius

        data.fields_updated = 0x1FFF; // All fields

        return data;
    }

    px4_bridge::GPSData SensorCollectorComponent::CollectGPSData()
    {
        auto entityId = GetEntityId();
        px4_bridge::GPSData gps{};

        AZ::Transform worldTm = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTm, entityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Vector3 pos = worldTm.GetTranslation();

        AZ::Vector3 linearVel = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(linearVel, entityId,
            &Physics::RigidBodyRequests::GetLinearVelocity);

        // Convert local NED position to lat/lon
        // Approximate: 1 deg lat ≈ 111320m, 1 deg lon ≈ 111320m * cos(lat)
        constexpr double METERS_PER_DEG_LAT = 111320.0;
        double cosLat = std::cos(m_gpsOriginLat * M_PI / 180.0);
        double metersPerDegLon = METERS_PER_DEG_LAT * cosLat;

        // O3DE: X=East, Y=North, Z=Up (ENU)
        double lat = m_gpsOriginLat + static_cast<double>(pos.GetY()) / METERS_PER_DEG_LAT;
        double lon = m_gpsOriginLon + static_cast<double>(pos.GetX()) / metersPerDegLon;
        double alt = m_gpsOriginAlt + static_cast<double>(pos.GetZ());

        gps.lat = static_cast<int32_t>(lat * 1e7);
        gps.lon = static_cast<int32_t>(lon * 1e7);
        gps.alt = static_cast<int32_t>(alt * 1000.0); // mm

        // Velocity NED (O3DE ENU → NED: N=Y, E=X, D=-Z)
        gps.vn = static_cast<int16_t>(linearVel.GetY() * 100.0f); // cm/s
        gps.ve = static_cast<int16_t>(linearVel.GetX() * 100.0f);
        gps.vd = static_cast<int16_t>(-linearVel.GetZ() * 100.0f);

        float groundSpeed = std::sqrt(
            linearVel.GetX() * linearVel.GetX() +
            linearVel.GetY() * linearVel.GetY());
        gps.vel = static_cast<uint16_t>(groundSpeed * 100.0f);

        // Course over ground (degrees * 100)
        float cog = std::atan2(linearVel.GetX(), linearVel.GetY()) * 180.0f / AZ::Constants::Pi;
        if (cog < 0.0f) cog += 360.0f;
        gps.cog = static_cast<uint16_t>(cog * 100.0f);

        gps.fix_type = 3; // 3D fix
        gps.satellites_visible = m_gpsSatCount;
        gps.eph = m_gpsEph;
        gps.epv = m_gpsEpv;

        return gps;
    }

} // namespace vtol_dynamics
