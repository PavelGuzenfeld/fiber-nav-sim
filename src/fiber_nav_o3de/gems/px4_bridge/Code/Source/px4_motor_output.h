#pragma once

#include <PX4Bridge/px4_bridge_bus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace px4_bridge
{
    /// O3DE Component that receives motor commands from PX4 (via PX4MotorOutputNotificationBus)
    /// and applies forces/torques to the vehicle's rigid body in PhysX.
    ///
    /// Attach to the vehicle entity that has a PhysX Rigid Body component.
    /// Configure motor positions and thrust parameters to match the airframe.
    class PX4MotorOutput
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public PX4MotorOutputNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(PX4MotorOutput, "{B7E3F1A2-4D5C-6E8F-9A0B-1C2D3E4F5A6B}");

        static void Reflect(AZ::ReflectContext* context);

        // Component
        void Activate() override;
        void Deactivate() override;

        // TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // PX4MotorOutputNotificationBus
        void OnMotorOutputsReceived(const MotorOutputs& outputs) override;

    private:
        /// Per-motor configuration (matches model_px4.sdf layout)
        struct MotorConfig
        {
            AZ::Vector3 position{AZ::Vector3::CreateZero()};  // relative to CoM
            float max_thrust{12.0f};       // N at full throttle
            float moment_constant{0.016f}; // torque/thrust ratio
            int8_t direction{1};           // +1 = CCW, -1 = CW (torque reaction)
        };

        // Airframe: 4 motors for quadtailsitter
        // Positions from model_px4.sdf rotor links
        static constexpr int NUM_MOTORS = 4;
        MotorConfig m_motors[NUM_MOTORS] = {
            // rotor_0: forward-right, CCW
            {{0.145f, -0.23f, 0.06f}, 12.0f, 0.016f, 1},
            // rotor_1: back-left, CCW
            {{-0.145f, 0.23f, 0.06f}, 12.0f, 0.016f, 1},
            // rotor_2: forward-left, CW
            {{0.145f, 0.23f, 0.06f}, 12.0f, 0.016f, -1},
            // rotor_3: back-right, CW
            {{-0.145f, -0.23f, 0.06f}, 12.0f, 0.016f, -1},
        };

        MotorOutputs m_latestOutputs{};
        bool m_hasNewOutputs{false};
    };

} // namespace px4_bridge
