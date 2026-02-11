#pragma once

#include <PX4Bridge/px4_bridge_bus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/vector.h>

namespace vtol_dynamics
{
    /// Per-motor configuration — serializable, editable in O3DE Editor.
    /// No hardcoded values; all parameters set via Editor or config files.
    struct MotorConfig
    {
        AZ_TYPE_INFO(MotorConfig, "{F1A2B3C4-D5E6-7890-ABCD-1234567890AB}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 position{AZ::Vector3::CreateZero()};
        float motor_constant{8.54858e-06f};
        float moment_constant{0.016f};
        float max_rot_velocity{1200.0f};
        float time_constant_up{0.0125f};
        float time_constant_down{0.025f};
        float rotor_drag_coeff{8.06428e-05f};
        float rolling_moment_coeff{1e-06f};
        int direction{1}; // +1 = CCW, -1 = CW

        // Runtime state (not serialized)
        float commanded_velocity{0.0f};
        float current_velocity{0.0f};
    };

    /// Applies motor thrust forces to the vehicle rigid body.
    /// All motor parameters are configurable via the O3DE Editor.
    class MulticopterMotorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public px4_bridge::PX4MotorOutputNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(MulticopterMotorComponent, "{D1E2F3A4-B5C6-7D8E-9F0A-1B2C3D4E5F6A}");

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        void OnMotorOutputsReceived(const px4_bridge::MotorOutputs& outputs) override;

    private:
        AZStd::vector<MotorConfig> m_motors;
        float m_rotorVelocitySlowdownSim{10.0f};
    };

} // namespace vtol_dynamics
