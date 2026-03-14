#include "multicopter_motor_component.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace vtol_dynamics
{
    void MotorConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorConfig>()
                ->Version(1)
                ->Field("Position", &MotorConfig::position)
                ->Field("MotorConstant", &MotorConfig::motor_constant)
                ->Field("MomentConstant", &MotorConfig::moment_constant)
                ->Field("MaxRotVelocity", &MotorConfig::max_rot_velocity)
                ->Field("TimeConstantUp", &MotorConfig::time_constant_up)
                ->Field("TimeConstantDown", &MotorConfig::time_constant_down)
                ->Field("RotorDragCoeff", &MotorConfig::rotor_drag_coeff)
                ->Field("RollingMomentCoeff", &MotorConfig::rolling_moment_coeff)
                ->Field("Direction", &MotorConfig::direction)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<MotorConfig>("Motor Configuration", "Single motor parameters")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::position,
                        "Position", "Motor position relative to CoM (m)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::motor_constant,
                        "Motor Constant", "Thrust = constant * omega^2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::moment_constant,
                        "Moment Constant", "Torque / thrust ratio")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::max_rot_velocity,
                        "Max Rot Velocity", "Maximum rotor velocity (rad/s)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::time_constant_up,
                        "Time Constant Up", "Spin-up time constant (s)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::time_constant_down,
                        "Time Constant Down", "Spin-down time constant (s)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfig::direction,
                        "Direction", "+1 = CCW, -1 = CW")
                    ;
            }
        }
    }

    void MulticopterMotorComponent::Reflect(AZ::ReflectContext* context)
    {
        MotorConfig::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MulticopterMotorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Motors", &MulticopterMotorComponent::m_motors)
                ->Field("RotorSlowdown", &MulticopterMotorComponent::m_rotorVelocitySlowdownSim)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<MulticopterMotorComponent>("Multicopter Motors",
                    "Applies motor thrust/torque forces to rigid body")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "VTOLDynamics")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MulticopterMotorComponent::m_motors,
                        "Motors", "Motor configurations (one per motor)")
                    ->DataElement(AZ::Edit::UIHandlers::Default,
                        &MulticopterMotorComponent::m_rotorVelocitySlowdownSim,
                        "Rotor Slowdown", "Visual slowdown factor for rotor spin")
                    ;
            }
        }
    }

    void MulticopterMotorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        px4_bridge::PX4MotorOutputNotificationBus::Handler::BusConnect();
    }

    void MulticopterMotorComponent::Deactivate()
    {
        px4_bridge::PX4MotorOutputNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void MulticopterMotorComponent::OnMotorOutputsReceived(const px4_bridge::MotorOutputs& outputs)
    {
        for (size_t i = 0; i < m_motors.size() && i < px4_bridge::MotorOutputs::MAX_MOTORS; ++i)
        {
            float normalized = AZStd::clamp(outputs.controls[i], 0.0f, 1.0f);
            m_motors[i].commanded_velocity = normalized * m_motors[i].max_rot_velocity;
        }
    }

    void MulticopterMotorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        if (m_motors.empty())
        {
            return;
        }

        auto entityId = GetEntityId();

        AZ::Vector3 totalForce = AZ::Vector3::CreateZero();
        AZ::Vector3 totalTorque = AZ::Vector3::CreateZero();

        for (auto& motor : m_motors)
        {
            // First-order motor dynamics (different up/down time constants)
            float tau = (motor.commanded_velocity > motor.current_velocity)
                ? motor.time_constant_up
                : motor.time_constant_down;

            if (tau > 0.0f)
            {
                float alpha = deltaTime / (tau + deltaTime);
                motor.current_velocity += alpha * (motor.commanded_velocity - motor.current_velocity);
            }
            else
            {
                motor.current_velocity = motor.commanded_velocity;
            }

            float omega = motor.current_velocity;
            float omega2 = omega * omega;

            // Thrust = motor_constant * omega^2 (along body +Z)
            float thrust = motor.motor_constant * omega2;
            AZ::Vector3 thrustVec(0.0f, 0.0f, thrust);

            // Torque from arm (position cross thrust)
            AZ::Vector3 armTorque = motor.position.Cross(thrustVec);

            // Reaction torque from rotor drag (yaw axis)
            float reactionTorque = motor.moment_constant * thrust
                                 * static_cast<float>(motor.direction);
            AZ::Vector3 reactionVec(0.0f, 0.0f, reactionTorque);

            totalForce += thrustVec;
            totalTorque += armTorque + reactionVec;
        }

        // Transform from body frame to world frame
        AZ::Transform worldTm = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTm, entityId, &AZ::TransformBus::Events::GetWorldTM);

        AZ::Vector3 worldForce = worldTm.GetRotation().TransformVector(totalForce);
        AZ::Vector3 worldTorque = worldTm.GetRotation().TransformVector(totalTorque);

        // Convert force to impulse: impulse = force * deltaTime
        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyLinearImpulse, worldForce * deltaTime);
        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyAngularImpulse, worldTorque * deltaTime);
    }

} // namespace vtol_dynamics
