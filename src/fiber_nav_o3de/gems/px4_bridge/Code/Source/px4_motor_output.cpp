#include "px4_motor_output.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace px4_bridge
{
    void PX4MotorOutput::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PX4MotorOutput, AZ::Component>()
                ->Version(1)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<PX4MotorOutput>("PX4 Motor Output",
                    "Applies motor forces from PX4 to the vehicle rigid body")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "PX4Bridge")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ;
            }
        }
    }

    void PX4MotorOutput::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        PX4MotorOutputNotificationBus::Handler::BusConnect();
    }

    void PX4MotorOutput::Deactivate()
    {
        PX4MotorOutputNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void PX4MotorOutput::OnMotorOutputsReceived(const MotorOutputs& outputs)
    {
        m_latestOutputs = outputs;
        m_hasNewOutputs = true;
    }

    void PX4MotorOutput::OnTick(float /*deltaTime*/, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_hasNewOutputs)
        {
            return;
        }
        m_hasNewOutputs = false;

        auto entityId = GetEntityId();

        // Apply forces from each motor
        AZ::Vector3 totalForce = AZ::Vector3::CreateZero();
        AZ::Vector3 totalTorque = AZ::Vector3::CreateZero();

        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            // PX4 outputs normalized [0..1] motor commands
            float throttle = AZStd::clamp(m_latestOutputs.controls[i], 0.0f, 1.0f);

            // Thrust along motor axis (body +Z for tailsitter = upward in MC, forward in FW)
            float thrust = throttle * m_motors[i].max_thrust;
            AZ::Vector3 thrustVec(0.0f, 0.0f, thrust);

            // Torque from motor position (cross product)
            AZ::Vector3 armTorque = m_motors[i].position.Cross(thrustVec);

            // Reaction torque from motor spin (yaw axis)
            float reactionTorque = thrust * m_motors[i].moment_constant
                                 * static_cast<float>(m_motors[i].direction);
            AZ::Vector3 reactionVec(0.0f, 0.0f, reactionTorque);

            totalForce += thrustVec;
            totalTorque += armTorque + reactionVec;
        }

        // Apply in body frame via PhysX rigid body
        // The RigidBodyRequestBus expects world-frame forces, so we need to transform
        AZ::Transform worldTm = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTm, entityId, &AZ::TransformBus::Events::GetWorldTM);

        AZ::Vector3 worldForce = worldTm.GetRotation().TransformVector(totalForce);
        AZ::Vector3 worldTorque = worldTm.GetRotation().TransformVector(totalTorque);

        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyLinearForce, worldForce);
        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyAngularForce, worldTorque);
    }

} // namespace px4_bridge
