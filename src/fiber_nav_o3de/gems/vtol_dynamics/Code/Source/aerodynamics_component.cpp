#include "aerodynamics_component.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>

#include <cmath>

namespace vtol_dynamics
{
    void AeroCoefficients::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AeroCoefficients>()
                ->Version(1)
                ->Field("a0", &AeroCoefficients::a0)
                ->Field("CL0", &AeroCoefficients::CL0)
                ->Field("CLa", &AeroCoefficients::CLa)
                ->Field("CLa_stall", &AeroCoefficients::CLa_stall)
                ->Field("CD0", &AeroCoefficients::CD0)
                ->Field("cda", &AeroCoefficients::cda)
                ->Field("CDa_stall", &AeroCoefficients::CDa_stall)
                ->Field("Cem0", &AeroCoefficients::Cem0)
                ->Field("Cema", &AeroCoefficients::Cema)
                ->Field("CYb", &AeroCoefficients::CYb)
                ->Field("Cellb", &AeroCoefficients::Cellb)
                ->Field("Cenb", &AeroCoefficients::Cenb)
                ->Field("CDp", &AeroCoefficients::CDp)
                ->Field("CYp", &AeroCoefficients::CYp)
                ->Field("CLp", &AeroCoefficients::CLp)
                ->Field("Cellp", &AeroCoefficients::Cellp)
                ->Field("Cemp", &AeroCoefficients::Cemp)
                ->Field("Cenp", &AeroCoefficients::Cenp)
                ->Field("CDq", &AeroCoefficients::CDq)
                ->Field("CYq", &AeroCoefficients::CYq)
                ->Field("CLq", &AeroCoefficients::CLq)
                ->Field("Cellq", &AeroCoefficients::Cellq)
                ->Field("Cemq", &AeroCoefficients::Cemq)
                ->Field("Cenq", &AeroCoefficients::Cenq)
                ->Field("CDr", &AeroCoefficients::CDr)
                ->Field("CYr", &AeroCoefficients::CYr)
                ->Field("CLr", &AeroCoefficients::CLr)
                ->Field("Cellr", &AeroCoefficients::Cellr)
                ->Field("Cemr", &AeroCoefficients::Cemr)
                ->Field("Cenr", &AeroCoefficients::Cenr)
                ->Field("alpha_stall", &AeroCoefficients::alpha_stall)
                ;
        }
    }

    void WingGeometry::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WingGeometry>()
                ->Version(1)
                ->Field("AspectRatio", &WingGeometry::aspect_ratio)
                ->Field("Efficiency", &WingGeometry::efficiency)
                ->Field("Area", &WingGeometry::area)
                ->Field("MAC", &WingGeometry::mac)
                ->Field("AirDensity", &WingGeometry::air_density)
                ->Field("RefPt", &WingGeometry::ref_pt)
                ->Field("Forward", &WingGeometry::forward)
                ->Field("Upward", &WingGeometry::upward)
                ;
        }
    }

    void AerodynamicsComponent::Reflect(AZ::ReflectContext* context)
    {
        AeroCoefficients::Reflect(context);
        WingGeometry::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AerodynamicsComponent, AZ::Component>()
                ->Version(1)
                ->Field("Coefficients", &AerodynamicsComponent::m_coeffs)
                ->Field("Wing", &AerodynamicsComponent::m_wing)
                ->Field("MinAirspeed", &AerodynamicsComponent::m_minAirspeed)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<AerodynamicsComponent>("Aerodynamics",
                    "AdvancedLiftDrag model — all parameters configurable")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "VTOLDynamics")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AerodynamicsComponent::m_coeffs,
                        "Aero Coefficients", "Aerodynamic coefficient set")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AerodynamicsComponent::m_wing,
                        "Wing Geometry", "Wing physical parameters")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AerodynamicsComponent::m_minAirspeed,
                        "Min Airspeed", "Below this speed (m/s), aerodynamic forces are zero")
                    ;
            }
        }
    }

    void AerodynamicsComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void AerodynamicsComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    float AerodynamicsComponent::ComputeCL(float alpha) const
    {
        float alpha_eff = alpha - m_coeffs.a0;

        if (std::abs(alpha_eff) <= m_coeffs.alpha_stall)
        {
            return m_coeffs.CL0 + m_coeffs.CLa * alpha_eff;
        }

        float sign = (alpha_eff > 0.0f) ? 1.0f : -1.0f;
        float cl_at_stall = m_coeffs.CL0 + m_coeffs.CLa * sign * m_coeffs.alpha_stall;
        float delta_alpha = alpha_eff - sign * m_coeffs.alpha_stall;
        return cl_at_stall + m_coeffs.CLa_stall * delta_alpha;
    }

    float AerodynamicsComponent::ComputeCD(float alpha) const
    {
        float cl = ComputeCL(alpha);
        float cdi = (m_wing.aspect_ratio > 0.0f && m_wing.efficiency > 0.0f)
            ? (cl * cl) / (AZ::Constants::Pi * m_wing.aspect_ratio * m_wing.efficiency)
            : 0.0f;

        float alpha_eff = alpha - m_coeffs.a0;

        if (std::abs(alpha_eff) <= m_coeffs.alpha_stall)
        {
            return m_coeffs.CD0 + cdi;
        }

        float sign = (alpha_eff > 0.0f) ? 1.0f : -1.0f;
        float cd_at_stall = m_coeffs.CD0 + cdi;
        float delta_alpha = alpha_eff - sign * m_coeffs.alpha_stall;
        return cd_at_stall - m_coeffs.CDa_stall * delta_alpha;
    }

    void AerodynamicsComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        auto entityId = GetEntityId();

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

        AZ::Vector3 bodyVel = rotInv.TransformVector(linearVel);
        AZ::Vector3 bodyAngVel = rotInv.TransformVector(angularVel);

        float vel_forward = bodyVel.Dot(m_wing.forward);
        float vel_up = bodyVel.Dot(m_wing.upward);
        float vel_lateral = bodyVel.GetY();

        float airspeed = bodyVel.GetLength();
        m_aeroState.airspeed = airspeed;

        if (airspeed < m_minAirspeed)
        {
            m_aeroState.lift_force = AZ::Vector3::CreateZero();
            m_aeroState.drag_force = AZ::Vector3::CreateZero();
            m_aeroState.aero_moment = AZ::Vector3::CreateZero();
            return;
        }

        float alpha = std::atan2(-vel_up, vel_forward);
        float beta = std::asin(AZStd::clamp(vel_lateral / airspeed, -1.0f, 1.0f));
        m_aeroState.angle_of_attack = alpha;
        m_aeroState.sideslip_angle = beta;

        float q = 0.5f * m_wing.air_density * airspeed * airspeed;
        float qS = q * m_wing.area;
        float qSc = qS * m_wing.mac;

        float half_v = m_wing.mac / (2.0f * airspeed);
        float p_hat = bodyAngVel.GetX() * half_v;
        float q_hat = bodyAngVel.GetY() * half_v;
        float r_hat = bodyAngVel.GetZ() * half_v;

        float CL = ComputeCL(alpha) + m_coeffs.CLq * q_hat + m_coeffs.CLp * p_hat + m_coeffs.CLr * r_hat;
        float CD = ComputeCD(alpha) + m_coeffs.CDq * q_hat + m_coeffs.CDp * p_hat + m_coeffs.CDr * r_hat;
        float CY = m_coeffs.CYb * beta + m_coeffs.CYq * q_hat + m_coeffs.CYp * p_hat + m_coeffs.CYr * r_hat;

        float Cell = m_coeffs.Cellb * beta + m_coeffs.Cellq * q_hat + m_coeffs.Cellp * p_hat + m_coeffs.Cellr * r_hat;
        float Cem = m_coeffs.Cem0 + m_coeffs.Cema * alpha + m_coeffs.Cemq * q_hat + m_coeffs.Cemp * p_hat + m_coeffs.Cemr * r_hat;
        float Cen = m_coeffs.Cenb * beta + m_coeffs.Cenq * q_hat + m_coeffs.Cenp * p_hat + m_coeffs.Cenr * r_hat;

        AZ::Vector3 velDir = bodyVel.GetNormalized();
        AZ::Vector3 liftDir = velDir.Cross(AZ::Vector3(0.0f, 1.0f, 0.0f)).GetNormalized();
        if (liftDir.Dot(m_wing.upward) < 0.0f)
        {
            liftDir = -liftDir;
        }

        AZ::Vector3 liftForce = liftDir * (CL * qS);
        AZ::Vector3 dragForce = -velDir * (CD * qS);
        AZ::Vector3 sideForce = AZ::Vector3(0.0f, 1.0f, 0.0f) * (CY * qS);

        AZ::Vector3 totalAeroForce = liftForce + dragForce + sideForce;
        AZ::Vector3 aeroMoment(Cell * qSc, Cem * qSc, Cen * qSc);

        m_aeroState.lift_force = liftForce;
        m_aeroState.drag_force = dragForce;
        m_aeroState.aero_moment = aeroMoment;

        AZ::Vector3 worldForce = rot.TransformVector(totalAeroForce);
        AZ::Vector3 worldTorque = rot.TransformVector(aeroMoment);

        // Convert force to impulse: impulse = force * deltaTime
        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyLinearImpulse, worldForce * deltaTime);
        Physics::RigidBodyRequestBus::Event(entityId,
            &Physics::RigidBodyRequests::ApplyAngularImpulse, worldTorque * deltaTime);
    }

} // namespace vtol_dynamics
