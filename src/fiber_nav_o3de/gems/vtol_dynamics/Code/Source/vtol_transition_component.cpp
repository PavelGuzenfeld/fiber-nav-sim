#include "vtol_transition_component.h"
#include "aerodynamics_component.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Console/ILogger.h>

namespace vtol_dynamics
{
    void VTOLTransitionComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VTOLTransitionComponent, AZ::Component>()
                ->Version(1)
                ->Field("FWTransitionSpeed", &VTOLTransitionComponent::m_fwTransitionSpeed)
                ->Field("MCTransitionSpeed", &VTOLTransitionComponent::m_mcTransitionSpeed)
                ->Field("TransitionTimeout", &VTOLTransitionComponent::m_transitionTimeout)
                ->Field("BackTransitionTime", &VTOLTransitionComponent::m_backTransitionTime)
                ;

            if (auto* edit = serialize->GetEditContext())
            {
                edit->Class<VTOLTransitionComponent>("VTOL Transition",
                    "MC/FW transition state machine for quadtailsitter")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "VTOLDynamics")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &VTOLTransitionComponent::m_fwTransitionSpeed,
                        "FW Transition Speed", "Airspeed (m/s) to enter FW mode")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &VTOLTransitionComponent::m_mcTransitionSpeed,
                        "MC Transition Speed", "Airspeed (m/s) to revert to MC mode")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &VTOLTransitionComponent::m_transitionTimeout,
                        "Transition Timeout", "Max time (s) in transition before reverting")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &VTOLTransitionComponent::m_backTransitionTime,
                        "Back Transition Time", "Duration (s) of FW→MC back transition")
                    ;
            }
        }
    }

    void VTOLTransitionComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        VTOLStateRequestBus::Handler::BusConnect();
    }

    void VTOLTransitionComponent::Deactivate()
    {
        VTOLStateRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    AeroState VTOLTransitionComponent::GetAeroState() const
    {
        // Query the AerodynamicsComponent on the same entity
        auto* aeroComp = GetEntity()->FindComponent<AerodynamicsComponent>();
        if (aeroComp)
        {
            return aeroComp->GetAeroState();
        }
        return {};
    }

    void VTOLTransitionComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        // Get airspeed from AerodynamicsComponent
        auto* aeroComp = GetEntity()->FindComponent<AerodynamicsComponent>();
        m_currentAirspeed = aeroComp ? aeroComp->GetAeroState().airspeed : 0.0f;

        switch (m_flightMode)
        {
        case VTOLFlightMode::Multicopter:
            if (m_currentAirspeed >= m_fwTransitionSpeed)
            {
                m_flightMode = VTOLFlightMode::TransitionToFW;
                m_transitionTimer = 0.0f;
                AZLOG_INFO("VTOL: MC → TransitionToFW (airspeed=%.1f m/s)", m_currentAirspeed);
            }
            break;

        case VTOLFlightMode::TransitionToFW:
            m_transitionTimer += deltaTime;
            if (m_currentAirspeed >= m_fwTransitionSpeed)
            {
                m_flightMode = VTOLFlightMode::FixedWing;
                AZLOG_INFO("VTOL: TransitionToFW → FixedWing (%.1fs, %.1f m/s)",
                    m_transitionTimer, m_currentAirspeed);
            }
            else if (m_transitionTimer >= m_transitionTimeout)
            {
                m_flightMode = VTOLFlightMode::Multicopter;
                AZLOG_WARN("VTOL: Transition timeout → MC");
            }
            break;

        case VTOLFlightMode::FixedWing:
            if (m_currentAirspeed < m_mcTransitionSpeed)
            {
                m_flightMode = VTOLFlightMode::TransitionToMC;
                m_transitionTimer = 0.0f;
                AZLOG_INFO("VTOL: FW → TransitionToMC (airspeed=%.1f m/s)", m_currentAirspeed);
            }
            break;

        case VTOLFlightMode::TransitionToMC:
            m_transitionTimer += deltaTime;
            if (m_transitionTimer >= m_backTransitionTime)
            {
                m_flightMode = VTOLFlightMode::Multicopter;
                AZLOG_INFO("VTOL: TransitionToMC → MC (%.1fs)", m_transitionTimer);
            }
            break;
        }
    }

} // namespace vtol_dynamics
