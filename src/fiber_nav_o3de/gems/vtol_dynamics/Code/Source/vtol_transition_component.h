#pragma once

#include <VTOLDynamics/vtol_dynamics_bus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace vtol_dynamics
{
    /// MC ↔ FW transition state machine for quadtailsitter.
    /// Monitors airspeed and vehicle attitude to determine flight mode.
    /// Publishes flight mode via VTOLStateRequestBus.
    ///
    /// Transition logic:
    ///   MC → FW: When airspeed exceeds fw_transition_speed and pitch < threshold
    ///   FW → MC: When airspeed drops below mc_transition_speed
    ///
    /// Parameters are configurable (not hardcoded).
    class VTOLTransitionComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public VTOLStateRequestBus::Handler
    {
    public:
        AZ_COMPONENT(VTOLTransitionComponent, "{F3A4B5C6-D7E8-9F01-2345-6789ABCDEF02}");

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // VTOLStateRequestBus
        VTOLFlightMode GetFlightMode() const override { return m_flightMode; }
        AeroState GetAeroState() const override;
        float GetAirspeed() const override { return m_currentAirspeed; }

    private:
        // Transition parameters (all configurable via Editor)
        float m_fwTransitionSpeed{15.0f};    // m/s — speed to enter FW mode
        float m_mcTransitionSpeed{8.0f};     // m/s — speed to revert to MC mode
        float m_transitionTimeout{5.0f};     // s — max time in transition state
        float m_backTransitionTime{5.0f};    // s — FW→MC back transition duration

        // Runtime
        VTOLFlightMode m_flightMode{VTOLFlightMode::Multicopter};
        float m_transitionTimer{0.0f};
        float m_currentAirspeed{0.0f};
    };

} // namespace vtol_dynamics
