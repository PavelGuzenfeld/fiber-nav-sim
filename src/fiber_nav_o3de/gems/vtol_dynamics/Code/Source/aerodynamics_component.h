#pragma once

#include <VTOLDynamics/vtol_dynamics_bus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace vtol_dynamics
{
    /// Aerodynamic coefficient set — fully serializable, no hardcoded values.
    /// Loaded from Editor properties or config files.
    struct AeroCoefficients
    {
        AZ_TYPE_INFO(AeroCoefficients, "{A2B3C4D5-E6F7-8901-2345-6789ABCDEF01}");
        static void Reflect(AZ::ReflectContext* context);

        // Zero-lift angle of attack
        float a0{0.0f};

        // Lift coefficients
        float CL0{0.0f};
        float CLa{0.0f};
        float CLa_stall{0.0f};

        // Drag coefficients
        float CD0{0.0f};
        float cda{0.0f};
        float CDa_stall{0.0f};

        // Pitch moment
        float Cem0{0.0f};
        float Cema{0.0f};
        float Cema_stall{0.0f};

        // Sideslip
        float CYb{0.0f};
        float Cellb{0.0f};
        float Cenb{0.0f};

        // Roll rate (p) derivatives
        float CDp{0.0f};
        float CYp{0.0f};
        float CLp{0.0f};
        float Cellp{0.0f};
        float Cemp{0.0f};
        float Cenp{0.0f};

        // Pitch rate (q) derivatives
        float CDq{0.0f};
        float CYq{0.0f};
        float CLq{0.0f};
        float Cellq{0.0f};
        float Cemq{0.0f};
        float Cenq{0.0f};

        // Yaw rate (r) derivatives
        float CDr{0.0f};
        float CYr{0.0f};
        float CLr{0.0f};
        float Cellr{0.0f};
        float Cemr{0.0f};
        float Cenr{0.0f};

        // Stall angle (rad)
        float alpha_stall{0.3f};
    };

    /// Wing geometry — fully serializable
    struct WingGeometry
    {
        AZ_TYPE_INFO(WingGeometry, "{B3C4D5E6-F789-0123-4567-89ABCDEF0123}");
        static void Reflect(AZ::ReflectContext* context);

        float aspect_ratio{1.0f};
        float efficiency{0.97f};
        float area{0.1f};           // m^2
        float mac{0.1f};            // Mean aerodynamic chord (m)
        float air_density{1.225f};  // kg/m^3
        AZ::Vector3 ref_pt{AZ::Vector3::CreateZero()};
        AZ::Vector3 forward{0.0f, 0.0f, 1.0f};   // Body forward direction
        AZ::Vector3 upward{-1.0f, 0.0f, 0.0f};    // Body upward direction
    };

    /// AdvancedLiftDrag equivalent for O3DE.
    /// All parameters configurable via O3DE Editor — no hardcoded aerodynamic data.
    class AerodynamicsComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(AerodynamicsComponent, "{E2F3A4B5-C6D7-8E9F-0A1B-2C3D4E5F6A7B}");

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        const AeroState& GetAeroState() const { return m_aeroState; }

    private:
        float ComputeCL(float alpha) const;
        float ComputeCD(float alpha) const;

        AeroCoefficients m_coeffs;
        WingGeometry m_wing;
        float m_minAirspeed{2.0f}; // m/s — below this, no aero forces

        AeroState m_aeroState;
    };

} // namespace vtol_dynamics
