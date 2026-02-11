#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>

namespace vtol_dynamics
{
    /// VTOL flight mode for transition state machine
    enum class VTOLFlightMode : uint8_t
    {
        Multicopter,     // MC hover / climb
        TransitionToFW,  // MC → FW (pitching forward)
        FixedWing,       // FW cruise
        TransitionToMC,  // FW → MC (pitching back)
    };

    /// Aerodynamic state for telemetry / debugging
    struct AeroState
    {
        float airspeed{0.0f};           // m/s
        float angle_of_attack{0.0f};    // rad
        float sideslip_angle{0.0f};     // rad
        AZ::Vector3 lift_force{AZ::Vector3::CreateZero()};
        AZ::Vector3 drag_force{AZ::Vector3::CreateZero()};
        AZ::Vector3 aero_moment{AZ::Vector3::CreateZero()};
    };

    /// EBus for querying VTOL state
    class VTOLStateRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual ~VTOLStateRequests() = default;

        virtual VTOLFlightMode GetFlightMode() const = 0;
        virtual AeroState GetAeroState() const = 0;
        virtual float GetAirspeed() const = 0;
    };
    using VTOLStateRequestBus = AZ::EBus<VTOLStateRequests>;

} // namespace vtol_dynamics
