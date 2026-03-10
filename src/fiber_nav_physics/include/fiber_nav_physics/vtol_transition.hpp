#pragma once

#include "fiber_nav_physics/types.hpp"

namespace fiber_nav_physics
{

/// VTOL MC/FW transition state machine.
/// Engine-agnostic — extracted from O3DE VTOLTransitionComponent.
struct VtolTransition
{
    // Configuration (all configurable, no hardcoded values)
    float fw_transition_speed{15.0f};   // m/s — airspeed to enter FW mode
    float mc_transition_speed{8.0f};    // m/s — airspeed to revert to MC mode
    float transition_timeout{5.0f};     // s — max time in transition before reverting
    float back_transition_time{5.0f};   // s — FW→MC back transition duration

    // State
    VTOLFlightMode flight_mode{VTOLFlightMode::Multicopter};
    float transition_timer{0.0f};

    /// Update state machine based on current airspeed.
    /// @param airspeed  current airspeed (m/s)
    /// @param dt        time step (s)
    /// @return true if flight mode changed this tick
    bool update(float airspeed, float dt);

    /// Reset to MC mode
    void reset();
};

} // namespace fiber_nav_physics
