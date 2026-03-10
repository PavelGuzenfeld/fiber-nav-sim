#include "fiber_nav_physics/vtol_transition.hpp"

namespace fiber_nav_physics
{

bool VtolTransition::update(float airspeed, float dt)
{
    auto prev = flight_mode;

    switch (flight_mode)
    {
    case VTOLFlightMode::Multicopter:
        if (airspeed >= fw_transition_speed)
        {
            flight_mode = VTOLFlightMode::TransitionToFW;
            transition_timer = 0.0f;
        }
        break;

    case VTOLFlightMode::TransitionToFW:
        transition_timer += dt;
        if (airspeed >= fw_transition_speed)
        {
            flight_mode = VTOLFlightMode::FixedWing;
        }
        else if (transition_timer >= transition_timeout)
        {
            flight_mode = VTOLFlightMode::Multicopter;
        }
        break;

    case VTOLFlightMode::FixedWing:
        if (airspeed < mc_transition_speed)
        {
            flight_mode = VTOLFlightMode::TransitionToMC;
            transition_timer = 0.0f;
        }
        break;

    case VTOLFlightMode::TransitionToMC:
        transition_timer += dt;
        if (transition_timer >= back_transition_time)
        {
            flight_mode = VTOLFlightMode::Multicopter;
        }
        break;
    }

    return flight_mode != prev;
}

void VtolTransition::reset()
{
    flight_mode = VTOLFlightMode::Multicopter;
    transition_timer = 0.0f;
}

} // namespace fiber_nav_physics
