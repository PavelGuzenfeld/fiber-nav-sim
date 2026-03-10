#pragma once

#include "fiber_nav_physics/types.hpp"

#include <span>

namespace fiber_nav_physics
{

/// First-order motor model with asymmetric spin-up/down time constants.
/// Computes body-frame thrust and torque from PX4 actuator commands.
///
/// Extracted from O3DE MulticopterMotorComponent.

/// Update motor states from PX4 actuator commands.
/// @param configs   motor configuration (read-only)
/// @param states    motor runtime state (read-write, same length as configs)
/// @param outputs   PX4 actuator outputs [0..1]
void update_motor_commands(
    std::span<const MotorConfig> configs,
    std::span<MotorState> states,
    const MotorOutputs& outputs);

/// Step motor dynamics forward by dt.
/// First-order lag with asymmetric time constants.
/// @param configs   motor configuration
/// @param states    motor runtime state (updated in-place)
/// @param dt        time step (s)
void step_motor_dynamics(
    std::span<const MotorConfig> configs,
    std::span<MotorState> states,
    float dt);

/// Compute total body-frame thrust and torque from current motor states.
/// @param configs   motor configuration
/// @param states    motor runtime state
/// @return forces + torques in body frame
ForcesTorques compute_motor_forces(
    std::span<const MotorConfig> configs,
    std::span<const MotorState> states);

} // namespace fiber_nav_physics
