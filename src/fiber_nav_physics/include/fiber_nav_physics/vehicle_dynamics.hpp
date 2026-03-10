#pragma once

#include "fiber_nav_physics/aerodynamics.hpp"
#include "fiber_nav_physics/motor_dynamics.hpp"
#include "fiber_nav_physics/types.hpp"
#include "fiber_nav_physics/vtol_transition.hpp"

#include <span>
#include <vector>

namespace fiber_nav_physics
{

/// Vehicle configuration
struct VehicleConfig
{
    float mass{2.0f};            // kg
    Vec3 inertia{0.03f, 0.03f, 0.01f}; // diagonal inertia tensor (Ixx, Iyy, Izz)

    AerodynamicsModel aero;
    std::vector<MotorConfig> motors;
    VtolTransition vtol;
};

/// Full VTOL vehicle dynamics integrator.
/// Combines motor thrust, aerodynamics, gravity, and external forces.
/// Runs at a fixed rate (e.g. 200 Hz) to produce physics state updates.
///
/// The integrator is physics-engine-agnostic: it takes external forces
/// as input and produces position/orientation as output. The caller
/// is responsible for injecting the pose into whatever renderer
/// (AirSim, O3DE, Gazebo).
class VehicleDynamics
{
public:
    /// Initialize with configuration and starting state
    void init(const VehicleConfig& config, const VehicleState& initial_state);

    /// Step the simulation forward by dt.
    /// @param dt              time step (s), typically 1/200
    /// @param external_force  additional body-frame force (e.g. cable tension)
    /// @param external_torque additional body-frame torque
    void step(float dt, Vec3 external_force = {}, Vec3 external_torque = {});

    /// Apply motor commands from PX4 (call when HIL_ACTUATOR_CONTROLS received)
    void set_motor_outputs(const MotorOutputs& outputs);

    /// Get current vehicle state
    const VehicleState& state() const { return state_; }

    /// Get current aero state (for telemetry)
    const AeroState& aero_state() const { return aero_state_; }

    /// Get current VTOL flight mode
    VTOLFlightMode flight_mode() const { return config_.vtol.flight_mode; }

    /// Get motor states (for telemetry)
    std::span<const MotorState> motor_states() const { return motor_states_; }

private:
    VehicleConfig config_;
    VehicleState state_;
    AeroState aero_state_;
    std::vector<MotorState> motor_states_;
};

} // namespace fiber_nav_physics
