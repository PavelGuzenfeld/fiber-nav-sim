#pragma once

#include "fiber_nav_physics/types.hpp"

namespace fiber_nav_physics
{

/// Pure-math aerodynamic force computation.
/// No engine dependency — takes body-frame velocity, returns body-frame forces.
///
/// Extracted from O3DE AerodynamicsComponent.
struct AerodynamicsModel
{
    AeroCoefficients coeffs;
    WingGeometry wing;
    float min_airspeed{2.0f}; // m/s — below this, no aero forces

    /// Compute lift coefficient (with post-stall model)
    float compute_cl(float alpha) const;

    /// Compute drag coefficient (induced + parasitic, with post-stall model)
    float compute_cd(float alpha) const;

    /// Compute all aerodynamic forces and moments in body frame.
    /// @param body_velocity  velocity in body frame (m/s)
    /// @param body_angular_velocity  angular velocity in body frame (rad/s)
    /// @return forces + torques in body frame, plus AeroState for telemetry
    AeroState compute(Vec3 body_velocity, Vec3 body_angular_velocity) const;
};

} // namespace fiber_nav_physics
