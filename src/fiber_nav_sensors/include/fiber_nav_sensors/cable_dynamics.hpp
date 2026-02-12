#pragma once

#include <algorithm>
#include <cmath>

namespace fiber_nav_sensors {

/// Physical properties of the fiber optic cable.
struct CableProperties {
    double mass_per_meter{0.003};       // kg/m (jacketed fiber ~3 g/m)
    double diameter{0.0009};            // m (0.9mm jacketed fiber)
    double breaking_strength{50.0};     // N
    double drag_coefficient{1.1};       // Cd for thin cylinder in crossflow
    double drag_shape_factor{0.4};      // fraction of airborne length in crossflow
    double spool_friction_static{0.5};  // N (constant payout resistance)
    double spool_friction_kinetic{0.02};// N/(m/s) (velocity-dependent friction)
    double air_density{1.225};          // kg/m³
    double gravity{9.81};              // m/s²
};

/// Force vector in world frame (ENU / Gazebo convention).
struct ForceVec {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

/// Result of cable force computation for a single timestep.
struct CableForceResult {
    ForceVec total_force;       // World-frame force on drone (N)
    double tension;             // Scalar tension at attachment (N)
    double airborne_length;     // Cable suspended in air (m)
    double drag_magnitude;      // Aero drag component (N)
    double weight_magnitude;    // Weight component (N)
    double friction_magnitude;  // Spool friction component (N)
    bool is_broken;             // Cable exceeded breaking strength
};

/// Compute the length of cable suspended in the air.
/// @param deployed Total deployed cable length (m)
/// @param altitude Height above ground (m), must be >= 0
/// @return Airborne cable length (m)
inline double compute_airborne_length(double deployed, double altitude) {
    if (deployed <= 0.0 || altitude <= 0.0) return 0.0;

    // The cable hangs from the drone at 'altitude' to the ground.
    // Minimum airborne length = altitude (straight down).
    // As drone moves horizontally, the cable forms a catenary.
    // The horizontal distance from the ground contact to the drone
    // determines the catenary sag. For a lightweight fiber, the cable
    // hangs nearly vertically under the drone with a long tail on the ground.
    //
    // Simplified: airborne portion is bounded by altitude * pi/2 (quarter-circle)
    // and by the total deployed length.
    double max_airborne = altitude * (std::numbers::pi / 2.0);
    return std::min(deployed, std::max(altitude, max_airborne));
}

/// Compute aerodynamic drag force on the airborne cable.
/// @param props Cable physical properties
/// @param airborne_length Length of cable in the air (m)
/// @param vx Horizontal velocity component X (m/s)
/// @param vy Horizontal velocity component Y (m/s)
/// @return Drag force opposing horizontal velocity (N)
inline ForceVec compute_cable_drag(CableProperties const& props,
                                   double airborne_length,
                                   double vx, double vy) {
    double v_horiz_sq = vx * vx + vy * vy;
    if (v_horiz_sq < 1e-6 || airborne_length < 1e-3) return {};

    double v_horiz = std::sqrt(v_horiz_sq);

    // F_drag = 0.5 * rho * Cd * d * L_effective * V²
    // L_effective = airborne_length * shape_factor (not all cable sees full airflow)
    double effective_length = airborne_length * props.drag_shape_factor;
    double drag_mag = 0.5 * props.air_density * props.drag_coefficient
                      * props.diameter * effective_length * v_horiz_sq;

    // Direction: opposing horizontal velocity
    double inv_v = 1.0 / v_horiz;
    return {-drag_mag * vx * inv_v, -drag_mag * vy * inv_v, 0.0};
}

/// Compute weight force from the airborne cable.
/// @param props Cable physical properties
/// @param airborne_length Length of cable in the air (m)
/// @return Weight force (downward in world Z)
inline ForceVec compute_cable_weight(CableProperties const& props,
                                     double airborne_length) {
    if (airborne_length < 1e-3) return {};
    // Weight pulls straight down (negative Z in Gazebo ENU)
    double weight = props.mass_per_meter * props.gravity * airborne_length;
    return {0.0, 0.0, -weight};
}

/// Compute spool payout friction force.
/// @param props Cable physical properties
/// @param vx Velocity X (m/s)
/// @param vy Velocity Y (m/s)
/// @param vz Velocity Z (m/s)
/// @param payout_speed Spool payout rate (m/s)
/// @return Friction force opposing drone velocity (N)
inline ForceVec compute_spool_friction(CableProperties const& props,
                                       double vx, double vy, double vz,
                                       double payout_speed) {
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed < 1e-3 || payout_speed < 1e-3) return {};

    double friction = props.spool_friction_static
                      + props.spool_friction_kinetic * payout_speed;

    // Direction: opposing drone velocity
    double inv_speed = 1.0 / speed;
    return {-friction * vx * inv_speed,
            -friction * vy * inv_speed,
            -friction * vz * inv_speed};
}

/// Compute all cable forces for a single timestep.
/// @param props Cable physical properties
/// @param deployed_length Total cable deployed (m)
/// @param altitude Drone height above ground (m)
/// @param vx World-frame velocity X (m/s)
/// @param vy World-frame velocity Y (m/s)
/// @param vz World-frame velocity Z (m/s)
/// @param payout_speed Spool payout rate (m/s)
/// @param already_broken If true, cable is already broken — return zero forces
inline CableForceResult compute_cable_forces(CableProperties const& props,
                                              double deployed_length,
                                              double altitude,
                                              double vx, double vy, double vz,
                                              double payout_speed,
                                              bool already_broken) {
    CableForceResult result{};

    if (already_broken || deployed_length < 1e-3) {
        result.is_broken = already_broken;
        return result;
    }

    // Airborne length
    result.airborne_length = compute_airborne_length(deployed_length, altitude);

    // Individual force components
    auto drag = compute_cable_drag(props, result.airborne_length, vx, vy);
    auto weight = compute_cable_weight(props, result.airborne_length);
    auto friction = compute_spool_friction(props, vx, vy, vz, payout_speed);

    result.drag_magnitude = drag.magnitude();
    result.weight_magnitude = weight.magnitude();
    result.friction_magnitude = friction.magnitude();

    // Total force on drone
    result.total_force = {
        drag.x + weight.x + friction.x,
        drag.y + weight.y + friction.y,
        drag.z + weight.z + friction.z,
    };

    // Tension = magnitude of cable pull (drag + weight component)
    result.tension = std::sqrt(
        (drag.x + friction.x) * (drag.x + friction.x) +
        (drag.y + friction.y) * (drag.y + friction.y) +
        weight.z * weight.z +
        friction.magnitude() * friction.magnitude()
    );

    // Check breakage
    result.is_broken = result.tension > props.breaking_strength;
    if (result.is_broken) {
        result.total_force = {};  // Cable snapped — no more forces
    }

    return result;
}

}  // namespace fiber_nav_sensors
