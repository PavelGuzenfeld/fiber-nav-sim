#include "fiber_nav_physics/aerodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fiber_nav_physics
{

float AerodynamicsModel::compute_cl(float alpha) const
{
    float alpha_eff = alpha - coeffs.a0;

    if (std::abs(alpha_eff) <= coeffs.alpha_stall)
    {
        return coeffs.CL0 + coeffs.CLa * alpha_eff;
    }

    float sign = (alpha_eff > 0.0f) ? 1.0f : -1.0f;
    float cl_at_stall = coeffs.CL0 + coeffs.CLa * sign * coeffs.alpha_stall;
    float delta_alpha = alpha_eff - sign * coeffs.alpha_stall;
    return cl_at_stall + coeffs.CLa_stall * delta_alpha;
}

float AerodynamicsModel::compute_cd(float alpha) const
{
    float cl = compute_cl(alpha);
    float cdi = (wing.aspect_ratio > 0.0f && wing.efficiency > 0.0f)
        ? (cl * cl) / (std::numbers::pi_v<float> * wing.aspect_ratio * wing.efficiency)
        : 0.0f;

    float alpha_eff = alpha - coeffs.a0;

    if (std::abs(alpha_eff) <= coeffs.alpha_stall)
    {
        return coeffs.CD0 + cdi;
    }

    float sign = (alpha_eff > 0.0f) ? 1.0f : -1.0f;
    float cd_at_stall = coeffs.CD0 + cdi;
    float delta_alpha = alpha_eff - sign * coeffs.alpha_stall;
    return cd_at_stall - coeffs.CDa_stall * delta_alpha;
}

AeroState AerodynamicsModel::compute(Vec3 body_vel, Vec3 body_ang_vel) const
{
    AeroState state{};
    float airspeed = body_vel.length();
    state.airspeed = airspeed;

    if (airspeed < min_airspeed)
    {
        return state;
    }

    float vel_forward = body_vel.dot(wing.forward);
    float vel_up = body_vel.dot(wing.upward);
    float vel_lateral = body_vel.y;

    float alpha = std::atan2(-vel_up, vel_forward);
    float beta = std::asin(std::clamp(vel_lateral / airspeed, -1.0f, 1.0f));
    state.angle_of_attack = alpha;
    state.sideslip_angle = beta;

    float q = 0.5f * wing.air_density * airspeed * airspeed;
    float qS = q * wing.area;
    float qSc = qS * wing.mac;

    float half_v = wing.mac / (2.0f * airspeed);
    float p_hat = body_ang_vel.x * half_v;
    float q_hat = body_ang_vel.y * half_v;
    float r_hat = body_ang_vel.z * half_v;

    float CL = compute_cl(alpha)
        + coeffs.CLq * q_hat + coeffs.CLp * p_hat + coeffs.CLr * r_hat;
    float CD = compute_cd(alpha)
        + coeffs.CDq * q_hat + coeffs.CDp * p_hat + coeffs.CDr * r_hat;
    float CY = coeffs.CYb * beta
        + coeffs.CYq * q_hat + coeffs.CYp * p_hat + coeffs.CYr * r_hat;

    float Cell = coeffs.Cellb * beta
        + coeffs.Cellq * q_hat + coeffs.Cellp * p_hat + coeffs.Cellr * r_hat;
    float Cem = coeffs.Cem0 + coeffs.Cema * alpha
        + coeffs.Cemq * q_hat + coeffs.Cemp * p_hat + coeffs.Cemr * r_hat;
    float Cen = coeffs.Cenb * beta
        + coeffs.Cenq * q_hat + coeffs.Cenp * p_hat + coeffs.Cenr * r_hat;

    Vec3 vel_dir = body_vel.normalized();
    Vec3 lateral_axis{0.0f, 1.0f, 0.0f};
    Vec3 lift_dir = vel_dir.cross(lateral_axis).normalized();
    if (lift_dir.dot(wing.upward) < 0.0f)
    {
        lift_dir = -lift_dir;
    }

    state.lift_force = lift_dir * (CL * qS);
    state.drag_force = -vel_dir * (CD * qS);
    Vec3 side_force = lateral_axis * (CY * qS);
    state.aero_moment = {Cell * qSc, Cem * qSc, Cen * qSc};

    return state;
}

} // namespace fiber_nav_physics
