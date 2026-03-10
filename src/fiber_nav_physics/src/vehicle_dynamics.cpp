#include "fiber_nav_physics/vehicle_dynamics.hpp"

namespace fiber_nav_physics
{

void VehicleDynamics::init(const VehicleConfig& config, const VehicleState& initial_state)
{
    config_ = config;
    state_ = initial_state;
    motor_states_.resize(config_.motors.size());
}

void VehicleDynamics::set_motor_outputs(const MotorOutputs& outputs)
{
    update_motor_commands(config_.motors, motor_states_, outputs);
}

void VehicleDynamics::step(float dt, Vec3 external_force, Vec3 external_torque)
{
    // 1. Step motor dynamics
    step_motor_dynamics(config_.motors, motor_states_, dt);

    // 2. Compute motor forces (body frame)
    auto motor_ft = compute_motor_forces(config_.motors, motor_states_);

    // 3. Compute body-frame velocity for aerodynamics
    Quat rot_inv = state_.orientation.conjugate();
    Vec3 body_vel = rot_inv.rotate(state_.velocity);
    Vec3 body_ang_vel = rot_inv.rotate(state_.angular_velocity);

    // 4. Compute aerodynamic forces (body frame)
    aero_state_ = config_.aero.compute(body_vel, body_ang_vel);
    Vec3 aero_force = aero_state_.lift_force + aero_state_.drag_force;
    Vec3 aero_torque = aero_state_.aero_moment;

    // 5. Update VTOL transition state
    config_.vtol.update(aero_state_.airspeed, dt);

    // 6. Sum all body-frame forces
    Vec3 total_body_force = motor_ft.force + aero_force + external_force;
    Vec3 total_body_torque = motor_ft.torque + aero_torque + external_torque;

    // 7. Transform to NED world frame
    Vec3 world_force = state_.orientation.rotate(total_body_force);
    Vec3 world_torque = state_.orientation.rotate(total_body_torque);

    // 8. Add gravity (NED: z-down = positive)
    constexpr float g = 9.81f;
    world_force.z += config_.mass * g;

    // 9. Semi-implicit Euler integration
    // Linear: a = F/m, v += a*dt, p += v*dt
    Vec3 accel = world_force / config_.mass;
    state_.velocity += accel * dt;
    state_.position += state_.velocity * dt;

    // Angular: alpha = tau/I (diagonal inertia), omega += alpha*dt
    // Convert world torque back to body for inertia application
    Vec3 body_total_torque = rot_inv.rotate(world_torque);
    Vec3 ang_accel{
        (config_.inertia.x > 0.0f) ? body_total_torque.x / config_.inertia.x : 0.0f,
        (config_.inertia.y > 0.0f) ? body_total_torque.y / config_.inertia.y : 0.0f,
        (config_.inertia.z > 0.0f) ? body_total_torque.z / config_.inertia.z : 0.0f,
    };

    // Update angular velocity (body frame → world frame for storage)
    Vec3 body_ang_accel_world = state_.orientation.rotate(ang_accel);
    state_.angular_velocity += body_ang_accel_world * dt;

    // 10. Integrate orientation (quaternion derivative)
    // q_dot = 0.5 * q * omega_body_quat
    Vec3 omega_body = rot_inv.rotate(state_.angular_velocity);
    Quat omega_quat{0.0f, omega_body.x, omega_body.y, omega_body.z};
    Quat q_dot = state_.orientation * omega_quat;
    state_.orientation.w += 0.5f * q_dot.w * dt;
    state_.orientation.x += 0.5f * q_dot.x * dt;
    state_.orientation.y += 0.5f * q_dot.y * dt;
    state_.orientation.z += 0.5f * q_dot.z * dt;
    state_.orientation = state_.orientation.normalized();

    // 11. Ground clamp: prevent falling below ground (NED: z > 0 = below ground)
    if (state_.position.z > 0.0f)
    {
        state_.position.z = 0.0f;
        if (state_.velocity.z > 0.0f)
        {
            state_.velocity.z = 0.0f;
        }
    }
}

} // namespace fiber_nav_physics
