#include "fiber_nav_physics/motor_dynamics.hpp"

#include <algorithm>
#include <cstddef>

namespace fiber_nav_physics
{

void update_motor_commands(
    std::span<const MotorConfig> configs,
    std::span<MotorState> states,
    const MotorOutputs& outputs)
{
    for (size_t i = 0; i < configs.size() && i < MotorOutputs::max_motors; ++i)
    {
        float normalized = std::clamp(outputs.controls[i], 0.0f, 1.0f);
        states[i].commanded_velocity = normalized * configs[i].max_rot_velocity;
    }
}

void step_motor_dynamics(
    std::span<const MotorConfig> configs,
    std::span<MotorState> states,
    float dt)
{
    for (size_t i = 0; i < configs.size(); ++i)
    {
        auto& motor = states[i];
        float tau = (motor.commanded_velocity > motor.current_velocity)
            ? configs[i].time_constant_up
            : configs[i].time_constant_down;

        if (tau > 0.0f)
        {
            float alpha = dt / (tau + dt);
            motor.current_velocity += alpha * (motor.commanded_velocity - motor.current_velocity);
        }
        else
        {
            motor.current_velocity = motor.commanded_velocity;
        }
    }
}

ForcesTorques compute_motor_forces(
    std::span<const MotorConfig> configs,
    std::span<const MotorState> states)
{
    ForcesTorques result{};

    for (size_t i = 0; i < configs.size(); ++i)
    {
        float omega = states[i].current_velocity;
        float omega2 = omega * omega;

        // Thrust = motor_constant * omega^2 (along body +Z for tailsitter)
        float thrust = configs[i].motor_constant * omega2;
        Vec3 thrust_vec{0.0f, 0.0f, thrust};

        // Torque from arm (position cross thrust)
        Vec3 arm_torque = configs[i].position.cross(thrust_vec);

        // Reaction torque from rotor drag (yaw axis)
        float reaction = configs[i].moment_constant * thrust
                       * static_cast<float>(configs[i].direction);
        Vec3 reaction_vec{0.0f, 0.0f, reaction};

        result.force += thrust_vec;
        result.torque += arm_torque + reaction_vec;
    }

    return result;
}

} // namespace fiber_nav_physics
