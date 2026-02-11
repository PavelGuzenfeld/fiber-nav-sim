// Copyright 2026 Pavel Guzenfeld — All rights reserved.
// PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
// Version: 0.0.1

#pragma once

#include <memory>
#include <string>

namespace fiber_nav_sensors {

/// Abstract interface for applying force/torque to a simulation entity.
/// Implementations exist for Gazebo (gz::transport) and ROS 2 (WrenchStamped).
class WrenchOutput {
public:
    virtual ~WrenchOutput() = default;

    /// Initialize the output channel.
    /// @param world_name  Simulation world name (used by Gazebo impl)
    /// @param model_name  Model to apply wrench to
    /// @return true on success
    virtual bool initialize(std::string const& world_name,
                            std::string const& model_name) = 0;

    /// Set the entity ID (model ID in physics engine).
    virtual void set_entity_id(int entity_id) = 0;

    /// Publish a one-time wrench (force + torque in world frame).
    virtual void publish(double fx, double fy, double fz,
                         double tx, double ty, double tz) = 0;
};

/// Factory: create the compile-time-selected WrenchOutput implementation.
std::unique_ptr<WrenchOutput> create_wrench_output();

}  // namespace fiber_nav_sensors
