// Copyright 2026 Pavel Guzenfeld — All rights reserved.
// PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
// Version: 0.0.1

#include <fiber_nav_sensors/cable_dynamics.hpp>
#include <fiber_nav_sensors/wrench_output.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>
#include <fiber_nav_sensors/msg/cable_status.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <cstdio>
#include <regex>
#include <string>

namespace fiber_nav_sensors {

class CableDynamicsNode : public rclcpp::Node {
public:
    CableDynamicsNode() : Node("cable_dynamics_node") {
        // Cable properties
        declare_parameter("world_name", "terrain_world");
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("enabled", true);
        declare_parameter("mass_per_meter", 0.003);
        declare_parameter("diameter", 0.0009);
        declare_parameter("breaking_strength", 50.0);
        declare_parameter("drag_coefficient", 1.1);
        declare_parameter("drag_shape_factor", 0.4);
        declare_parameter("spool_friction_static", 0.5);
        declare_parameter("spool_friction_kinetic", 0.02);
        declare_parameter("air_density", 1.225);
        declare_parameter("update_rate", 50.0);

        enabled_ = get_parameter("enabled").as_bool();
        world_name_ = get_parameter("world_name").as_string();
        model_name_ = get_parameter("model_name").as_string();

        props_.mass_per_meter = get_parameter("mass_per_meter").as_double();
        props_.diameter = get_parameter("diameter").as_double();
        props_.breaking_strength = get_parameter("breaking_strength").as_double();
        props_.drag_coefficient = get_parameter("drag_coefficient").as_double();
        props_.drag_shape_factor = get_parameter("drag_shape_factor").as_double();
        props_.spool_friction_static = get_parameter("spool_friction_static").as_double();
        props_.spool_friction_kinetic = get_parameter("spool_friction_kinetic").as_double();
        props_.air_density = get_parameter("air_density").as_double();

        double rate = get_parameter("update_rate").as_double();

        // Wrench output (Gazebo transport)
        wrench_output_ = create_wrench_output();
        wrench_output_->initialize(world_name_, model_name_);

        // Subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/" + model_name_ + "/odometry", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_ = *msg;
                have_odom_ = true;
            });

        spool_sub_ = create_subscription<fiber_nav_sensors::msg::SpoolStatus>(
            "/sensors/fiber_spool/status", 10,
            [this](fiber_nav_sensors::msg::SpoolStatus::ConstSharedPtr msg) {
                spool_ = *msg;
                have_spool_ = true;
            });

        // Publishers
        status_pub_ = create_publisher<fiber_nav_sensors::msg::CableStatus>(
            "/cable/status", 10);
        tension_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/cable/tension", 10);

        // Update timer
        auto period_ms = static_cast<int>(1000.0 / rate);
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&CableDynamicsNode::update, this));

        // Discover entity ID
        discover_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CableDynamicsNode::discover_entity_id, this));

        RCLCPP_INFO(get_logger(), "Cable dynamics node initialized");
        RCLCPP_INFO(get_logger(),
            "  mass=%.1f g/m, d=%.1f mm, break=%.0f N, Cd=%.1f, shape=%.1f",
            props_.mass_per_meter * 1000.0,
            props_.diameter * 1000.0,
            props_.breaking_strength,
            props_.drag_coefficient,
            props_.drag_shape_factor);

        if (!enabled_) {
            RCLCPP_WARN(get_logger(), "Cable dynamics DISABLED (enabled=false)");
        }
    }

private:
    void discover_entity_id() {
        discover_timer_->cancel();

        std::ostringstream cmd;
        cmd << "gz model -m " << model_name_ << " 2>/dev/null";

        std::array<char, 1024> buffer;
        std::string result;
        FILE* pipe = popen(cmd.str().c_str(), "r");
        if (pipe) {
            while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
                result += buffer.data();
            }
            pclose(pipe);
        }

        std::regex model_regex("Model: \\[(\\d+)\\]");
        std::smatch match;
        if (std::regex_search(result, match, model_regex)) {
            entity_id_ = std::stoi(match[1].str());
            wrench_output_->set_entity_id(entity_id_);
            RCLCPP_INFO(get_logger(), "Found model entity ID: %d", entity_id_);
        } else {
            RCLCPP_WARN(get_logger(), "Entity ID not found, retrying...");
            discover_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&CableDynamicsNode::discover_entity_id, this));
        }
    }

    void update() {
        if (!have_odom_ || !have_spool_ || entity_id_ <= 0) return;

        // Extract state from odometry (Gazebo ENU frame)
        double vx = odom_.twist.twist.linear.x;
        double vy = odom_.twist.twist.linear.y;
        double vz = odom_.twist.twist.linear.z;
        double altitude = odom_.pose.pose.position.z;  // ENU: Z = up

        double deployed = spool_.total_length;
        double payout = spool_.velocity;

        // Compute forces
        auto result = compute_cable_forces(
            props_, deployed, altitude, vx, vy, vz, payout, cable_broken_);

        // Latch broken state
        if (result.is_broken && !cable_broken_) {
            cable_broken_ = true;
            RCLCPP_ERROR(get_logger(),
                "CABLE BROKEN! Tension %.1f N exceeded breaking strength %.1f N "
                "(deployed=%.0fm, airborne=%.0fm)",
                result.tension, props_.breaking_strength,
                deployed, result.airborne_length);
        }

        // Apply wrench to drone (force only, no torque)
        if (enabled_ && !cable_broken_) {
            wrench_output_->publish(
                result.total_force.x, result.total_force.y, result.total_force.z,
                0.0, 0.0, 0.0);
        }

        // Publish status
        auto status = fiber_nav_sensors::msg::CableStatus();
        status.header.stamp = now();
        status.tension = static_cast<float>(result.tension);
        status.deployed_length = static_cast<float>(deployed);
        status.airborne_length = static_cast<float>(result.airborne_length);
        status.drag_force = static_cast<float>(result.drag_magnitude);
        status.weight_force = static_cast<float>(result.weight_magnitude);
        status.spool_friction = static_cast<float>(result.friction_magnitude);
        status.is_broken = cable_broken_;
        status_pub_->publish(status);

        // Publish scalar tension for plotting
        auto tension_msg = std_msgs::msg::Float64();
        tension_msg.data = result.tension;
        tension_pub_->publish(tension_msg);

        // Log at 1 Hz
        log_counter_++;
        if (log_counter_ % 50 == 0 && deployed > 1.0) {
            RCLCPP_INFO(get_logger(),
                "[Cable] T=%.1fN drag=%.1fN weight=%.1fN "
                "deployed=%.0fm airborne=%.0fm alt=%.0fm",
                result.tension, result.drag_magnitude, result.weight_magnitude,
                deployed, result.airborne_length, altitude);
        }
    }

    // Parameters
    CableProperties props_;
    std::string world_name_;
    std::string model_name_;
    bool enabled_{true};

    // State
    nav_msgs::msg::Odometry odom_;
    fiber_nav_sensors::msg::SpoolStatus spool_;
    bool have_odom_{false};
    bool have_spool_{false};
    int entity_id_{0};
    bool cable_broken_{false};
    int log_counter_{0};

    // I/O
    std::unique_ptr<WrenchOutput> wrench_output_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Publisher<fiber_nav_sensors::msg::CableStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tension_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr discover_timer_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::CableDynamicsNode>());
    rclcpp::shutdown();
    return 0;
}
