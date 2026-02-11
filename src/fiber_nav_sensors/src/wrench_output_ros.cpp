// Copyright 2026 Pavel Guzenfeld — All rights reserved.
// PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
// Version: 0.0.1

#include <fiber_nav_sensors/wrench_output.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace fiber_nav_sensors {

/// ROS 2 WrenchStamped publisher for non-Gazebo backends (O3DE, etc.).
/// Publishes to /model/{model_name}/wrench which can be consumed by
/// an O3DE ROS 2 Gem component or any wrench-applying bridge.
class RosWrenchOutput final : public WrenchOutput {
public:
    bool initialize(std::string const& /*world_name*/,
                    std::string const& model_name) override {
        // Create a minimal node for publishing (lifecycle managed externally)
        if (!rclcpp::ok()) return false;
        node_ = rclcpp::Node::make_shared("wrench_output_ros");
        pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/model/" + model_name + "/wrench", 10);
        model_name_ = model_name;
        return true;
    }

    void set_entity_id(int entity_id) override {
        entity_id_ = entity_id;
    }

    void publish(double fx, double fy, double fz,
                 double tx, double ty, double tz) override {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = model_name_;
        msg.wrench.force.x = fx;
        msg.wrench.force.y = fy;
        msg.wrench.force.z = fz;
        msg.wrench.torque.x = tx;
        msg.wrench.torque.y = ty;
        msg.wrench.torque.z = tz;
        pub_->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
    std::string model_name_;
    int entity_id_{0};
};

std::unique_ptr<WrenchOutput> create_wrench_output() {
    return std::make_unique<RosWrenchOutput>();
}

}  // namespace fiber_nav_sensors
