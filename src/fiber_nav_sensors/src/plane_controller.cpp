#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdlib>
#include <string>
#include <sstream>
#include <regex>
#include <array>
#include <memory>

namespace fiber_nav_sensors {

/// Plane controller that applies persistent forces via Gazebo CLI.
/// This keeps the plane flying for testing the sensor fusion pipeline.
/// Automatically discovers the link entity ID at startup.
class PlaneController : public rclcpp::Node {
public:
    PlaneController() : Node("plane_controller") {
        // Parameters
        declare_parameter("thrust", 20.0);      // Forward force (N)
        declare_parameter("lift", 20.0);        // Upward force (N)
        declare_parameter("world_name", "canyon_world");
        declare_parameter("model_name", "plane");
        declare_parameter("link_name", "base_link");
        declare_parameter("update_rate", 1.0);  // Hz for checking/reapplying

        thrust_ = get_parameter("thrust").as_double();
        lift_ = get_parameter("lift").as_double();
        world_name_ = get_parameter("world_name").as_string();
        model_name_ = get_parameter("model_name").as_string();
        link_name_ = get_parameter("link_name").as_string();
        double rate = get_parameter("update_rate").as_double();

        // Service to enable/disable flight
        enable_srv_ = create_service<std_srvs::srv::SetBool>(
            "~/enable",
            std::bind(&PlaneController::enable_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Service to re-discover entity ID
        discover_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/discover",
            std::bind(&PlaneController::discover_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Plane controller initialized");
        RCLCPP_INFO(get_logger(), "  Thrust: %.1f N, Lift: %.1f N", thrust_, lift_);
        RCLCPP_INFO(get_logger(), "  Model: %s, Link: %s", model_name_.c_str(), link_name_.c_str());

        // Discover entity ID after a short delay (let Gazebo spawn the model)
        discover_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PlaneController::discover_entity_id, this));
    }

    ~PlaneController() {
        // Clear wrench on shutdown
        if (entity_id_ > 0) {
            clear_wrench();
        }
    }

private:
    void discover_entity_id() {
        // Only run once
        discover_timer_->cancel();

        RCLCPP_INFO(get_logger(), "Discovering entity ID for %s::%s...",
                    model_name_.c_str(), link_name_.c_str());

        // Run gz model command to get entity ID
        std::ostringstream cmd;
        cmd << "gz model -m " << model_name_ << " --link 2>/dev/null";

        std::array<char, 1024> buffer;
        std::string result;

        FILE* pipe = popen(cmd.str().c_str(), "r");
        if (pipe) {
            while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
                result += buffer.data();
            }
            pclose(pipe);
        }

        // Parse output to find link ID
        // Format: "- Link [45]"
        std::regex link_regex("Link \\[(\\d+)\\]");
        std::smatch match;
        if (std::regex_search(result, match, link_regex)) {
            entity_id_ = std::stoi(match[1].str());
            RCLCPP_INFO(get_logger(), "Found entity ID: %d", entity_id_);

            // Start enabled and apply wrench
            enabled_ = true;
            apply_persistent_wrench();
        } else {
            RCLCPP_ERROR(get_logger(), "Could not find entity ID. Output was: %s",
                         result.substr(0, 200).c_str());

            // Retry in 5 seconds
            discover_timer_ = create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&PlaneController::discover_entity_id, this));
        }
    }

    void enable_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (entity_id_ <= 0) {
            response->success = false;
            response->message = "Entity ID not discovered yet";
            return;
        }

        enabled_ = request->data;
        if (enabled_) {
            apply_persistent_wrench();
            response->message = "Flight enabled";
        } else {
            clear_wrench();
            response->message = "Flight disabled";
        }
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void discover_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        entity_id_ = 0;
        discover_entity_id();
        response->success = (entity_id_ > 0);
        response->message = response->success ?
            "Entity discovered: " + std::to_string(entity_id_) :
            "Discovery started, check logs";
    }

    void apply_persistent_wrench() {
        if (entity_id_ <= 0) {
            RCLCPP_WARN(get_logger(), "Cannot apply wrench: entity ID not known");
            return;
        }

        // Use gz topic to publish persistent wrench with entity ID
        std::ostringstream cmd;
        cmd << "gz topic -t /world/" << world_name_ << "/wrench/persistent "
            << "-m gz.msgs.EntityWrench "
            << "-p \"entity: {id: " << entity_id_ << ", type: 2} "
            << "wrench: {force: {x: " << thrust_ << ", z: " << lift_ << "}}\"";

        int result = std::system(cmd.str().c_str());
        if (result != 0) {
            RCLCPP_WARN(get_logger(), "Failed to apply wrench (exit code: %d)", result);
        } else {
            RCLCPP_INFO(get_logger(), "Applied persistent wrench: thrust=%.1f, lift=%.1f (entity %d)",
                        thrust_, lift_, entity_id_);
        }
    }

    void clear_wrench() {
        if (entity_id_ <= 0) return;

        // Clear all persistent wrenches on this entity
        std::ostringstream cmd;
        cmd << "gz topic -t /world/" << world_name_ << "/wrench/clear "
            << "-m gz.msgs.Entity "
            << "-p \"id: " << entity_id_ << ", type: 2\"";

        std::system(cmd.str().c_str());
        RCLCPP_INFO(get_logger(), "Cleared wrench on entity %d", entity_id_);
    }

    // Parameters
    double thrust_;
    double lift_;
    std::string world_name_;
    std::string model_name_;
    std::string link_name_;
    int entity_id_{0};
    bool enabled_{false};

    // ROS interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr discover_srv_;
    rclcpp::TimerBase::SharedPtr discover_timer_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::PlaneController>());
    rclcpp::shutdown();
    return 0;
}
