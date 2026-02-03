#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

#include <random>
#include <cmath>

namespace fiber_nav_sensors {

class SpoolSimDriver : public rclcpp::Node {
public:
    SpoolSimDriver() : Node("spool_sim_driver") {
        // Parameters
        declare_parameter("model_name", "plane");
        declare_parameter("noise_stddev", 0.1);      // m/s
        declare_parameter("slack_factor", 1.05);     // Over-payout bias
        declare_parameter("publish_rate", 50.0);     // Hz

        model_name_ = get_parameter("model_name").as_string();
        noise_stddev_ = get_parameter("noise_stddev").as_double();
        slack_factor_ = get_parameter("slack_factor").as_double();

        // Publishers
        velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/sensors/fiber_spool/velocity", 10);

        // Subscribers
        model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&SpoolSimDriver::model_states_callback, this, std::placeholders::_1));

        // Random number generator
        std::random_device rd;
        rng_ = std::mt19937(rd());
        noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);

        RCLCPP_INFO(get_logger(), "Spool sim driver initialized");
        RCLCPP_INFO(get_logger(), "  Model: %s", model_name_.c_str());
        RCLCPP_INFO(get_logger(), "  Noise stddev: %.3f m/s", noise_stddev_);
        RCLCPP_INFO(get_logger(), "  Slack factor: %.3f", slack_factor_);
    }

private:
    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        // Find our model
        auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
        if (it == msg->name.end()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Model '%s' not found in model_states", model_name_.c_str());
            return;
        }

        size_t idx = std::distance(msg->name.begin(), it);
        const auto& twist = msg->twist[idx];

        // Calculate velocity magnitude
        double vx = twist.linear.x;
        double vy = twist.linear.y;
        double vz = twist.linear.z;
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

        // Add noise
        double speed_noisy = speed + noise_dist_(rng_);

        // Apply slack factor (over-payout bias)
        double speed_measured = speed_noisy * slack_factor_;

        // Clamp to non-negative
        speed_measured = std::max(0.0, speed_measured);

        // Publish
        auto msg_out = std_msgs::msg::Float32();
        msg_out.data = static_cast<float>(speed_measured);
        velocity_pub_->publish(msg_out);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;

    // Parameters
    std::string model_name_;
    double noise_stddev_;
    double slack_factor_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::SpoolSimDriver>());
    rclcpp::shutdown();
    return 0;
}
