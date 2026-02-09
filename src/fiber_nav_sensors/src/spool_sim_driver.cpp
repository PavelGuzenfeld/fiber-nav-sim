#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>

#include <random>
#include <cmath>
#include <optional>

namespace fiber_nav_sensors {

class SpoolSimDriver : public rclcpp::Node {
public:
    SpoolSimDriver() : Node("spool_sim_driver") {
        // Parameters
        declare_parameter("odom_topic", "/model/quadtailsitter/odometry");
        declare_parameter("noise_stddev", 0.1);      // m/s
        declare_parameter("slack_factor", 1.05);     // Over-payout bias
        declare_parameter("publish_rate", 50.0);     // Hz
        declare_parameter("moving_threshold", 0.05); // m/s

        odom_topic_ = get_parameter("odom_topic").as_string();
        noise_stddev_ = get_parameter("noise_stddev").as_double();
        slack_factor_ = get_parameter("slack_factor").as_double();
        moving_threshold_ = get_parameter("moving_threshold").as_double();

        // Publishers
        velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/sensors/fiber_spool/velocity", 10);
        status_pub_ = create_publisher<fiber_nav_sensors::msg::SpoolStatus>(
            "/sensors/fiber_spool/status", 10);

        // Subscribers - listen to odometry from Gazebo bridge
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&SpoolSimDriver::odom_callback, this, std::placeholders::_1));

        // Random number generator
        std::random_device rd;
        rng_ = std::mt19937(rd());
        noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);

        RCLCPP_INFO(get_logger(), "Spool sim driver initialized");
        RCLCPP_INFO(get_logger(), "  Odom topic: %s", odom_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Noise stddev: %.3f m/s", noise_stddev_);
        RCLCPP_INFO(get_logger(), "  Slack factor: %.3f", slack_factor_);
        RCLCPP_INFO(get_logger(), "  Moving threshold: %.3f m/s", moving_threshold_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& twist = msg->twist.twist;

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

        // Accumulate total length
        auto current_time = rclcpp::Time(msg->header.stamp);
        if (last_odom_time_.has_value()) {
            double dt = (current_time - *last_odom_time_).seconds();
            if (dt > 0.0 && dt < 1.0) {
                total_unrolled_length_ += speed_measured * dt;
            }
        }
        last_odom_time_ = current_time;

        // Publish velocity (backward compatible)
        auto msg_out = std_msgs::msg::Float32();
        msg_out.data = static_cast<float>(speed_measured);
        velocity_pub_->publish(msg_out);

        // Publish status
        auto status = fiber_nav_sensors::msg::SpoolStatus();
        status.header.stamp = current_time;
        status.velocity = static_cast<float>(speed_measured);
        status.total_length = static_cast<float>(total_unrolled_length_);
        status.is_moving = (speed_measured > moving_threshold_);
        status_pub_->publish(status);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Publisher<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Parameters
    std::string odom_topic_;
    double noise_stddev_;
    double slack_factor_;
    double moving_threshold_;

    // State
    double total_unrolled_length_{0.0};
    std::optional<rclcpp::Time> last_odom_time_;

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
