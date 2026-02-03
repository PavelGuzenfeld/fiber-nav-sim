#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <random>
#include <cmath>

namespace fiber_nav_sensors {

class VisionDirectionSim : public rclcpp::Node {
public:
    VisionDirectionSim() : Node("vision_direction_sim") {
        // Parameters
        declare_parameter("odom_topic", "/model/plane/odometry");
        declare_parameter("drift_rate", 0.001);      // rad/s random walk
        declare_parameter("min_velocity", 0.5);      // m/s - below this, direction is unreliable

        odom_topic_ = get_parameter("odom_topic").as_string();
        drift_rate_ = get_parameter("drift_rate").as_double();
        min_velocity_ = get_parameter("min_velocity").as_double();

        // Publishers
        direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10);

        // Subscribers - listen to odometry from Gazebo bridge
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&VisionDirectionSim::odom_callback, this, std::placeholders::_1));

        // Random number generator for drift
        std::random_device rd;
        rng_ = std::mt19937(rd());
        drift_dist_ = std::normal_distribution<double>(0.0, drift_rate_);

        // Initialize drift angles
        drift_yaw_ = 0.0;
        drift_pitch_ = 0.0;

        last_time_ = now();

        RCLCPP_INFO(get_logger(), "Vision direction sim initialized");
        RCLCPP_INFO(get_logger(), "  Odom topic: %s", odom_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Drift rate: %.4f rad/s", drift_rate_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& twist = msg->twist.twist;

        double vx = twist.linear.x;
        double vy = twist.linear.y;
        double vz = twist.linear.z;
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

        // Don't publish if velocity is too low (direction undefined)
        if (speed < min_velocity_) {
            return;
        }

        // Normalize to unit vector
        double ux = vx / speed;
        double uy = vy / speed;
        double uz = vz / speed;

        // Apply random walk drift
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        drift_yaw_ += drift_dist_(rng_) * std::sqrt(dt);
        drift_pitch_ += drift_dist_(rng_) * std::sqrt(dt);

        // Apply drift rotation (simplified - rotate around z then y)
        double cos_yaw = std::cos(drift_yaw_);
        double sin_yaw = std::sin(drift_yaw_);
        double cos_pitch = std::cos(drift_pitch_);
        double sin_pitch = std::sin(drift_pitch_);

        // Rotate by yaw (around z)
        double ux_yaw = cos_yaw * ux - sin_yaw * uy;
        double uy_yaw = sin_yaw * ux + cos_yaw * uy;
        double uz_yaw = uz;

        // Rotate by pitch (around y)
        double ux_final = cos_pitch * ux_yaw + sin_pitch * uz_yaw;
        double uy_final = uy_yaw;
        double uz_final = -sin_pitch * ux_yaw + cos_pitch * uz_yaw;

        // Re-normalize (rotation should preserve norm, but floating point...)
        double norm = std::sqrt(ux_final*ux_final + uy_final*uy_final + uz_final*uz_final);
        ux_final /= norm;
        uy_final /= norm;
        uz_final /= norm;

        // Publish
        auto msg_out = geometry_msgs::msg::Vector3Stamped();
        msg_out.header.stamp = current_time;
        msg_out.header.frame_id = "base_link";  // Body frame
        msg_out.vector.x = ux_final;
        msg_out.vector.y = uy_final;
        msg_out.vector.z = uz_final;
        direction_pub_->publish(msg_out);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Parameters
    std::string odom_topic_;
    double drift_rate_;
    double min_velocity_;

    // Drift state
    double drift_yaw_;
    double drift_pitch_;
    rclcpp::Time last_time_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> drift_dist_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::VisionDirectionSim>());
    rclcpp::shutdown();
    return 0;
}
