#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <optional>
#include <cmath>

namespace fiber_nav_fusion {

class FiberVisionFusion : public rclcpp::Node {
public:
    FiberVisionFusion() : Node("fiber_vision_fusion") {
        // Parameters
        declare_parameter("slack_factor", 1.05);     // Known/estimated slack
        declare_parameter("publish_rate", 50.0);     // Hz
        declare_parameter("max_data_age", 0.1);      // seconds

        slack_factor_ = get_parameter("slack_factor").as_double();
        max_data_age_ = get_parameter("max_data_age").as_double();
        double rate = get_parameter("publish_rate").as_double();

        // Publishers - PX4 visual odometry input
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        // Subscribers
        spool_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/sensors/fiber_spool/velocity", 10,
            std::bind(&FiberVisionFusion::spool_callback, this, std::placeholders::_1));

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10,
            std::bind(&FiberVisionFusion::direction_callback, this, std::placeholders::_1));

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", 10,
            std::bind(&FiberVisionFusion::attitude_callback, this, std::placeholders::_1));

        // Timer for fusion
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&FiberVisionFusion::fusion_callback, this));

        RCLCPP_INFO(get_logger(), "Fiber vision fusion initialized");
        RCLCPP_INFO(get_logger(), "  Slack factor: %.3f", slack_factor_);
        RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", rate);
    }

private:
    void spool_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spool_velocity_ = msg->data;
        spool_time_ = now();
    }

    void direction_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        direction_x_ = msg->vector.x;
        direction_y_ = msg->vector.y;
        direction_z_ = msg->vector.z;
        direction_time_ = now();
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // PX4 quaternion: [w, x, y, z]
        attitude_q_.setW(msg->q[0]);
        attitude_q_.setX(msg->q[1]);
        attitude_q_.setY(msg->q[2]);
        attitude_q_.setZ(msg->q[3]);
        attitude_time_ = now();
        has_attitude_ = true;
    }

    void fusion_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        auto current_time = now();

        // Check data freshness
        if (!spool_time_ || (current_time - *spool_time_).seconds() > max_data_age_) {
            return;  // Spool data stale
        }
        if (!direction_time_ || (current_time - *direction_time_).seconds() > max_data_age_) {
            return;  // Direction data stale
        }
        if (!has_attitude_ || !attitude_time_ ||
            (current_time - *attitude_time_).seconds() > max_data_age_) {
            return;  // Attitude data stale
        }

        // Reconstruct velocity in body frame
        // v_body = (spool_velocity / slack_factor) * direction_unit_vector
        double corrected_speed = spool_velocity_ / slack_factor_;
        double vx_body = corrected_speed * direction_x_;
        double vy_body = corrected_speed * direction_y_;
        double vz_body = corrected_speed * direction_z_;

        // Rotate from body frame to NED frame using attitude quaternion
        tf2::Vector3 v_body(vx_body, vy_body, vz_body);
        tf2::Vector3 v_ned = tf2::quatRotate(attitude_q_, v_body);

        // Create odometry message
        auto odom_msg = px4_msgs::msg::VehicleOdometry();
        odom_msg.timestamp = current_time.nanoseconds() / 1000;  // PX4 uses microseconds
        odom_msg.timestamp_sample = odom_msg.timestamp;

        // Position unknown (NaN)
        odom_msg.position[0] = std::nan("");
        odom_msg.position[1] = std::nan("");
        odom_msg.position[2] = std::nan("");

        // Velocity in NED frame
        odom_msg.velocity[0] = static_cast<float>(v_ned.x());
        odom_msg.velocity[1] = static_cast<float>(v_ned.y());
        odom_msg.velocity[2] = static_cast<float>(v_ned.z());

        // Attitude unknown (let EKF use its own estimate)
        odom_msg.q[0] = std::nan("");

        // Angular velocity unknown
        odom_msg.angular_velocity[0] = std::nan("");
        odom_msg.angular_velocity[1] = std::nan("");
        odom_msg.angular_velocity[2] = std::nan("");

        // Frame: NED, local frame
        odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        // Covariance (diagonal - velocity only)
        // Position variance infinite (unknown)
        odom_msg.position_variance[0] = 1e6f;
        odom_msg.position_variance[1] = 1e6f;
        odom_msg.position_variance[2] = 1e6f;

        // Velocity variance based on sensor noise
        float vel_var = 0.1f * 0.1f;  // σ² from spool noise
        odom_msg.velocity_variance[0] = vel_var;
        odom_msg.velocity_variance[1] = vel_var;
        odom_msg.velocity_variance[2] = vel_var;

        odom_pub_->publish(odom_msg);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double slack_factor_;
    double max_data_age_;

    // Data storage (protected by mutex)
    std::mutex data_mutex_;
    double spool_velocity_{0.0};
    double direction_x_{1.0}, direction_y_{0.0}, direction_z_{0.0};
    tf2::Quaternion attitude_q_;
    bool has_attitude_{false};

    std::optional<rclcpp::Time> spool_time_;
    std::optional<rclcpp::Time> direction_time_;
    std::optional<rclcpp::Time> attitude_time_;
};

}  // namespace fiber_nav_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_fusion::FiberVisionFusion>());
    rclcpp::shutdown();
    return 0;
}
