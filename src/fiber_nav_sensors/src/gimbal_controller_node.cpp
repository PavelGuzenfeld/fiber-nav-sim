#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>

namespace fiber_nav_sensors {

/// Single-axis gimbal filter: low-pass + rate limiter.
struct AxisFilter {
    float filtered = 0.f;
    float prev = 0.f;

    float update(float target, float dt, float tau, float max_rate) {
        if (dt > 0.f) {
            float alpha = dt / (tau + dt);
            filtered += alpha * (target - filtered);

            float max_delta = max_rate * dt;
            float delta = filtered - prev;
            if (std::abs(delta) > max_delta) {
                filtered = prev + std::copysign(max_delta, delta);
            }
        } else {
            filtered = target;
        }
        prev = filtered;
        return filtered;
    }
};

class GimbalControllerNode : public rclcpp::Node {
public:
    GimbalControllerNode() : Node("gimbal_controller_node") {
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("update_rate", 50.0);
        // Yaw axis params
        declare_parameter("gain", 1.0);
        declare_parameter("filter_tau", 0.3);
        declare_parameter("max_rate", 1.0);
        declare_parameter("max_angle", 0.7);
        declare_parameter("gx_threshold", 0.3);
        // Pitch axis params (same defaults as yaw)
        declare_parameter("pitch_gain", 1.0);
        declare_parameter("pitch_filter_tau", 0.3);
        declare_parameter("pitch_max_rate", 1.0);
        declare_parameter("pitch_max_angle", 0.7);

        auto model = get_parameter("model_name").as_string();
        double rate = get_parameter("update_rate").as_double();
        yaw_gain_ = static_cast<float>(get_parameter("gain").as_double());
        yaw_tau_ = static_cast<float>(get_parameter("filter_tau").as_double());
        yaw_max_rate_ = static_cast<float>(get_parameter("max_rate").as_double());
        yaw_max_angle_ = static_cast<float>(get_parameter("max_angle").as_double());
        gx_threshold_ = static_cast<float>(get_parameter("gx_threshold").as_double());
        pitch_gain_ = static_cast<float>(get_parameter("pitch_gain").as_double());
        pitch_tau_ = static_cast<float>(get_parameter("pitch_filter_tau").as_double());
        pitch_max_rate_ = static_cast<float>(get_parameter("pitch_max_rate").as_double());
        pitch_max_angle_ = static_cast<float>(get_parameter("pitch_max_angle").as_double());

        // Yaw axis publishers
        yaw_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/cmd_pos", 10);
        yaw_sat_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/saturation", 10);
        // Pitch axis publishers
        pitch_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/pitch_cmd_pos", 10);
        pitch_sat_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/pitch_saturation", 10);

        // Use Gazebo odometry — its quaternion uses the actual SDF body frame
        // (not PX4's virtual frame which has a 90° rotation for tailsitters).
        // SDF body frame: Z=nose(forward), X=belly(down when sitting).
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/quadtailsitter/odometry", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                auto& q = msg->pose.pose.orientation;
                float w = static_cast<float>(q.w);
                float x = static_cast<float>(q.x);
                float y = static_cast<float>(q.y);
                float z = static_cast<float>(q.z);

                // Gravity vector in SDF body frame.
                // Gazebo ENU: gravity = [0, 0, -1] in world frame.
                // Odometry quaternion = body→world (R_bw).
                // g_body = R_bw^T * [0,0,-1] = -(row 3 of R_bw).
                float gx = 2.f * (w * y - x * z);
                float gy = -2.f * (y * z + w * x);
                float gz = 2.f * (x * x + y * y) - 1.f;

                // SDF tailsitter body: Z=nose, X=belly(down when sitting).
                // In hover (nose up): body Z≈up, body X≈horizontal → gx≈0
                // In FW flight: body Z≈forward, body X≈down → gx≈1
                //
                // Yaw gimbal axis = SDF Z (nose). Camera points along SDF X.
                // To keep camera nadir, yaw correction = -atan2(gy, gx).
                //
                // Pitch gimbal axis = SDF Y (lateral). Camera points along SDF X.
                // Pitch correction = -atan2(gz, gx): compensates nose-up/down tilt.
                //
                // Only compensate when body is roughly horizontal (FW flight):
                // gx > threshold means gravity has large component along body X (down).
                float yaw_target = 0.f;
                float pitch_target = 0.f;

                if (gx > gx_threshold_) {
                    yaw_target = std::clamp(
                        -yaw_gain_ * std::atan2(gy, gx), -yaw_max_angle_, yaw_max_angle_);
                    pitch_target = std::clamp(
                        -pitch_gain_ * std::atan2(gz, gx), -pitch_max_angle_, pitch_max_angle_);
                }

                // Compute dt from message timestamps
                rclcpp::Time now(msg->header.stamp);
                float dt = 0.f;
                if (last_time_.nanoseconds() > 0) {
                    dt = static_cast<float>((now - last_time_).seconds());
                    dt = std::clamp(dt, 0.001f, 0.5f);
                }
                last_time_ = now;

                // Filter and rate-limit both axes
                float yaw_cmd = yaw_filter_.update(yaw_target, dt, yaw_tau_, yaw_max_rate_);
                float pitch_cmd = pitch_filter_.update(pitch_target, dt, pitch_tau_, pitch_max_rate_);

                // Publish yaw
                auto yaw_msg = std_msgs::msg::Float64();
                yaw_msg.data = yaw_cmd;
                yaw_cmd_pub_->publish(yaw_msg);

                float yaw_sat = (yaw_max_angle_ > 0.f)
                    ? std::abs(yaw_cmd) / yaw_max_angle_ : 0.f;
                auto yaw_sat_msg = std_msgs::msg::Float64();
                yaw_sat_msg.data = yaw_sat;
                yaw_sat_pub_->publish(yaw_sat_msg);

                // Publish pitch
                auto pitch_msg = std_msgs::msg::Float64();
                pitch_msg.data = pitch_cmd;
                pitch_cmd_pub_->publish(pitch_msg);

                float pitch_sat = (pitch_max_angle_ > 0.f)
                    ? std::abs(pitch_cmd) / pitch_max_angle_ : 0.f;
                auto pitch_sat_msg = std_msgs::msg::Float64();
                pitch_sat_msg.data = pitch_sat;
                pitch_sat_pub_->publish(pitch_sat_msg);

                // Debug: log every ~2 seconds
                if (++log_counter_ % 20 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "gx=%.2f gy=%.2f gz=%.2f yaw=%.3f(%.0f%%) pitch=%.3f(%.0f%%)",
                        gx, gy, gz, yaw_cmd, yaw_sat * 100.f,
                        pitch_cmd, pitch_sat * 100.f);
                }
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            []() {});

        RCLCPP_INFO(get_logger(),
            "Gimbal controller started (model=%s, yaw: gain=%.1f tau=%.2f max_rate=%.1f "
            "max_angle=%.2f, pitch: gain=%.1f tau=%.2f max_rate=%.1f max_angle=%.2f, "
            "gx_thresh=%.2f)",
            model.c_str(), yaw_gain_, yaw_tau_, yaw_max_rate_, yaw_max_angle_,
            pitch_gain_, pitch_tau_, pitch_max_rate_, pitch_max_angle_, gx_threshold_);
    }

private:
    // Yaw axis
    float yaw_gain_;
    float yaw_tau_;
    float yaw_max_rate_;
    float yaw_max_angle_;
    float gx_threshold_;
    AxisFilter yaw_filter_;

    // Pitch axis
    float pitch_gain_;
    float pitch_tau_;
    float pitch_max_rate_;
    float pitch_max_angle_;
    AxisFilter pitch_filter_;

    int log_counter_ = 0;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_sat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_sat_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::GimbalControllerNode>());
    rclcpp::shutdown();
    return 0;
}
