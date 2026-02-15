#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fiber_nav_sensors/gimbal_math.hpp>

namespace fiber_nav_sensors {

class GimbalControllerNode : public rclcpp::Node {
public:
    GimbalControllerNode() : Node("gimbal_controller_node") {
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("update_rate", 50.0);
        // Yaw axis (roll compensation)
        declare_parameter("gain", 1.0);
        declare_parameter("filter_tau", 0.2);
        declare_parameter("max_rate", 1.0);
        declare_parameter("max_angle", 0.8);
        // Pitch axis
        declare_parameter("pitch_gain", 1.0);
        declare_parameter("pitch_filter_tau", 0.2);
        declare_parameter("pitch_max_rate", 1.0);
        declare_parameter("pitch_max_angle", 1.7);

        auto model = get_parameter("model_name").as_string();
        double rate = get_parameter("update_rate").as_double();
        yaw_gain_ = static_cast<float>(get_parameter("gain").as_double());
        yaw_tau_ = static_cast<float>(get_parameter("filter_tau").as_double());
        yaw_max_rate_ = static_cast<float>(get_parameter("max_rate").as_double());
        yaw_max_angle_ = static_cast<float>(get_parameter("max_angle").as_double());
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

                auto g = gravityInBody(w, x, y, z);
                auto targets = gimbalTargetsNadir(g,
                    yaw_gain_, yaw_max_angle_, pitch_gain_, pitch_max_angle_);

                // Compute dt from message timestamps
                rclcpp::Time now(msg->header.stamp);
                float dt = 0.f;
                if (last_time_.nanoseconds() > 0) {
                    dt = static_cast<float>((now - last_time_).seconds());
                    dt = std::clamp(dt, 0.001f, 0.5f);
                }
                last_time_ = now;

                // Filter and rate-limit both axes
                float yaw_cmd = yaw_filter_.update(
                    targets.yaw, dt, yaw_tau_, yaw_max_rate_);
                float pitch_cmd = pitch_filter_.update(
                    targets.pitch, dt, pitch_tau_, pitch_max_rate_);

                // Publish yaw
                auto yaw_msg = std_msgs::msg::Float64();
                yaw_msg.data = yaw_cmd;
                yaw_cmd_pub_->publish(yaw_msg);

                float yaw_sat = saturationRatio(yaw_cmd, yaw_max_angle_);
                auto yaw_sat_msg = std_msgs::msg::Float64();
                yaw_sat_msg.data = yaw_sat;
                yaw_sat_pub_->publish(yaw_sat_msg);

                // Publish pitch
                auto pitch_msg = std_msgs::msg::Float64();
                pitch_msg.data = pitch_cmd;
                pitch_cmd_pub_->publish(pitch_msg);

                float pitch_sat = saturationRatio(pitch_cmd, pitch_max_angle_);
                auto pitch_sat_msg = std_msgs::msg::Float64();
                pitch_sat_msg.data = pitch_sat;
                pitch_sat_pub_->publish(pitch_sat_msg);

                // Debug: log every ~2 seconds
                if (++log_counter_ % 100 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "gx=%.2f gy=%.2f gz=%.2f yaw=%.3f(%.0f%%) pitch=%.3f(%.0f%%)",
                        g.gx, g.gy, g.gz, yaw_cmd, yaw_sat * 100.f,
                        pitch_cmd, pitch_sat * 100.f);
                }
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            []() {});

        RCLCPP_INFO(get_logger(),
            "Nadir tracker started (model=%s, roll: gain=%.1f tau=%.2f rate=%.1f "
            "max=%.2f, pitch: gain=%.1f tau=%.2f rate=%.1f max=%.2f)",
            model.c_str(), yaw_gain_, yaw_tau_, yaw_max_rate_, yaw_max_angle_,
            pitch_gain_, pitch_tau_, pitch_max_rate_, pitch_max_angle_);
    }

private:
    // Yaw axis (roll compensation)
    float yaw_gain_;
    float yaw_tau_;
    float yaw_max_rate_;
    float yaw_max_angle_;
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
