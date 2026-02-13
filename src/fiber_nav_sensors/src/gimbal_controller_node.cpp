#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>

namespace fiber_nav_sensors {

class GimbalControllerNode : public rclcpp::Node {
public:
    GimbalControllerNode() : Node("gimbal_controller_node") {
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("update_rate", 50.0);
        declare_parameter("gain", 1.0);
        declare_parameter("filter_tau", 1.0);
        declare_parameter("max_rate", 0.3);

        auto model = get_parameter("model_name").as_string();
        double rate = get_parameter("update_rate").as_double();
        gain_ = static_cast<float>(get_parameter("gain").as_double());
        filter_tau_ = static_cast<float>(get_parameter("filter_tau").as_double());
        max_rate_ = static_cast<float>(get_parameter("max_rate").as_double());

        cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/cmd_pos", 10);

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

                // SDF tailsitter body: Z=nose, X=belly(down when sitting).
                // In hover (nose up): body Z≈up, body X≈horizontal → gx≈0
                // In FW flight: body Z≈forward, body X≈down → gx≈1
                //
                // Gimbal axis = SDF Z (nose). Camera points along SDF X when angle=0.
                // To keep camera nadir, correction angle = -atan2(gy, gx).
                //
                // Only compensate when body is roughly horizontal (FW flight):
                // gx > 0.5 means gravity has a large component along body X (down).
                float target = 0.f;

                if (gx > 0.5f) {
                    float angle = std::atan2(gy, gx);
                    target = std::clamp(-gain_ * angle, -0.5f, 0.5f);
                }

                // Compute dt from message timestamps
                rclcpp::Time now(msg->header.stamp);
                float dt = 0.f;
                if (last_time_.nanoseconds() > 0) {
                    dt = static_cast<float>((now - last_time_).seconds());
                    dt = std::clamp(dt, 0.001f, 0.5f);
                }
                last_time_ = now;

                // Low-pass filter: smoothly approach target
                if (dt > 0.f) {
                    float alpha = dt / (filter_tau_ + dt);
                    filtered_cmd_ += alpha * (target - filtered_cmd_);

                    // Rate limit: max change per second
                    float max_delta = max_rate_ * dt;
                    float delta = filtered_cmd_ - prev_cmd_;
                    if (std::abs(delta) > max_delta) {
                        filtered_cmd_ = prev_cmd_ + std::copysign(max_delta, delta);
                    }
                } else {
                    filtered_cmd_ = target;
                }
                prev_cmd_ = filtered_cmd_;

                auto cmd_msg = std_msgs::msg::Float64();
                cmd_msg.data = filtered_cmd_;

                // Debug: log every ~2 seconds
                if (++log_counter_ % 20 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "gx=%.2f gy=%.2f tgt=%.3f cmd=%.3f",
                        gx, gy, target, filtered_cmd_);
                }

                cmd_pub_->publish(cmd_msg);
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            []() {});

        RCLCPP_INFO(get_logger(),
            "Gimbal controller started (model=%s, gain=%.1f, tau=%.1fs, max_rate=%.1f)",
            model.c_str(), gain_, filter_tau_, max_rate_);
    }

private:
    float gain_;
    float filter_tau_;
    float max_rate_;
    float filtered_cmd_ = 0.f;
    float prev_cmd_ = 0.f;
    int log_counter_ = 0;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
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
