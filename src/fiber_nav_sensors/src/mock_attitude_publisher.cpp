#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <cmath>

namespace fiber_nav_sensors {

/// Mock attitude publisher for testing without PX4 SITL.
/// Publishes identity quaternion (level flight, heading north).
class MockAttitudePublisher : public rclcpp::Node {
public:
    MockAttitudePublisher() : Node("mock_attitude_publisher") {
        declare_parameter("publish_rate", 50.0);  // Hz
        declare_parameter("roll", 0.0);           // rad
        declare_parameter("pitch", 0.0);          // rad
        declare_parameter("yaw", 0.0);            // rad

        double rate = get_parameter("publish_rate").as_double();
        roll_ = get_parameter("roll").as_double();
        pitch_ = get_parameter("pitch").as_double();
        yaw_ = get_parameter("yaw").as_double();

        // Publisher
        attitude_pub_ = create_publisher<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", 10);

        // Timer
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&MockAttitudePublisher::publish_attitude, this));

        // Compute quaternion from Euler angles (ZYX convention)
        compute_quaternion();

        RCLCPP_INFO(get_logger(), "Mock attitude publisher initialized");
        RCLCPP_INFO(get_logger(), "  Rate: %.1f Hz", rate);
        RCLCPP_INFO(get_logger(), "  Roll/Pitch/Yaw: %.2f/%.2f/%.2f rad",
                    roll_, pitch_, yaw_);
    }

private:
    void compute_quaternion() {
        // Euler ZYX to quaternion
        double cr = std::cos(roll_ * 0.5);
        double sr = std::sin(roll_ * 0.5);
        double cp = std::cos(pitch_ * 0.5);
        double sp = std::sin(pitch_ * 0.5);
        double cy = std::cos(yaw_ * 0.5);
        double sy = std::sin(yaw_ * 0.5);

        q_w_ = cr * cp * cy + sr * sp * sy;
        q_x_ = sr * cp * cy - cr * sp * sy;
        q_y_ = cr * sp * cy + sr * cp * sy;
        q_z_ = cr * cp * sy - sr * sp * cy;
    }

    void publish_attitude() {
        auto msg = px4_msgs::msg::VehicleAttitude();

        // PX4 timestamp in microseconds
        msg.timestamp = now().nanoseconds() / 1000;
        msg.timestamp_sample = msg.timestamp;

        // Quaternion [w, x, y, z]
        msg.q[0] = static_cast<float>(q_w_);
        msg.q[1] = static_cast<float>(q_x_);
        msg.q[2] = static_cast<float>(q_y_);
        msg.q[3] = static_cast<float>(q_z_);

        // No rotation rate covariance
        msg.delta_q_reset[0] = 1.0f;
        msg.delta_q_reset[1] = 0.0f;
        msg.delta_q_reset[2] = 0.0f;
        msg.delta_q_reset[3] = 0.0f;
        msg.quat_reset_counter = 0;

        attitude_pub_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double roll_, pitch_, yaw_;
    double q_w_, q_x_, q_y_, q_z_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::MockAttitudePublisher>());
    rclcpp::shutdown();
    return 0;
}
