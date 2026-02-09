#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>

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
        declare_parameter("zupt_threshold", 0.05);   // m/s
        declare_parameter("zupt_velocity_variance", 0.001);
        declare_parameter("enable_position_clamping", true);
        declare_parameter("k_drag", 0.0005);
        declare_parameter("tunnel_heading_deg", 90.0);
        declare_parameter("position_variance_longitudinal", 1.0);
        declare_parameter("position_variance_lateral", 100.0);

        slack_factor_ = get_parameter("slack_factor").as_double();
        max_data_age_ = get_parameter("max_data_age").as_double();
        double rate = get_parameter("publish_rate").as_double();
        zupt_threshold_ = get_parameter("zupt_threshold").as_double();
        zupt_velocity_variance_ = get_parameter("zupt_velocity_variance").as_double();
        enable_position_clamping_ = get_parameter("enable_position_clamping").as_bool();
        k_drag_ = get_parameter("k_drag").as_double();
        tunnel_heading_deg_ = get_parameter("tunnel_heading_deg").as_double();
        position_variance_longitudinal_ = get_parameter("position_variance_longitudinal").as_double();
        position_variance_lateral_ = get_parameter("position_variance_lateral").as_double();

        // Publishers - PX4 visual odometry input
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        // Subscribers
        spool_sub_ = create_subscription<fiber_nav_sensors::msg::SpoolStatus>(
            "/sensors/fiber_spool/status", 10,
            std::bind(&FiberVisionFusion::spool_callback, this, std::placeholders::_1));

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10,
            std::bind(&FiberVisionFusion::direction_callback, this, std::placeholders::_1));

        // PX4 publishes with BEST_EFFORT QoS — must match for compatibility
        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();
        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4_qos,
            std::bind(&FiberVisionFusion::attitude_callback, this, std::placeholders::_1));

        lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4_qos,
            std::bind(&FiberVisionFusion::lpos_callback, this, std::placeholders::_1));

        // Timer for fusion
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&FiberVisionFusion::fusion_callback, this));

        RCLCPP_INFO(get_logger(), "Fiber vision fusion initialized");
        RCLCPP_INFO(get_logger(), "  Slack factor: %.3f", slack_factor_);
        RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", rate);
        RCLCPP_INFO(get_logger(), "  ZUPT threshold: %.3f m/s", zupt_threshold_);
        RCLCPP_INFO(get_logger(), "  Position clamping: %s", enable_position_clamping_ ? "enabled" : "disabled");
        if (enable_position_clamping_) {
            RCLCPP_INFO(get_logger(), "  Tunnel heading: %.1f deg", tunnel_heading_deg_);
            RCLCPP_INFO(get_logger(), "  k_drag: %.4f", k_drag_);
        }
    }

private:
    void spool_callback(const fiber_nav_sensors::msg::SpoolStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spool_velocity_ = msg->velocity;
        spool_total_length_ = msg->total_length;
        spool_is_moving_ = msg->is_moving;
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

    void lpos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ekf_x_ = msg->x;
        ekf_y_ = msg->y;
        ekf_z_ = msg->z;
        has_ekf_z_ = true;
    }

    void fusion_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        auto current_time = now();

        // Spool data is always required
        if (!spool_time_ || (current_time - *spool_time_).seconds() > max_data_age_) {
            return;  // Spool data stale
        }

        // Attitude is always required (for frame reference even during ZUPT)
        if (!has_attitude_ || !attitude_time_ ||
            (current_time - *attitude_time_).seconds() > max_data_age_) {
            return;  // Attitude data stale
        }

        // ZUPT check: when stopped, we don't need vision direction
        bool zupt_active = (spool_velocity_ < zupt_threshold_);

        // Vision direction required only for normal (non-ZUPT) fusion
        bool direction_fresh = direction_time_.has_value() &&
            (current_time - *direction_time_).seconds() <= max_data_age_;
        if (!zupt_active && !direction_fresh) {
            return;  // Direction data stale and not in ZUPT mode
        }

        // Create odometry message
        auto odom_msg = px4_msgs::msg::VehicleOdometry();
        odom_msg.timestamp = current_time.nanoseconds() / 1000;  // PX4 uses microseconds
        odom_msg.timestamp_sample = odom_msg.timestamp;

        // Velocity: ZUPT or normal fusion
        if (zupt_active) {
            odom_msg.velocity[0] = 0.0f;
            odom_msg.velocity[1] = 0.0f;
            odom_msg.velocity[2] = 0.0f;
            float zv = static_cast<float>(zupt_velocity_variance_);
            odom_msg.velocity_variance[0] = zv;
            odom_msg.velocity_variance[1] = zv;
            odom_msg.velocity_variance[2] = zv;
        } else {
            // Reconstruct velocity in body frame
            double corrected_speed = spool_velocity_ / slack_factor_;
            double vx_body = corrected_speed * direction_x_;
            double vy_body = corrected_speed * direction_y_;
            double vz_body = corrected_speed * direction_z_;

            // Rotate from body frame to NED frame using attitude quaternion
            tf2::Vector3 v_body(vx_body, vy_body, vz_body);
            tf2::Vector3 v_ned = tf2::quatRotate(attitude_q_, v_body);

            odom_msg.velocity[0] = static_cast<float>(v_ned.x());
            odom_msg.velocity[1] = static_cast<float>(v_ned.y());
            odom_msg.velocity[2] = static_cast<float>(v_ned.z());

            float vel_var = 0.1f * 0.1f;  // σ² from spool noise
            odom_msg.velocity_variance[0] = vel_var;
            odom_msg.velocity_variance[1] = vel_var;
            odom_msg.velocity_variance[2] = vel_var;
        }

        // Position: Echo EKF's own estimate back with very high variance.
        // This keeps EV position "active" in PX4 without any disagreement
        // (zero innovation). When GPS is disabled, EV velocity provides the
        // velocity constraint, and this echo prevents unbounded position drift
        // by giving the EKF a weak self-consistent position reference.
        // When the spool is moving (GPS-denied zone), the drag bow model
        // provides actual position corrections along the tunnel heading.
        if (has_ekf_z_) {
            if (enable_position_clamping_ && spool_is_moving_) {
                // Spool moving: use drag bow model for horizontal position
                double v = spool_velocity_;
                double x_est = spool_total_length_ * (1.0 - k_drag_ * v * v);
                double heading_rad = tunnel_heading_deg_ * M_PI / 180.0;
                odom_msg.position[0] = static_cast<float>(x_est * std::cos(heading_rad));
                odom_msg.position[1] = static_cast<float>(x_est * std::sin(heading_rad));
                odom_msg.position[2] = ekf_z_;
                float pos_var = static_cast<float>(position_variance_lateral_);
                odom_msg.position_variance[0] = pos_var;
                odom_msg.position_variance[1] = pos_var;
                odom_msg.position_variance[2] = 1e6f;
            } else {
                // Echo EKF position back — zero innovation, keeps EV active
                odom_msg.position[0] = ekf_x_;
                odom_msg.position[1] = ekf_y_;
                odom_msg.position[2] = ekf_z_;
                odom_msg.position_variance[0] = 1e6f;
                odom_msg.position_variance[1] = 1e6f;
                odom_msg.position_variance[2] = 1e6f;
            }
        } else {
            // No EKF data yet — NaN to avoid any position constraint
            odom_msg.position[0] = std::nanf("");
            odom_msg.position[1] = std::nanf("");
            odom_msg.position[2] = std::nanf("");
        }

        // Attitude - NaN = no attitude data from this source
        odom_msg.q[0] = std::nanf("");
        odom_msg.q[1] = std::nanf("");
        odom_msg.q[2] = std::nanf("");
        odom_msg.q[3] = std::nanf("");

        // Angular velocity - NaN = no angular velocity data
        odom_msg.angular_velocity[0] = std::nanf("");
        odom_msg.angular_velocity[1] = std::nanf("");
        odom_msg.angular_velocity[2] = std::nanf("");

        // Frame: NED, local frame
        odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        // Quality (0-255, higher is better) - required for PX4 EKF to accept data
        odom_msg.quality = 100;

        odom_pub_->publish(odom_msg);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double slack_factor_;
    double max_data_age_;
    double zupt_threshold_;
    double zupt_velocity_variance_;
    bool enable_position_clamping_;
    double k_drag_;
    double tunnel_heading_deg_;
    double position_variance_longitudinal_;
    double position_variance_lateral_;

    // Data storage (protected by mutex)
    std::mutex data_mutex_;
    double spool_velocity_{0.0};
    double spool_total_length_{0.0};
    bool spool_is_moving_{false};
    double direction_x_{1.0}, direction_y_{0.0}, direction_z_{0.0};
    tf2::Quaternion attitude_q_;
    bool has_attitude_{false};
    float ekf_x_{0.0f}, ekf_y_{0.0f}, ekf_z_{0.0f};
    bool has_ekf_z_{false};

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
