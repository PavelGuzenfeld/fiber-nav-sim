#include <fiber_nav_fusion/position_ekf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>
#include <fiber_nav_sensors/msg/cable_status.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <optional>
#include <cmath>
#include <sstream>

namespace fiber_nav_fusion {

class PositionEkfNode : public rclcpp::Node {
public:
    PositionEkfNode() : Node("position_ekf_node") {
        declare_parameter("enabled", false);
        declare_parameter("publish_rate", 50.0);
        declare_parameter("q_pos", 0.1);
        declare_parameter("q_vel", 1.0);
        declare_parameter("q_wind", 0.01);
        declare_parameter("r_velocity", 0.5);
        declare_parameter("r_speed", 0.1);
        declare_parameter("of_quality_min", 0.3);
        declare_parameter("of_quality_scale", 5.0);
        declare_parameter("cable_margin", 0.95);
        declare_parameter("p0_pos", 1.0);
        declare_parameter("p0_vel", 1.0);
        declare_parameter("p0_wind", 4.0);
        declare_parameter("feed_px4", true);
        declare_parameter("position_variance_to_px4", 10.0);

        enabled_ = get_parameter("enabled").as_bool();
        double rate = get_parameter("publish_rate").as_double();
        feed_px4_ = get_parameter("feed_px4").as_bool();
        position_variance_to_px4_ = static_cast<float>(
            get_parameter("position_variance_to_px4").as_double());

        ekf_config_.q_pos = static_cast<float>(get_parameter("q_pos").as_double());
        ekf_config_.q_vel = static_cast<float>(get_parameter("q_vel").as_double());
        ekf_config_.q_wind = static_cast<float>(get_parameter("q_wind").as_double());
        ekf_config_.r_velocity = static_cast<float>(get_parameter("r_velocity").as_double());
        ekf_config_.r_speed = static_cast<float>(get_parameter("r_speed").as_double());
        ekf_config_.of_quality_min = static_cast<float>(get_parameter("of_quality_min").as_double());
        ekf_config_.of_quality_scale = static_cast<float>(get_parameter("of_quality_scale").as_double());
        ekf_config_.cable_margin = static_cast<float>(get_parameter("cable_margin").as_double());
        ekf_config_.p0_pos = static_cast<float>(get_parameter("p0_pos").as_double());
        ekf_config_.p0_vel = static_cast<float>(get_parameter("p0_vel").as_double());
        ekf_config_.p0_wind = static_cast<float>(get_parameter("p0_wind").as_double());

        // Publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/position_ekf/estimate", 10);
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/position_ekf/velocity", 10);
        wind_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/position_ekf/wind", 10);
        diag_pub_ = create_publisher<std_msgs::msg::String>(
            "/position_ekf/diagnostics", 10);

        sigma_x_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/position_ekf/sigma_x", 10);
        sigma_y_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/position_ekf/sigma_y", 10);
        dist_home_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/position_ekf/distance_home", 10);

        if (feed_px4_) {
            odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", 10);
        }

        // Subscribers
        spool_sub_ = create_subscription<fiber_nav_sensors::msg::SpoolStatus>(
            "/sensors/fiber_spool/status", 10,
            [this](fiber_nav_sensors::msg::SpoolStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                spool_velocity_ = msg->velocity;
                spool_total_length_ = msg->total_length;
                spool_time_ = now();
            });

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10,
            [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                dir_x_ = static_cast<float>(msg->vector.x);
                dir_y_ = static_cast<float>(msg->vector.y);
                dir_z_ = static_cast<float>(msg->vector.z);
                dir_time_ = now();
            });

        quality_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/sensors/optical_flow/quality", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                of_quality_ = static_cast<float>(msg->data);
            });

        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4_qos,
            [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                att_q_.setW(msg->q[0]);
                att_q_.setX(msg->q[1]);
                att_q_.setY(msg->q[2]);
                att_q_.setZ(msg->q[3]);
                has_attitude_ = true;
            });

        lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                ekf_x_ = msg->x;
                ekf_y_ = msg->y;
                ekf_z_ = msg->z;
                gps_healthy_ = msg->xy_global;
            });

        cable_sub_ = create_subscription<fiber_nav_sensors::msg::CableStatus>(
            "/cable/status", rclcpp::QoS(1).best_effort(),
            [this](fiber_nav_sensors::msg::CableStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                cable_deployed_ = msg->deployed_length;
                cable_valid_ = true;
            });

        // Timer
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&PositionEkfNode::update_callback, this));

        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PositionEkfNode::diagnostics_callback, this));

        RCLCPP_INFO(get_logger(), "Position EKF node initialized (enabled=%s, feed_px4=%s)",
            enabled_ ? "true" : "false", feed_px4_ ? "true" : "false");
    }

private:
    void update_callback() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!enabled_) return;

        auto current_time = now();

        // Check data freshness
        bool spool_fresh = spool_time_.has_value() &&
            (current_time - *spool_time_).seconds() < 0.2;
        bool dir_fresh = dir_time_.has_value() &&
            (current_time - *dir_time_).seconds() < 0.2;

        if (!spool_fresh || !dir_fresh || !has_attitude_) return;

        // GPS transition: when GPS goes from healthy to denied, initialize EKF
        if (was_gps_healthy_ && !gps_healthy_ && !ekf_state_.initialized) {
            RCLCPP_INFO(get_logger(), "GPS denied detected — initializing Position EKF at (%.1f, %.1f)",
                ekf_x_, ekf_y_);
            ekf_state_ = initializeAt(ekf_config_, ekf_x_, ekf_y_);
        }

        // GPS re-acquired: reset position
        if (!was_gps_healthy_ && gps_healthy_ && ekf_state_.initialized) {
            RCLCPP_INFO(get_logger(), "GPS re-acquired — resetting EKF position to (%.1f, %.1f)",
                ekf_x_, ekf_y_);
            ekf_state_ = resetPosition(ekf_state_, ekf_x_, ekf_y_, 1.0f);
        }

        was_gps_healthy_ = gps_healthy_;

        // Only run EKF when GPS is denied
        if (gps_healthy_ || !ekf_state_.initialized) return;

        // Compute NED velocity from spool + OF direction + attitude
        float speed = spool_velocity_;
        float vx_body = speed * dir_x_;
        float vy_body = speed * dir_y_;
        float vz_body = speed * dir_z_;

        tf2::Vector3 v_body(vx_body, vy_body, vz_body);
        tf2::Vector3 v_ned = tf2::quatRotate(att_q_, v_body);

        float vn = static_cast<float>(v_ned.x());
        float ve = static_cast<float>(v_ned.y());

        // dt
        float dt = last_update_time_.has_value()
            ? static_cast<float>((current_time - *last_update_time_).seconds())
            : 0.02f;
        dt = std::clamp(dt, 0.001f, 0.1f);
        last_update_time_ = current_time;

        // EKF steps
        ekf_state_ = predict(ekf_state_, ekf_config_, dt, vn, ve);
        ekf_state_ = updateVelocity(ekf_state_, ekf_config_, vn, ve, of_quality_);
        ekf_state_ = updateSpeedConsistency(ekf_state_, ekf_config_, speed);

        if (cable_valid_ && cable_deployed_ > 0.f) {
            ekf_state_ = applyCableConstraint(ekf_state_, ekf_config_, cable_deployed_);
        }

        // Publish position estimate
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = ekf_state_.x(0);
        pose_msg.pose.pose.position.y = ekf_state_.x(1);
        pose_msg.pose.pose.position.z = 0.0;
        // 6x6 row-major covariance — fill XY block
        pose_msg.pose.covariance[0] = ekf_state_.P(0, 0);   // xx
        pose_msg.pose.covariance[1] = ekf_state_.P(0, 1);   // xy
        pose_msg.pose.covariance[6] = ekf_state_.P(1, 0);   // yx
        pose_msg.pose.covariance[7] = ekf_state_.P(1, 1);   // yy
        pose_pub_->publish(pose_msg);

        // Publish sigma for Foxglove
        auto sigma = positionSigma(ekf_state_);
        auto sx_msg = std_msgs::msg::Float64();
        sx_msg.data = sigma[0];
        sigma_x_pub_->publish(sx_msg);
        auto sy_msg = std_msgs::msg::Float64();
        sy_msg.data = sigma[1];
        sigma_y_pub_->publish(sy_msg);

        // Publish distance from home
        auto dh_msg = std_msgs::msg::Float64();
        dh_msg.data = distanceFromHome(ekf_state_);
        dist_home_pub_->publish(dh_msg);

        // Publish velocity
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = "odom";
        twist_msg.twist.linear.x = ekf_state_.x(2);
        twist_msg.twist.linear.y = ekf_state_.x(3);
        twist_pub_->publish(twist_msg);

        // Publish wind estimate
        auto wind_msg = geometry_msgs::msg::Vector3Stamped();
        wind_msg.header.stamp = current_time;
        wind_msg.header.frame_id = "odom";
        wind_msg.vector.x = ekf_state_.x(4);
        wind_msg.vector.y = ekf_state_.x(5);
        wind_pub_->publish(wind_msg);

        // Publish position to PX4 (position only, velocity = NaN)
        if (feed_px4_ && odom_pub_) {
            auto odom = px4_msgs::msg::VehicleOdometry();
            odom.timestamp = current_time.nanoseconds() / 1000;
            odom.timestamp_sample = odom.timestamp;

            odom.position[0] = ekf_state_.x(0);
            odom.position[1] = ekf_state_.x(1);
            odom.position[2] = ekf_z_;
            odom.position_variance[0] = position_variance_to_px4_;
            odom.position_variance[1] = position_variance_to_px4_;
            odom.position_variance[2] = 1e6f;

            // NaN velocity — don't override fusion node's velocity
            odom.velocity[0] = std::nanf("");
            odom.velocity[1] = std::nanf("");
            odom.velocity[2] = std::nanf("");

            // NaN attitude and angular velocity
            odom.q[0] = std::nanf("");
            odom.q[1] = std::nanf("");
            odom.q[2] = std::nanf("");
            odom.q[3] = std::nanf("");
            odom.angular_velocity[0] = std::nanf("");
            odom.angular_velocity[1] = std::nanf("");
            odom.angular_velocity[2] = std::nanf("");

            odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
            odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
            odom.quality = 50;  // Lower than fusion node (100)

            odom_pub_->publish(odom);
        }
    }

    void diagnostics_callback() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!enabled_ || !ekf_state_.initialized) return;

        auto pos = position(ekf_state_);
        auto vel = velocity(ekf_state_);
        auto w = wind(ekf_state_);
        auto sigma = positionSigma(ekf_state_);
        float dist = distanceFromHome(ekf_state_);

        std::ostringstream ss;
        ss << "{"
           << "\"pos_n\":" << pos[0]
           << ",\"pos_e\":" << pos[1]
           << ",\"vel_n\":" << vel[0]
           << ",\"vel_e\":" << vel[1]
           << ",\"wind_n\":" << w[0]
           << ",\"wind_e\":" << w[1]
           << ",\"sigma_n\":" << sigma[0]
           << ",\"sigma_e\":" << sigma[1]
           << ",\"dist_home\":" << dist
           << ",\"cable_deployed\":" << cable_deployed_
           << ",\"of_quality\":" << of_quality_
           << ",\"gps_healthy\":" << (gps_healthy_ ? "true" : "false")
           << "}";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        diag_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "PosEKF: pos=(%.1f, %.1f) sigma=(%.1f, %.1f) dist=%.1f cable=%.1f wind=(%.1f, %.1f)",
            pos[0], pos[1], sigma[0], sigma[1], dist, cable_deployed_, w[0], w[1]);
    }

    // Parameters
    bool enabled_;
    bool feed_px4_;
    float position_variance_to_px4_;
    PositionEkfConfig ekf_config_;

    // EKF state
    PositionEkfState ekf_state_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sigma_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sigma_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_home_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;

    // Subscribers
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::CableStatus>::SharedPtr cable_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Sensor data (protected by mutex)
    std::mutex mutex_;
    float spool_velocity_{0.f};
    float spool_total_length_{0.f};
    float dir_x_{1.f}, dir_y_{0.f}, dir_z_{0.f};
    float of_quality_{0.f};
    tf2::Quaternion att_q_;
    bool has_attitude_{false};
    float ekf_x_{0.f}, ekf_y_{0.f}, ekf_z_{0.f};
    bool gps_healthy_{true};
    bool was_gps_healthy_{true};
    float cable_deployed_{0.f};
    bool cable_valid_{false};

    std::optional<rclcpp::Time> spool_time_;
    std::optional<rclcpp::Time> dir_time_;
    std::optional<rclcpp::Time> last_update_time_;
};

}  // namespace fiber_nav_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_fusion::PositionEkfNode>());
    rclcpp::shutdown();
    return 0;
}
