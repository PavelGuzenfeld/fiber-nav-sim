#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <optional>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

namespace fiber_nav_fusion {

enum class FlightPhase : uint8_t {
    MC,
    FW,
    TRANSITION_FW,
    TRANSITION_MC,
};

inline const char* flight_phase_str(FlightPhase p) {
    switch (p) {
        case FlightPhase::MC:            return "MC";
        case FlightPhase::FW:            return "FW";
        case FlightPhase::TRANSITION_FW: return "TRANSITION_FW";
        case FlightPhase::TRANSITION_MC: return "TRANSITION_MC";
    }
    return "UNKNOWN";
}

// Determine flight phase from VehicleStatus fields
inline FlightPhase determine_flight_phase(uint8_t vehicle_type,
                                          bool in_transition_mode,
                                          bool in_transition_to_fw) {
    if (in_transition_mode) {
        return in_transition_to_fw ? FlightPhase::TRANSITION_FW
                                   : FlightPhase::TRANSITION_MC;
    }
    // vehicle_type: 1=ROTARY_WING, 2=FIXED_WING
    if (vehicle_type == 2) return FlightPhase::FW;
    return FlightPhase::MC;
}

// Velocity variance for a given flight phase (non-ZUPT)
inline float phase_velocity_variance(FlightPhase phase, float normal_var, float transition_var) {
    switch (phase) {
        case FlightPhase::MC:            return normal_var;
        case FlightPhase::FW:            return normal_var;
        case FlightPhase::TRANSITION_FW: return transition_var;
        case FlightPhase::TRANSITION_MC: return transition_var;
    }
    return normal_var;
}

// Health-based variance scaling: low health → high variance
// health_frac in [0,1]: 1.0=100% → 1x, 0.5=50% → 4x, 0.1=10% → 100x
inline float health_variance_scale(float health_pct) {
    float frac = std::max(0.01f, health_pct / 100.f);
    return 1.f / (frac * frac);
}

// Attitude staleness variance multiplier
// < 200ms: 1x, 200-500ms: 2x, 500ms-1s: 4x, >1s: skip (return 0)
inline float attitude_staleness_scale(double age_seconds) {
    if (age_seconds < 0.2) return 1.f;
    if (age_seconds < 0.5) return 2.f;
    if (age_seconds < 1.0) return 4.f;
    return 0.f;  // signal: skip publish entirely
}

// Spool-EKF cross-validation: compare speed magnitudes
// Returns variance multiplier: 1x when agreement, higher when disagreement
inline float cross_validation_scale(float spool_speed, float ekf_speed) {
    if (spool_speed < 0.5f && ekf_speed < 0.5f) return 1.f;  // both slow, no comparison
    float ref = std::max(spool_speed, ekf_speed);
    float innovation = std::abs(spool_speed - ekf_speed) / ref;
    if (innovation < 0.3f) return 1.f;   // agreement
    // Proportional scale: 30% → 1x, 60% → 4x, 90% → 9x
    float excess = (innovation - 0.3f) / 0.3f;  // 0..2+ range
    return 1.f + 3.f * excess * excess;
}

// Rolling sensor health tracker using a ring buffer
struct SensorHealth {
    size_t window_size{50};

    void init(size_t ws) {
        window_size = ws;
        buffer.assign(ws, false);
        pos = 0;
        count = 0;
        hits = 0;
    }

    // Record that a message was expected this tick (called each fusion cycle).
    // received=true means a new message arrived since the last tick.
    void record(bool received) {
        if (buffer.empty()) return;
        // Remove oldest sample from hits count
        if (count >= window_size && buffer[pos]) {
            --hits;
        }
        buffer[pos] = received;
        if (received) ++hits;
        pos = (pos + 1) % window_size;
        if (count < window_size) ++count;
    }

    float health_pct() const {
        if (count == 0) return 100.0f;
        return 100.0f * static_cast<float>(hits) / static_cast<float>(count);
    }

private:
    std::vector<bool> buffer;
    size_t pos{0};
    size_t count{0};   // samples recorded so far (up to window_size)
    size_t hits{0};    // number of true entries in current window
};

class FiberVisionFusion : public rclcpp::Node {
public:
    FiberVisionFusion() : Node("fiber_vision_fusion") {
        // Parameters
        declare_parameter("slack_factor", 1.05);     // Known/estimated slack
        declare_parameter("publish_rate", 50.0);     // Hz
        declare_parameter("max_data_age", 0.1);      // seconds
        declare_parameter("zupt_threshold", 0.05);   // m/s
        declare_parameter("zupt_velocity_variance", 0.001);
        declare_parameter("velocity_variance", 0.01);
        declare_parameter("transition_velocity_variance", 0.04);
        declare_parameter("enable_position_clamping", true);
        declare_parameter("k_drag", 0.0005);
        declare_parameter("tunnel_heading_deg", 90.0);
        declare_parameter("position_variance_longitudinal", 1.0);
        declare_parameter("position_variance_lateral", 100.0);
        declare_parameter("health_window_size", 50);
        declare_parameter("health_warn_threshold", 80.0);

        slack_factor_ = get_parameter("slack_factor").as_double();
        max_data_age_ = get_parameter("max_data_age").as_double();
        double rate = get_parameter("publish_rate").as_double();
        zupt_threshold_ = get_parameter("zupt_threshold").as_double();
        zupt_velocity_variance_ = get_parameter("zupt_velocity_variance").as_double();
        velocity_variance_ = get_parameter("velocity_variance").as_double();
        transition_velocity_variance_ = get_parameter("transition_velocity_variance").as_double();
        enable_position_clamping_ = get_parameter("enable_position_clamping").as_bool();
        k_drag_ = get_parameter("k_drag").as_double();
        tunnel_heading_deg_ = get_parameter("tunnel_heading_deg").as_double();
        position_variance_longitudinal_ = get_parameter("position_variance_longitudinal").as_double();
        position_variance_lateral_ = get_parameter("position_variance_lateral").as_double();
        auto health_window = static_cast<size_t>(get_parameter("health_window_size").as_int());
        health_warn_threshold_ = get_parameter("health_warn_threshold").as_double();

        spool_health_.init(health_window);
        direction_health_.init(health_window);

        // Publishers
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
        diag_pub_ = create_publisher<std_msgs::msg::String>(
            "/sensors/fusion/diagnostics", 10);

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

        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos,
            std::bind(&FiberVisionFusion::vehicle_status_callback, this, std::placeholders::_1));

        // Timer for fusion at publish_rate
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&FiberVisionFusion::fusion_callback, this));

        // Diagnostics timer at 1 Hz
        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FiberVisionFusion::diagnostics_callback, this));

        RCLCPP_INFO(get_logger(), "Fiber vision fusion initialized");
        RCLCPP_INFO(get_logger(), "  Slack factor: %.3f", slack_factor_);
        RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", rate);
        RCLCPP_INFO(get_logger(), "  ZUPT threshold: %.3f m/s", zupt_threshold_);
        RCLCPP_INFO(get_logger(), "  Velocity variance: %.4f (normal), %.4f (transition)",
                     velocity_variance_, transition_velocity_variance_);
        RCLCPP_INFO(get_logger(), "  Health window: %zu, warn threshold: %.0f%%",
                     health_window, health_warn_threshold_);
        RCLCPP_INFO(get_logger(), "  Position clamping: %s", enable_position_clamping_ ? "enabled" : "disabled");
        if (enable_position_clamping_) {
            RCLCPP_INFO(get_logger(), "  Tunnel heading: %.1f deg", tunnel_heading_deg_);
            RCLCPP_INFO(get_logger(), "  k_drag: %.4f", k_drag_);
        }
        RCLCPP_INFO(get_logger(), "  Enhanced: health scaling, attitude staleness, cross-validation");
    }

private:
    void spool_callback(const fiber_nav_sensors::msg::SpoolStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spool_velocity_ = msg->velocity;
        spool_total_length_ = msg->total_length;
        spool_is_moving_ = msg->is_moving;
        spool_time_ = now();
        spool_received_flag_ = true;
    }

    void direction_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        direction_x_ = msg->vector.x;
        direction_y_ = msg->vector.y;
        direction_z_ = msg->vector.z;
        direction_time_ = now();
        direction_received_flag_ = true;
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
        ekf_vx_ = msg->vx;
        ekf_vy_ = msg->vy;
        ekf_vz_ = msg->vz;
        has_ekf_z_ = true;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto new_phase = determine_flight_phase(
            msg->vehicle_type, msg->in_transition_mode, msg->in_transition_to_fw);
        if (new_phase != flight_phase_) {
            RCLCPP_INFO(get_logger(), "Flight phase: %s -> %s",
                         flight_phase_str(flight_phase_), flight_phase_str(new_phase));
            flight_phase_ = new_phase;
        }
    }

    void fusion_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        auto current_time = now();

        // Track sensor health: record whether a new message arrived since last tick
        spool_health_.record(spool_received_flag_);
        spool_received_flag_ = false;
        direction_health_.record(direction_received_flag_);
        direction_received_flag_ = false;

        // Check health and warn
        float spool_hp = spool_health_.health_pct();
        float dir_hp = direction_health_.health_pct();
        if (spool_hp < health_warn_threshold_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Spool health low: %.0f%%", spool_hp);
        }
        if (dir_hp < health_warn_threshold_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Direction health low: %.0f%%", dir_hp);
        }

        // Spool data is always required
        if (!spool_time_ || (current_time - *spool_time_).seconds() > max_data_age_) {
            return;  // Spool data stale
        }

        // Attitude is always required (for frame reference even during ZUPT)
        if (!has_attitude_ || !attitude_time_) {
            return;  // No attitude data at all
        }

        // 3b: Attitude staleness check
        double attitude_age = (current_time - *attitude_time_).seconds();
        float att_scale = attitude_staleness_scale(attitude_age);
        last_attitude_age_ = attitude_age;
        last_attitude_scale_ = att_scale;

        if (att_scale == 0.f) {
            // Attitude too stale (>1s) — skip publish entirely
            return;
        }

        // ZUPT check: when stopped, we don't need vision direction
        bool zupt_active = (spool_velocity_ < zupt_threshold_);
        last_zupt_active_ = zupt_active;

        // Vision direction required only for normal (non-ZUPT) fusion
        bool direction_fresh = direction_time_.has_value() &&
            (current_time - *direction_time_).seconds() <= max_data_age_;
        if (!zupt_active && !direction_fresh) {
            return;  // Direction data stale and not in ZUPT mode
        }

        // 3a: Health-based variance scaling
        float spool_health_scale = health_variance_scale(spool_hp);
        float dir_health_scale = health_variance_scale(dir_hp);
        // Use the worse of the two sensor health scales
        float combined_health_scale = std::max(spool_health_scale, dir_health_scale);
        last_health_scale_ = combined_health_scale;

        // 3c: Spool-EKF cross-validation
        float spool_speed = static_cast<float>(spool_velocity_ / slack_factor_);
        float ekf_speed = std::sqrt(ekf_vx_ * ekf_vx_ + ekf_vy_ * ekf_vy_ + ekf_vz_ * ekf_vz_);
        float xval_scale = cross_validation_scale(spool_speed, ekf_speed);
        last_xval_innovation_ = (ekf_speed > 0.5f) ?
            std::abs(spool_speed - ekf_speed) / std::max(spool_speed, ekf_speed) : 0.f;
        last_xval_scale_ = xval_scale;

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

            // Adaptive variance: base phase variance × health × staleness × cross-validation
            float vel_var = phase_velocity_variance(
                flight_phase_,
                static_cast<float>(velocity_variance_),
                static_cast<float>(transition_velocity_variance_));
            vel_var *= combined_health_scale * att_scale * xval_scale;
            odom_msg.velocity_variance[0] = vel_var;
            odom_msg.velocity_variance[1] = vel_var;
            odom_msg.velocity_variance[2] = vel_var;
        }
        last_velocity_variance_ = odom_msg.velocity_variance[0];

        // Position: Echo EKF's own estimate back with very high variance.
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
        last_position_variance_ = odom_msg.position_variance[0];

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

    void diagnostics_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        std::ostringstream ss;
        ss << "{"
           << "\"flight_phase\":\"" << flight_phase_str(flight_phase_) << "\""
           << ",\"spool_health_pct\":" << spool_health_.health_pct()
           << ",\"direction_health_pct\":" << direction_health_.health_pct()
           << ",\"zupt_active\":" << (last_zupt_active_ ? "true" : "false")
           << ",\"velocity_variance\":" << last_velocity_variance_
           << ",\"position_variance\":" << last_position_variance_
           // 3d: Enhanced diagnostics
           << ",\"health_variance_scale\":" << last_health_scale_
           << ",\"attitude_age_ms\":" << (last_attitude_age_ * 1000.0)
           << ",\"attitude_staleness_scale\":" << last_attitude_scale_
           << ",\"xval_innovation\":" << last_xval_innovation_
           << ",\"xval_scale\":" << last_xval_scale_
           << "}";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        diag_pub_->publish(msg);
    }

    // Publishers/Subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Parameters
    double slack_factor_;
    double max_data_age_;
    double zupt_threshold_;
    double zupt_velocity_variance_;
    double velocity_variance_;
    double transition_velocity_variance_;
    bool enable_position_clamping_;
    double k_drag_;
    double tunnel_heading_deg_;
    double position_variance_longitudinal_;
    double position_variance_lateral_;
    double health_warn_threshold_;

    // Data storage (protected by mutex)
    std::mutex data_mutex_;
    double spool_velocity_{0.0};
    double spool_total_length_{0.0};
    bool spool_is_moving_{false};
    double direction_x_{1.0}, direction_y_{0.0}, direction_z_{0.0};
    tf2::Quaternion attitude_q_;
    bool has_attitude_{false};
    float ekf_x_{0.0f}, ekf_y_{0.0f}, ekf_z_{0.0f};
    float ekf_vx_{0.0f}, ekf_vy_{0.0f}, ekf_vz_{0.0f};
    bool has_ekf_z_{false};
    FlightPhase flight_phase_{FlightPhase::MC};
    bool spool_received_flag_{false};
    bool direction_received_flag_{false};

    // Sensor health tracking
    SensorHealth spool_health_;
    SensorHealth direction_health_;

    // Diagnostics state (written in fusion_callback, read in diagnostics_callback)
    bool last_zupt_active_{false};
    float last_velocity_variance_{0.0f};
    float last_position_variance_{0.0f};
    float last_health_scale_{1.0f};
    double last_attitude_age_{0.0};
    float last_attitude_scale_{1.0f};
    float last_xval_innovation_{0.0f};
    float last_xval_scale_{1.0f};

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
