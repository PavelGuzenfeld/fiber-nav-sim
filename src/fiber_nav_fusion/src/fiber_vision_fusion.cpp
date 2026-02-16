#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <optional>
#include <cmath>
#include <numeric>
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

// 2a: Online slack calibration — EMA of spool/EKF speed ratio
// Returns updated slack factor, clamped to [0.8, 1.2]
inline float slack_calibration_update(float current_slack, float spool_speed,
                                       float ekf_speed, float ema_alpha) {
    if (spool_speed < 1.0f || ekf_speed < 1.0f) return current_slack;
    float ratio = spool_speed / ekf_speed;
    float updated = current_slack + ema_alpha * (ratio - current_slack);
    return std::clamp(updated, 0.8f, 1.2f);
}

// 2b: Heading cross-check — compare two headings (radians)
// Returns variance multiplier: 1.0 when headings agree, higher when divergent
inline float heading_crosscheck_scale(float heading1_rad, float heading2_rad) {
    float diff = std::remainder(heading1_rad - heading2_rad,
                                2.f * static_cast<float>(M_PI));
    float abs_diff = std::abs(diff);
    if (abs_diff < 0.1f) return 1.f;
    float excess = (abs_diff - 0.1f) / 0.2f;
    return std::min(10.f, 1.f + 3.f * excess * excess);
}

// 2c: Adaptive ZUPT threshold from noise floor RMS
inline float adaptive_zupt_threshold(float noise_rms) {
    return std::max(0.01f, 3.f * noise_rms);
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

// Running RMS noise tracker for adaptive ZUPT
struct ZuptNoiseTracker {
    void init(size_t ws, float thresh) {
        window_size_ = ws;
        speed_threshold_ = thresh;
        buffer_.assign(ws, 0.f);
        pos_ = 0;
        count_ = 0;
    }

    void record(float speed) {
        if (speed >= speed_threshold_) return;
        if (buffer_.empty()) return;
        buffer_[pos_] = speed;
        pos_ = (pos_ + 1) % window_size_;
        if (count_ < window_size_) ++count_;
    }

    float rms() const {
        if (count_ == 0) return 0.f;
        float sum_sq = std::transform_reduce(
            buffer_.begin(), buffer_.begin() + static_cast<ptrdiff_t>(count_),
            0.f, std::plus<>{}, [](float v) { return v * v; });
        return std::sqrt(sum_sq / static_cast<float>(count_));
    }

    float threshold() const {
        return adaptive_zupt_threshold(rms());
    }

private:
    std::vector<float> buffer_;
    size_t window_size_{100};
    float speed_threshold_{0.5f};
    size_t pos_{0};
    size_t count_{0};
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
        declare_parameter("slack_ema_alpha", 0.01);
        declare_parameter("zupt_noise_window", 100);
        declare_parameter("zupt_noise_speed_threshold", 0.5);
        declare_parameter("terrain_altitude_variance", 100.0);
        declare_parameter("terrain_z_max_age", 2.0);
        declare_parameter("terrain_z_hold", true);

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
        slack_ema_alpha_ = get_parameter("slack_ema_alpha").as_double();
        terrain_alt_variance_ = static_cast<float>(get_parameter("terrain_altitude_variance").as_double());
        terrain_z_max_age_ = static_cast<float>(get_parameter("terrain_z_max_age").as_double());
        auto zupt_noise_window = static_cast<size_t>(get_parameter("zupt_noise_window").as_int());
        auto zupt_noise_thresh = get_parameter("zupt_noise_speed_threshold").as_double();

        spool_health_.init(health_window);
        direction_health_.init(health_window);
        zupt_noise_tracker_.init(zupt_noise_window, static_cast<float>(zupt_noise_thresh));
        calibrated_slack_ = static_cast<float>(slack_factor_);

        // Publishers
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
        diag_pub_ = create_publisher<std_msgs::msg::String>(
            "/sensors/fusion/diagnostics", 10);
        diag_spool_health_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/fusion/spool_health", 10);
        diag_direction_health_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/fusion/direction_health", 10);
        diag_velocity_var_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/fusion/velocity_variance", 10);
        diag_position_var_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/fusion/position_variance", 10);
        diag_health_scale_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/fusion/health_scale", 10);

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

        // EstimatorStatusFlags: cs_gnss_pos reflects actual GPS fusion state in EKF2.
        // VehicleLocalPosition.xy_global stays true permanently once NED origin is set,
        // which is WRONG for detecting GPS-denied mode.
        ekf_flags_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags", px4_qos,
            [this](px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                bool was = gps_fusing_;
                gps_fusing_ = msg->cs_gnss_pos;
                ev_vel_fusing_ = msg->cs_ev_vel;
                if (was && !gps_fusing_) {
                    RCLCPP_WARN(get_logger(), "GPS fusion LOST — disabling cross-validation");
                }
                if (!was && gps_fusing_) {
                    RCLCPP_INFO(get_logger(), "GPS fusion RESTORED — enabling cross-validation");
                }
            });

        // Terrain-anchored Z from position_ekf_node (rangefinder + DEM)
        terrain_z_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/position_ekf/terrain_z", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                terrain_z_ = static_cast<float>(msg->data);
                terrain_z_time_ = now();
                has_terrain_z_ = true;
            });

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
        RCLCPP_INFO(get_logger(), "  Slack EMA alpha: %.4f", slack_ema_alpha_);
        RCLCPP_INFO(get_logger(), "  ZUPT noise window: %zu, speed threshold: %.2f",
                     zupt_noise_window, zupt_noise_thresh);
        RCLCPP_INFO(get_logger(), "  Enhanced: health scaling, staleness, cross-val, slack cal, heading check, adaptive ZUPT");
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
        // Store PX4 timestamp for visual odometry messages — critical for SITL lockstep.
        // PX4 SITL uses simulation time (starts at 0), while ROS uses wall clock.
        px4_timestamp_us_ = msg->timestamp;
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 15000,
                "Spool health low: %.0f%%", spool_hp);
        }
        if (dir_hp < health_warn_threshold_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 15000,
                "Direction health low: %.0f%%", dir_hp);
        }

        // Spool data is always required
        if (!spool_time_ || (current_time - *spool_time_).seconds() > max_data_age_) {
            return;  // Spool data stale
        }

        // 2c: Track noise floor for adaptive ZUPT
        float raw_spool_speed = static_cast<float>(spool_velocity_);
        zupt_noise_tracker_.record(raw_spool_speed);
        float noise_rms = zupt_noise_tracker_.rms();
        float adaptive_threshold = zupt_noise_tracker_.threshold();
        last_zupt_threshold_ = adaptive_threshold;
        last_noise_rms_ = noise_rms;

        // 2a: Online slack calibration (only when GPS actively fused and moving)
        if (gps_fusing_ && raw_spool_speed > 1.0f) {
            float ekf_speed_h = std::hypot(ekf_vx_, ekf_vy_);
            calibrated_slack_ = slack_calibration_update(
                calibrated_slack_, raw_spool_speed, ekf_speed_h,
                static_cast<float>(slack_ema_alpha_));
        }
        last_calibrated_slack_ = calibrated_slack_;

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

        // ZUPT check: adaptive threshold from noise floor
        bool zupt_active = (spool_velocity_ < adaptive_threshold);
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

        // 3c: Spool-EKF cross-validation (use calibrated slack)
        // DISABLED when GPS is not fusing: without GPS, EKF velocity drifts and
        // cross-validation creates a death spiral (inflated variance → EKF ignores
        // fusion velocity → EKF velocity diverges more → variance inflates more).
        float spool_speed = static_cast<float>(spool_velocity_ / calibrated_slack_);
        float xval_scale = 1.f;
        if (gps_fusing_) {
            float ekf_speed = std::hypot(ekf_vx_, ekf_vy_, ekf_vz_);
            xval_scale = cross_validation_scale(spool_speed, ekf_speed);
            last_xval_innovation_ = (ekf_speed > 0.5f) ?
                std::abs(spool_speed - ekf_speed) / std::max(spool_speed, ekf_speed) : 0.f;
        } else {
            last_xval_innovation_ = 0.f;
        }
        last_xval_scale_ = xval_scale;

        // Create odometry message
        auto odom_msg = px4_msgs::msg::VehicleOdometry();
        // Use PX4's timestamp — critical for EKF acceptance in SITL lockstep.
        // PX4 SITL uses simulation time (starts at 0), while ROS uses wall clock.
        odom_msg.timestamp = px4_timestamp_us_;
        odom_msg.timestamp_sample = px4_timestamp_us_;

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
            // Reconstruct velocity in body frame (2a: use calibrated slack)
            double corrected_speed = spool_velocity_ / calibrated_slack_;
            double vx_body = corrected_speed * direction_x_;
            double vy_body = corrected_speed * direction_y_;
            double vz_body = corrected_speed * direction_z_;

            // Direction from sensors is in SDF/FLU body frame (X=fwd, Y=left, Z=up).
            // PX4 VehicleAttitude.q rotates FRD → NED.
            // Convert FLU → FRD: negate Y and Z.
            tf2::Vector3 v_body_frd(vx_body, -vy_body, -vz_body);
            tf2::Vector3 v_ned = tf2::quatRotate(attitude_q_, v_body_frd);

            float vn = static_cast<float>(v_ned.x());
            float ve = static_cast<float>(v_ned.y());
            float vd = static_cast<float>(v_ned.z());

            odom_msg.velocity[0] = vn;
            odom_msg.velocity[1] = ve;
            odom_msg.velocity[2] = vd;

            // 2b: Heading cross-check (fused NED velocity vs EKF velocity)
            // DISABLED when GPS is not fusing: EKF velocity heading is unreliable
            // without GPS, so comparing against it would inflate variance incorrectly.
            float heading_check = 1.f;
            if (gps_fusing_) {
                float fused_speed_h = std::hypot(vn, ve);
                float ekf_speed_h = std::hypot(ekf_vx_, ekf_vy_);
                if (fused_speed_h > 1.0f && ekf_speed_h > 1.0f) {
                    float fused_heading = std::atan2(ve, vn);
                    float ekf_heading = std::atan2(ekf_vy_, ekf_vx_);
                    heading_check = heading_crosscheck_scale(fused_heading, ekf_heading);
                }
            }
            last_heading_check_scale_ = heading_check;

            // Adaptive variance: base × health × staleness × cross-val × heading
            float vel_var = phase_velocity_variance(
                flight_phase_,
                static_cast<float>(velocity_variance_),
                static_cast<float>(transition_velocity_variance_));
            vel_var *= combined_health_scale * att_scale * xval_scale * heading_check;
            odom_msg.velocity_variance[0] = vel_var;
            odom_msg.velocity_variance[1] = vel_var;
            odom_msg.velocity_variance[2] = vel_var;
        }
        last_velocity_variance_ = odom_msg.velocity_variance[0];

        // Terrain-anchored Z: always include when available so PX4 continuously
        // fuses EV height alongside baro. This enables smooth EKF2_HGT_REF switch
        // from baro to EV when GPS is disabled (no cold-start height reset).
        // Hold mechanism: when terrain_z goes stale, keep publishing the last value
        // with degraded (4x) variance instead of NaN. This prevents PX4 from
        // losing EV height fusion during slow Gazebo sim updates.
        float z_val = std::nanf("");
        float z_var = 0.f;
        if (has_terrain_z_) {
            bool fresh = terrain_z_time_.has_value() &&
                (current_time - *terrain_z_time_).seconds() < terrain_z_max_age_;
            z_val = terrain_z_;  // always use last value (hold)
            z_var = fresh ? terrain_alt_variance_ : terrain_alt_variance_ * 4.f;  // degrade if stale
        }

        // Position: Echo EKF's own estimate back with very high variance.
        if (has_ekf_z_) {
            if (enable_position_clamping_ && spool_is_moving_) {
                // Spool moving: use drag bow model for horizontal position
                double v = spool_velocity_;
                double x_est = spool_total_length_ * (1.0 - k_drag_ * v * v);
                double heading_rad = tunnel_heading_deg_ * M_PI / 180.0;
                odom_msg.position[0] = static_cast<float>(x_est * std::cos(heading_rad));
                odom_msg.position[1] = static_cast<float>(x_est * std::sin(heading_rad));
                odom_msg.position[2] = z_val;
                float pos_var = static_cast<float>(position_variance_lateral_);
                odom_msg.position_variance[0] = pos_var;
                odom_msg.position_variance[1] = pos_var;
                odom_msg.position_variance[2] = z_var;
            } else {
                // Echo EKF position back — zero innovation, keeps EV active
                odom_msg.position[0] = ekf_x_;
                odom_msg.position[1] = ekf_y_;
                odom_msg.position[2] = z_val;
                odom_msg.position_variance[0] = 1e6f;
                odom_msg.position_variance[1] = 1e6f;
                odom_msg.position_variance[2] = z_var;
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
           << ",\"calibrated_slack\":" << last_calibrated_slack_
           << ",\"heading_check_scale\":" << last_heading_check_scale_
           << ",\"adaptive_zupt_threshold\":" << last_zupt_threshold_
           << ",\"noise_rms\":" << last_noise_rms_
           << ",\"gps_fusing\":" << (gps_fusing_ ? "true" : "false")
           << ",\"ev_vel_fusing\":" << (ev_vel_fusing_ ? "true" : "false")
           << "}";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        diag_pub_->publish(msg);

        auto f64 = [](double v) { std_msgs::msg::Float64 m; m.data = v; return m; };
        diag_spool_health_pub_->publish(f64(spool_health_.health_pct()));
        diag_direction_health_pub_->publish(f64(direction_health_.health_pct()));
        diag_velocity_var_pub_->publish(f64(last_velocity_variance_));
        diag_position_var_pub_->publish(f64(last_position_variance_));
        diag_health_scale_pub_->publish(f64(last_health_scale_));
    }

    // Publishers/Subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_spool_health_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_direction_health_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_velocity_var_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_position_var_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_health_scale_pub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr ekf_flags_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr terrain_z_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Parameters
    double slack_factor_;
    double max_data_age_;
    double zupt_threshold_;        // base threshold (fallback)
    double zupt_velocity_variance_;
    double slack_ema_alpha_;
    double velocity_variance_;
    double transition_velocity_variance_;
    bool enable_position_clamping_;
    double k_drag_;
    double tunnel_heading_deg_;
    double position_variance_longitudinal_;
    double position_variance_lateral_;
    double health_warn_threshold_;
    float terrain_alt_variance_{4.f};
    float terrain_z_max_age_{0.5f};

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
    uint64_t px4_timestamp_us_{0};
    bool gps_fusing_{true};     // from estimator_status_flags.cs_gnss_pos
    bool ev_vel_fusing_{false}; // from estimator_status_flags.cs_ev_vel
    float calibrated_slack_{1.05f};
    FlightPhase flight_phase_{FlightPhase::MC};
    bool spool_received_flag_{false};
    bool direction_received_flag_{false};
    float terrain_z_{0.f};
    bool has_terrain_z_{false};
    std::optional<rclcpp::Time> terrain_z_time_;

    // Sensor health tracking
    SensorHealth spool_health_;
    SensorHealth direction_health_;
    ZuptNoiseTracker zupt_noise_tracker_;

    // Diagnostics state (written in fusion_callback, read in diagnostics_callback)
    bool last_zupt_active_{false};
    float last_velocity_variance_{0.0f};
    float last_position_variance_{0.0f};
    float last_health_scale_{1.0f};
    double last_attitude_age_{0.0};
    float last_attitude_scale_{1.0f};
    float last_xval_innovation_{0.0f};
    float last_xval_scale_{1.0f};
    float last_calibrated_slack_{1.05f};
    float last_heading_check_scale_{1.0f};
    float last_zupt_threshold_{0.05f};
    float last_noise_rms_{0.0f};

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
