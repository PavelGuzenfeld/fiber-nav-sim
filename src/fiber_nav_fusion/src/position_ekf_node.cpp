#include <fiber_nav_fusion/position_ekf.hpp>
#include <fiber_nav_fusion/tercom.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("use_odometry_direction", false);
        declare_parameter("direction_alpha", 0.15);
        declare_parameter("terrain_altitude_enabled", false);
        declare_parameter("terrain_altitude_variance", 4.0);
        declare_parameter("terrain_data_path", std::string(""));
        declare_parameter("rangefinder_max_age", 0.5);
        declare_parameter("terrain_z_filter_tau", 0.5);

        enabled_ = get_parameter("enabled").as_bool();
        double rate = get_parameter("publish_rate").as_double();
        feed_px4_ = get_parameter("feed_px4").as_bool();
        position_variance_to_px4_ = static_cast<float>(
            get_parameter("position_variance_to_px4").as_double());
        model_name_ = get_parameter("model_name").as_string();
        use_odom_dir_ = get_parameter("use_odometry_direction").as_bool();
        direction_alpha_ = static_cast<float>(get_parameter("direction_alpha").as_double());

        terrain_alt_enabled_ = get_parameter("terrain_altitude_enabled").as_bool();
        terrain_alt_variance_ = static_cast<float>(
            get_parameter("terrain_altitude_variance").as_double());
        rangefinder_max_age_ = static_cast<float>(
            get_parameter("rangefinder_max_age").as_double());
        terrain_z_tau_ = static_cast<float>(
            get_parameter("terrain_z_filter_tau").as_double());

        // Load terrain DEM for terrain-anchored altitude
        if (terrain_alt_enabled_) {
            auto terrain_path = get_parameter("terrain_data_path").as_string();
            auto logger = get_logger();
            terrain_map_ = load_terrain_map(terrain_path, [&](const std::string& msg) {
                RCLCPP_INFO(logger, "[terrain_alt] %s", msg.c_str());
            });
            terrain_map_loaded_ = (terrain_map_.width > 0);
            if (!terrain_map_loaded_) {
                RCLCPP_WARN(get_logger(), "Terrain altitude enabled but DEM load failed — disabled");
                terrain_alt_enabled_ = false;
            } else {
                RCLCPP_INFO(get_logger(), "Terrain altitude enabled (var=%.1f, tau=%.1f)",
                    terrain_alt_variance_, terrain_z_tau_);
            }
        }

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

        // Publish terrain-anchored Z for fiber_vision_fusion to include in its VehicleOdometry
        if (terrain_alt_enabled_) {
            terrain_z_pub_ = create_publisher<std_msgs::msg::Float64>(
                "/position_ekf/terrain_z", 10);
        }

        // Publish EKF state for TERCOM node to use as search center
        state_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/position_ekf/state", 10);

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
                px4_timestamp_us_ = msg->timestamp;

                // Calibrate terrain altitude reference on first valid position
                if (!terrain_alt_calibrated_ && terrain_map_loaded_ && gps_healthy_) {
                    ground_terrain_gz_ = terrain_map_.height_at(0.f, 0.f);
                    terrain_alt_calibrated_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[terrain_alt] Calibrated: ground_terrain_gz=%.2f", ground_terrain_gz_);
                }
            });

        // Use estimator_status_flags.cs_gnss_pos for GPS health detection.
        // vehicle_local_position.xy_global stays true permanently once NED origin
        // is established (even after GPS fusion is disabled). cs_gnss_pos reflects
        // whether GPS position fusion is actually active in the EKF.
        ekf_flags_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags", px4_qos,
            [this](px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                gps_healthy_ = msg->cs_gnss_pos;
                ekf_flags_received_ = true;
            });

        cable_sub_ = create_subscription<fiber_nav_sensors::msg::CableStatus>(
            "/cable/status", rclcpp::QoS(1).best_effort(),
            [this](fiber_nav_sensors::msg::CableStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                cable_deployed_ = msg->deployed_length;
                cable_valid_ = true;
            });

        // TERCOM position fix subscriber
        tercom_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/tercom/position", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!ekf_state_.initialized) return;

                float tx = static_cast<float>(msg->point.x);
                float ty = static_cast<float>(msg->point.y);
                float sigma = static_cast<float>(msg->point.z);  // encoded in z
                float variance = sigma * sigma;

                auto prev_pos = position(ekf_state_);
                ekf_state_ = updatePosition(ekf_state_, tx, ty, variance);
                auto new_pos = position(ekf_state_);

                RCLCPP_INFO(get_logger(),
                    "TERCOM fix applied: (%.1f,%.1f)→(%.1f,%.1f) delta=(%.1f,%.1f) sigma=%.1fm",
                    prev_pos[0], prev_pos[1], new_pos[0], new_pos[1],
                    new_pos[0] - prev_pos[0], new_pos[1] - prev_pos[1], sigma);
            });

        // Rangefinder for terrain-anchored altitude
        if (terrain_alt_enabled_) {
            dist_sensor_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
                "/fmu/in/distance_sensor", px4_qos,
                [this](px4_msgs::msg::DistanceSensor::SharedPtr msg) {
                    // Filter: downward-facing only, valid signal
                    if (msg->orientation != 25 || msg->signal_quality == 0) return;
                    std::lock_guard<std::mutex> lock(mutex_);
                    rangefinder_agl_ = msg->current_distance;
                    rangefinder_valid_ = true;
                    rangefinder_time_ = now();
                });
        }

        // Gazebo odometry for debug comparison / fallback direction
        std::string odom_topic = "/model/" + model_name_ + "/odometry";
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                odom_vx_ = static_cast<float>(msg->twist.twist.linear.x);
                odom_vy_ = static_cast<float>(msg->twist.twist.linear.y);
                odom_vz_ = static_cast<float>(msg->twist.twist.linear.z);
            });

        // Timer
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&PositionEkfNode::update_callback, this));

        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PositionEkfNode::diagnostics_callback, this));

        RCLCPP_INFO(get_logger(),
            "Position EKF node initialized (enabled=%s, feed_px4=%s, terrain_alt=%s)",
            enabled_ ? "true" : "false", feed_px4_ ? "true" : "false",
            terrain_alt_enabled_ ? "true" : "false");
    }

private:
    void update_callback() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!enabled_) return;

        auto current_time = now();

        // Track GPS state transitions BEFORE sensor freshness gate.
        // GPS transition must not be missed even if sensors are momentarily stale.
        if (was_gps_healthy_ && !gps_healthy_ && !ekf_state_.initialized) {
            RCLCPP_INFO(get_logger(), "GPS denied detected — initializing Position EKF at (%.1f, %.1f)",
                ekf_x_, ekf_y_);
            ekf_state_ = initializeAt(ekf_config_, ekf_x_, ekf_y_);
        }

        // Fallback: if GPS is already off when we first receive status, initialize.
        // This handles BEST_EFFORT message drops during the exact transition moment.
        if (!gps_healthy_ && !ekf_state_.initialized && ekf_flags_received_) {
            RCLCPP_INFO(get_logger(), "GPS already denied — late-initializing Position EKF at (%.1f, %.1f)",
                ekf_x_, ekf_y_);
            ekf_state_ = initializeAt(ekf_config_, ekf_x_, ekf_y_);
        }

        if (!was_gps_healthy_ && gps_healthy_ && ekf_state_.initialized) {
            RCLCPP_INFO(get_logger(), "GPS re-acquired — resetting EKF position to (%.1f, %.1f)",
                ekf_x_, ekf_y_);
            ekf_state_ = resetPosition(ekf_state_, ekf_x_, ekf_y_, 1.0f);
        }

        was_gps_healthy_ = gps_healthy_;

        // Always publish terrain-anchored altitude to PX4, even before EKF init.
        // Terrain altitude only needs rangefinder + DEM, not the position EKF.
        publishTerrainAltitude();

        // Always publish metrics from best available source
        if (gps_healthy_ || !ekf_state_.initialized) {
            // GPS healthy or EKF not yet initialized: use PX4 local position
            auto dh_msg = std_msgs::msg::Float64();
            dh_msg.data = std::sqrt(ekf_x_ * ekf_x_ + ekf_y_ * ekf_y_);
            dist_home_pub_->publish(dh_msg);

            auto sx_msg = std_msgs::msg::Float64();
            sx_msg.data = gps_healthy_ ? 0.5 : 999.0;
            sigma_x_pub_->publish(sx_msg);
            auto sy_msg = std_msgs::msg::Float64();
            sy_msg.data = gps_healthy_ ? 0.5 : 999.0;
            sigma_y_pub_->publish(sy_msg);

            // Position estimate from PX4 local position (NED)
            auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            pose_msg.header.stamp = current_time;
            pose_msg.header.frame_id = "odom";
            pose_msg.pose.pose.position.x = ekf_x_;
            pose_msg.pose.pose.position.y = ekf_y_;
            pose_pub_->publish(pose_msg);

            // Wind: zeros when GPS healthy (unknown)
            auto wind_msg = geometry_msgs::msg::Vector3Stamped();
            wind_msg.header.stamp = current_time;
            wind_msg.header.frame_id = "odom";
            wind_msg.vector.x = 0.0;
            wind_msg.vector.y = 0.0;
            wind_pub_->publish(wind_msg);

            return;
        }

        // Check data freshness (only needed for EKF updates, not GPS tracking)
        bool spool_fresh = spool_time_.has_value() &&
            (current_time - *spool_time_).seconds() < 0.2;
        bool dir_fresh = dir_time_.has_value() &&
            (current_time - *dir_time_).seconds() < 0.2;

        if (!spool_fresh || !dir_fresh || !has_attitude_) return;

        // Body direction: from OF or from Gazebo odometry (debug mode)
        float dx = dir_x_, dy = dir_y_, dz = dir_z_;
        if (use_odom_dir_) {
            float odom_speed = std::sqrt(odom_vx_ * odom_vx_ + odom_vy_ * odom_vy_
                                         + odom_vz_ * odom_vz_);
            if (odom_speed > 0.5f) {
                dx = odom_vx_ / odom_speed;
                dy = odom_vy_ / odom_speed;
                dz = odom_vz_ / odom_speed;
            }
        }

        // EMA filter on direction to reject OF noise.
        // Innovation gating: reject samples that disagree >90° with filtered direction.
        if (!dir_filter_init_) {
            filt_dx_ = dx;
            filt_dy_ = dy;
            filt_dz_ = dz;
            dir_filter_init_ = true;
        } else {
            float dot = dx * filt_dx_ + dy * filt_dy_ + dz * filt_dz_;
            if (dot > 0.f) {
                // Accept: apply EMA
                float a = direction_alpha_;
                filt_dx_ = a * dx + (1.f - a) * filt_dx_;
                filt_dy_ = a * dy + (1.f - a) * filt_dy_;
                filt_dz_ = a * dz + (1.f - a) * filt_dz_;
                // Renormalize
                float norm = std::sqrt(filt_dx_ * filt_dx_ + filt_dy_ * filt_dy_
                                       + filt_dz_ * filt_dz_);
                if (norm > 1e-6f) {
                    filt_dx_ /= norm;
                    filt_dy_ /= norm;
                    filt_dz_ /= norm;
                }
                dir_rejected_count_ = 0;
            } else {
                // Reject: keep filtered direction, count consecutive rejections
                ++dir_rejected_count_;
                if (dir_rejected_count_ == 10) {
                    RCLCPP_WARN(get_logger(),
                        "Direction filter: 10 consecutive rejections — OF may have flipped");
                }
            }
        }
        dx = filt_dx_;
        dy = filt_dy_;
        dz = filt_dz_;

        // Compute NED velocity from spool + direction + attitude
        float speed = spool_velocity_;
        float vx_body = speed * dx;
        float vy_body = speed * dy;
        float vz_body = speed * dz;

        // Direction from sensors is in SDF/FLU body frame (X=fwd, Y=left, Z=up).
        // PX4 VehicleAttitude.q rotates FRD → NED.
        // Convert FLU → FRD: negate Y and Z.
        tf2::Vector3 v_body_frd(vx_body, -vy_body, -vz_body);
        tf2::Vector3 v_ned = tf2::quatRotate(att_q_, v_body_frd);

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

        // Publish EKF state for TERCOM search center
        auto state_msg = geometry_msgs::msg::PointStamped();
        state_msg.header.stamp = current_time;
        state_msg.header.frame_id = "odom";
        state_msg.point.x = ekf_state_.x(0);
        state_msg.point.y = ekf_state_.x(1);
        state_pub_->publish(state_msg);

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

        // Publish EKF-derived position to PX4 (XY from position EKF, Z from terrain)
        // Note: publishTerrainAltitude() already called above for Z-only path.
        // This path adds XY position from the initialized EKF.
        if (feed_px4_ && odom_pub_) {
            auto odom = px4_msgs::msg::VehicleOdometry();
            odom.timestamp = px4_timestamp_us_;
            odom.timestamp_sample = px4_timestamp_us_;

            odom.position[0] = ekf_state_.x(0);
            odom.position[1] = ekf_state_.x(1);
            odom.position[2] = std::nanf("");  // Z handled by publishTerrainAltitude()
            odom.position_variance[0] = position_variance_to_px4_;
            odom.position_variance[1] = position_variance_to_px4_;
            odom.position_variance[2] = 0.f;

            odom.velocity[0] = std::nanf("");
            odom.velocity[1] = std::nanf("");
            odom.velocity[2] = std::nanf("");

            odom.q[0] = std::nanf("");
            odom.q[1] = std::nanf("");
            odom.q[2] = std::nanf("");
            odom.q[3] = std::nanf("");
            odom.angular_velocity[0] = std::nanf("");
            odom.angular_velocity[1] = std::nanf("");
            odom.angular_velocity[2] = std::nanf("");

            odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
            odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
            odom.quality = 50;

            odom_pub_->publish(odom);
        }
    }

    /// Compute and publish terrain-anchored NED Z on /position_ekf/terrain_z.
    /// fiber_vision_fusion subscribes and includes it in VehicleOdometry to PX4.
    void publishTerrainAltitude() {
        if (!terrain_alt_enabled_ || !terrain_z_pub_) return;

        if (terrain_alt_calibrated_ && rangefinder_fresh()) {
            float terrain_gz = terrain_map_.height_at(ekf_x_, ekf_y_);
            float terrain_delta = terrain_gz - ground_terrain_gz_;
            float raw_z = -(rangefinder_agl_ + terrain_delta);
            last_terrain_z_ = raw_z;
            terrain_z_valid_ = true;
        }

        // Always publish when we have a value (hold last when rangefinder goes stale)
        if (terrain_z_valid_) {
            auto msg = std_msgs::msg::Float64();
            msg.data = last_terrain_z_;
            terrain_z_pub_->publish(msg);
        }

        // Diagnostic log at ~1Hz (every 50 calls at 50Hz)
        if (terrain_alt_calibrated_ && ++terrain_diag_count_ >= 50) {
            terrain_diag_count_ = 0;
            RCLCPP_INFO(get_logger(),
                "[terrain_z] z=%.1f agl=%.1f rf_fresh=%d valid=%d ekf_pos=(%.0f,%.0f)",
                last_terrain_z_, rangefinder_agl_, rangefinder_fresh(),
                terrain_z_valid_, ekf_x_, ekf_y_);
        }
    }

    bool rangefinder_fresh() const {
        if (!rangefinder_valid_) return false;
        return (now() - rangefinder_time_).seconds() < rangefinder_max_age_;
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

        // Debug: compare OF direction, filtered direction, and odometry direction
        float odom_speed = std::sqrt(odom_vx_ * odom_vx_ + odom_vy_ * odom_vy_
                                     + odom_vz_ * odom_vz_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "PosEKF: pos=(%.1f,%.1f) vel=(%.1f,%.1f) w=(%.1f,%.1f) | "
            "dir_raw=(%.2f,%.2f,%.2f) dir_filt=(%.2f,%.2f,%.2f) dir_odom=(%.2f,%.2f,%.2f) "
            "rej=%d spd=%.1f%s",
            pos[0], pos[1], vel[0], vel[1], w[0], w[1],
            dir_x_, dir_y_, dir_z_,
            filt_dx_, filt_dy_, filt_dz_,
            odom_speed > 0.5f ? odom_vx_ / odom_speed : 0.f,
            odom_speed > 0.5f ? odom_vy_ / odom_speed : 0.f,
            odom_speed > 0.5f ? odom_vz_ / odom_speed : 0.f,
            dir_rejected_count_, spool_velocity_,
            use_odom_dir_ ? " [ODOM_DIR]" : "");
    }

    // Parameters
    bool enabled_;
    bool feed_px4_;
    float position_variance_to_px4_;
    std::string model_name_;
    bool use_odom_dir_;
    float direction_alpha_;
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr terrain_z_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr state_pub_;

    // Subscribers
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr ekf_flags_sub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::CableStatus>::SharedPtr cable_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tercom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr dist_sensor_sub_;

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
    uint64_t px4_timestamp_us_{0};
    bool gps_healthy_{true};
    bool was_gps_healthy_{true};
    bool ekf_flags_received_{false};
    float cable_deployed_{0.f};
    bool cable_valid_{false};
    float odom_vx_{0.f}, odom_vy_{0.f}, odom_vz_{0.f};

    // EMA-filtered direction (body frame)
    float filt_dx_{0.f}, filt_dy_{0.f}, filt_dz_{1.f};  // init forward (+Z in FLU tailsitter)
    bool dir_filter_init_{false};
    int dir_rejected_count_{0};

    std::optional<rclcpp::Time> spool_time_;
    std::optional<rclcpp::Time> dir_time_;
    std::optional<rclcpp::Time> last_update_time_;

    // Terrain-anchored altitude
    TerrainMap terrain_map_;
    bool terrain_map_loaded_{false};
    bool terrain_alt_enabled_{false};
    float terrain_alt_variance_{4.f};
    float ground_terrain_gz_{0.f};
    bool terrain_alt_calibrated_{false};
    float rangefinder_agl_{0.f};
    bool rangefinder_valid_{false};
    rclcpp::Time rangefinder_time_{0, 0, RCL_ROS_TIME};
    float rangefinder_max_age_{0.5f};
    float terrain_z_tau_{0.5f};       // unused (filter removed)
    float last_terrain_z_{0.f};       // last computed terrain Z (NED)
    bool terrain_z_valid_{false};     // at least one valid computation done
    int terrain_diag_count_{0};       // counter for 1Hz diagnostic log
};

}  // namespace fiber_nav_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_fusion::PositionEkfNode>());
    rclcpp::shutdown();
    return 0;
}
