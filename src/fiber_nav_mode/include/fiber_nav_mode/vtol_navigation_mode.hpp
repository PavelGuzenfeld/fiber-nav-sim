#pragma once

#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <fiber_nav_mode/terrain_altitude_controller.hpp>
#include <fiber_nav_sensors/msg/cable_status.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fiber_nav_mode {

struct CableMonitorConfig
{
    bool enabled = false;
    float tension_warn_percent = 70.f;   // % of breaking_strength → log warning
    float tension_abort_percent = 85.f;  // % of breaking_strength → abort to return
    float breaking_strength = 50.f;      // N (must match cable_dynamics_node param)
    float spool_capacity = -1.f;         // m (-1 = don't monitor spool length)
    float spool_warn_percent = 80.f;     // % deployed → log warning
    float spool_abort_percent = 95.f;    // % deployed → abort to return
};

struct GpsDeniedConfig
{
    bool enabled = false;
    float wp_time_s = 30.f;           // seconds to fly before accepting WP
    float return_time_s = 120.f;      // seconds of FW return flight before MC transition
    float descent_time_s = 60.f;      // seconds of MC descent before DONE
    float altitude_kp = 0.5f;         // P-gain for altitude hold
    float altitude_max_vz = 3.f;      // max vertical rate [m/s]
    float fw_speed = 18.f;            // FW cruise speed [m/s] for velocity setpoints
    float return_heading = NAN;       // pre-computed return heading [rad] (auto-computed if NAN)

    // Position-based navigation (when position_ekf is available)
    bool use_position_ekf = false;       // Enable position-based steering
    float ekf_wp_accept_radius = 80.f;   // WP acceptance radius [m]
    float ekf_home_accept_radius = 100.f;// Home acceptance radius [m]
    float ekf_max_uncertainty = 200.f;   // Fallback to time-based if sigma > this [m]
};

struct GimbalAccommodationConfig
{
    bool enabled = false;
    float saturation_threshold = 0.7f;   // Start throttling turns above this [0..1]
    float base_turn_rate = 5.f;          // [deg/s] max turn rate when gimbal is free
    float min_turn_rate = 1.f;           // [deg/s] min turn rate when gimbal is saturated
};

struct VtolNavConfig
{
    float cruise_alt_m = 50.f;           // AGL [m]
    float climb_rate = 2.f;              // [m/s]
    float fw_accept_radius = 50.f;       // [m] flythrough radius for FW waypoints
    float mc_transition_dist = 200.f;    // [m] transition to MC when this close to home
    float mc_approach_speed = 5.f;       // [m/s]
    float fw_transition_timeout = 30.f;  // [s]
    float mc_transition_timeout = 60.f;  // [s]
    TerrainFollowConfig terrain_follow;
    CableMonitorConfig cable_monitor;
    GpsDeniedConfig gps_denied;
    GimbalAccommodationConfig gimbal;
};

struct VtolWaypoint
{
    float x;                // NED north [m]
    float y;                // NED east [m]
    float heading;          // [rad] (optional, NaN = don't care)
    float acceptance_radius;// [m] (0 = use config default)
};

/// Generalized VTOL navigation mode with automatic MC/FW mode switching.
///
/// State machine:
///   MC_CLIMB → TRANSITION_FW → FW_NAVIGATE → FW_RETURN →
///   TRANSITION_MC → MC_APPROACH → DONE
///
/// FW navigation uses PX4's NPFG+TECS controllers via course+altitude setpoints.
/// RTL is done in FW mode until close enough to home for safe MC transition.
class VtolNavigationMode : public px4_ros2::ModeBase
{
public:
    static constexpr auto kModeName = "FiberNav VTOL Navigation";
    static constexpr float kUpdateRate = 50.f;
    static constexpr float kAltitudeTolerance = 2.f;   // [m]
    static constexpr float kHomeApproachDist = 5.f;     // [m] within this = DONE
    static constexpr float kReturnTurnRate = 3.f;       // [deg/s] max course change rate during FW_RETURN
    static constexpr float kFwLogInterval = 10.f;       // [s] periodic FW status log (diagnostic)
    static constexpr float kCableWarnInterval = 5.f;    // [s] periodic cable warning log

    enum class State
    {
        McClimb,
        TransitionFw,
        FwNavigate,
        FwReturn,
        TransitionMc,
        McApproach,
        Done,
    };

    explicit VtolNavigationMode(rclcpp::Node& node, const VtolNavConfig& config,
                                const std::string& topic_namespace_prefix = "")
        : ModeBase(node, Settings{kModeName}, topic_namespace_prefix)
        , config_(config)
    {
        setSetpointUpdateRate(kUpdateRate);

        // Create setpoint types (first created = initial active)
        trajectory_sp_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        fw_sp_ = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
        goto_sp_ = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);

        // VTOL transition helper
        vtol_ = std::make_shared<px4_ros2::VTOL>(*this);

        // Position feedback
        local_pos_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        // Subscribe to global position directly (not via OdometryGlobalPosition)
        // to avoid setting global_position mode requirement — we can fly without GNSS
        global_pos_sub_ = node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "fmu/out/vehicle_global_position",
            rclcpp::QoS(1).best_effort(),
            [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
                last_global_pos_ = *msg;
                global_pos_valid_ = true;
            });

        // Cable tension monitor
        if (config_.cable_monitor.enabled) {
            cable_sub_ = node.create_subscription<fiber_nav_sensors::msg::CableStatus>(
                "/cable/status", rclcpp::QoS(1).best_effort(),
                [this](fiber_nav_sensors::msg::CableStatus::UniquePtr msg) {
                    last_cable_status_ = *msg;
                    cable_status_valid_ = true;
                });
        }

        // Position EKF subscription (for closed-loop GPS-denied navigation)
        if (config_.gps_denied.use_position_ekf) {
            ekf_pos_sub_ = node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/position_ekf/estimate", rclcpp::QoS(1).best_effort(),
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
                    ekf_pos_x_ = static_cast<float>(msg->pose.pose.position.x);
                    ekf_pos_y_ = static_cast<float>(msg->pose.pose.position.y);
                    ekf_pos_sigma_x_ = static_cast<float>(std::sqrt(msg->pose.covariance[0]));
                    ekf_pos_sigma_y_ = static_cast<float>(std::sqrt(msg->pose.covariance[7]));
                    ekf_pos_valid_ = true;
                });
        }

        // Gimbal saturation subscription (for turn rate accommodation)
        if (config_.gimbal.enabled) {
            gimbal_sat_sub_ = node.create_subscription<std_msgs::msg::Float64>(
                "/gimbal/saturation", rclcpp::QoS(1).best_effort(),
                [this](std_msgs::msg::Float64::UniquePtr msg) {
                    gimbal_saturation_ = static_cast<float>(msg->data);
                });
        }

        // Terrain-following controller (subscribes to VehicleLocalPosition internally)
        terrain_ctrl_ = std::make_unique<TerrainAltitudeController>(node, config.terrain_follow);
    }

    void setWaypoints(std::vector<VtolWaypoint> waypoints)
    {
        waypoints_ = std::move(waypoints);
        computeLegHeadings();
    }

    /// Compute fixed heading for each waypoint leg from waypoint geometry.
    /// Heading for leg i = atan2(wp[i].y - prev.y, wp[i].x - prev.x)
    /// where prev = (0,0) for the first leg.
    static std::vector<float> computeLegHeadingsFromWaypoints(
        const std::vector<VtolWaypoint>& waypoints)
    {
        std::vector<float> headings;
        headings.reserve(waypoints.size());
        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            const float from_x = (i == 0) ? 0.f : waypoints[i - 1].x;
            const float from_y = (i == 0) ? 0.f : waypoints[i - 1].y;
            headings.push_back(
                std::atan2(waypoints[i].y - from_y, waypoints[i].x - from_x));
        }
        return headings;
    }

    /// Compute return heading from last waypoint back to home (0,0).
    /// If config provides an explicit return_heading, use that.
    static float computeReturnHeading(
        const std::vector<VtolWaypoint>& waypoints, float config_heading)
    {
        if (!std::isnan(config_heading)) {
            return config_heading;
        }
        if (waypoints.empty()) {
            return 0.f;  // no waypoints, head north
        }
        const auto& last = waypoints.back();
        return std::atan2(-last.y, -last.x);
    }

    /// Altitude P-controller for GPS-denied flight.
    /// Returns vertical velocity to hold cruise altitude.
    static float altitudeHoldVz(float cruise_alt_m, float current_agl,
                                float kp, float max_vz)
    {
        const float error = cruise_alt_m - current_agl;
        return std::clamp(error * kp, -max_vz, max_vz);
    }

    void onActivate() override
    {
        state_ = State::McClimb;
        current_wp_index_ = 0;
        state_elapsed_ = 0.f;
        wp_leg_elapsed_ = 0.f;
        return_course_primed_ = false;

        // Record AMSL reference: ground-level AMSL = current AMSL - current AGL
        const float current_alt_agl = -local_pos_->positionNed().z();
        if (global_pos_valid_) {
            alt_amsl_ref_ = static_cast<float>(last_global_pos_.alt) - current_alt_agl;
        } else {
            alt_amsl_ref_ = 0.f;
            RCLCPP_WARN(node().get_logger(),
                "Global position not valid, using 0 as AMSL reference");
        }

        if (waypoints_.empty()) {
            RCLCPP_WARN(node().get_logger(),
                "No waypoints set, skipping to FW_RETURN");
            state_ = State::FwReturn;
        }

        RCLCPP_INFO(node().get_logger(),
            "VTOL navigation activated: %zu waypoints, cruise_alt=%.0fm, "
            "amsl_ref=%.1fm, gps_denied=%s",
            waypoints_.size(), config_.cruise_alt_m, alt_amsl_ref_,
            config_.gps_denied.enabled ? "ON" : "OFF");

        if (config_.gps_denied.enabled && !waypoints_.empty()) {
            RCLCPP_INFO(node().get_logger(),
                "GPS-denied: wp_time=%.0fs, return_time=%.0fs, descent_time=%.0fs, "
                "fw_speed=%.0fm/s, return_hdg=%.1fdeg, use_ekf=%s",
                config_.gps_denied.wp_time_s, config_.gps_denied.return_time_s,
                config_.gps_denied.descent_time_s, config_.gps_denied.fw_speed,
                return_heading_ * 180.f / static_cast<float>(M_PI),
                config_.gps_denied.use_position_ekf ? "ON" : "OFF");
        }
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(),
            "VTOL navigation deactivated in state %s, wp %zu/%zu",
            stateToString(state_), current_wp_index_, waypoints_.size());
    }

    void updateSetpoint(float dt_s) override
    {
        dt_s_ = dt_s;
        state_elapsed_ += dt_s;

        // Check cable tension before any state updates
        if (checkCableTension(dt_s)) return;

        // Pass velocity info to terrain controller for look-ahead queries
        const auto vel = local_pos_->velocityNed();
        const float vx = vel.x();
        const float vy = vel.y();
        const float gspd = std::hypot(vx, vy);
        terrain_ctrl_->update(dt_s, gspd, vx, vy, currentAmsl());

        switch (state_) {
        case State::McClimb:
            updateMcClimb();
            break;
        case State::TransitionFw:
            updateTransitionFw();
            break;
        case State::FwNavigate:
            updateFwNavigate();
            break;
        case State::FwReturn:
            updateFwReturn();
            break;
        case State::TransitionMc:
            updateTransitionMc();
            break;
        case State::McApproach:
            updateMcApproach();
            break;
        case State::Done:
            completed(px4_ros2::Result::Success);
            break;
        }
    }

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] std::size_t currentWaypointIndex() const { return current_wp_index_; }
    [[nodiscard]] std::size_t waypointCount() const { return waypoints_.size(); }

    /// Compute course angle [rad] from current position to target (x, y) in NED.
    /// Returns atan2(east, north) which gives heading from north, CW positive.
    static float courseToTarget(const Eigen::Vector3f& pos, float target_x, float target_y)
    {
        return std::atan2(target_y - pos.y(), target_x - pos.x());
    }

    /// Normalize angle to [-π, π].
    static float normalizeAngle(float rad)
    {
        rad = std::fmod(rad + static_cast<float>(M_PI), 2.f * static_cast<float>(M_PI));
        if (rad < 0.f) rad += 2.f * static_cast<float>(M_PI);
        return rad - static_cast<float>(M_PI);
    }

    /// Shortest signed angular difference, wrapping correctly around ±π.
    static float angleDiff(float target, float current)
    {
        return normalizeAngle(target - current);
    }

    /// Rate-limited course towards target. Steps toward target_course at most
    /// max_rate_deg_s * dt radians per call. Returns the new course [rad].
    static float rateLimitedCourse(float current_course, float target_course,
                                    float max_rate_deg_s, float dt_s)
    {
        const float max_delta = max_rate_deg_s * static_cast<float>(M_PI) / 180.f * dt_s;
        const float diff = angleDiff(target_course, current_course);
        const float delta = std::clamp(diff, -max_delta, max_delta);
        return normalizeAngle(current_course + delta);
    }

    /// Compute effective turn rate [deg/s] based on gimbal saturation.
    /// When saturation exceeds the threshold, linearly reduce from base_rate
    /// to min_rate. This slows turns to keep the gimbal within its operating range.
    static float effectiveTurnRate(float saturation, float threshold,
                                    float base_rate, float min_rate)
    {
        if (saturation <= threshold) {
            return base_rate;
        }
        // Linear interpolation: threshold → base_rate, 1.0 → min_rate
        const float t = std::clamp(
            (saturation - threshold) / (1.f - threshold), 0.f, 1.f);
        return std::lerp(base_rate, min_rate, t);
    }

    /// Horizontal distance from current position to target (x, y).
    static float horizontalDistTo(const Eigen::Vector3f& pos, float target_x, float target_y)
    {
        return std::hypot(target_x - pos.x(), target_y - pos.y());
    }

    static const char* stateToString(State s)
    {
        switch (s) {
        case State::McClimb:       return "MC_CLIMB";
        case State::TransitionFw:  return "TRANSITION_FW";
        case State::FwNavigate:    return "FW_NAVIGATE";
        case State::FwReturn:      return "FW_RETURN";
        case State::TransitionMc:  return "TRANSITION_MC";
        case State::McApproach:    return "MC_APPROACH";
        case State::Done:          return "DONE";
        }
        return "UNKNOWN";
    }

private:
    float targetAltAmsl() const
    {
        return alt_amsl_ref_ + config_.cruise_alt_m;
    }

    float currentAltAgl() const
    {
        return -local_pos_->positionNed().z();
    }

    float currentAmsl() const
    {
        return global_pos_valid_
            ? static_cast<float>(last_global_pos_.alt)
            : (alt_amsl_ref_ - local_pos_->positionNed().z());
    }

    void transitionTo(State new_state)
    {
        RCLCPP_INFO(node().get_logger(), "%s -> %s (elapsed %.1fs)",
            stateToString(state_), stateToString(new_state), state_elapsed_);
        state_ = new_state;
        state_elapsed_ = 0.f;
        last_fw_log_time_ = 0.f;
    }

    /// Check if vehicle is no longer in FW mode (quad-chute or other MC reversion).
    /// Returns true if detected and state was changed.
    bool checkQuadchute()
    {
        const auto vtol_state = vtol_->getCurrentState();
        // Detect any state that is NOT FixedWing while we expect FW flight.
        // Includes: Multicopter, TransitionToMulticopter, TransitionToFixedWing.
        // Excludes: Undefined (stale subscription), FixedWing (expected).
        if (vtol_state != px4_ros2::VTOL::State::FixedWing &&
            vtol_state != px4_ros2::VTOL::State::Undefined)
        {
            const auto pos = local_pos_->positionNed();
            RCLCPP_WARN(node().get_logger(),
                "VTOL no longer in FW mode (state=%d) in %s! "
                "pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm. "
                "Switching to MC_APPROACH.",
                static_cast<int>(vtol_state), stateToString(state_),
                pos.x(), pos.y(), currentAltAgl(),
                horizontalDistTo(pos, 0.f, 0.f));
            transitionTo(State::McApproach);
            return true;
        }
        return false;
    }

    /// Check cable tension and abort mission if threshold exceeded.
    /// Returns true if a state transition was triggered (caller should return).
    bool checkCableTension(float dt_s)
    {
        if (!config_.cable_monitor.enabled || !cable_status_valid_) {
            return false;
        }

        const auto& cable = last_cable_status_;
        const float breaking = config_.cable_monitor.breaking_strength;

        // Cable already broken — emergency MC approach
        if (cable.is_broken) {
            if (!cable_abort_triggered_) {
                cable_abort_triggered_ = true;
                RCLCPP_ERROR(node().get_logger(),
                    "CABLE BROKEN! tension=%.1fN deployed=%.1fm. "
                    "Aborting to MC_APPROACH.",
                    cable.tension, cable.deployed_length);
                transitionTo(State::McApproach);
            }
            return true;
        }

        const float abort_threshold = breaking * config_.cable_monitor.tension_abort_percent / 100.f;
        const float warn_threshold = breaking * config_.cable_monitor.tension_warn_percent / 100.f;

        // Abort threshold exceeded
        if (cable.tension > abort_threshold && !cable_abort_triggered_) {
            cable_abort_triggered_ = true;
            RCLCPP_WARN(node().get_logger(),
                "Cable tension ABORT: %.1fN > %.1fN (%.0f%% of %.0fN). "
                "deployed=%.1fm state=%s",
                cable.tension, abort_threshold,
                config_.cable_monitor.tension_abort_percent, breaking,
                cable.deployed_length, stateToString(state_));

            switch (state_) {
            case State::FwNavigate:
                transitionTo(State::FwReturn);
                break;
            case State::McClimb:
            case State::TransitionFw:
                transitionTo(State::McApproach);
                break;
            default:
                // FwReturn, TransitionMc, McApproach, Done — already returning
                break;
            }
            return true;
        }

        // Warning threshold — log periodically
        if (cable.tension > warn_threshold && cable.tension <= abort_threshold) {
            cable_warn_elapsed_ += dt_s;
            if (cable_warn_elapsed_ >= kCableWarnInterval) {
                cable_warn_elapsed_ = 0.f;
                RCLCPP_WARN(node().get_logger(),
                    "Cable tension WARNING: %.1fN > %.1fN (%.0f%% of %.0fN). "
                    "deployed=%.1fm",
                    cable.tension, warn_threshold,
                    config_.cable_monitor.tension_warn_percent, breaking,
                    cable.deployed_length);
            }
        } else {
            cable_warn_elapsed_ = 0.f;
        }

        // Spool exhaustion check (only if capacity is configured)
        if (config_.cable_monitor.spool_capacity > 0.f) {
            const float deployed = cable.deployed_length;
            const float capacity = config_.cable_monitor.spool_capacity;
            const float deployed_percent = (deployed / capacity) * 100.f;
            const float abort_pct = config_.cable_monitor.spool_abort_percent;
            const float warn_pct = config_.cable_monitor.spool_warn_percent;

            if (deployed_percent >= abort_pct && !spool_abort_triggered_) {
                spool_abort_triggered_ = true;
                RCLCPP_WARN(node().get_logger(),
                    "Spool exhaustion ABORT: %.0fm / %.0fm (%.0f%% >= %.0f%%). "
                    "remaining=%.0fm state=%s",
                    deployed, capacity, deployed_percent, abort_pct,
                    capacity - deployed, stateToString(state_));

                switch (state_) {
                case State::FwNavigate:
                    transitionTo(State::FwReturn);
                    break;
                case State::McClimb:
                case State::TransitionFw:
                    transitionTo(State::McApproach);
                    break;
                default:
                    break;
                }
                return true;
            }

            if (deployed_percent >= warn_pct && deployed_percent < abort_pct) {
                spool_warn_elapsed_ += dt_s;
                if (spool_warn_elapsed_ >= kCableWarnInterval) {
                    spool_warn_elapsed_ = 0.f;
                    RCLCPP_WARN(node().get_logger(),
                        "Spool WARNING: %.0fm / %.0fm (%.0f%%). remaining=%.0fm",
                        deployed, capacity, deployed_percent, capacity - deployed);
                }
            } else {
                spool_warn_elapsed_ = 0.f;
            }
        }

        return false;
    }

    /// Get dead-reckoned position if available and trustworthy.
    /// Returns nullopt if EKF not enabled, not valid, or uncertainty too high.
    std::optional<Eigen::Vector2f> drPosition() const
    {
        if (!config_.gps_denied.use_position_ekf || !ekf_pos_valid_) {
            return std::nullopt;
        }
        float max_sigma = std::max(ekf_pos_sigma_x_, ekf_pos_sigma_y_);
        if (max_sigma > config_.gps_denied.ekf_max_uncertainty) {
            return std::nullopt;
        }
        return Eigen::Vector2f{ekf_pos_x_, ekf_pos_y_};
    }

    static const char* vtolStateToString(px4_ros2::VTOL::State s)
    {
        switch (s) {
        case px4_ros2::VTOL::State::Undefined:              return "Undefined";
        case px4_ros2::VTOL::State::TransitionToFixedWing:  return "TransToFW";
        case px4_ros2::VTOL::State::TransitionToMulticopter:return "TransToMC";
        case px4_ros2::VTOL::State::Multicopter:            return "MC";
        case px4_ros2::VTOL::State::FixedWing:              return "FW";
        }
        return "?";
    }

    /// Periodic status log during FW flight (every kFwLogInterval seconds).
    void logFwStatusPeriodic(const Eigen::Vector3f& pos, float cmd_course = NAN)
    {
        if (state_elapsed_ - last_fw_log_time_ >= kFwLogInterval) {
            last_fw_log_time_ = state_elapsed_;

            const auto vel = local_pos_->velocityNed();
            const float gspd = std::hypot(vel.x(), vel.y());
            const float vel_hdg = std::atan2(vel.y(), vel.x()) * 180.f / static_cast<float>(M_PI);
            const float px4_hdg = local_pos_->heading() * 180.f / static_cast<float>(M_PI);
            const float cmd_deg = std::isnan(cmd_course) ? NAN
                : cmd_course * 180.f / static_cast<float>(M_PI);

            if (terrain_ctrl_->isActive()) {
                const auto dist = terrain_ctrl_->filteredDistBottom();
                const auto err = terrain_ctrl_->aglError();
                RCLCPP_INFO(node().get_logger(),
                    "[%s] pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm "
                    "px4_hdg=%.1f vel_hdg=%.1f cmd=%.1f gspd=%.1f vtol=%s "
                    "terrain_agl=%.1fm err=%.1fm%s",
                    stateToString(state_), pos.x(), pos.y(),
                    currentAltAgl(), horizontalDistTo(pos, 0.f, 0.f),
                    px4_hdg, vel_hdg, cmd_deg, gspd,
                    vtolStateToString(vtol_->getCurrentState()),
                    dist.value_or(0.f), err.value_or(0.f),
                    terrain_ctrl_->isEmergencyClimbActive() ? " SAFETY_CLIMB" : "");
            } else {
                RCLCPP_INFO(node().get_logger(),
                    "[%s] pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm "
                    "px4_hdg=%.1f vel_hdg=%.1f cmd=%.1f gspd=%.1f vtol=%s",
                    stateToString(state_), pos.x(), pos.y(),
                    currentAltAgl(), horizontalDistTo(pos, 0.f, 0.f),
                    px4_hdg, vel_hdg, cmd_deg, gspd,
                    vtolStateToString(vtol_->getCurrentState()));
            }
        }
    }

    float wpAcceptRadius(const VtolWaypoint& wp) const
    {
        return wp.acceptance_radius > 0.f ? wp.acceptance_radius : config_.fw_accept_radius;
    }

    // --- PX4 parameter helpers ---

    /// Set a PX4 parameter at runtime via px4-param CLI.
    /// Returns true on success. Only works when running in the same
    /// container as PX4 SITL (shared /tmp for uORB).
    static bool setPx4Param(const char* name, const char* value,
                            rclcpp::Logger logger)
    {
        const std::string cmd =
            "cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs && "
            "../bin/px4-param set " + std::string(name) + " " + value +
            " > /dev/null 2>&1";
        const int rc = std::system(cmd.c_str());
        if (rc != 0) {
            RCLCPP_WARN(logger, "Failed to set PX4 param %s=%s (rc=%d)",
                name, value, rc);
            return false;
        }
        return true;
    }

    /// Disable GPS fusion and set failsafe params for GPS-denied flight.
    /// Called once after MC climb reaches cruise altitude (home position
    /// already established from GPS).
    void disableGpsForDeniedFlight()
    {
        auto logger = node().get_logger();
        RCLCPP_INFO(logger, "Disabling GPS for GPS-denied flight...");

        // Widen position/velocity failsafe thresholds (prevent RTL on GPS loss)
        setPx4Param("COM_POS_FS_EPH", "9999", logger);
        setPx4Param("COM_POS_FS_EPV", "9999", logger);
        setPx4Param("COM_VEL_FS_EVH", "9999", logger);

        // Increase EV velocity noise (without GPS anchor, velocity innovations
        // grow and get rejected at the default 0.15 noise level)
        setPx4Param("EKF2_EVV_NOISE", "1.0", logger);

        // Disable GPS fusion — triggers position_ekf_node initialization
        setPx4Param("EKF2_GPS_CTRL", "0", logger);

        RCLCPP_INFO(logger, "GPS disabled (EKF2_GPS_CTRL=0), failsafes widened");
    }

    // --- State update functions ---

    void updateMcClimb()
    {
        // Climb vertically at configured rate, yaw toward first WP
        // so FW transition pitches toward the target
        const Eigen::Vector3f vel{0.f, 0.f, -config_.climb_rate};
        float yaw = 0.f;
        if (!waypoints_.empty()) {
            yaw = courseToTarget(local_pos_->positionNed(),
                waypoints_[0].x, waypoints_[0].y);
        }
        trajectory_sp_->update(vel, std::nullopt, yaw);

        // Check if we've reached cruise altitude
        if (currentAltAgl() >= config_.cruise_alt_m - kAltitudeTolerance) {
            // Disable GPS before FW transition (home position established)
            if (config_.gps_denied.enabled && !gps_disabled_) {
                gps_disabled_ = true;
                disableGpsForDeniedFlight();
            }
            transitionTo(State::TransitionFw);
        }
    }

    void updateTransitionFw()
    {
        vtol_->toFixedwing();

        const auto vtol_state = vtol_->getCurrentState();

        if (vtol_state == px4_ros2::VTOL::State::FixedWing) {
            // Transition complete
            if (!waypoints_.empty()) {
                transitionTo(State::FwNavigate);
            } else {
                transitionTo(State::FwReturn);
            }
            return;
        }

        // Timeout check
        if (state_elapsed_ > config_.fw_transition_timeout) {
            RCLCPP_WARN(node().get_logger(),
                "FW transition timeout (%.1fs), vtol_state=%s, aborting to MC_APPROACH",
                state_elapsed_, vtolStateToString(vtol_state));
            transitionTo(State::McApproach);
            return;
        }

        // During transition: send both trajectory and FW setpoints
        // Trajectory: hold altitude with zero vertical velocity
        const Eigen::Vector3f vel{NAN, NAN, 0.f};
        const Eigen::Vector3f accel = vtol_->computeAccelerationSetpointDuringTransition();
        trajectory_sp_->update(vel, accel);

        // FW: hold height rate 0, course toward first waypoint (or home)
        float course = 0.f;
        if (!waypoints_.empty()) {
            course = courseToTarget(local_pos_->positionNed(),
                waypoints_[0].x, waypoints_[0].y);
        }
        fw_sp_->updateWithHeightRate(0.f, course);
    }

    void updateFwNavigate()
    {
        if (checkQuadchute()) return;

        if (current_wp_index_ >= waypoints_.size()) {
            transitionTo(State::FwReturn);
            return;
        }

        const auto& wp = waypoints_[current_wp_index_];
        const auto pos = local_pos_->positionNed();

        if (config_.gps_denied.enabled) {
            // GPS-denied navigation: position-based steering with time-based fallback
            float course;
            bool wp_reached = false;

            auto dr_pos = drPosition();
            if (dr_pos.has_value()) {
                // Position-based steering: course toward WP
                course = std::atan2(wp.y - dr_pos->y(), wp.x - dr_pos->x());

                // Distance-based WP acceptance
                const float dist = std::hypot(wp.x - dr_pos->x(), wp.y - dr_pos->y());
                wp_reached = (dist < config_.gps_denied.ekf_wp_accept_radius);
            } else {
                // Fallback: fixed heading (open-loop)
                course = leg_headings_[current_wp_index_];
            }

            const float height_rate = altitudeHoldVz(
                config_.cruise_alt_m, currentAltAgl(),
                config_.gps_denied.altitude_kp, config_.gps_denied.altitude_max_vz);
            fw_sp_->updateWithHeightRate(height_rate, course);

            logFwStatusPeriodic(pos, course);

            // Time-based fallback always active as safety net
            wp_leg_elapsed_ += 1.f / kUpdateRate;
            if (wp_reached || wp_leg_elapsed_ >= config_.gps_denied.wp_time_s) {
                RCLCPP_INFO(node().get_logger(),
                    "WP %zu accepted (%s, time=%.1fs), hdg=%.1fdeg alt_agl=%.1fm",
                    current_wp_index_,
                    wp_reached ? "distance" : "time",
                    wp_leg_elapsed_,
                    course * 180.f / static_cast<float>(M_PI), currentAltAgl());
                ++current_wp_index_;
                wp_leg_elapsed_ = 0.f;

                if (current_wp_index_ >= waypoints_.size()) {
                    RCLCPP_INFO(node().get_logger(), "All %zu waypoints reached",
                        waypoints_.size());
                    transitionTo(State::FwReturn);
                }
            }
        } else {
            // Normal GPS mode: course toward WP + distance-based acceptance.
            // No rate limiting here — PX4's NPFG controller handles smooth
            // course transitions internally. External rate limiting causes the
            // aircraft to lag behind on heading and drift off course.
            const float course = courseToTarget(pos, wp.x, wp.y);

            if (terrain_ctrl_->isActive()) {
                // Terrain-following: rate-limited AMSL altitude target.
                // Uses filtered terrain elevation (computed from raw AGL in update())
                // to avoid positive feedback loop. Rate limiter (rate_slew m/s)
                // prevents sudden altitude target jumps.
                // currentAmsl() passed for emergency safety floor check.
                const float amsl = terrain_ctrl_->computeSmoothedTargetAmsl(
                    targetAltAmsl(), dt_s_, currentAmsl());
                fw_sp_->updateWithAltitude(amsl, course);
            } else {
                fw_sp_->updateWithAltitude(targetAltAmsl(), course);
            }

            logFwStatusPeriodic(pos, course);

            const float dist = horizontalDistTo(pos, wp.x, wp.y);
            const float accept = wpAcceptRadius(wp);

            if (dist < accept) {
                RCLCPP_INFO(node().get_logger(),
                    "WP %zu reached (d=%.1fm < %.1fm), pos=(%.1f, %.1f) alt_agl=%.1fm",
                    current_wp_index_, dist, accept, pos.x(), pos.y(),
                    currentAltAgl());
                ++current_wp_index_;

                if (current_wp_index_ >= waypoints_.size()) {
                    RCLCPP_INFO(node().get_logger(), "All %zu waypoints reached",
                        waypoints_.size());
                    transitionTo(State::FwReturn);
                }
            }
        }
    }

    void updateFwReturn()
    {
        if (checkQuadchute()) return;

        const auto pos = local_pos_->positionNed();

        if (config_.gps_denied.enabled) {
            // GPS-denied return: position-based steering with time-based fallback
            float course;
            bool close_enough = false;

            auto dr_pos = drPosition();
            if (dr_pos.has_value()) {
                // Position-based steering: course toward home (0,0)
                course = std::atan2(-dr_pos->y(), -dr_pos->x());

                // Distance-based MC transition
                float dist_home = dr_pos->norm();
                close_enough = (dist_home < config_.gps_denied.ekf_home_accept_radius);
            } else {
                // Fallback: fixed return heading (open-loop)
                course = return_heading_;
            }

            const float height_rate = altitudeHoldVz(
                config_.cruise_alt_m, currentAltAgl(),
                config_.gps_denied.altitude_kp, config_.gps_denied.altitude_max_vz);
            fw_sp_->updateWithHeightRate(height_rate, course);

            logFwStatusPeriodic(pos, course);

            // Time-based fallback always active as safety net
            if (close_enough || state_elapsed_ >= config_.gps_denied.return_time_s) {
                RCLCPP_INFO(node().get_logger(),
                    "FW return complete (%s, elapsed=%.1fs), transitioning to MC",
                    close_enough ? "distance" : "time", state_elapsed_);
                transitionTo(State::TransitionMc);
            }
        } else {
            // Normal GPS mode: course toward home + distance-based transition
            const float dist_home = horizontalDistTo(pos, 0.f, 0.f);
            const float target_course = courseToTarget(pos, 0.f, 0.f);

            // Rate-limit the course change to prevent the tailsitter from
            // entering a spiral dive during the 180° turn-around.
            // The tailsitter uses differential thrust (no ailerons), so steep
            // bank angles cause rapid altitude loss.
            if (!return_course_primed_) {
                // Initialize from current velocity heading (not px4 heading which is noisy)
                const auto vel = local_pos_->velocityNed();
                return_course_smoothed_ = std::atan2(vel.y(), vel.x());
                return_course_primed_ = true;
            }
            // Modulate turn rate based on gimbal saturation: when gimbal is near
            // its physical limit, slow down the turn so the camera stays nadir.
            float turn_rate = kReturnTurnRate;
            if (config_.gimbal.enabled) {
                turn_rate = effectiveTurnRate(
                    gimbal_saturation_, config_.gimbal.saturation_threshold,
                    kReturnTurnRate, config_.gimbal.min_turn_rate);
            }
            return_course_smoothed_ = rateLimitedCourse(
                return_course_smoothed_, target_course, turn_rate, dt_s_);

            // During FW_RETURN, always use cruise altitude (no terrain following).
            // The turn causes altitude loss from banking; using full cruise
            // altitude commands TECS to climb during the turn for safety margin.
            fw_sp_->updateWithAltitude(targetAltAmsl(), return_course_smoothed_);

            logFwStatusPeriodic(pos, return_course_smoothed_);

            if (dist_home < config_.mc_transition_dist) {
                RCLCPP_INFO(node().get_logger(),
                    "Within MC transition distance (%.0fm < %.0fm), transitioning to MC",
                    dist_home, config_.mc_transition_dist);
                transitionTo(State::TransitionMc);
            }
        }
    }

    void updateTransitionMc()
    {
        vtol_->toMulticopter();

        const auto vtol_state = vtol_->getCurrentState();

        if (vtol_state == px4_ros2::VTOL::State::Multicopter) {
            transitionTo(State::McApproach);
            return;
        }

        // Timeout: PX4 may have transitioned internally
        if (state_elapsed_ > config_.mc_transition_timeout) {
            RCLCPP_WARN(node().get_logger(),
                "MC transition timeout (%.1fs), forcing MC_APPROACH",
                state_elapsed_);
            transitionTo(State::McApproach);
            return;
        }

        // During back-transition: send both setpoint types with deceleration
        const Eigen::Vector3f vel{NAN, NAN, 0.f};
        const Eigen::Vector3f accel = vtol_->computeAccelerationSetpointDuringTransition();
        trajectory_sp_->update(vel, accel);

        // FW: maintain altitude, course toward home
        const float course = courseToTarget(local_pos_->positionNed(), 0.f, 0.f);
        fw_sp_->updateWithHeightRate(0.f, course);
    }

    void updateMcApproach()
    {
        if (config_.gps_denied.enabled) {
            auto dr_pos = drPosition();
            if (dr_pos.has_value()) {
                // MC approach toward home using trajectory velocity
                float course = std::atan2(-dr_pos->y(), -dr_pos->x());
                float dist = dr_pos->norm();

                if (dist < kHomeApproachDist * 2.f) {
                    RCLCPP_INFO(node().get_logger(),
                        "GPS-denied MC_APPROACH: near home (%.1fm), delegating landing",
                        dist);
                    transitionTo(State::Done);
                } else {
                    float vn = config_.mc_approach_speed * std::cos(course);
                    float ve = config_.mc_approach_speed * std::sin(course);
                    float vd = -altitudeHoldVz(config_.cruise_alt_m, currentAltAgl(),
                        config_.gps_denied.altitude_kp, config_.gps_denied.altitude_max_vz);
                    trajectory_sp_->update(
                        Eigen::Vector3f{vn, ve, vd}, std::nullopt, course);
                }
            } else {
                // No position estimate — delegate landing to executor
                RCLCPP_INFO(node().get_logger(),
                    "GPS-denied MC_APPROACH: no position, delegating landing (alt_agl=%.1fm)",
                    currentAltAgl());
                transitionTo(State::Done);
            }
        } else {
            // Normal GPS mode: goto home position
            const Eigen::Vector3f home_pos{0.f, 0.f, -config_.cruise_alt_m};
            const float heading = courseToTarget(local_pos_->positionNed(), 0.f, 0.f);

            goto_sp_->update(home_pos, heading, config_.mc_approach_speed);

            const auto pos = local_pos_->positionNed();
            const float dist = horizontalDistTo(pos, 0.f, 0.f);

            if (dist < kHomeApproachDist) {
                RCLCPP_INFO(node().get_logger(),
                    "Over home position (d=%.1fm), navigation complete", dist);
                transitionTo(State::Done);
            }
        }
    }

    void computeLegHeadings()
    {
        leg_headings_ = computeLegHeadingsFromWaypoints(waypoints_);
        return_heading_ = computeReturnHeading(
            waypoints_, config_.gps_denied.return_heading);
    }

    // Config
    VtolNavConfig config_;

    // Setpoint types
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_sp_;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> fw_sp_;
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> goto_sp_;

    // VTOL helper
    std::shared_ptr<px4_ros2::VTOL> vtol_;

    // Terrain-following controller
    std::unique_ptr<TerrainAltitudeController> terrain_ctrl_;

    // Position feedback
    std::shared_ptr<px4_ros2::OdometryLocalPosition> local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_sub_;
    px4_msgs::msg::VehicleGlobalPosition last_global_pos_;
    bool global_pos_valid_{false};

    // State machine
    State state_{State::McClimb};
    float state_elapsed_{0.f};
    float last_fw_log_time_{0.f};
    float alt_amsl_ref_{0.f};
    float dt_s_{1.f / kUpdateRate};

    // Waypoints
    std::vector<VtolWaypoint> waypoints_;
    std::size_t current_wp_index_{0};

    // GPS-denied navigation
    std::vector<float> leg_headings_;
    float return_heading_{0.f};
    float wp_leg_elapsed_{0.f};
    bool gps_disabled_{false};

    // FW_RETURN rate-limited course (with gimbal accommodation)
    bool return_course_primed_{false};
    float return_course_smoothed_{0.f};

    // Gimbal saturation feedback
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_sat_sub_;
    float gimbal_saturation_{0.f};

    // Position EKF (GPS-denied closed-loop navigation)
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pos_sub_;
    float ekf_pos_x_{0.f};
    float ekf_pos_y_{0.f};
    float ekf_pos_sigma_x_{0.f};
    float ekf_pos_sigma_y_{0.f};
    bool ekf_pos_valid_{false};

    // Cable tension monitor
    rclcpp::Subscription<fiber_nav_sensors::msg::CableStatus>::SharedPtr cable_sub_;
    fiber_nav_sensors::msg::CableStatus last_cable_status_;
    bool cable_status_valid_{false};
    float cable_warn_elapsed_{0.f};
    bool cable_abort_triggered_{false};
    float spool_warn_elapsed_{0.f};
    bool spool_abort_triggered_{false};
};

}  // namespace fiber_nav_mode
