#pragma once

#include <cmath>
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
    static constexpr float kFwLogInterval = 30.f;       // [s] periodic FW status log
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
        , _config(config)
    {
        setSetpointUpdateRate(kUpdateRate);

        // Create setpoint types (first created = initial active)
        _trajectory_sp = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _fw_sp = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
        _goto_sp = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);

        // VTOL transition helper
        _vtol = std::make_shared<px4_ros2::VTOL>(*this);

        // Position feedback
        _local_pos = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        // Subscribe to global position directly (not via OdometryGlobalPosition)
        // to avoid setting global_position mode requirement — we can fly without GNSS
        _global_pos_sub = node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "fmu/out/vehicle_global_position",
            rclcpp::QoS(1).best_effort(),
            [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
                _last_global_pos = *msg;
                _global_pos_valid = true;
            });

        // Cable tension monitor
        if (_config.cable_monitor.enabled) {
            _cable_sub = node.create_subscription<fiber_nav_sensors::msg::CableStatus>(
                "/cable/status", rclcpp::QoS(1).best_effort(),
                [this](fiber_nav_sensors::msg::CableStatus::UniquePtr msg) {
                    _last_cable_status = *msg;
                    _cable_status_valid = true;
                });
        }

        // Position EKF subscription (for closed-loop GPS-denied navigation)
        if (_config.gps_denied.use_position_ekf) {
            _ekf_pos_sub = node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/position_ekf/estimate", rclcpp::QoS(1).best_effort(),
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
                    _ekf_pos_x = static_cast<float>(msg->pose.pose.position.x);
                    _ekf_pos_y = static_cast<float>(msg->pose.pose.position.y);
                    _ekf_pos_sigma_x = static_cast<float>(std::sqrt(msg->pose.covariance[0]));
                    _ekf_pos_sigma_y = static_cast<float>(std::sqrt(msg->pose.covariance[7]));
                    _ekf_pos_valid = true;
                });
        }

        // Terrain-following controller (subscribes to VehicleLocalPosition internally)
        _terrain_ctrl = std::make_unique<TerrainAltitudeController>(node, config.terrain_follow);
    }

    void setWaypoints(std::vector<VtolWaypoint> waypoints)
    {
        _waypoints = std::move(waypoints);
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
        _state = State::McClimb;
        _current_wp_index = 0;
        _state_elapsed = 0.f;
        _wp_leg_elapsed = 0.f;

        // Record AMSL reference: ground-level AMSL = current AMSL - current AGL
        const float current_alt_agl = -_local_pos->positionNed().z();
        if (_global_pos_valid) {
            _alt_amsl_ref = static_cast<float>(_last_global_pos.alt) - current_alt_agl;
        } else {
            _alt_amsl_ref = 0.f;
            RCLCPP_WARN(node().get_logger(),
                "Global position not valid, using 0 as AMSL reference");
        }

        if (_waypoints.empty()) {
            RCLCPP_WARN(node().get_logger(),
                "No waypoints set, skipping to FW_RETURN");
            _state = State::FwReturn;
        }

        RCLCPP_INFO(node().get_logger(),
            "VTOL navigation activated: %zu waypoints, cruise_alt=%.0fm, "
            "amsl_ref=%.1fm, gps_denied=%s",
            _waypoints.size(), _config.cruise_alt_m, _alt_amsl_ref,
            _config.gps_denied.enabled ? "ON" : "OFF");

        if (_config.gps_denied.enabled && !_waypoints.empty()) {
            RCLCPP_INFO(node().get_logger(),
                "GPS-denied: wp_time=%.0fs, return_time=%.0fs, descent_time=%.0fs, "
                "fw_speed=%.0fm/s, return_hdg=%.1fdeg",
                _config.gps_denied.wp_time_s, _config.gps_denied.return_time_s,
                _config.gps_denied.descent_time_s, _config.gps_denied.fw_speed,
                _return_heading * 180.f / static_cast<float>(M_PI));
        }
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(),
            "VTOL navigation deactivated in state %s, wp %zu/%zu",
            stateToString(_state), _current_wp_index, _waypoints.size());
    }

    void updateSetpoint(float dt_s) override
    {
        _state_elapsed += dt_s;

        // Check cable tension before any state updates
        if (checkCableTension(dt_s)) return;

        // Pass velocity info to terrain controller for look-ahead queries
        const auto vel = _local_pos->velocityNed();
        const float vx = vel.x();
        const float vy = vel.y();
        const float gspd = std::sqrt(vx * vx + vy * vy);
        _terrain_ctrl->update(dt_s, gspd, vx, vy);

        switch (_state) {
        case State::McClimb:
            updateMcClimb();
            break;
        case State::TransitionFw:
            updateTransitionFw(dt_s);
            break;
        case State::FwNavigate:
            updateFwNavigate();
            break;
        case State::FwReturn:
            updateFwReturn();
            break;
        case State::TransitionMc:
            updateTransitionMc(dt_s);
            break;
        case State::McApproach:
            updateMcApproach();
            break;
        case State::Done:
            completed(px4_ros2::Result::Success);
            break;
        }
    }

    [[nodiscard]] State state() const { return _state; }
    [[nodiscard]] std::size_t currentWaypointIndex() const { return _current_wp_index; }
    [[nodiscard]] std::size_t waypointCount() const { return _waypoints.size(); }

    /// Compute course angle [rad] from current position to target (x, y) in NED.
    /// Returns atan2(east, north) which gives heading from north, CW positive.
    static float courseToTarget(const Eigen::Vector3f& pos, float target_x, float target_y)
    {
        return std::atan2(target_y - pos.y(), target_x - pos.x());
    }

    /// Horizontal distance from current position to target (x, y).
    static float horizontalDistTo(const Eigen::Vector3f& pos, float target_x, float target_y)
    {
        const float dx = target_x - pos.x();
        const float dy = target_y - pos.y();
        return std::sqrt(dx * dx + dy * dy);
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
        return _alt_amsl_ref + _config.cruise_alt_m;
    }

    float currentAltAgl() const
    {
        return -_local_pos->positionNed().z();
    }

    float currentAmsl() const
    {
        return _global_pos_valid
            ? static_cast<float>(_last_global_pos.alt)
            : (_alt_amsl_ref - _local_pos->positionNed().z());
    }

    void transitionTo(State new_state)
    {
        RCLCPP_INFO(node().get_logger(), "%s -> %s (elapsed %.1fs)",
            stateToString(_state), stateToString(new_state), _state_elapsed);
        _state = new_state;
        _state_elapsed = 0.f;
        _last_fw_log_time = 0.f;
    }

    /// Check if vehicle is no longer in FW mode (quad-chute or other MC reversion).
    /// Returns true if detected and state was changed.
    bool checkQuadchute()
    {
        const auto vtol_state = _vtol->getCurrentState();
        // Detect any state that is NOT FixedWing while we expect FW flight.
        // Includes: Multicopter, TransitionToMulticopter, TransitionToFixedWing.
        // Excludes: Undefined (stale subscription), FixedWing (expected).
        if (vtol_state != px4_ros2::VTOL::State::FixedWing &&
            vtol_state != px4_ros2::VTOL::State::Undefined)
        {
            const auto pos = _local_pos->positionNed();
            RCLCPP_WARN(node().get_logger(),
                "VTOL no longer in FW mode (state=%d) in %s! "
                "pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm. "
                "Switching to MC_APPROACH.",
                static_cast<int>(vtol_state), stateToString(_state),
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
        if (!_config.cable_monitor.enabled || !_cable_status_valid) {
            return false;
        }

        const auto& cable = _last_cable_status;
        const float breaking = _config.cable_monitor.breaking_strength;

        // Cable already broken — emergency MC approach
        if (cable.is_broken) {
            if (!_cable_abort_triggered) {
                _cable_abort_triggered = true;
                RCLCPP_ERROR(node().get_logger(),
                    "CABLE BROKEN! tension=%.1fN deployed=%.1fm. "
                    "Aborting to MC_APPROACH.",
                    cable.tension, cable.deployed_length);
                transitionTo(State::McApproach);
            }
            return true;
        }

        const float abort_threshold = breaking * _config.cable_monitor.tension_abort_percent / 100.f;
        const float warn_threshold = breaking * _config.cable_monitor.tension_warn_percent / 100.f;

        // Abort threshold exceeded
        if (cable.tension > abort_threshold && !_cable_abort_triggered) {
            _cable_abort_triggered = true;
            RCLCPP_WARN(node().get_logger(),
                "Cable tension ABORT: %.1fN > %.1fN (%.0f%% of %.0fN). "
                "deployed=%.1fm state=%s",
                cable.tension, abort_threshold,
                _config.cable_monitor.tension_abort_percent, breaking,
                cable.deployed_length, stateToString(_state));

            switch (_state) {
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
            _cable_warn_elapsed += dt_s;
            if (_cable_warn_elapsed >= kCableWarnInterval) {
                _cable_warn_elapsed = 0.f;
                RCLCPP_WARN(node().get_logger(),
                    "Cable tension WARNING: %.1fN > %.1fN (%.0f%% of %.0fN). "
                    "deployed=%.1fm",
                    cable.tension, warn_threshold,
                    _config.cable_monitor.tension_warn_percent, breaking,
                    cable.deployed_length);
            }
        } else {
            _cable_warn_elapsed = 0.f;
        }

        // Spool exhaustion check (only if capacity is configured)
        if (_config.cable_monitor.spool_capacity > 0.f) {
            const float deployed = cable.deployed_length;
            const float capacity = _config.cable_monitor.spool_capacity;
            const float deployed_percent = (deployed / capacity) * 100.f;
            const float abort_pct = _config.cable_monitor.spool_abort_percent;
            const float warn_pct = _config.cable_monitor.spool_warn_percent;

            if (deployed_percent >= abort_pct && !_spool_abort_triggered) {
                _spool_abort_triggered = true;
                RCLCPP_WARN(node().get_logger(),
                    "Spool exhaustion ABORT: %.0fm / %.0fm (%.0f%% >= %.0f%%). "
                    "remaining=%.0fm state=%s",
                    deployed, capacity, deployed_percent, abort_pct,
                    capacity - deployed, stateToString(_state));

                switch (_state) {
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
                _spool_warn_elapsed += dt_s;
                if (_spool_warn_elapsed >= kCableWarnInterval) {
                    _spool_warn_elapsed = 0.f;
                    RCLCPP_WARN(node().get_logger(),
                        "Spool WARNING: %.0fm / %.0fm (%.0f%%). remaining=%.0fm",
                        deployed, capacity, deployed_percent, capacity - deployed);
                }
            } else {
                _spool_warn_elapsed = 0.f;
            }
        }

        return false;
    }

    /// Get dead-reckoned position if available and trustworthy.
    /// Returns nullopt if EKF not enabled, not valid, or uncertainty too high.
    std::optional<Eigen::Vector2f> drPosition() const
    {
        if (!_config.gps_denied.use_position_ekf || !_ekf_pos_valid) {
            return std::nullopt;
        }
        float max_sigma = std::max(_ekf_pos_sigma_x, _ekf_pos_sigma_y);
        if (max_sigma > _config.gps_denied.ekf_max_uncertainty) {
            return std::nullopt;
        }
        return Eigen::Vector2f{_ekf_pos_x, _ekf_pos_y};
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
    void logFwStatusPeriodic(const Eigen::Vector3f& pos)
    {
        if (_state_elapsed - _last_fw_log_time >= kFwLogInterval) {
            _last_fw_log_time = _state_elapsed;

            if (_terrain_ctrl->isActive()) {
                const auto dist = _terrain_ctrl->filteredDistBottom();
                const auto err = _terrain_ctrl->aglError();
                RCLCPP_INFO(node().get_logger(),
                    "[%s] pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm "
                    "hdg=%.1fdeg vtol=%s terrain_agl=%.1fm err=%.1fm",
                    stateToString(_state), pos.x(), pos.y(),
                    currentAltAgl(), horizontalDistTo(pos, 0.f, 0.f),
                    _local_pos->heading() * 180.f / M_PI,
                    vtolStateToString(_vtol->getCurrentState()),
                    dist.value_or(0.f), err.value_or(0.f));
            } else {
                RCLCPP_INFO(node().get_logger(),
                    "[%s] pos=(%.0f, %.0f) alt_agl=%.0fm dist_home=%.0fm "
                    "hdg=%.1fdeg vtol=%s",
                    stateToString(_state), pos.x(), pos.y(),
                    currentAltAgl(), horizontalDistTo(pos, 0.f, 0.f),
                    _local_pos->heading() * 180.f / M_PI,
                    vtolStateToString(_vtol->getCurrentState()));
            }
        }
    }

    float wpAcceptRadius(const VtolWaypoint& wp) const
    {
        return wp.acceptance_radius > 0.f ? wp.acceptance_radius : _config.fw_accept_radius;
    }

    // --- State update functions ---

    void updateMcClimb()
    {
        // Climb vertically at configured rate, yaw toward first WP
        // so FW transition pitches toward the target
        const Eigen::Vector3f vel{0.f, 0.f, -_config.climb_rate};
        float yaw = 0.f;
        if (!_waypoints.empty()) {
            yaw = courseToTarget(_local_pos->positionNed(),
                _waypoints[0].x, _waypoints[0].y);
        }
        _trajectory_sp->update(vel, std::nullopt, yaw);

        // Check if we've reached cruise altitude
        if (currentAltAgl() >= _config.cruise_alt_m - kAltitudeTolerance) {
            transitionTo(State::TransitionFw);
        }
    }

    void updateTransitionFw([[maybe_unused]] float dt_s)
    {
        _vtol->toFixedwing();

        const auto vtol_state = _vtol->getCurrentState();

        if (vtol_state == px4_ros2::VTOL::State::FixedWing) {
            // Transition complete
            if (!_waypoints.empty()) {
                transitionTo(State::FwNavigate);
            } else {
                transitionTo(State::FwReturn);
            }
            return;
        }

        // Timeout check
        if (_state_elapsed > _config.fw_transition_timeout) {
            RCLCPP_WARN(node().get_logger(),
                "FW transition timeout (%.1fs), vtol_state=%s, aborting to MC_APPROACH",
                _state_elapsed, vtolStateToString(vtol_state));
            transitionTo(State::McApproach);
            return;
        }

        // During transition: send both trajectory and FW setpoints
        // Trajectory: hold altitude with zero vertical velocity
        const Eigen::Vector3f vel{NAN, NAN, 0.f};
        const Eigen::Vector3f accel = _vtol->computeAccelerationSetpointDuringTransition();
        _trajectory_sp->update(vel, accel);

        // FW: hold height rate 0, course toward first waypoint (or home)
        float course = 0.f;
        if (!_waypoints.empty()) {
            course = courseToTarget(_local_pos->positionNed(),
                _waypoints[0].x, _waypoints[0].y);
        }
        _fw_sp->updateWithHeightRate(0.f, course);
    }

    void updateFwNavigate()
    {
        if (checkQuadchute()) return;

        if (_current_wp_index >= _waypoints.size()) {
            transitionTo(State::FwReturn);
            return;
        }

        const auto& wp = _waypoints[_current_wp_index];
        const auto pos = _local_pos->positionNed();

        if (_config.gps_denied.enabled) {
            // GPS-denied navigation: position-based steering with time-based fallback
            float course;
            bool wp_reached = false;

            auto dr_pos = drPosition();
            if (dr_pos.has_value()) {
                // Position-based steering: course toward WP
                course = std::atan2(wp.y - dr_pos->y(), wp.x - dr_pos->x());

                // Distance-based WP acceptance
                const float dx = wp.x - dr_pos->x();
                const float dy = wp.y - dr_pos->y();
                const float dist = std::sqrt(dx * dx + dy * dy);
                wp_reached = (dist < _config.gps_denied.ekf_wp_accept_radius);
            } else {
                // Fallback: fixed heading (open-loop)
                course = _leg_headings[_current_wp_index];
            }

            const float height_rate = altitudeHoldVz(
                _config.cruise_alt_m, currentAltAgl(),
                _config.gps_denied.altitude_kp, _config.gps_denied.altitude_max_vz);
            _fw_sp->updateWithHeightRate(height_rate, course);

            logFwStatusPeriodic(pos);

            // Time-based fallback always active as safety net
            _wp_leg_elapsed += 1.f / kUpdateRate;
            if (wp_reached || _wp_leg_elapsed >= _config.gps_denied.wp_time_s) {
                RCLCPP_INFO(node().get_logger(),
                    "WP %zu accepted (%s, time=%.1fs), hdg=%.1fdeg alt_agl=%.1fm",
                    _current_wp_index,
                    wp_reached ? "distance" : "time",
                    _wp_leg_elapsed,
                    course * 180.f / static_cast<float>(M_PI), currentAltAgl());
                ++_current_wp_index;
                _wp_leg_elapsed = 0.f;

                if (_current_wp_index >= _waypoints.size()) {
                    RCLCPP_INFO(node().get_logger(), "All %zu waypoints reached",
                        _waypoints.size());
                    transitionTo(State::FwReturn);
                }
            }
        } else {
            // Normal GPS mode: course toward WP + distance-based acceptance
            const float course = courseToTarget(pos, wp.x, wp.y);
            const float alt = _terrain_ctrl->isActive()
                ? _terrain_ctrl->computeTargetAmsl(currentAmsl(), targetAltAmsl())
                : targetAltAmsl();
            _fw_sp->updateWithAltitude(alt, course);

            logFwStatusPeriodic(pos);

            const float dist = horizontalDistTo(pos, wp.x, wp.y);
            const float accept = wpAcceptRadius(wp);

            if (dist < accept) {
                RCLCPP_INFO(node().get_logger(),
                    "WP %zu reached (d=%.1fm < %.1fm), pos=(%.1f, %.1f) alt_agl=%.1fm",
                    _current_wp_index, dist, accept, pos.x(), pos.y(),
                    currentAltAgl());
                ++_current_wp_index;

                if (_current_wp_index >= _waypoints.size()) {
                    RCLCPP_INFO(node().get_logger(), "All %zu waypoints reached",
                        _waypoints.size());
                    transitionTo(State::FwReturn);
                }
            }
        }
    }

    void updateFwReturn()
    {
        if (checkQuadchute()) return;

        const auto pos = _local_pos->positionNed();

        if (_config.gps_denied.enabled) {
            // GPS-denied return: position-based steering with time-based fallback
            float course;
            bool close_enough = false;

            auto dr_pos = drPosition();
            if (dr_pos.has_value()) {
                // Position-based steering: course toward home (0,0)
                course = std::atan2(-dr_pos->y(), -dr_pos->x());

                // Distance-based MC transition
                float dist_home = dr_pos->norm();
                close_enough = (dist_home < _config.gps_denied.ekf_home_accept_radius);
            } else {
                // Fallback: fixed return heading (open-loop)
                course = _return_heading;
            }

            const float height_rate = altitudeHoldVz(
                _config.cruise_alt_m, currentAltAgl(),
                _config.gps_denied.altitude_kp, _config.gps_denied.altitude_max_vz);
            _fw_sp->updateWithHeightRate(height_rate, course);

            logFwStatusPeriodic(pos);

            // Time-based fallback always active as safety net
            if (close_enough || _state_elapsed >= _config.gps_denied.return_time_s) {
                RCLCPP_INFO(node().get_logger(),
                    "FW return complete (%s, elapsed=%.1fs), transitioning to MC",
                    close_enough ? "distance" : "time", _state_elapsed);
                transitionTo(State::TransitionMc);
            }
        } else {
            // Normal GPS mode: course toward home + distance-based transition
            const float dist_home = horizontalDistTo(pos, 0.f, 0.f);
            const float course = courseToTarget(pos, 0.f, 0.f);
            const float alt = _terrain_ctrl->isActive()
                ? _terrain_ctrl->computeTargetAmsl(currentAmsl(), targetAltAmsl())
                : targetAltAmsl();
            _fw_sp->updateWithAltitude(alt, course);

            logFwStatusPeriodic(pos);

            if (dist_home < _config.mc_transition_dist) {
                RCLCPP_INFO(node().get_logger(),
                    "Within MC transition distance (%.0fm < %.0fm), transitioning to MC",
                    dist_home, _config.mc_transition_dist);
                transitionTo(State::TransitionMc);
            }
        }
    }

    void updateTransitionMc([[maybe_unused]] float dt_s)
    {
        _vtol->toMulticopter();

        const auto vtol_state = _vtol->getCurrentState();

        if (vtol_state == px4_ros2::VTOL::State::Multicopter) {
            transitionTo(State::McApproach);
            return;
        }

        // Timeout: PX4 may have transitioned internally
        if (_state_elapsed > _config.mc_transition_timeout) {
            RCLCPP_WARN(node().get_logger(),
                "MC transition timeout (%.1fs), forcing MC_APPROACH",
                _state_elapsed);
            transitionTo(State::McApproach);
            return;
        }

        // During back-transition: send both setpoint types with deceleration
        const Eigen::Vector3f vel{NAN, NAN, 0.f};
        const Eigen::Vector3f accel = _vtol->computeAccelerationSetpointDuringTransition();
        _trajectory_sp->update(vel, accel);

        // FW: maintain altitude, course toward home
        const float course = courseToTarget(_local_pos->positionNed(), 0.f, 0.f);
        _fw_sp->updateWithHeightRate(0.f, course);
    }

    void updateMcApproach()
    {
        if (_config.gps_denied.enabled) {
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
                    float vn = _config.mc_approach_speed * std::cos(course);
                    float ve = _config.mc_approach_speed * std::sin(course);
                    float vd = -altitudeHoldVz(_config.cruise_alt_m, currentAltAgl(),
                        _config.gps_denied.altitude_kp, _config.gps_denied.altitude_max_vz);
                    _trajectory_sp->update(
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
            const Eigen::Vector3f home_pos{0.f, 0.f, -_config.cruise_alt_m};
            const float heading = courseToTarget(_local_pos->positionNed(), 0.f, 0.f);

            _goto_sp->update(home_pos, heading, _config.mc_approach_speed);

            const auto pos = _local_pos->positionNed();
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
        _leg_headings = computeLegHeadingsFromWaypoints(_waypoints);
        _return_heading = computeReturnHeading(
            _waypoints, _config.gps_denied.return_heading);
    }

    // Config
    VtolNavConfig _config;

    // Setpoint types
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_sp;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_sp;
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_sp;

    // VTOL helper
    std::shared_ptr<px4_ros2::VTOL> _vtol;

    // Terrain-following controller
    std::unique_ptr<TerrainAltitudeController> _terrain_ctrl;

    // Position feedback
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_pos;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _global_pos_sub;
    px4_msgs::msg::VehicleGlobalPosition _last_global_pos;
    bool _global_pos_valid{false};

    // State machine
    State _state{State::McClimb};
    float _state_elapsed{0.f};
    float _last_fw_log_time{0.f};
    float _alt_amsl_ref{0.f};

    // Waypoints
    std::vector<VtolWaypoint> _waypoints;
    std::size_t _current_wp_index{0};

    // GPS-denied navigation
    std::vector<float> _leg_headings;
    float _return_heading{0.f};
    float _wp_leg_elapsed{0.f};

    // Position EKF (GPS-denied closed-loop navigation)
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _ekf_pos_sub;
    float _ekf_pos_x{0.f};
    float _ekf_pos_y{0.f};
    float _ekf_pos_sigma_x{0.f};
    float _ekf_pos_sigma_y{0.f};
    bool _ekf_pos_valid{false};

    // Cable tension monitor
    rclcpp::Subscription<fiber_nav_sensors::msg::CableStatus>::SharedPtr _cable_sub;
    fiber_nav_sensors::msg::CableStatus _last_cable_status;
    bool _cable_status_valid{false};
    float _cable_warn_elapsed{0.f};
    bool _cable_abort_triggered{false};
    float _spool_warn_elapsed{0.f};
    bool _spool_abort_triggered{false};
};

}  // namespace fiber_nav_mode
