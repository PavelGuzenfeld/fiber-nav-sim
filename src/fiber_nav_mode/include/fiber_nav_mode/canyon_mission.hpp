#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fiber_nav_mode {

struct Waypoint
{
    Eigen::Vector3f position;  // NED [m]
    float heading;             // [rad]
    float acceptance_radius;   // [m]
};

/// Mode that navigates through a sequence of waypoints using GotoSetpoint.
/// Calls completed(Success) when all waypoints are reached.
class CanyonWaypointMode : public px4_ros2::ModeBase
{
public:
    static constexpr auto kModeName = "FiberNav Waypoint";
    static constexpr float kUpdateRate = 50.f;
    static constexpr float kDefaultAcceptRadius = 2.f;
    static constexpr float kDefaultMaxSpeed = 5.f;
    static constexpr float kVelocityThreshold = 0.5f;

    explicit CanyonWaypointMode(rclcpp::Node& node,
                                const std::string& topic_namespace_prefix = "")
        : ModeBase(node, Settings{kModeName}, topic_namespace_prefix)
    {
        setSetpointUpdateRate(kUpdateRate);
        _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

    void setWaypoints(std::vector<Waypoint> waypoints)
    {
        _waypoints = std::move(waypoints);
    }

    void onActivate() override
    {
        _current_wp_index = 0;
        if (_waypoints.empty()) {
            RCLCPP_WARN(node().get_logger(), "No waypoints set, completing immediately");
            completed(px4_ros2::Result::Success);
            return;
        }
        RCLCPP_INFO(node().get_logger(),
            "Canyon waypoint mode activated with %zu waypoints", _waypoints.size());
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(),
            "Canyon waypoint mode deactivated at wp %zu/%zu",
            _current_wp_index, _waypoints.size());
    }

    void updateSetpoint(float /*dt_s*/) override
    {
        if (_current_wp_index >= _waypoints.size()) {
            completed(px4_ros2::Result::Success);
            return;
        }

        const auto& wp = _waypoints[_current_wp_index];
        _goto_setpoint->update(wp.position, wp.heading, kDefaultMaxSpeed);

        // Check if waypoint reached
        const Eigen::Vector3f pos = _vehicle_local_position->positionNed();
        const float dist = (wp.position - pos).norm();
        const float vel = _vehicle_local_position->velocityNed().norm();

        if (dist < wp.acceptance_radius && vel < kVelocityThreshold) {
            RCLCPP_INFO(node().get_logger(),
                "Waypoint %zu reached (d=%.1fm, v=%.1fm/s)",
                _current_wp_index, dist, vel);
            ++_current_wp_index;
            if (_current_wp_index >= _waypoints.size()) {
                RCLCPP_INFO(node().get_logger(), "All waypoints reached");
                completed(px4_ros2::Result::Success);
            }
        }
    }

    [[nodiscard]] std::size_t currentWaypointIndex() const { return _current_wp_index; }
    [[nodiscard]] std::size_t waypointCount() const { return _waypoints.size(); }

private:
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::vector<Waypoint> _waypoints;
    std::size_t _current_wp_index{0};
};

/// Mission executor that sequences: arm -> takeoff -> waypoints -> RTL -> land -> disarm.
class CanyonMissionExecutor : public px4_ros2::ModeExecutorBase
{
public:
    static constexpr auto kModeName = "FiberNav Canyon Mission";

    explicit CanyonMissionExecutor(CanyonWaypointMode& owned_mode)
        : ModeExecutorBase(
              Settings{}.activate(Settings::Activation::ActivateAlways),
              owned_mode)
        , _waypoint_mode(owned_mode)
    {
    }

    enum class State
    {
        Arming,
        TakingOff,
        Navigating,
        RTL,
        WaitDisarmed,
    };

    void onActivate() override
    {
        RCLCPP_INFO(node().get_logger(), "Canyon mission started");
        runState(State::Arming, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override
    {
        RCLCPP_INFO(node().get_logger(), "Canyon mission deactivated (reason=%d)",
            static_cast<int>(reason));
    }

private:
    void runState(State state, px4_ros2::Result previous_result)
    {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(node().get_logger(),
                "State transition failed, aborting mission");
            return;
        }

        switch (state) {
        case State::Arming:
            RCLCPP_INFO(node().get_logger(), "Arming...");
            arm([this](px4_ros2::Result result) {
                runState(State::TakingOff, result);
            });
            break;

        case State::TakingOff:
            RCLCPP_INFO(node().get_logger(), "Taking off...");
            takeoff([this](px4_ros2::Result result) {
                runState(State::Navigating, result);
            });
            break;

        case State::Navigating:
            RCLCPP_INFO(node().get_logger(), "Navigating canyon waypoints...");
            scheduleMode(
                _waypoint_mode.id(),
                [this](px4_ros2::Result result) {
                    runState(State::RTL, result);
                });
            break;

        case State::RTL:
            RCLCPP_INFO(node().get_logger(), "Returning to launch...");
            rtl([this](px4_ros2::Result result) {
                runState(State::WaitDisarmed, result);
            });
            break;

        case State::WaitDisarmed:
            RCLCPP_INFO(node().get_logger(), "Waiting for disarm...");
            waitUntilDisarmed([this](px4_ros2::Result result) {
                if (result == px4_ros2::Result::Success) {
                    RCLCPP_INFO(node().get_logger(), "Canyon mission complete");
                } else {
                    RCLCPP_WARN(node().get_logger(), "Canyon mission ended with error");
                }
            });
            break;
        }
    }

    CanyonWaypointMode& _waypoint_mode;
};

/// Build default canyon waypoints in NED frame.
/// The canyon runs roughly along the +X axis, starting at spawn position.
inline std::vector<Waypoint> defaultCanyonWaypoints()
{
    constexpr float altitude = -50.f;  // 50m above ground (NED: negative = up)
    constexpr float accept = 3.f;

    return {
        // Fly north along canyon
        {{0.f, 0.f, altitude}, 0.f, accept},
        {{50.f, 0.f, altitude}, 0.f, accept},
        {{100.f, 10.f, altitude}, 0.1f, accept},
        {{150.f, 0.f, altitude}, 0.f, accept},
        // Turn around
        {{150.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
        // Return along canyon
        {{100.f, -10.f, altitude}, static_cast<float>(M_PI), accept},
        {{50.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
        {{0.f, 0.f, altitude}, static_cast<float>(M_PI), accept},
    };
}

}  // namespace fiber_nav_mode
