#pragma once

#include <fiber_nav_mode/vtol_navigation_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fiber_nav_mode {

/// Mission executor that sequences: arm → takeoff → VTOL navigation → land → disarm.
///
/// The VtolNavigationMode handles all waypoint navigation and MC/FW transitions.
/// This executor just manages the arm/takeoff/land lifecycle around it.
class VtolMissionExecutor : public px4_ros2::ModeExecutorBase
{
public:
    static constexpr auto kModeName = "FiberNav VTOL Mission";
    static constexpr int kFailsafeTimeoutS = 600;  // 10 minutes
    static constexpr int kMaxArmRetries = 15;
    static constexpr int kArmRetryIntervalMs = 2000;

    enum class State
    {
        Arming,
        TakingOff,
        Navigating,
        Landing,
        WaitDisarmed,
    };

    explicit VtolMissionExecutor(VtolNavigationMode& nav_mode)
        : ModeExecutorBase(
              Settings{}.activate(Settings::Activation::ActivateImmediately),
              nav_mode)
        , _nav_mode(nav_mode)
    {
        setSkipMessageCompatibilityCheck();
        // ActivateImmediately: PX4 activates this executor right after
        // registration, without requiring manual mode selection. The executor
        // then arms and starts the mission autonomously.
    }

    void onActivate() override
    {
        // Prevent auto-restart after mid-flight deactivation.
        // ActivateImmediately causes PX4 to re-activate us after deactivation,
        // but restarting a mission mid-air is unsafe.
        if (_mission_aborted) {
            RCLCPP_WARN(node().get_logger(),
                "VTOL mission was previously aborted, not restarting. "
                "Restart the node to retry.");
            return;
        }

        RCLCPP_INFO(node().get_logger(), "VTOL mission started");
        // Note: deferFailsafesSync() cannot be used with ActivateImmediately
        // because it does a synchronous spin that conflicts with the
        // registration wait set. Failsafe timeout is handled by PX4 internally.
        _arm_retries = 0;
        runState(State::Arming, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override
    {
        RCLCPP_INFO(node().get_logger(), "VTOL mission deactivated (reason=%d)",
            static_cast<int>(reason));
    }

private:
    void runState(State state, px4_ros2::Result previous_result)
    {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(node().get_logger(),
                "State %s failed (result=%s), aborting mission",
                stateToString(state),
                px4_ros2::resultToString(previous_result));
            _mission_aborted = true;
            return;
        }

        switch (state) {
        case State::Arming:
            RCLCPP_INFO(node().get_logger(), "Arming...");
            arm([this](px4_ros2::Result result) {
                if (result == px4_ros2::Result::Rejected &&
                    _arm_retries < kMaxArmRetries)
                {
                    ++_arm_retries;
                    RCLCPP_WARN(node().get_logger(),
                        "Arm rejected, retry %d/%d in %dms...",
                        _arm_retries, kMaxArmRetries, kArmRetryIntervalMs);
                    _arm_timer = node().create_wall_timer(
                        std::chrono::milliseconds(kArmRetryIntervalMs),
                        [this]() {
                            _arm_timer->cancel();
                            runState(State::Arming, px4_ros2::Result::Success);
                        });
                } else {
                    runState(State::TakingOff, result);
                }
            });
            break;

        case State::TakingOff:
            RCLCPP_INFO(node().get_logger(), "Taking off...");
            takeoff([this](px4_ros2::Result result) {
                runState(State::Navigating, result);
            });
            break;

        case State::Navigating:
            RCLCPP_INFO(node().get_logger(), "Starting VTOL navigation...");
            scheduleMode(
                _nav_mode.id(),
                [this](px4_ros2::Result result) {
                    runState(State::Landing, result);
                });
            break;

        case State::Landing:
            RCLCPP_INFO(node().get_logger(), "Landing...");
            land([this](px4_ros2::Result result) {
                runState(State::WaitDisarmed, result);
            });
            break;

        case State::WaitDisarmed:
            RCLCPP_INFO(node().get_logger(), "Waiting for disarm...");
            waitUntilDisarmed([this](px4_ros2::Result result) {
                if (result == px4_ros2::Result::Success) {
                    RCLCPP_INFO(node().get_logger(), "VTOL mission complete");
                } else {
                    RCLCPP_WARN(node().get_logger(),
                        "VTOL mission ended with error: %s",
                        px4_ros2::resultToString(result));
                }
            });
            break;
        }
    }

    static const char* stateToString(State s)
    {
        switch (s) {
        case State::Arming:       return "ARMING";
        case State::TakingOff:    return "TAKING_OFF";
        case State::Navigating:   return "NAVIGATING";
        case State::Landing:      return "LANDING";
        case State::WaitDisarmed: return "WAIT_DISARMED";
        }
        return "UNKNOWN";
    }

    VtolNavigationMode& _nav_mode;
    int _arm_retries{0};
    bool _mission_aborted{false};
    rclcpp::TimerBase::SharedPtr _arm_timer;
};

}  // namespace fiber_nav_mode
