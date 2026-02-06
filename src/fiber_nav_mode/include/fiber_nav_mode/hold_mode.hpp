#pragma once

#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fiber_nav_mode {

class HoldMode : public px4_ros2::ModeBase
{
public:
    static constexpr auto kModeName = "FiberNav Hold";
    static constexpr float kUpdateRate = 50.f;

    explicit HoldMode(rclcpp::Node& node)
        : ModeBase(node, Settings{kModeName})
    {
        setSetpointUpdateRate(kUpdateRate);
        _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

    void onActivate() override
    {
        _hold_position = _vehicle_local_position->positionNed();
        _hold_heading = _vehicle_local_position->heading();
        RCLCPP_INFO(node().get_logger(),
            "Hold activated at NED [%.1f, %.1f, %.1f] hdg=%.1f°",
            _hold_position.x(), _hold_position.y(), _hold_position.z(),
            _hold_heading * 180.f / M_PI);
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(), "Hold deactivated");
    }

    void updateSetpoint(float /*dt_s*/) override
    {
        _goto_setpoint->update(_hold_position, _hold_heading);
    }

private:
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    Eigen::Vector3f _hold_position{0.f, 0.f, 0.f};
    float _hold_heading{0.f};
};

}  // namespace fiber_nav_mode
