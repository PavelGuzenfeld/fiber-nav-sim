#include <fiber_nav_mode/hold_mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<px4_ros2::NodeWithMode<fiber_nav_mode::HoldMode>>(
            "hold_mode_node"));
    rclcpp::shutdown();
    return 0;
}
