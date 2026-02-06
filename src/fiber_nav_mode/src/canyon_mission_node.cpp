#include <fiber_nav_mode/canyon_mission.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("canyon_mission_node");

    // Declare waypoint parameters
    node->declare_parameter<double>("altitude", -50.0);
    node->declare_parameter<double>("accept_radius", 3.0);
    node->declare_parameter<std::vector<double>>("waypoints_x", {0., 50., 100., 150.});
    node->declare_parameter<std::vector<double>>("waypoints_y", {0., 0., 10., 0.});
    node->declare_parameter<std::vector<double>>("waypoints_heading", {0., 0., 0.1, 0.});

    const auto altitude = static_cast<float>(node->get_parameter("altitude").as_double());
    const auto accept = static_cast<float>(node->get_parameter("accept_radius").as_double());
    const auto wx = node->get_parameter("waypoints_x").as_double_array();
    const auto wy = node->get_parameter("waypoints_y").as_double_array();
    const auto wh = node->get_parameter("waypoints_heading").as_double_array();

    // Build waypoints from parameters (or use defaults if sizes mismatch)
    std::vector<fiber_nav_mode::Waypoint> waypoints;
    if (wx.size() == wy.size() && wx.size() == wh.size() && !wx.empty()) {
        for (std::size_t i = 0; i < wx.size(); ++i) {
            waypoints.push_back({
                {static_cast<float>(wx[i]), static_cast<float>(wy[i]), altitude},
                static_cast<float>(wh[i]),
                accept
            });
        }
    } else {
        RCLCPP_WARN(node->get_logger(),
            "Waypoint parameter sizes mismatch, using defaults");
        waypoints = fiber_nav_mode::defaultCanyonWaypoints();
    }

    RCLCPP_INFO(node->get_logger(),
        "Canyon mission: %zu waypoints, alt=%.0fm, accept=%.1fm",
        waypoints.size(), altitude, accept);

    auto waypoint_mode = std::make_unique<fiber_nav_mode::CanyonWaypointMode>(*node);
    waypoint_mode->setWaypoints(std::move(waypoints));

    auto executor = std::make_unique<fiber_nav_mode::CanyonMissionExecutor>(
        *waypoint_mode);

    if (!executor->doRegister()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to register canyon mission executor");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
