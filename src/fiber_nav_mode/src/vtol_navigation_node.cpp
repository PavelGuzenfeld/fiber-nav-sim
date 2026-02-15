#include <limits>

#include <fiber_nav_mode/vtol_mission_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vtol_navigation_node");

    // Declare navigation config parameters
    node->declare_parameter<double>("cruise_altitude", 50.0);
    node->declare_parameter<double>("climb_rate", 2.0);
    node->declare_parameter<double>("fw_accept_radius", 50.0);
    node->declare_parameter<double>("mc_transition_dist", 200.0);
    node->declare_parameter<double>("mc_approach_speed", 5.0);
    node->declare_parameter<double>("fw_transition_timeout", 30.0);
    node->declare_parameter<double>("mc_transition_timeout", 60.0);

    // Declare terrain-following parameters
    node->declare_parameter<bool>("terrain_follow.enabled", false);
    node->declare_parameter<double>("terrain_follow.target_agl", 30.0);
    node->declare_parameter<double>("terrain_follow.kp", 0.5);
    node->declare_parameter<double>("terrain_follow.max_rate", 3.0);
    node->declare_parameter<double>("terrain_follow.rate_slew", 1.0);
    node->declare_parameter<double>("terrain_follow.filter_tau", 0.5);
    node->declare_parameter<double>("terrain_follow.fallback_timeout", 5.0);
    node->declare_parameter<double>("terrain_follow.min_agl", 10.0);
    node->declare_parameter<double>("terrain_follow.max_agl", 200.0);
    node->declare_parameter<double>("terrain_follow.lookahead_time", 3.0);
    node->declare_parameter<double>("terrain_follow.lookahead_max", 100.0);
    node->declare_parameter<double>("terrain_follow.feedforward_gain", 0.8);
    node->declare_parameter<double>("terrain_follow.max_slope", 0.5);
    node->declare_parameter<double>("terrain_follow.slope_tau", 1.0);

    // Declare cable monitor parameters
    node->declare_parameter<bool>("cable_monitor.enabled", false);
    node->declare_parameter<double>("cable_monitor.tension_warn_percent", 70.0);
    node->declare_parameter<double>("cable_monitor.tension_abort_percent", 85.0);
    node->declare_parameter<double>("cable_monitor.breaking_strength", 50.0);
    node->declare_parameter<double>("cable_monitor.spool_capacity", -1.0);
    node->declare_parameter<double>("cable_monitor.spool_warn_percent", 80.0);
    node->declare_parameter<double>("cable_monitor.spool_abort_percent", 95.0);

    // Declare gimbal accommodation parameters
    node->declare_parameter<bool>("gimbal.enabled", false);
    node->declare_parameter<double>("gimbal.saturation_threshold", 0.7);
    node->declare_parameter<double>("gimbal.base_turn_rate", 5.0);
    node->declare_parameter<double>("gimbal.min_turn_rate", 1.0);
    node->declare_parameter<double>("gimbal.min_alt_rate_scale", 0.3);

    // Declare GPS-denied navigation parameters
    node->declare_parameter<bool>("gps_denied.enabled", false);
    node->declare_parameter<double>("gps_denied.wp_time_s", 30.0);
    node->declare_parameter<double>("gps_denied.return_time_s", 120.0);
    node->declare_parameter<double>("gps_denied.descent_time_s", 60.0);
    node->declare_parameter<double>("gps_denied.altitude_kp", 0.5);
    node->declare_parameter<double>("gps_denied.altitude_max_vz", 3.0);
    node->declare_parameter<double>("gps_denied.fw_speed", 18.0);
    node->declare_parameter<double>("gps_denied.return_heading",
        std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>("gps_denied.turn_heading_tolerance_deg", 10.0);
    node->declare_parameter<bool>("gps_denied.use_position_ekf", false);
    node->declare_parameter<double>("gps_denied.ekf_wp_accept_radius", 80.0);
    node->declare_parameter<double>("gps_denied.ekf_home_accept_radius", 100.0);
    node->declare_parameter<double>("gps_denied.ekf_max_uncertainty", 200.0);

    // Declare waypoint parameters (parallel arrays)
    node->declare_parameter<std::vector<double>>("waypoints.x", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("waypoints.y", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("waypoints.heading", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("waypoints.accept_radius", std::vector<double>{});

    // Build config from parameters
    fiber_nav_mode::VtolNavConfig config;
    config.cruise_alt_m =
        static_cast<float>(node->get_parameter("cruise_altitude").as_double());
    config.climb_rate =
        static_cast<float>(node->get_parameter("climb_rate").as_double());
    config.fw_accept_radius =
        static_cast<float>(node->get_parameter("fw_accept_radius").as_double());
    config.mc_transition_dist =
        static_cast<float>(node->get_parameter("mc_transition_dist").as_double());
    config.mc_approach_speed =
        static_cast<float>(node->get_parameter("mc_approach_speed").as_double());
    config.fw_transition_timeout =
        static_cast<float>(node->get_parameter("fw_transition_timeout").as_double());
    config.mc_transition_timeout =
        static_cast<float>(node->get_parameter("mc_transition_timeout").as_double());

    // Load terrain-following config
    config.terrain_follow.enabled =
        node->get_parameter("terrain_follow.enabled").as_bool();
    config.terrain_follow.target_agl =
        static_cast<float>(node->get_parameter("terrain_follow.target_agl").as_double());
    config.terrain_follow.kp =
        static_cast<float>(node->get_parameter("terrain_follow.kp").as_double());
    config.terrain_follow.max_rate =
        static_cast<float>(node->get_parameter("terrain_follow.max_rate").as_double());
    config.terrain_follow.rate_slew =
        static_cast<float>(node->get_parameter("terrain_follow.rate_slew").as_double());
    config.terrain_follow.filter_tau =
        static_cast<float>(node->get_parameter("terrain_follow.filter_tau").as_double());
    config.terrain_follow.fallback_timeout =
        static_cast<float>(node->get_parameter("terrain_follow.fallback_timeout").as_double());
    config.terrain_follow.min_agl =
        static_cast<float>(node->get_parameter("terrain_follow.min_agl").as_double());
    config.terrain_follow.max_agl =
        static_cast<float>(node->get_parameter("terrain_follow.max_agl").as_double());
    config.terrain_follow.lookahead_time =
        static_cast<float>(node->get_parameter("terrain_follow.lookahead_time").as_double());
    config.terrain_follow.lookahead_max =
        static_cast<float>(node->get_parameter("terrain_follow.lookahead_max").as_double());
    config.terrain_follow.feedforward_gain =
        static_cast<float>(node->get_parameter("terrain_follow.feedforward_gain").as_double());
    config.terrain_follow.max_slope =
        static_cast<float>(node->get_parameter("terrain_follow.max_slope").as_double());
    config.terrain_follow.slope_tau =
        static_cast<float>(node->get_parameter("terrain_follow.slope_tau").as_double());

    // Load cable monitor config
    config.cable_monitor.enabled =
        node->get_parameter("cable_monitor.enabled").as_bool();
    config.cable_monitor.tension_warn_percent =
        static_cast<float>(node->get_parameter("cable_monitor.tension_warn_percent").as_double());
    config.cable_monitor.tension_abort_percent =
        static_cast<float>(node->get_parameter("cable_monitor.tension_abort_percent").as_double());
    config.cable_monitor.breaking_strength =
        static_cast<float>(node->get_parameter("cable_monitor.breaking_strength").as_double());
    config.cable_monitor.spool_capacity =
        static_cast<float>(node->get_parameter("cable_monitor.spool_capacity").as_double());
    config.cable_monitor.spool_warn_percent =
        static_cast<float>(node->get_parameter("cable_monitor.spool_warn_percent").as_double());
    config.cable_monitor.spool_abort_percent =
        static_cast<float>(node->get_parameter("cable_monitor.spool_abort_percent").as_double());

    // Load gimbal accommodation config
    config.gimbal.enabled =
        node->get_parameter("gimbal.enabled").as_bool();
    config.gimbal.saturation_threshold =
        static_cast<float>(node->get_parameter("gimbal.saturation_threshold").as_double());
    config.gimbal.base_turn_rate =
        static_cast<float>(node->get_parameter("gimbal.base_turn_rate").as_double());
    config.gimbal.min_turn_rate =
        static_cast<float>(node->get_parameter("gimbal.min_turn_rate").as_double());
    config.gimbal.min_alt_rate_scale =
        static_cast<float>(node->get_parameter("gimbal.min_alt_rate_scale").as_double());

    // Load GPS-denied config
    config.gps_denied.enabled = node->get_parameter("gps_denied.enabled").as_bool();
    config.gps_denied.wp_time_s =
        static_cast<float>(node->get_parameter("gps_denied.wp_time_s").as_double());
    config.gps_denied.return_time_s =
        static_cast<float>(node->get_parameter("gps_denied.return_time_s").as_double());
    config.gps_denied.descent_time_s =
        static_cast<float>(node->get_parameter("gps_denied.descent_time_s").as_double());
    config.gps_denied.altitude_kp =
        static_cast<float>(node->get_parameter("gps_denied.altitude_kp").as_double());
    config.gps_denied.altitude_max_vz =
        static_cast<float>(node->get_parameter("gps_denied.altitude_max_vz").as_double());
    config.gps_denied.fw_speed =
        static_cast<float>(node->get_parameter("gps_denied.fw_speed").as_double());
    config.gps_denied.return_heading =
        static_cast<float>(node->get_parameter("gps_denied.return_heading").as_double());
    config.gps_denied.turn_heading_tolerance_deg =
        static_cast<float>(node->get_parameter("gps_denied.turn_heading_tolerance_deg").as_double());
    config.gps_denied.use_position_ekf =
        node->get_parameter("gps_denied.use_position_ekf").as_bool();
    config.gps_denied.ekf_wp_accept_radius =
        static_cast<float>(node->get_parameter("gps_denied.ekf_wp_accept_radius").as_double());
    config.gps_denied.ekf_home_accept_radius =
        static_cast<float>(node->get_parameter("gps_denied.ekf_home_accept_radius").as_double());
    config.gps_denied.ekf_max_uncertainty =
        static_cast<float>(node->get_parameter("gps_denied.ekf_max_uncertainty").as_double());

    // Build waypoints from parameters
    const auto wx = node->get_parameter("waypoints.x").as_double_array();
    const auto wy = node->get_parameter("waypoints.y").as_double_array();
    const auto wh = node->get_parameter("waypoints.heading").as_double_array();
    const auto wr = node->get_parameter("waypoints.accept_radius").as_double_array();

    std::vector<fiber_nav_mode::VtolWaypoint> waypoints;

    if (wx.size() != wy.size() || wx.empty())
    {
        RCLCPP_WARN(node->get_logger(),
                    "Waypoint x/y sizes mismatch or empty (x=%zu, y=%zu), "
                    "will navigate without waypoints (direct FW_RETURN)",
                    wx.size(), wy.size());
    }
    else
    {
        waypoints.reserve(wx.size());
        for (std::size_t i = 0; i < wx.size(); ++i)
        {
            fiber_nav_mode::VtolWaypoint wp;
            wp.x = static_cast<float>(wx[i]);
            wp.y = static_cast<float>(wy[i]);
            wp.heading = (i < wh.size()) ? static_cast<float>(wh[i]) : NAN;
            wp.acceptance_radius = (i < wr.size()) ? static_cast<float>(wr[i]) : 0.f;
            waypoints.push_back(wp);
        }
    }

    RCLCPP_INFO(node->get_logger(),
                "VTOL navigation: %zu waypoints, cruise_alt=%.0fm, "
                "fw_accept=%.0fm, mc_transition_dist=%.0fm, "
                "cable_monitor=%s, gps_denied=%s, gimbal_accom=%s",
                waypoints.size(), config.cruise_alt_m,
                config.fw_accept_radius, config.mc_transition_dist,
                config.cable_monitor.enabled ? "ON" : "OFF",
                config.gps_denied.enabled ? "ON" : "OFF",
                config.gimbal.enabled ? "ON" : "OFF");
    if (config.gps_denied.enabled) {
        RCLCPP_INFO(node->get_logger(),
                    "GPS-denied: wp_time=%.0fs, return_time=%.0fs, "
                    "descent_time=%.0fs, fw_speed=%.0fm/s, alt_kp=%.2f",
                    config.gps_denied.wp_time_s,
                    config.gps_denied.return_time_s,
                    config.gps_denied.descent_time_s,
                    config.gps_denied.fw_speed,
                    config.gps_denied.altitude_kp);
    }
    if (config.cable_monitor.enabled) {
        RCLCPP_INFO(node->get_logger(),
                    "Cable monitor: warn=%.0f%% abort=%.0f%% breaking=%.0fN "
                    "spool=%s warn=%.0f%% abort=%.0f%%",
                    config.cable_monitor.tension_warn_percent,
                    config.cable_monitor.tension_abort_percent,
                    config.cable_monitor.breaking_strength,
                    config.cable_monitor.spool_capacity > 0.f
                        ? (std::to_string(static_cast<int>(config.cable_monitor.spool_capacity)) + "m").c_str()
                        : "unlimited",
                    config.cable_monitor.spool_warn_percent,
                    config.cable_monitor.spool_abort_percent);
    }
    if (config.gimbal.enabled) {
        RCLCPP_INFO(node->get_logger(),
                    "Gimbal accommodation: threshold=%.0f%%, yaw_rate=%.1f-%.1f°/s, "
                    "pitch_alt_scale=%.1f",
                    config.gimbal.saturation_threshold * 100.f,
                    config.gimbal.min_turn_rate,
                    config.gimbal.base_turn_rate,
                    config.gimbal.min_alt_rate_scale);
    }

    auto nav_mode = std::make_unique<fiber_nav_mode::VtolNavigationMode>(
        *node, config);
    nav_mode->setWaypoints(std::move(waypoints));

    auto executor = std::make_unique<fiber_nav_mode::VtolMissionExecutor>(
        *nav_mode);

    if (!executor->doRegister())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to register VTOL mission executor");
        return 1;
    }

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
    } catch (...) {
        RCLCPP_FATAL(node->get_logger(), "Unhandled unknown exception");
    }
    rclcpp::shutdown();
    return 0;
}
