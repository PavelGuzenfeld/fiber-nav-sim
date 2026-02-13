// Copyright 2026 Pavel Guzenfeld — All rights reserved.
// PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
// Version: 0.0.1

#include <fiber_nav_sensors/cable_dynamics.hpp>
#include <fiber_nav_sensors/wrench_output.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fiber_nav_sensors/msg/spool_status.hpp>
#include <fiber_nav_sensors/msg/cable_status.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <regex>
#include <string>
#include <vector>

#ifdef USE_GAZEBO_WRENCH
#include <gz/transport/Node.hh>
#include <gz/msgs/marker.pb.h>
#endif

namespace fiber_nav_sensors {

class CableDynamicsNode : public rclcpp::Node {
public:
    CableDynamicsNode() : Node("cable_dynamics_node") {
        // Cable properties
        declare_parameter("world_name", "terrain_world");
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("enabled", true);
        declare_parameter("mass_per_meter", 0.003);
        declare_parameter("diameter", 0.0009);
        declare_parameter("breaking_strength", 50.0);
        declare_parameter("drag_coefficient", 1.1);
        declare_parameter("drag_shape_factor", 0.4);
        declare_parameter("spool_friction_static", 0.5);
        declare_parameter("spool_friction_kinetic", 0.02);
        declare_parameter("air_density", 1.225);
        declare_parameter("spool_capacity", 7500.0);  // meters (total fiber on spool)
        declare_parameter("update_rate", 50.0);

        enabled_ = get_parameter("enabled").as_bool();
        world_name_ = get_parameter("world_name").as_string();
        model_name_ = get_parameter("model_name").as_string();

        props_.mass_per_meter = get_parameter("mass_per_meter").as_double();
        props_.diameter = get_parameter("diameter").as_double();
        props_.breaking_strength = get_parameter("breaking_strength").as_double();
        props_.drag_coefficient = get_parameter("drag_coefficient").as_double();
        props_.drag_shape_factor = get_parameter("drag_shape_factor").as_double();
        props_.spool_friction_static = get_parameter("spool_friction_static").as_double();
        props_.spool_friction_kinetic = get_parameter("spool_friction_kinetic").as_double();
        props_.air_density = get_parameter("air_density").as_double();
        spool_capacity_ = get_parameter("spool_capacity").as_double();

        double rate = get_parameter("update_rate").as_double();

        // Wrench output (Gazebo transport)
        wrench_output_ = create_wrench_output();
        wrench_output_->initialize(world_name_, model_name_);

        // Subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/" + model_name_ + "/odometry", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_ = *msg;
                have_odom_ = true;
            });

        spool_sub_ = create_subscription<fiber_nav_sensors::msg::SpoolStatus>(
            "/sensors/fiber_spool/status", 10,
            [this](fiber_nav_sensors::msg::SpoolStatus::ConstSharedPtr msg) {
                spool_ = *msg;
                have_spool_ = true;
            });

        // Publishers
        status_pub_ = create_publisher<fiber_nav_sensors::msg::CableStatus>(
            "/cable/status", 10);
        tension_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/cable/tension", 10);
        remaining_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/cable/remaining", 10);

        // Update timer
        auto period_ms = static_cast<int>(1000.0 / rate);
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&CableDynamicsNode::update, this));

        // Discover entity ID
        discover_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CableDynamicsNode::discover_entity_id, this));

        // Visual cable rendering (Gazebo markers)
#ifdef USE_GAZEBO_WRENCH
        declare_parameter("visual_enabled", true);
        declare_parameter("visual_segments", 30);
        declare_parameter("visual_color_r", 0.1);
        declare_parameter("visual_color_g", 1.0);
        declare_parameter("visual_color_b", 0.1);
        declare_parameter("visual_color_a", 1.0);
        declare_parameter("visual_width", 0.15);
        declare_parameter("visual_rate", 10.0);

        visual_enabled_ = get_parameter("visual_enabled").as_bool();
        visual_segments_ = get_parameter("visual_segments").as_int();
        visual_color_r_ = static_cast<float>(get_parameter("visual_color_r").as_double());
        visual_color_g_ = static_cast<float>(get_parameter("visual_color_g").as_double());
        visual_color_b_ = static_cast<float>(get_parameter("visual_color_b").as_double());
        visual_color_a_ = static_cast<float>(get_parameter("visual_color_a").as_double());
        visual_width_ = get_parameter("visual_width").as_double();
        double visual_rate = get_parameter("visual_rate").as_double();

        if (visual_enabled_) {
            auto visual_period_ms = static_cast<int>(1000.0 / visual_rate);
            visual_timer_ = create_wall_timer(
                std::chrono::milliseconds(visual_period_ms),
                std::bind(&CableDynamicsNode::update_visual, this));
        }
#endif

        RCLCPP_INFO(get_logger(), "Cable dynamics node initialized");
        RCLCPP_INFO(get_logger(),
            "  mass=%.1f g/m, d=%.1f mm, break=%.0f N, Cd=%.1f, shape=%.1f, spool=%s",
            props_.mass_per_meter * 1000.0,
            props_.diameter * 1000.0,
            props_.breaking_strength,
            props_.drag_coefficient,
            props_.drag_shape_factor,
            spool_capacity_ > 0.0
                ? (std::to_string(static_cast<int>(spool_capacity_)) + "m").c_str()
                : "unlimited");

        if (!enabled_) {
            RCLCPP_WARN(get_logger(), "Cable dynamics DISABLED (enabled=false)");
        }
#ifdef USE_GAZEBO_WRENCH
        if (visual_enabled_) {
            RCLCPP_INFO(get_logger(),
                "  visual: %d segs, color=(%.1f,%.1f,%.1f), width=%.2fm, rate=%.0fHz",
                visual_segments_, visual_color_r_, visual_color_g_, visual_color_b_,
                visual_width_, visual_rate);
        }
#endif
    }

private:
    void discover_entity_id() {
        discover_timer_->cancel();

        std::ostringstream cmd;
        cmd << "gz model -m " << model_name_ << " 2>/dev/null";

        std::array<char, 1024> buffer;
        std::string result;
        FILE* pipe = popen(cmd.str().c_str(), "r");
        if (pipe) {
            while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
                result += buffer.data();
            }
            pclose(pipe);
        }

        std::regex model_regex("Model: \\[(\\d+)\\]");
        std::smatch match;
        if (std::regex_search(result, match, model_regex)) {
            entity_id_ = std::stoi(match[1].str());
            wrench_output_->set_entity_id(entity_id_);
            RCLCPP_INFO(get_logger(), "Found model entity ID: %d", entity_id_);
        } else {
            RCLCPP_WARN(get_logger(), "Entity ID not found, retrying...");
            discover_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&CableDynamicsNode::discover_entity_id, this));
        }
    }

    void update() {
        if (!have_odom_ || !have_spool_ || entity_id_ <= 0) return;

        // Extract state from odometry (Gazebo ENU frame)
        double vx = odom_.twist.twist.linear.x;
        double vy = odom_.twist.twist.linear.y;
        double vz = odom_.twist.twist.linear.z;
        double altitude = odom_.pose.pose.position.z;  // ENU: Z = up

        double deployed = spool_.total_length;
        double payout = spool_.velocity;

        // Compute forces
        auto result = compute_cable_forces(
            props_, deployed, altitude, vx, vy, vz, payout, cable_broken_);

        // Latch broken state
        if (result.is_broken && !cable_broken_) {
            cable_broken_ = true;
            RCLCPP_ERROR(get_logger(),
                "CABLE BROKEN! Tension %.1f N exceeded breaking strength %.1f N "
                "(deployed=%.0fm, airborne=%.0fm)",
                result.tension, props_.breaking_strength,
                deployed, result.airborne_length);
        }

        // Apply wrench to drone (force only, no torque)
        if (enabled_ && !cable_broken_) {
            wrench_output_->publish(
                result.total_force.x, result.total_force.y, result.total_force.z,
                0.0, 0.0, 0.0);
        }

        // Compute remaining spool length (-1 = no capacity set)
        double remaining = (spool_capacity_ > 0.0)
            ? std::max(0.0, spool_capacity_ - deployed) : -1.0;

        // Publish status
        auto status = fiber_nav_sensors::msg::CableStatus();
        status.header.stamp = now();
        status.tension = static_cast<float>(result.tension);
        status.deployed_length = static_cast<float>(deployed);
        status.airborne_length = static_cast<float>(result.airborne_length);
        status.remaining_length = static_cast<float>(remaining);
        status.drag_force = static_cast<float>(result.drag_magnitude);
        status.weight_force = static_cast<float>(result.weight_magnitude);
        status.spool_friction = static_cast<float>(result.friction_magnitude);
        status.is_broken = cable_broken_;
        status_pub_->publish(status);

        // Publish scalar tension for plotting
        auto tension_msg = std_msgs::msg::Float64();
        tension_msg.data = result.tension;
        tension_pub_->publish(tension_msg);

        // Publish scalar remaining for Foxglove gauge
        auto remaining_msg = std_msgs::msg::Float64();
        remaining_msg.data = remaining;
        remaining_pub_->publish(remaining_msg);

        // Log at 1 Hz
        log_counter_++;
        if (log_counter_ % 50 == 0 && deployed > 1.0) {
            RCLCPP_INFO(get_logger(),
                "[Cable] T=%.1fN drag=%.1fN weight=%.1fN "
                "deployed=%.0fm airborne=%.0fm alt=%.0fm",
                result.tension, result.drag_magnitude, result.weight_magnitude,
                deployed, result.airborne_length, altitude);
        }
    }

#ifdef USE_GAZEBO_WRENCH
    void update_visual() {
        if (!have_odom_) return;

        // Track launch position from first odometry
        if (!have_launch_pos_) {
            launch_x_ = odom_.pose.pose.position.x;
            launch_y_ = odom_.pose.pose.position.y;
            launch_z_ = odom_.pose.pose.position.z;
            have_launch_pos_ = true;
        }

        double deployed = have_spool_ ? spool_.total_length : 0.0;
        if (deployed < 0.5 || cable_broken_) {
            if (visual_marker_active_) delete_visual();
            return;
        }

        // Current drone position (Gazebo ENU frame)
        double drone_x = odom_.pose.pose.position.x;
        double drone_y = odom_.pose.pose.position.y;
        double drone_z = odom_.pose.pose.position.z;

        // Altitude above launch ground level
        double altitude = std::max(0.0, drone_z - launch_z_);

        // Airborne cable length
        double airborne = compute_airborne_length(deployed, altitude);
        double ground_length = deployed - airborne;

        // Direction from launch to drone (2D projection)
        double dx = drone_x - launch_x_;
        double dy = drone_y - launch_y_;
        double d_horiz = std::sqrt(dx * dx + dy * dy);
        double dir_x = (d_horiz > 0.1) ? dx / d_horiz : 1.0;
        double dir_y = (d_horiz > 0.1) ? dy / d_horiz : 0.0;

        // Ground contact point (where cable lifts off)
        double contact_dist = std::min(ground_length, d_horiz);
        double contact_x = launch_x_ + dir_x * contact_dist;
        double contact_y = launch_y_ + dir_y * contact_dist;
        double contact_z = launch_z_;

        // Build list of cable path points (vertices of the polyline)
        std::vector<std::array<double, 3>> pts;
        pts.reserve(static_cast<size_t>(visual_segments_) + 2);

        int total_segs = visual_segments_;
        int ground_segs = 0;
        int airborne_segs = total_segs;
        if (ground_length > 0.5 && deployed > 0.5) {
            ground_segs = std::max(1, static_cast<int>(
                total_segs * ground_length / deployed));
            airborne_segs = total_segs - ground_segs;
        }
        airborne_segs = std::max(airborne_segs, 3);

        // Ground vertices: straight line from launch to contact
        if (ground_segs > 0) {
            for (int i = 0; i <= ground_segs; ++i) {
                double t = static_cast<double>(i) / ground_segs;
                pts.push_back({
                    launch_x_ + dir_x * contact_dist * t,
                    launch_y_ + dir_y * contact_dist * t,
                    contact_z + 0.2});
            }
        } else {
            pts.push_back({contact_x, contact_y, contact_z + 0.2});
        }

        // Airborne vertices: quadratic catenary from contact to drone
        for (int i = 1; i <= airborne_segs; ++i) {
            double t = static_cast<double>(i) / airborne_segs;
            pts.push_back({
                contact_x + (drone_x - contact_x) * t,
                contact_y + (drone_y - contact_y) * t,
                contact_z + (drone_z - contact_z) * t * t});
        }

        // Publish a cylinder marker for each consecutive pair of points
        int seg_idx = 0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            publish_segment(seg_idx++, pts[i], pts[i + 1]);
        }

        // Delete excess segments from previous frame
        for (int i = seg_idx; i < prev_seg_count_; ++i) {
            delete_segment(i);
        }
        prev_seg_count_ = seg_idx;
        visual_marker_active_ = true;
    }

    void publish_segment(int id,
                         const std::array<double, 3>& a,
                         const std::array<double, 3>& b) {
        // Midpoint
        double mx = (a[0] + b[0]) * 0.5;
        double my = (a[1] + b[1]) * 0.5;
        double mz = (a[2] + b[2]) * 0.5;

        // Segment vector and length
        double sx = b[0] - a[0];
        double sy = b[1] - a[1];
        double sz = b[2] - a[2];
        double len = std::sqrt(sx * sx + sy * sy + sz * sz);
        if (len < 0.01) return;

        // Cylinder default axis is Z. Compute quaternion to rotate Z to segment.
        // axis = normalize(Z x seg), angle = acos(Z . seg / |seg|)
        double nx = sx / len, ny = sy / len, nz = sz / len;
        // Cross product of (0,0,1) x (nx,ny,nz) = (-ny, nx, 0)
        double cx = -ny, cy = nx, cz = 0.0;
        double cross_len = std::sqrt(cx * cx + cy * cy);
        double dot = nz;  // (0,0,1) . (nx,ny,nz)

        double qx = 0, qy = 0, qz = 0, qw = 1;
        if (cross_len > 1e-6) {
            double half_angle = std::atan2(cross_len, dot) * 0.5;
            double s = std::sin(half_angle) / cross_len;
            qx = cx * s;
            qy = cy * s;
            qz = cz * s;
            qw = std::cos(half_angle);
        } else if (dot < 0) {
            // 180 degree rotation — segment points straight down
            qx = 1; qy = 0; qz = 0; qw = 0;
        }

        gz::msgs::Marker msg;
        msg.set_ns("fiber_cable");
        msg.set_id(id);
        msg.set_action(gz::msgs::Marker::ADD_MODIFY);
        msg.set_type(gz::msgs::Marker::CYLINDER);

        auto* pose = msg.mutable_pose();
        auto* pos = pose->mutable_position();
        pos->set_x(mx);
        pos->set_y(my);
        pos->set_z(mz);
        auto* ori = pose->mutable_orientation();
        ori->set_x(qx);
        ori->set_y(qy);
        ori->set_z(qz);
        ori->set_w(qw);

        // Scale: x,y = diameter, z = length
        auto* scale = msg.mutable_scale();
        scale->set_x(visual_width_);
        scale->set_y(visual_width_);
        scale->set_z(len);

        auto* mat = msg.mutable_material();
        auto* diffuse = mat->mutable_diffuse();
        diffuse->set_r(visual_color_r_);
        diffuse->set_g(visual_color_g_);
        diffuse->set_b(visual_color_b_);
        diffuse->set_a(visual_color_a_);
        auto* ambient = mat->mutable_ambient();
        ambient->set_r(visual_color_r_);
        ambient->set_g(visual_color_g_);
        ambient->set_b(visual_color_b_);
        ambient->set_a(visual_color_a_);

        gz_marker_node_.Request("/sensors/marker", msg);
    }

    void delete_segment(int id) {
        gz::msgs::Marker msg;
        msg.set_ns("fiber_cable");
        msg.set_id(id);
        msg.set_action(gz::msgs::Marker::DELETE_MARKER);
        gz_marker_node_.Request("/sensors/marker", msg);
    }

    void delete_visual() {
        for (int i = 0; i < prev_seg_count_; ++i) {
            delete_segment(i);
        }
        prev_seg_count_ = 0;
        visual_marker_active_ = false;
    }
#endif

    // Parameters
    CableProperties props_;
    std::string world_name_;
    std::string model_name_;
    bool enabled_{true};
    double spool_capacity_{7500.0};

    // State
    nav_msgs::msg::Odometry odom_;
    fiber_nav_sensors::msg::SpoolStatus spool_;
    bool have_odom_{false};
    bool have_spool_{false};
    int entity_id_{0};
    bool cable_broken_{false};
    int log_counter_{0};

    // Visual cable rendering (Gazebo markers)
#ifdef USE_GAZEBO_WRENCH
    gz::transport::Node gz_marker_node_;
    bool visual_enabled_{false};
    int visual_segments_{30};
    float visual_color_r_{0.1f};
    float visual_color_g_{1.0f};
    float visual_color_b_{0.1f};
    float visual_color_a_{1.0f};
    double visual_width_{0.15};
    double launch_x_{0.0};
    double launch_y_{0.0};
    double launch_z_{0.0};
    bool have_launch_pos_{false};
    bool visual_marker_active_{false};
    int prev_seg_count_{0};
    rclcpp::TimerBase::SharedPtr visual_timer_;
#endif

    // I/O
    std::unique_ptr<WrenchOutput> wrench_output_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<fiber_nav_sensors::msg::SpoolStatus>::SharedPtr spool_sub_;
    rclcpp::Publisher<fiber_nav_sensors::msg::CableStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tension_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr remaining_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr discover_timer_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::CableDynamicsNode>());
    rclcpp::shutdown();
    return 0;
}
