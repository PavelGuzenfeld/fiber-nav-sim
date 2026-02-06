#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <cmath>
#include <array>
#include <cstdio>
#include <regex>
#include <sstream>
#include <string>
#include <numbers>
#include <vector>

namespace fiber_nav_sensors {

struct Vec3 { double x, y, z; };
struct Quat { double w, x, y, z; };

inline Vec3 quaternion_to_euler(Quat const& q) {
    // ZYX convention (roll, pitch, yaw)
    double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr, cosr);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = (std::abs(sinp) >= 1.0)
        ? std::copysign(std::numbers::pi / 2.0, sinp)
        : std::asin(sinp);

    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny, cosy);

    return {roll, pitch, yaw};
}

inline Vec3 rotate_vector(Quat const& q, Vec3 const& v) {
    // q * v * q^-1 via cross-product form
    Vec3 u{q.x, q.y, q.z};
    double s = q.w;
    // t = 2 * (u x v)
    Vec3 t{2.0 * (u.y * v.z - u.z * v.y),
            2.0 * (u.z * v.x - u.x * v.z),
            2.0 * (u.x * v.y - u.y * v.x)};
    return {v.x + s * t.x + (u.y * t.z - u.z * t.y),
            v.y + s * t.y + (u.z * t.x - u.x * t.z),
            v.z + s * t.z + (u.x * t.y - u.y * t.x)};
}

inline double wrap_angle(double a) {
    while (a > std::numbers::pi) a -= 2.0 * std::numbers::pi;
    while (a < -std::numbers::pi) a += 2.0 * std::numbers::pi;
    return a;
}

inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

class StabilizedFlightController : public rclcpp::Node {
public:
    StabilizedFlightController() : Node("stabilized_flight_controller") {
        // Parameters
        declare_parameter("world_name", "canyon_world");
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("target_altitude", 50.0);
        declare_parameter("target_speed", 15.0);
        declare_parameter("auto_enable", true);
        declare_parameter("acceptance_radius", 20.0);
        declare_parameter("physics_step_size", 0.001);

        // PD gains — tuned for Ixx=0.113, Iyy=0.030, Izz=0.084, mass=1.635
        declare_parameter("kp_roll", 8.0);
        declare_parameter("kd_roll", 2.0);
        declare_parameter("kp_pitch", 3.0);
        declare_parameter("kd_pitch", 0.8);
        declare_parameter("kp_yaw", 3.0);
        declare_parameter("kd_yaw", 1.0);
        declare_parameter("kp_alt", 8.0);
        declare_parameter("kd_alt", 6.0);
        declare_parameter("kp_lateral", 3.0);
        declare_parameter("kd_lateral", 3.0);
        declare_parameter("kp_speed", 2.0);
        declare_parameter("kd_speed", 0.5);

        world_name_ = get_parameter("world_name").as_string();
        model_name_ = get_parameter("model_name").as_string();
        target_alt_ = get_parameter("target_altitude").as_double();
        target_speed_ = get_parameter("target_speed").as_double();
        auto_enable_ = get_parameter("auto_enable").as_bool();
        accept_radius_ = get_parameter("acceptance_radius").as_double();
        physics_step_ = get_parameter("physics_step_size").as_double();

        kp_roll_ = get_parameter("kp_roll").as_double();
        kd_roll_ = get_parameter("kd_roll").as_double();
        kp_pitch_ = get_parameter("kp_pitch").as_double();
        kd_pitch_ = get_parameter("kd_pitch").as_double();
        kp_yaw_ = get_parameter("kp_yaw").as_double();
        kd_yaw_ = get_parameter("kd_yaw").as_double();
        kp_alt_ = get_parameter("kp_alt").as_double();
        kd_alt_ = get_parameter("kd_alt").as_double();
        kp_lateral_ = get_parameter("kp_lateral").as_double();
        kd_lateral_ = get_parameter("kd_lateral").as_double();
        kp_speed_ = get_parameter("kp_speed").as_double();
        kd_speed_ = get_parameter("kd_speed").as_double();

        // Weight: mass * g (base 1.6 + 4 rotors 0.005 + airspeed 0.015)
        weight_ = 1.635 * 9.81;

        // Racetrack waypoints (repeating oval at target altitude)
        waypoints_ = {
            {0,    20,  target_alt_},
            {200,  20,  target_alt_},
            {400,  20,  target_alt_},
            {600,  20,  target_alt_},
            {720,  10,  target_alt_},
            {720, -10,  target_alt_},
            {600, -20,  target_alt_},
            {400, -20,  target_alt_},
            {200, -20,  target_alt_},
            {80,  -10,  target_alt_},
            {80,   10,  target_alt_},
        };

        // Gz transport: use ONE-TIME wrench topic (not persistent!)
        // Gazebo's persistent wrench accumulates with every publish (push_back),
        // so repeated publishing at 50Hz causes force to grow without bound.
        // One-time wrenches are cleared after each physics step.
        wrench_topic_ = "/world/" + world_name_ + "/wrench";
        gz_pub_ = gz_node_.Advertise<gz::msgs::EntityWrench>(wrench_topic_);

        // ROS subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/" + model_name_ + "/odometry", 10,
            std::bind(&StabilizedFlightController::odom_callback, this,
                      std::placeholders::_1));

        // Enable/disable service
        enable_srv_ = create_service<std_srvs::srv::SetBool>(
            "~/enable",
            std::bind(&StabilizedFlightController::enable_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 50Hz control timer (wall clock — wrench is scaled by sim dt)
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&StabilizedFlightController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Stabilized flight controller initialized");
        RCLCPP_INFO(get_logger(), "  Model: %s, Target alt: %.0fm, Speed: %.0f m/s",
                    model_name_.c_str(), target_alt_, target_speed_);

        // Discover entity ID quickly
        discover_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&StabilizedFlightController::discover_entity_id, this));
    }

private:
    void discover_entity_id() {
        discover_timer_->cancel();

        RCLCPP_INFO(get_logger(), "Discovering model entity ID for %s...",
                    model_name_.c_str());

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

        // Parse "Model: [44]" to get model entity ID
        std::regex model_regex("Model: \\[(\\d+)\\]");
        std::smatch match;
        if (std::regex_search(result, match, model_regex)) {
            entity_id_ = std::stoi(match[1].str());
            RCLCPP_INFO(get_logger(), "Found model entity ID: %d", entity_id_);
            RCLCPP_INFO(get_logger(), "Control enabled, starting racetrack");
        } else {
            RCLCPP_ERROR(get_logger(), "Could not find entity ID. Output: %s",
                         result.substr(0, 200).c_str());
            discover_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&StabilizedFlightController::discover_entity_id, this));
        }
    }

    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
    }

    void enable_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (entity_id_ <= 0) {
            response->success = false;
            response->message = "Entity ID not discovered yet";
            return;
        }
        enabled_ = request->data;
        response->message = enabled_ ? "Control enabled" : "Control disabled";
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    static bool is_finite(double v) {
        return std::isfinite(v) && std::abs(v) < 1e6;
    }

    double get_sim_time() const {
        return odom_.header.stamp.sec + odom_.header.stamp.nanosec * 1e-9;
    }

    void control_loop() {
        if (entity_id_ <= 0 || !have_odom_) return;

        auto const& pos = odom_.pose.pose.position;
        auto const& orient = odom_.pose.pose.orientation;
        auto const& lin_vel = odom_.twist.twist.linear;
        auto const& ang_vel = odom_.twist.twist.angular;

        bool odom_valid = is_finite(pos.x) && is_finite(pos.y) && is_finite(pos.z) &&
            is_finite(orient.w) && is_finite(orient.x) &&
            is_finite(orient.y) && is_finite(orient.z) &&
            is_finite(lin_vel.x) && is_finite(ang_vel.x);

        if (!odom_valid) {
            valid_odom_count_ = 0;
            if (!state_invalid_) {
                RCLCPP_WARN(get_logger(),
                    "Invalid odom: pos=(%.2g,%.2g,%.2g)",
                    pos.x, pos.y, pos.z);
                state_invalid_ = true;
            }
            return;
        }

        ++valid_odom_count_;

        if (state_invalid_) {
            if (valid_odom_count_ < required_valid_count_) return;
            RCLCPP_INFO(get_logger(), "Odometry stable (%d valid readings)",
                        valid_odom_count_);
            state_invalid_ = false;
            control_start_time_ = 0.0;
        }

        // Auto-enable after valid odom
        if (!enabled_ && auto_enable_ && valid_odom_count_ >= required_valid_count_) {
            enabled_ = true;
            RCLCPP_INFO(get_logger(),
                "Control enabled at pos=(%.1f,%.1f,%.1f)",
                pos.x, pos.y, pos.z);
        }

        if (!enabled_) return;

        // Compute sim time delta for impulse scaling
        double sim_time = get_sim_time();
        if (last_publish_sim_time_ <= 0.0) {
            last_publish_sim_time_ = sim_time;
            control_start_time_ = sim_time;
            return;  // Skip first tick to establish baseline
        }
        double sim_dt = sim_time - last_publish_sim_time_;
        if (sim_dt < physics_step_) return;  // No new physics step yet
        last_publish_sim_time_ = sim_time;

        // Impulse scale: number of physics steps since last publish
        double scale = clamp(sim_dt / physics_step_, 1.0, 500.0);

        // Startup ramp over 5 sim-seconds
        constexpr double ramp_duration = 5.0;
        double elapsed = sim_time - control_start_time_;
        double ramp = clamp(elapsed / ramp_duration, 0.0, 1.0);

        Quat q{orient.w, orient.x, orient.y, orient.z};
        auto [roll, pitch, yaw] = quaternion_to_euler(q);

        // Rotate body-frame linear velocity to world frame
        Vec3 vel_world = rotate_vector(q, {lin_vel.x, lin_vel.y, lin_vel.z});

        // Current waypoint
        auto const& wp = waypoints_[wp_index_];
        double dx = wp.x - pos.x;
        double dy = wp.y - pos.y;
        double dist2d = std::sqrt(dx * dx + dy * dy);

        // Advance waypoint
        if (dist2d < accept_radius_) {
            wp_index_ = (wp_index_ + 1) % waypoints_.size();
            auto const& nwp = waypoints_[wp_index_];
            RCLCPP_INFO(get_logger(), "Reached WP %zu (%.0f, %.0f, %.0f)",
                        wp_index_, nwp.x, nwp.y, nwp.z);
        }

        // Desired heading toward waypoint
        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = wrap_angle(desired_yaw - yaw);

        // --- Attitude PD (body-frame torques) ---
        double tx = kp_roll_  * (0.0 - roll)  + kd_roll_  * (-ang_vel.x);
        double ty = kp_pitch_ * (0.0 - pitch) + kd_pitch_ * (-ang_vel.y);
        double tz = kp_yaw_   * yaw_error     + kd_yaw_   * (-ang_vel.z);

        constexpr double max_torque = 3.0;  // Nm
        tx = clamp(tx, -max_torque, max_torque);
        ty = clamp(ty, -max_torque, max_torque);
        tz = clamp(tz, -max_torque, max_torque);

        // Rotate torques to world frame
        Vec3 torque_world = rotate_vector(q, {tx, ty, tz});
        torque_world.x = clamp(torque_world.x, -max_torque, max_torque) * ramp;
        torque_world.y = clamp(torque_world.y, -max_torque, max_torque) * ramp;
        torque_world.z = clamp(torque_world.z, -max_torque, max_torque) * ramp;

        // --- Altitude PD ---
        double alt_error = clamp(target_alt_ - pos.z, -10.0, 10.0);
        double fz = weight_ + kp_alt_ * alt_error + kd_alt_ * (-vel_world.z);
        fz = clamp(fz, 0.0, 3.0 * weight_);

        // --- Forward speed PD ---
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double speed_forward = vel_world.x * cos_yaw + vel_world.y * sin_yaw;
        double speed_error = target_speed_ - speed_forward;
        double f_forward = kp_speed_ * speed_error + kd_speed_ * (-speed_forward);
        f_forward = clamp(f_forward, -8.0, 12.0);

        // --- Cross-track (lateral) PD ---
        double v_lateral = -vel_world.x * sin_yaw + vel_world.y * cos_yaw;
        double cross_track = clamp(-dx * sin_yaw + dy * cos_yaw, -20.0, 20.0);
        double f_lateral = kp_lateral_ * cross_track + kd_lateral_ * (-v_lateral);
        f_lateral = clamp(f_lateral, -8.0, 8.0);

        // Project forward and lateral into world X, Y, apply ramp
        double fx = (f_forward * cos_yaw - f_lateral * sin_yaw) * ramp;
        double fy = (f_forward * sin_yaw + f_lateral * cos_yaw) * ramp;
        // Altitude: always apply weight for hover, ramp only the correction
        fz = weight_ + (fz - weight_) * ramp;

        // Periodic logging
        if (++log_counter_ % 125 == 0) {
            RCLCPP_INFO(get_logger(),
                "pos=(%.1f,%.1f,%.1f) rpy=(%.2f,%.2f,%.2f) "
                "F=(%.1f,%.1f,%.1f) T=(%.2f,%.2f,%.2f) ramp=%.2f scale=%.0f wp=%zu",
                pos.x, pos.y, pos.z, roll, pitch, yaw,
                fx, fy, fz, torque_world.x, torque_world.y, torque_world.z,
                ramp, scale, wp_index_);
        }

        // Publish one-time wrench scaled by physics steps since last publish.
        // One-time wrench applies for a single physics step then is removed.
        // Scaling ensures the impulse equals continuous_force * sim_dt.
        publish_wrench(fx * scale, fy * scale, fz * scale,
                       torque_world.x * scale,
                       torque_world.y * scale,
                       torque_world.z * scale);
    }

    void publish_wrench(double fx, double fy, double fz,
                         double tx, double ty, double tz) {
        gz::msgs::EntityWrench msg;
        auto* entity = msg.mutable_entity();
        entity->set_id(entity_id_);
        entity->set_type(gz::msgs::Entity::MODEL);
        auto* wrench = msg.mutable_wrench();
        wrench->mutable_force()->set_x(fx);
        wrench->mutable_force()->set_y(fy);
        wrench->mutable_force()->set_z(fz);
        wrench->mutable_torque()->set_x(tx);
        wrench->mutable_torque()->set_y(ty);
        wrench->mutable_torque()->set_z(tz);
        gz_pub_.Publish(msg);
    }

    // Parameters
    std::string world_name_;
    std::string model_name_;
    double target_alt_;
    double target_speed_;
    bool auto_enable_;
    double accept_radius_;
    double weight_;
    double physics_step_;

    // PD gains
    double kp_roll_, kd_roll_;
    double kp_pitch_, kd_pitch_;
    double kp_yaw_, kd_yaw_;
    double kp_alt_, kd_alt_;
    double kp_lateral_, kd_lateral_;
    double kp_speed_, kd_speed_;

    // State
    int entity_id_{0};
    bool enabled_{false};
    bool have_odom_{false};
    bool state_invalid_{true};
    int valid_odom_count_{0};
    static constexpr int required_valid_count_ = 25;
    double control_start_time_{0.0};
    double last_publish_sim_time_{0.0};
    nav_msgs::msg::Odometry odom_;
    std::size_t wp_index_{0};
    int log_counter_{0};

    // Racetrack waypoints
    std::vector<Vec3> waypoints_;

    // Gz transport
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_pub_;
    std::string wrench_topic_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr discover_timer_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::StabilizedFlightController>());
    rclcpp::shutdown();
    return 0;
}
