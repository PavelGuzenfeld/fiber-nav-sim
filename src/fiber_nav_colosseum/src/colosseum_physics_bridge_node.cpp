/// Colosseum Physics Bridge Node
///
/// Full VTOL physics simulation + MAVLink HIL bridge for PX4.
///
/// Architecture:
///   PX4 SITL <── MAVLink TCP:4560 ──> this node
///                                       ├── VehicleDynamics (200Hz)
///                                       ├── SensorSynthesis → HIL_SENSOR/HIL_GPS → PX4
///                                       ├── AirsimRpcClient
///                                       │   ├── simSetVehiclePose() ← physics state
///                                       │   └── simGetImages() → /camera/compressed
///                                       └── ROS 2 topics (odometry, imu, gps, etc.)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "fiber_nav_colosseum/airsim_rpc_client.hpp"

#include <fiber_nav_physics/hil_sensor_publisher.hpp>
#include <fiber_nav_physics/mavlink_connection.hpp>
#include <fiber_nav_physics/sensor_synthesis.hpp>
#include <fiber_nav_physics/vehicle_dynamics.hpp>

#include <chrono>
#include <mutex>

namespace fiber_nav_colosseum
{

namespace fnp = fiber_nav_physics;

class ColosseumPhysicsBridgeNode : public rclcpp::Node
{
public:
    ColosseumPhysicsBridgeNode()
        : Node("colosseum_physics_bridge")
    {
        declare_parameters();
        create_publishers();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // AirSim client
        auto host = get_parameter("airsim_host").as_string();
        auto port = static_cast<uint16_t>(get_parameter("airsim_port").as_int());
        airsim_client_ = std::make_unique<AirsimRpcClient>(host, port);

        // MAVLink connection to PX4
        mavlink_conn_ = std::make_shared<fnp::MavlinkConnection>();
        mavlink_conn_->set_log_callback([this](const char* msg) {
            RCLCPP_INFO(get_logger(), "%s", msg);
        });
        mavlink_conn_->set_message_callback([this](const mavlink_message_t& msg) {
            on_mavlink_message(msg);
        });

        hil_publisher_.init(mavlink_conn_);

        // Connection retry timer (connects to both AirSim and PX4)
        connect_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() { attempt_connection(); });
    }

    ~ColosseumPhysicsBridgeNode() override
    {
        mavlink_conn_->stop();
    }

private:
    void declare_parameters()
    {
        // AirSim connection
        declare_parameter("airsim_host", "127.0.0.1");
        declare_parameter("airsim_port", 41451);
        declare_parameter("camera_name", "front_center");
        declare_parameter("image_rate_hz", 30.0);

        // PX4 MAVLink connection
        declare_parameter("px4_host", "127.0.0.1");
        declare_parameter("px4_port", 4560);

        // Physics
        declare_parameter("physics_rate_hz", 200.0);
        declare_parameter("vehicle_mass", 2.0);
        declare_parameter("inertia_xx", 0.03);
        declare_parameter("inertia_yy", 0.03);
        declare_parameter("inertia_zz", 0.01);

        // Aerodynamics (tailsitter defaults)
        declare_parameter("aero.CLa", 4.7);
        declare_parameter("aero.CD0", 0.02);
        declare_parameter("aero.alpha_stall", 0.3);
        declare_parameter("aero.wing_area", 0.15);
        declare_parameter("aero.wing_aspect_ratio", 4.0);
        declare_parameter("aero.wing_mac", 0.2);
        declare_parameter("aero.min_airspeed", 2.0);

        // Motors (4-motor quadtailsitter defaults)
        declare_parameter("motor.count", 4);
        declare_parameter("motor.motor_constant", 8.54858e-06);
        declare_parameter("motor.moment_constant", 0.016);
        declare_parameter("motor.max_rot_velocity", 1200.0);
        declare_parameter("motor.time_constant_up", 0.0125);
        declare_parameter("motor.time_constant_down", 0.025);
        declare_parameter("motor.arm_length", 0.174);  // m, diagonal from CoM

        // VTOL transition
        declare_parameter("vtol.fw_transition_speed", 15.0);
        declare_parameter("vtol.mc_transition_speed", 8.0);
        declare_parameter("vtol.transition_timeout", 5.0);
        declare_parameter("vtol.back_transition_time", 5.0);

        // GPS origin (Negev desert, matching Gazebo world)
        declare_parameter("gps.origin_lat", 31.164093);
        declare_parameter("gps.origin_lon", 34.532227);
        declare_parameter("gps.origin_alt", 141.0);
        declare_parameter("gps.enabled", true);

        // Initial spawn position (NED, relative to GPS origin)
        declare_parameter("spawn_x", 0.0);
        declare_parameter("spawn_y", 0.0);
        declare_parameter("spawn_z", -0.5);  // slightly above ground
    }

    void create_publishers()
    {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/model/quadtailsitter/odometry", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
            "/vehicle/nav_sat_fix", 10);
        baro_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/barometer/altitude", 10);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
            "/sensors/magnetometer", 10);
        range_pub_ = create_publisher<sensor_msgs::msg::Range>(
            "/sensors/rangefinder", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/compressed", 10);
        clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        vtol_status_pub_ = create_publisher<std_msgs::msg::String>(
            "/colosseum/vtol_status", 10);
    }

    void attempt_connection()
    {
        if (connected_)
        {
            return;
        }

        // Connect to AirSim
        if (!airsim_connected_)
        {
            RCLCPP_INFO(get_logger(), "Connecting to AirSim...");
            auto result = airsim_client_->connect();
            if (!result)
            {
                RCLCPP_WARN(get_logger(), "AirSim connection failed, retrying...");
                return;
            }
            auto ping = airsim_client_->ping();
            if (!ping || !*ping)
            {
                RCLCPP_WARN(get_logger(), "AirSim ping failed, retrying...");
                airsim_client_->disconnect();
                return;
            }
            RCLCPP_INFO(get_logger(), "Connected to AirSim API");
            airsim_connected_ = true;
        }

        // Start MAVLink connection to PX4
        if (!mavlink_started_)
        {
            auto px4_host = get_parameter("px4_host").as_string();
            auto px4_port = static_cast<uint16_t>(get_parameter("px4_port").as_int());

            fnp::MavlinkConnection::Config config{};
            config.host = px4_host.c_str();
            config.port = px4_port;
            config.connect_timeout_ms = 5000;
            config.reconnect_interval_ms = 2000;

            // Store host string for lifetime
            px4_host_str_ = px4_host;
            config.host = px4_host_str_.c_str();

            mavlink_conn_->start(config);
            mavlink_started_ = true;
            RCLCPP_INFO(get_logger(), "MAVLink connection started to %s:%u",
                         px4_host.c_str(), px4_port);
        }

        // Wait for MAVLink connection
        if (!mavlink_conn_->is_connected())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for PX4 MAVLink connection...");
            return;
        }

        RCLCPP_INFO(get_logger(), "Both AirSim and PX4 connected — starting physics");
        connected_ = true;
        connect_timer_->cancel();
        initialize_physics();
        start_simulation();
    }

    void initialize_physics()
    {
        // Build vehicle config from parameters
        fnp::VehicleConfig config{};
        config.mass = static_cast<float>(get_parameter("vehicle_mass").as_double());
        config.inertia = {
            static_cast<float>(get_parameter("inertia_xx").as_double()),
            static_cast<float>(get_parameter("inertia_yy").as_double()),
            static_cast<float>(get_parameter("inertia_zz").as_double()),
        };

        // Aerodynamics
        config.aero.coeffs.CLa = static_cast<float>(
            get_parameter("aero.CLa").as_double());
        config.aero.coeffs.CD0 = static_cast<float>(
            get_parameter("aero.CD0").as_double());
        config.aero.coeffs.alpha_stall = static_cast<float>(
            get_parameter("aero.alpha_stall").as_double());
        config.aero.wing.area = static_cast<float>(
            get_parameter("aero.wing_area").as_double());
        config.aero.wing.aspect_ratio = static_cast<float>(
            get_parameter("aero.wing_aspect_ratio").as_double());
        config.aero.wing.mac = static_cast<float>(
            get_parameter("aero.wing_mac").as_double());
        config.aero.min_airspeed = static_cast<float>(
            get_parameter("aero.min_airspeed").as_double());
        // Tailsitter body axes: forward=+Z, up=-X
        config.aero.wing.forward = {0.0f, 0.0f, 1.0f};
        config.aero.wing.upward = {-1.0f, 0.0f, 0.0f};

        // Motors (quadtailsitter X-config)
        int motor_count = get_parameter("motor.count").as_int();
        float arm = static_cast<float>(get_parameter("motor.arm_length").as_double());
        float kf = static_cast<float>(get_parameter("motor.motor_constant").as_double());
        float km = static_cast<float>(get_parameter("motor.moment_constant").as_double());
        float max_vel = static_cast<float>(
            get_parameter("motor.max_rot_velocity").as_double());
        float tau_up = static_cast<float>(
            get_parameter("motor.time_constant_up").as_double());
        float tau_down = static_cast<float>(
            get_parameter("motor.time_constant_down").as_double());

        // Standard quadrotor X-config motor positions and directions
        // Motor 0: front-right (CCW), Motor 1: rear-left (CCW)
        // Motor 2: front-left (CW), Motor 3: rear-right (CW)
        struct MotorDef { float px, py; int dir; };
        std::array<MotorDef, 4> quad_motors = {{
            { arm,  -arm, 1},  // front-right, CCW
            {-arm,   arm, 1},  // rear-left, CCW
            { arm,   arm, -1}, // front-left, CW
            {-arm,  -arm, -1}, // rear-right, CW
        }};

        for (int i = 0; i < motor_count && i < 4; ++i)
        {
            fnp::MotorConfig mc{};
            mc.position = {quad_motors[i].px, quad_motors[i].py, 0.0f};
            mc.motor_constant = kf;
            mc.moment_constant = km;
            mc.max_rot_velocity = max_vel;
            mc.time_constant_up = tau_up;
            mc.time_constant_down = tau_down;
            mc.direction = quad_motors[i].dir;
            config.motors.push_back(mc);
        }

        // VTOL transition
        config.vtol.fw_transition_speed = static_cast<float>(
            get_parameter("vtol.fw_transition_speed").as_double());
        config.vtol.mc_transition_speed = static_cast<float>(
            get_parameter("vtol.mc_transition_speed").as_double());
        config.vtol.transition_timeout = static_cast<float>(
            get_parameter("vtol.transition_timeout").as_double());
        config.vtol.back_transition_time = static_cast<float>(
            get_parameter("vtol.back_transition_time").as_double());

        // Initial state
        fnp::VehicleState initial{};
        initial.position = {
            static_cast<float>(get_parameter("spawn_x").as_double()),
            static_cast<float>(get_parameter("spawn_y").as_double()),
            static_cast<float>(get_parameter("spawn_z").as_double()),
        };
        // Tailsitter hover: body +Z (thrust) points world -Z (up in NED)
        // 180° pitch around Y axis
        initial.orientation = fnp::Quat::from_axis_angle(
            {0.0f, 1.0f, 0.0f}, 3.14159265f);

        dynamics_.init(config, initial);

        // Sensor config
        gps_config_.origin_lat = get_parameter("gps.origin_lat").as_double();
        gps_config_.origin_lon = get_parameter("gps.origin_lon").as_double();
        gps_config_.origin_alt = static_cast<float>(
            get_parameter("gps.origin_alt").as_double());
        gps_config_.enabled = get_parameter("gps.enabled").as_bool();

        prev_velocity_ = initial.velocity;

        RCLCPP_INFO(get_logger(),
            "Physics initialized: mass=%.1fkg, %d motors, arm=%.3fm",
            config.mass, motor_count, arm);
    }

    void start_simulation()
    {
        double physics_hz = get_parameter("physics_rate_hz").as_double();
        physics_dt_ = static_cast<float>(1.0 / physics_hz);

        // Main physics timer (200 Hz)
        physics_timer_ = create_wall_timer(
            std::chrono::duration<double>(physics_dt_),
            [this]() { physics_tick(); });

        // Camera image timer (30 Hz, separate from physics)
        double image_hz = get_parameter("image_rate_hz").as_double();
        image_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / image_hz),
            [this]() { publish_image(); });

        RCLCPP_INFO(get_logger(),
            "Simulation started: physics=%.0fHz, image=%.0fHz",
            physics_hz, image_hz);
    }

    void physics_tick()
    {
        // Step physics (lock protects against concurrent set_motor_outputs)
        {
            std::lock_guard lock(motor_mutex_);
            dynamics_.step(physics_dt_);
        }

        auto& state = dynamics_.state();
        auto stamp = now();
        uint64_t time_usec = static_cast<uint64_t>(
            stamp.nanoseconds() / 1000);

        // ── Send HIL_SENSOR to PX4 (every tick = 200 Hz) ──
        auto sensor_data = fnp::synthesize_sensors(
            state, prev_velocity_, physics_dt_, noise_config_, gps_config_);
        hil_publisher_.send_hil_sensor(sensor_data, time_usec);
        prev_velocity_ = state.velocity;

        // ── Send HIL_GPS to PX4 (every 20th tick = 10 Hz) ──
        if (++gps_counter_ >= 20)
        {
            gps_counter_ = 0;
            if (gps_config_.enabled)
            {
                auto gps_data = fnp::synthesize_gps(state, gps_config_);
                hil_publisher_.send_hil_gps(gps_data, time_usec);
            }
        }

        // ── Send HEARTBEAT to PX4 (every 200th tick = 1 Hz) ──
        if (++heartbeat_counter_ >= 200)
        {
            heartbeat_counter_ = 0;
            hil_publisher_.send_heartbeat();
        }

        // ── Push pose to AirSim for rendering ──
        if (++airsim_pose_counter_ >= 2) // 100 Hz pose update to AirSim
        {
            airsim_pose_counter_ = 0;
            push_pose_to_airsim(state);
        }

        // ── Publish ROS 2 topics ──
        publish_odometry(state, stamp);
        publish_imu(sensor_data, stamp);

        // Lower-rate ROS 2 topics
        if (gps_counter_ == 0)
        {
            publish_gps(state, stamp);
        }
        if (++baro_counter_ >= 4) // 50 Hz
        {
            baro_counter_ = 0;
            publish_barometer(sensor_data, stamp);
            publish_magnetometer(sensor_data, stamp);
        }

        // Clock
        publish_clock(stamp);

        // VTOL status (1 Hz)
        if (heartbeat_counter_ == 0)
        {
            publish_vtol_status();
        }
    }

    void on_mavlink_message(const mavlink_message_t& msg)
    {
        if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
        {
            mavlink_hil_actuator_controls_t actuators{};
            mavlink_msg_hil_actuator_controls_decode(&msg, &actuators);

            fnp::MotorOutputs outputs{};
            for (int i = 0; i < fnp::MotorOutputs::max_motors; ++i)
            {
                outputs.controls[i] = actuators.controls[i];
            }
            outputs.mode = actuators.mode;
            outputs.flags = actuators.flags;

            // Thread-safe: MAVLink receive runs on background thread
            std::lock_guard lock(motor_mutex_);
            dynamics_.set_motor_outputs(outputs);
        }
    }

    void push_pose_to_airsim(const fnp::VehicleState& state)
    {
        // Convert NED physics state to AirSim pose
        // AirSim uses NED, so direct mapping
        AirsimRpcClient::Pose pose{};
        pose.x = state.position.x;
        pose.y = state.position.y;
        pose.z = state.position.z;
        pose.qw = state.orientation.w;
        pose.qx = state.orientation.x;
        pose.qy = state.orientation.y;
        pose.qz = state.orientation.z;

        auto result = airsim_client_->set_pose(pose);
        if (!result)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Failed to set AirSim pose");
        }
    }

    // ── ROS 2 publishers ──

    void publish_odometry(const fnp::VehicleState& state,
                          const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = state.position.x;
        msg.pose.pose.position.y = state.position.y;
        msg.pose.pose.position.z = state.position.z;
        msg.pose.pose.orientation.w = state.orientation.w;
        msg.pose.pose.orientation.x = state.orientation.x;
        msg.pose.pose.orientation.y = state.orientation.y;
        msg.pose.pose.orientation.z = state.orientation.z;
        msg.twist.twist.linear.x = state.velocity.x;
        msg.twist.twist.linear.y = state.velocity.y;
        msg.twist.twist.linear.z = state.velocity.z;
        msg.twist.twist.angular.x = state.angular_velocity.x;
        msg.twist.twist.angular.y = state.angular_velocity.y;
        msg.twist.twist.angular.z = state.angular_velocity.z;
        odom_pub_->publish(msg);

        // TF broadcast
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = state.position.x;
        tf.transform.translation.y = state.position.y;
        tf.transform.translation.z = state.position.z;
        tf.transform.rotation.w = state.orientation.w;
        tf.transform.rotation.x = state.orientation.x;
        tf.transform.rotation.y = state.orientation.y;
        tf.transform.rotation.z = state.orientation.z;
        tf_broadcaster_->sendTransform(tf);
    }

    void publish_imu(const fnp::SensorData& data, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "imu_link";
        msg.angular_velocity.x = data.gyro.x;
        msg.angular_velocity.y = data.gyro.y;
        msg.angular_velocity.z = data.gyro.z;
        msg.linear_acceleration.x = data.accel.x;
        msg.linear_acceleration.y = data.accel.y;
        msg.linear_acceleration.z = data.accel.z;
        auto& q = dynamics_.state().orientation;
        msg.orientation.w = q.w;
        msg.orientation.x = q.x;
        msg.orientation.y = q.y;
        msg.orientation.z = q.z;
        imu_pub_->publish(msg);
    }

    void publish_gps(const fnp::VehicleState& state, const rclcpp::Time& stamp)
    {
        if (!gps_config_.enabled)
        {
            return;
        }

        auto gps = fnp::synthesize_gps(state, gps_config_);
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "gps_link";
        msg.latitude = static_cast<double>(gps.lat) / 1e7;
        msg.longitude = static_cast<double>(gps.lon) / 1e7;
        msg.altitude = static_cast<double>(gps.alt) / 1000.0;
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        float eph_m = static_cast<float>(gps.eph) / 100.0f;
        float epv_m = static_cast<float>(gps.epv) / 100.0f;
        msg.position_covariance[0] = eph_m * eph_m;
        msg.position_covariance[4] = eph_m * eph_m;
        msg.position_covariance[8] = epv_m * epv_m;
        msg.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_pub_->publish(msg);
    }

    void publish_barometer(const fnp::SensorData& data,
                           const rclcpp::Time& /*stamp*/)
    {
        std_msgs::msg::Float64 msg;
        msg.data = data.pressure_alt;
        baro_pub_->publish(msg);
    }

    void publish_magnetometer(const fnp::SensorData& data,
                              const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "mag_link";
        // Convert gauss → Tesla (1 Gauss = 1e-4 Tesla)
        msg.magnetic_field.x = data.mag.x * 1e-4;
        msg.magnetic_field.y = data.mag.y * 1e-4;
        msg.magnetic_field.z = data.mag.z * 1e-4;
        mag_pub_->publish(msg);
    }

    void publish_image()
    {
        if (!airsim_connected_)
        {
            return;
        }

        auto camera_name = get_parameter("camera_name").as_string();
        auto img_result = airsim_client_->get_image(camera_name, 0);
        if (!img_result)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Image fetch failed");
            return;
        }

        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = now();
        msg.header.frame_id = "camera_link";
        msg.format = "png";
        msg.data = std::move(*img_result);
        image_pub_->publish(msg);
        RCLCPP_INFO_ONCE(get_logger(), "First image published (%zu bytes)",
                         msg.data.size());
    }

    void publish_clock(const rclcpp::Time& stamp)
    {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = stamp;
        clock_pub_->publish(msg);
    }

    void publish_vtol_status()
    {
        auto mode = dynamics_.flight_mode();
        const char* mode_str = "UNKNOWN";
        switch (mode)
        {
        case fnp::VTOLFlightMode::Multicopter:    mode_str = "MC"; break;
        case fnp::VTOLFlightMode::TransitionToFW: mode_str = "TRANSITION_FW"; break;
        case fnp::VTOLFlightMode::FixedWing:      mode_str = "FW"; break;
        case fnp::VTOLFlightMode::TransitionToMC: mode_str = "TRANSITION_MC"; break;
        }

        auto& aero = dynamics_.aero_state();
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "{\"mode\":\"%s\",\"airspeed\":%.1f,\"aoa\":%.2f,\"sideslip\":%.2f,"
            "\"hil_sensor_count\":%lu,\"hil_gps_count\":%lu}",
            mode_str, aero.airspeed, aero.angle_of_attack, aero.sideslip_angle,
            hil_publisher_.sensor_count(), hil_publisher_.gps_count());

        std_msgs::msg::String msg;
        msg.data = buf;
        vtol_status_pub_->publish(msg);
    }

    // ── Members ──
    std::unique_ptr<AirsimRpcClient> airsim_client_;
    std::shared_ptr<fnp::MavlinkConnection> mavlink_conn_;
    fnp::HilSensorPublisher hil_publisher_;
    fnp::VehicleDynamics dynamics_;

    fnp::SensorNoiseConfig noise_config_;
    fnp::GPSConfig gps_config_;
    fnp::Vec3 prev_velocity_{};

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string px4_host_str_; // keeps host string alive for MavlinkConnection

    bool airsim_connected_{false};
    bool mavlink_started_{false};
    bool connected_{false};

    float physics_dt_{0.005f};
    int gps_counter_{0};
    int heartbeat_counter_{0};
    int airsim_pose_counter_{0};
    int baro_counter_{0};
    std::mutex motor_mutex_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr baro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vtol_status_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr physics_timer_;
    rclcpp::TimerBase::SharedPtr image_timer_;
};

} // namespace fiber_nav_colosseum

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<fiber_nav_colosseum::ColosseumPhysicsBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
