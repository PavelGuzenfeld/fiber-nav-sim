#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "fiber_nav_colosseum/airsim_rpc_client.hpp"

#include <chrono>
#include <string>

namespace fiber_nav_colosseum {

class ColosseumBridgeNode : public rclcpp::Node {
public:
    ColosseumBridgeNode() : Node("colosseum_bridge") {
        declare_parameter("host", "127.0.0.1");
        declare_parameter("port", 41451);
        declare_parameter("camera_name", "front_center");
        declare_parameter("image_rate_hz", 30.0);
        declare_parameter("odom_rate_hz", 100.0);
        declare_parameter("imu_rate_hz", 200.0);
        declare_parameter("baro_rate_hz", 50.0);
        declare_parameter("gps_rate_hz", 10.0);
        declare_parameter("mag_rate_hz", 50.0);
        declare_parameter("range_rate_hz", 50.0);
        declare_parameter("clock_rate_hz", 100.0);

        auto host = get_parameter("host").as_string();
        auto port = static_cast<uint16_t>(get_parameter("port").as_int());

        client_ = std::make_unique<AirsimRpcClient>(host, port);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/model/quadtailsitter/odometry", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/imu", 10);
        gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
            "/vehicle/nav_sat_fix", 10);
        baro_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/barometer/altitude", 10);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
            "/sensors/magnetometer", 10);
        range_pub_ = create_publisher<sensor_msgs::msg::Range>(
            "/sensors/rangefinder", 10);
        clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock", 10);

        // Connection retry timer
        connect_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() { attempt_connection(); });
    }

private:
    void attempt_connection() {
        if (connected_) return;

        auto host = get_parameter("host").as_string();
        auto port = get_parameter("port").as_int();
        RCLCPP_INFO(get_logger(), "Connecting to AirSim at %s:%ld...",
                     host.c_str(), port);

        auto result = client_->connect();
        if (!result) {
            RCLCPP_WARN(get_logger(), "Connection failed, retrying...");
            return;
        }

        auto ping = client_->ping();
        if (!ping || !*ping) {
            RCLCPP_WARN(get_logger(), "Ping failed, retrying...");
            client_->disconnect();
            return;
        }

        RCLCPP_INFO(get_logger(), "Connected to AirSim API");
        connected_ = true;
        connect_timer_->cancel();
        start_publishing();
    }

    void start_publishing() {
        auto make_timer = [this](double hz, auto callback) {
            return create_wall_timer(
                std::chrono::duration<double>(1.0 / hz), callback);
        };

        odom_timer_ = make_timer(
            get_parameter("odom_rate_hz").as_double(),
            [this]() { publish_odometry(); });

        imu_timer_ = make_timer(
            get_parameter("imu_rate_hz").as_double(),
            [this]() { publish_imu(); });

        image_timer_ = make_timer(
            get_parameter("image_rate_hz").as_double(),
            [this]() { publish_image(); });

        gps_timer_ = make_timer(
            get_parameter("gps_rate_hz").as_double(),
            [this]() { publish_gps(); });

        baro_timer_ = make_timer(
            get_parameter("baro_rate_hz").as_double(),
            [this]() { publish_barometer(); });

        mag_timer_ = make_timer(
            get_parameter("mag_rate_hz").as_double(),
            [this]() { publish_magnetometer(); });

        range_timer_ = make_timer(
            get_parameter("range_rate_hz").as_double(),
            [this]() { publish_rangefinder(); });

        clock_timer_ = make_timer(
            get_parameter("clock_rate_hz").as_double(),
            [this]() { publish_clock(); });

        RCLCPP_INFO(get_logger(), "All sensor publishers started");
    }

    void publish_odometry() {
        auto pose_result = client_->get_pose();
        if (!pose_result) return;

        auto& pose = *pose_result;
        auto stamp = now();

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = pose.x;
        msg.pose.pose.position.y = pose.y;
        msg.pose.pose.position.z = pose.z;
        msg.pose.pose.orientation.w = pose.qw;
        msg.pose.pose.orientation.x = pose.qx;
        msg.pose.pose.orientation.y = pose.qy;
        msg.pose.pose.orientation.z = pose.qz;
        odom_pub_->publish(msg);

        // TF broadcast
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = pose.x;
        tf.transform.translation.y = pose.y;
        tf.transform.translation.z = pose.z;
        tf.transform.rotation.w = pose.qw;
        tf.transform.rotation.x = pose.qx;
        tf.transform.rotation.y = pose.qy;
        tf.transform.rotation.z = pose.qz;
        tf_broadcaster_->sendTransform(tf);
    }

    void publish_imu() {
        auto imu_result = client_->get_imu_data();
        if (!imu_result) return;

        auto& imu = *imu_result;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = "imu_link";
        msg.angular_velocity.x = imu.angular_velocity_x;
        msg.angular_velocity.y = imu.angular_velocity_y;
        msg.angular_velocity.z = imu.angular_velocity_z;
        msg.linear_acceleration.x = imu.linear_acceleration_x;
        msg.linear_acceleration.y = imu.linear_acceleration_y;
        msg.linear_acceleration.z = imu.linear_acceleration_z;
        msg.orientation.w = imu.orientation_w;
        msg.orientation.x = imu.orientation_x;
        msg.orientation.y = imu.orientation_y;
        msg.orientation.z = imu.orientation_z;
        imu_pub_->publish(msg);
    }

    void publish_image() {
        auto camera_name = get_parameter("camera_name").as_string();
        auto img_result = client_->get_image(camera_name, 0);
        if (!img_result) return;

        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.header.frame_id = "camera_link";
        msg.encoding = "png";
        msg.data = std::move(*img_result);
        image_pub_->publish(msg);
    }

    void publish_gps() {
        auto gps_result = client_->get_gps_data();
        if (!gps_result || !gps_result->is_valid) return;

        auto& gps = *gps_result;
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now();
        msg.header.frame_id = "gps_link";
        msg.latitude = gps.latitude;
        msg.longitude = gps.longitude;
        msg.altitude = gps.altitude;
        msg.status.status = (gps.fix_type >= 3)
            ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
            : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.position_covariance[0] = gps.eph * gps.eph;
        msg.position_covariance[4] = gps.eph * gps.eph;
        msg.position_covariance[8] = gps.epv * gps.epv;
        msg.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_pub_->publish(msg);
    }

    void publish_barometer() {
        auto baro_result = client_->get_barometer_data();
        if (!baro_result) return;

        std_msgs::msg::Float64 msg;
        msg.data = baro_result->altitude;
        baro_pub_->publish(msg);
    }

    void publish_magnetometer() {
        auto mag_result = client_->get_magnetometer_data();
        if (!mag_result) return;

        auto& mag = *mag_result;
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = now();
        msg.header.frame_id = "mag_link";
        // AirSim reports in Gauss, ROS expects Tesla (1 Gauss = 1e-4 Tesla)
        msg.magnetic_field.x = mag.magnetic_field_x * 1e-4;
        msg.magnetic_field.y = mag.magnetic_field_y * 1e-4;
        msg.magnetic_field.z = mag.magnetic_field_z * 1e-4;
        mag_pub_->publish(msg);
    }

    void publish_rangefinder() {
        auto dist_result = client_->get_distance_sensor_data();
        if (!dist_result) return;

        auto& dist = *dist_result;
        sensor_msgs::msg::Range msg;
        msg.header.stamp = now();
        msg.header.frame_id = "rangefinder_link";
        msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        msg.field_of_view = 0.05f;
        msg.min_range = static_cast<float>(dist.min_distance);
        msg.max_range = static_cast<float>(dist.max_distance);
        msg.range = static_cast<float>(dist.distance);
        range_pub_->publish(msg);
    }

    void publish_clock() {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = now();
        clock_pub_->publish(msg);
    }

    std::unique_ptr<AirsimRpcClient> client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool connected_ = false;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr baro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::TimerBase::SharedPtr gps_timer_;
    rclcpp::TimerBase::SharedPtr baro_timer_;
    rclcpp::TimerBase::SharedPtr mag_timer_;
    rclcpp::TimerBase::SharedPtr range_timer_;
    rclcpp::TimerBase::SharedPtr clock_timer_;
};

} // namespace fiber_nav_colosseum

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_colosseum::ColosseumBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
