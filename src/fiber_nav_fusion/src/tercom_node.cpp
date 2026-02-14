#include <fiber_nav_fusion/tercom.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <optional>
#include <cmath>
#include <sstream>
#include <deque>


namespace fiber_nav_fusion {

class TercomNode : public rclcpp::Node {
public:
    TercomNode() : Node("tercom_node") {
        declare_parameter("terrain_data_path", "");
        declare_parameter("min_samples", 10);
        declare_parameter("sample_spacing", 12.0);
        declare_parameter("search_radius", 300.0);
        declare_parameter("search_step", 12.0);
        declare_parameter("min_ncc", 0.5);
        declare_parameter("par_threshold", 1.5);
        declare_parameter("par_sigma_scale", 50.0);
        declare_parameter("max_buffer_size", 100);

        config_.min_samples = get_parameter("min_samples").as_int();
        config_.sample_spacing = static_cast<float>(get_parameter("sample_spacing").as_double());
        config_.search_radius = static_cast<float>(get_parameter("search_radius").as_double());
        config_.search_step = static_cast<float>(get_parameter("search_step").as_double());
        config_.min_ncc = static_cast<float>(get_parameter("min_ncc").as_double());
        config_.par_threshold = static_cast<float>(get_parameter("par_threshold").as_double());
        config_.par_sigma_scale = static_cast<float>(get_parameter("par_sigma_scale").as_double());
        max_buffer_size_ = get_parameter("max_buffer_size").as_int();

        // Load DEM
        if (!loadTerrain()) {
            RCLCPP_ERROR(get_logger(), "Failed to load terrain DEM — TERCOM disabled");
            return;
        }

        // Publishers
        position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/tercom/position", 10);
        diag_pub_ = create_publisher<std_msgs::msg::String>(
            "/tercom/diagnostics", 10);
        quality_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/tercom/quality", 10);

        // Subscribers
        declare_parameter("laser_topic", "/laser_rangefinder");
        auto laser_topic = get_parameter("laser_topic").as_string();
        RCLCPP_INFO(get_logger(), "Subscribing to laser: %s", laser_topic.c_str());
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic, 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!msg->ranges.empty() && std::isfinite(msg->ranges[0]) &&
                    msg->ranges[0] >= msg->range_min && msg->ranges[0] <= msg->range_max) {
                    last_agl_ = msg->ranges[0];
                    agl_time_ = now();
                }
            });

        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();

        lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                baro_alt_ = -msg->z;  // PX4 z is down, baro_alt is up
                lpos_time_ = now();
            });

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4_qos,
            [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                att_q_.setW(msg->q[0]);
                att_q_.setX(msg->q[1]);
                att_q_.setY(msg->q[2]);
                att_q_.setZ(msg->q[3]);
                has_attitude_ = true;
            });

        ekf_state_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/position_ekf/state", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                ekf_x_ = static_cast<float>(msg->point.x);
                ekf_y_ = static_cast<float>(msg->point.y);
                has_ekf_state_ = true;
            });

        spool_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/sensors/fiber_spool/velocity", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                spool_speed_ = msg->data;
            });

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10,
            [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                dir_x_ = static_cast<float>(msg->vector.x);
                dir_y_ = static_cast<float>(msg->vector.y);
                dir_z_ = static_cast<float>(msg->vector.z);
            });

        // Timer: process at 10 Hz (matches laser rate)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TercomNode::update, this));

        RCLCPP_INFO(get_logger(),
            "TERCOM node started (DEM %dx%d, %.1f m/px, search_r=%.0fm, min_samples=%d)",
            terrain_map_.width, terrain_map_.height,
            terrain_map_.meters_per_pixel, config_.search_radius, config_.min_samples);
    }

private:
    bool loadTerrain() {
        auto path = get_parameter("terrain_data_path").as_string();
        auto logger = get_logger();
        terrain_map_ = load_terrain_map(path, [&](const std::string& msg) {
            RCLCPP_INFO(logger, "%s", msg.c_str());
        });
        if (terrain_map_.width <= 0) {
            RCLCPP_ERROR(logger, "Failed to load terrain DEM");
            return false;
        }
        return true;
    }

    void update() {
        std::lock_guard<std::mutex> lock(mutex_);

        auto current_time = now();

        // Check data freshness
        bool agl_fresh = agl_time_.has_value() &&
            (current_time - *agl_time_).seconds() < 0.5;
        bool lpos_fresh = lpos_time_.has_value() &&
            (current_time - *lpos_time_).seconds() < 0.5;

        if (!agl_fresh || !lpos_fresh || !has_attitude_) return;

        // Compute displacement since last sample
        float dt = last_sample_time_.has_value()
            ? static_cast<float>((current_time - *last_sample_time_).seconds())
            : 0.1f;
        dt = std::clamp(dt, 0.001f, 1.0f);

        // Body-frame velocity from spool speed + direction
        float vx_body = spool_speed_ * dir_x_;
        float vy_body = spool_speed_ * dir_y_;
        float vz_body = spool_speed_ * dir_z_;

        // Rotate body (FLU) → NED via PX4 attitude (FRD→NED)
        // FLU→FRD: negate Y and Z
        tf2::Vector3 v_body_frd(vx_body, -vy_body, -vz_body);
        tf2::Vector3 v_ned = tf2::quatRotate(att_q_, v_body_frd);

        float dx = static_cast<float>(v_ned.x()) * dt;
        float dy = static_cast<float>(v_ned.y()) * dt;

        accum_dx_ += dx;
        accum_dy_ += dy;
        float accum_dist = std::sqrt(accum_dx_ * accum_dx_ + accum_dy_ * accum_dy_);

        // Accumulate until we've traveled sample_spacing distance
        if (accum_dist < config_.sample_spacing) return;

        // Push new sample
        TerrainSample sample{
            .agl = last_agl_,
            .baro_alt = baro_alt_,
            .dx = accum_dx_,
            .dy = accum_dy_
        };
        samples_.push_back(sample);
        accum_dx_ = 0.f;
        accum_dy_ = 0.f;
        last_sample_time_ = current_time;

        // Trim buffer
        while (static_cast<int>(samples_.size()) > max_buffer_size_) {
            samples_.pop_front();
        }

        // Run TERCOM match when we have enough samples
        if (static_cast<int>(samples_.size()) < config_.min_samples) return;

        // Use EKF state as search center, fallback to (0,0)
        float cx = has_ekf_state_ ? ekf_x_ : 0.f;
        float cy = has_ekf_state_ ? ekf_y_ : 0.f;

        // Convert deque to contiguous vector for span
        std::vector<TerrainSample> samples_vec(samples_.begin(), samples_.end());

        auto result = tercom_match(terrain_map_, samples_vec, cx, cy, config_);

        // Publish diagnostics always
        publishDiagnostics(result, static_cast<int>(samples_vec.size()));

        // Publish quality for plotting
        auto q_msg = std_msgs::msg::Float64();
        q_msg.data = result.par;
        quality_pub_->publish(q_msg);

        // Only publish position fix if valid
        if (result.valid) {
            auto pos_msg = geometry_msgs::msg::PointStamped();
            pos_msg.header.stamp = current_time;
            pos_msg.header.frame_id = "odom";
            pos_msg.point.x = result.x;
            pos_msg.point.y = result.y;
            pos_msg.point.z = result.sigma;  // encode sigma in z for consumer
            position_pub_->publish(pos_msg);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "TERCOM fix: (%.1f, %.1f) NCC=%.3f PAR=%.2f sigma=%.1fm",
                result.x, result.y, result.ncc, result.par, result.sigma);
        }
    }

    void publishDiagnostics(const TercomResult& result, int n_samples) {
        std::ostringstream ss;
        ss << "{"
           << "\"valid\":" << (result.valid ? "true" : "false")
           << ",\"ncc\":" << result.ncc
           << ",\"par\":" << result.par
           << ",\"sigma\":" << result.sigma
           << ",\"x\":" << result.x
           << ",\"y\":" << result.y
           << ",\"n_samples\":" << n_samples
           << "}";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        diag_pub_->publish(msg);
    }

    // Terrain
    TerrainMap terrain_map_;
    TercomConfig config_;
    int max_buffer_size_{100};

    // Sample buffer
    std::deque<TerrainSample> samples_;
    float accum_dx_{0.f};
    float accum_dy_{0.f};
    std::optional<rclcpp::Time> last_sample_time_;

    // Sensor data (protected by mutex)
    std::mutex mutex_;
    float last_agl_{0.f};
    float baro_alt_{0.f};
    float spool_speed_{0.f};
    float dir_x_{1.f}, dir_y_{0.f}, dir_z_{0.f};
    tf2::Quaternion att_q_;
    bool has_attitude_{false};
    float ekf_x_{0.f}, ekf_y_{0.f};
    bool has_ekf_state_{false};

    std::optional<rclcpp::Time> agl_time_;
    std::optional<rclcpp::Time> lpos_time_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quality_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ekf_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr spool_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fiber_nav_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_fusion::TercomNode>());
    rclcpp::shutdown();
    return 0;
}
