#include <fiber_nav_sensors/optical_flow_direction.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <random>

namespace fiber_nav_sensors {

class OpticalFlowDirection : public rclcpp::Node {
public:
    OpticalFlowDirection() : Node("optical_flow_direction") {
        // Parameters
        declare_parameter("model_name", "quadtailsitter");
        declare_parameter("max_features", 200);
        declare_parameter("min_quality", 0.3);
        declare_parameter("process_scale", 0.5);
        declare_parameter("min_displacement", 0.5);
        declare_parameter("fallback_drift_rate", 0.001);
        declare_parameter("publish_rate", 15.0);
        declare_parameter("redetect_interval", 10);

        model_name_ = get_parameter("model_name").as_string();
        max_features_ = get_parameter("max_features").as_int();
        min_quality_ = get_parameter("min_quality").as_double();
        process_scale_ = get_parameter("process_scale").as_double();
        min_displacement_ = static_cast<float>(get_parameter("min_displacement").as_double());
        fallback_drift_rate_ = get_parameter("fallback_drift_rate").as_double();
        publish_rate_ = get_parameter("publish_rate").as_double();
        redetect_interval_ = get_parameter("redetect_interval").as_int();

        // Publishers
        direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/vision_direction", 10);
        quality_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/optical_flow/quality", 10);
        of_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/sensors/optical_flow/direction", 10);
        source_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/sensors/optical_flow/source", 10);

        // Subscribers
        auto sensor_qos = rclcpp::SensorDataQoS();
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera_down", sensor_qos,
            std::bind(&OpticalFlowDirection::image_callback, this, std::placeholders::_1));

        std::string odom_topic = "/model/" + model_name_ + "/odometry";
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OpticalFlowDirection::odom_callback, this, std::placeholders::_1));

        // Random number generator for fallback drift
        std::random_device rd;
        rng_ = std::mt19937(rd());
        drift_dist_ = std::normal_distribution<double>(0.0, fallback_drift_rate_);

        min_publish_interval_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        RCLCPP_INFO(get_logger(), "Optical flow direction initialized");
        RCLCPP_INFO(get_logger(), "  Model: %s", model_name_.c_str());
        RCLCPP_INFO(get_logger(), "  Max features: %d, process scale: %.1f",
                     max_features_, process_scale_);
        RCLCPP_INFO(get_logger(), "  Min quality: %.2f, fallback drift: %.4f rad/s",
                     min_quality_, fallback_drift_rate_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = msg;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Rate limit
        auto now_time = now();
        if (last_publish_time_.nanoseconds() > 0
            && (now_time - last_publish_time_) < min_publish_interval_) {
            return;
        }

        // Convert to grayscale
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "cv_bridge error: %s", e.what());
            return;
        }

        // Downsample
        cv::Mat gray;
        if (std::abs(process_scale_ - 1.0) > 0.01) {
            cv::resize(cv_ptr->image, gray, cv::Size(), process_scale_, process_scale_,
                        cv::INTER_AREA);
        } else {
            gray = cv_ptr->image;
        }

        // First frame: detect features, return
        if (prev_gray_.empty()) {
            prev_gray_ = gray.clone();
            detect_features(prev_gray_, prev_pts_);
            RCLCPP_INFO(get_logger(), "First frame: detected %zu features", prev_pts_.size());
            return;
        }

        // Track features
        if (prev_pts_.empty()) {
            detect_features(prev_gray_, prev_pts_);
            prev_gray_ = gray.clone();
            return;
        }

        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, curr_pts, status, err);

        // Compute dominant flow
        auto flow = compute_dominant_flow(prev_pts_, curr_pts, status, min_displacement_);

        // Publish quality
        auto quality_msg = std_msgs::msg::Float64();
        quality_msg.data = flow.quality;
        quality_pub_->publish(quality_msg);

        // Publish raw OF direction (before fallback decision)
        if (flow.magnitude >= min_displacement_ && flow.quality > 0.0f) {
            auto body_dir = flow_to_body_direction(flow.dir_x, flow.dir_y);
            auto of_msg = geometry_msgs::msg::Vector3Stamped();
            of_msg.header.stamp = now_time;
            of_msg.header.frame_id = "base_link";
            of_msg.vector.x = body_dir[0];
            of_msg.vector.y = body_dir[1];
            of_msg.vector.z = body_dir[2];
            of_direction_pub_->publish(of_msg);
        }

        // Decide: optical flow or fallback
        bool use_optical_flow = flow.quality >= min_quality_
                             && flow.magnitude >= min_displacement_;

        auto source_msg = std_msgs::msg::Float64();
        source_msg.data = use_optical_flow ? 1.0 : 0.0;
        source_pub_->publish(source_msg);

        if (use_optical_flow) {
            auto body_dir = flow_to_body_direction(flow.dir_x, flow.dir_y);
            auto dir_msg = geometry_msgs::msg::Vector3Stamped();
            dir_msg.header.stamp = now_time;
            dir_msg.header.frame_id = "base_link";
            dir_msg.vector.x = body_dir[0];
            dir_msg.vector.y = body_dir[1];
            dir_msg.vector.z = body_dir[2];
            direction_pub_->publish(dir_msg);
        } else {
            publish_fallback_direction(now_time);
        }

        last_publish_time_ = now_time;

        // Update previous frame
        prev_gray_ = gray.clone();

        // Re-detect features periodically or when too few remain
        frame_count_++;
        size_t tracked_count = 0;
        for (auto s : status) {
            if (s) tracked_count++;
        }

        if (frame_count_ % redetect_interval_ == 0
            || tracked_count < static_cast<size_t>(max_features_) / 3) {
            detect_features(prev_gray_, prev_pts_);
        } else {
            // Keep only successfully tracked points
            std::vector<cv::Point2f> good_pts;
            good_pts.reserve(tracked_count);
            for (size_t i = 0; i < curr_pts.size(); ++i) {
                if (status[i]) {
                    good_pts.push_back(curr_pts[i]);
                }
            }
            prev_pts_ = std::move(good_pts);
        }
    }

    void detect_features(const cv::Mat& gray, std::vector<cv::Point2f>& pts) {
        pts.clear();
        cv::goodFeaturesToTrack(gray, pts, max_features_, 0.01, 10);
    }

    void publish_fallback_direction(const rclcpp::Time& stamp) {
        if (!last_odom_) return;

        const auto& twist = last_odom_->twist.twist;
        double vx = twist.linear.x;
        double vy = twist.linear.y;
        double vz = twist.linear.z;
        double speed = std::hypot(vx, vy, vz);

        if (speed < 0.5) return;  // Direction undefined at low speed

        // Normalize
        double ux = vx / speed;
        double uy = vy / speed;
        double uz = vz / speed;

        // Apply random walk drift (same algorithm as vision_direction_sim)
        double dt = 1.0 / publish_rate_;
        drift_yaw_ += drift_dist_(rng_) * std::sqrt(dt);
        drift_pitch_ += drift_dist_(rng_) * std::sqrt(dt);

        double cos_yaw = std::cos(drift_yaw_);
        double sin_yaw = std::sin(drift_yaw_);
        double cos_pitch = std::cos(drift_pitch_);
        double sin_pitch = std::sin(drift_pitch_);

        double ux_yaw = cos_yaw * ux - sin_yaw * uy;
        double uy_yaw = sin_yaw * ux + cos_yaw * uy;
        double uz_yaw = uz;

        double ux_final = cos_pitch * ux_yaw + sin_pitch * uz_yaw;
        double uy_final = uy_yaw;
        double uz_final = -sin_pitch * ux_yaw + cos_pitch * uz_yaw;

        double norm = std::hypot(ux_final, uy_final, uz_final);
        ux_final /= norm;
        uy_final /= norm;
        uz_final /= norm;

        auto msg = geometry_msgs::msg::Vector3Stamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";
        msg.vector.x = ux_final;
        msg.vector.y = uy_final;
        msg.vector.z = uz_final;
        direction_pub_->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quality_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr of_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr source_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Parameters
    std::string model_name_;
    int max_features_ = 200;
    double min_quality_ = 0.3;
    double process_scale_ = 0.5;
    float min_displacement_ = 0.5f;
    double fallback_drift_rate_ = 0.001;
    double publish_rate_ = 15.0;
    int redetect_interval_ = 10;

    // Optical flow state
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int frame_count_ = 0;

    // Rate limiting
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Duration min_publish_interval_{0, 0};

    // Fallback state (odometry + drift)
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    double drift_yaw_ = 0.0;
    double drift_pitch_ = 0.0;
    std::mt19937 rng_;
    std::normal_distribution<double> drift_dist_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::OpticalFlowDirection>());
    rclcpp::shutdown();
    return 0;
}
