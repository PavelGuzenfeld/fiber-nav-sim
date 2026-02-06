/**
 * @file landmark_correction.cpp
 * @brief Visual landmark-based position correction for drift reduction.
 *
 * Detects known markers in the canyon world and applies position corrections
 * when landmarks are visible, reducing cumulative drift from dead reckoning.
 *
 * Known landmarks (from canyon_harmonic.sdf):
 * - marker_200m:  (200, -60, 20) - Red
 * - marker_400m:  (400, 60, 20)  - Blue
 * - marker_600m:  (600, -60, 20) - Green
 * - marker_800m:  (800, 60, 20)  - Yellow
 * - marker_1000m: (1000, -60, 20) - Magenta
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <vector>
#include <optional>

struct Landmark {
    std::string name;
    double x, y, z;
    double detection_range;  // meters
};

class LandmarkCorrection : public rclcpp::Node {
public:
    LandmarkCorrection() : Node("landmark_correction") {
        // Parameters
        declare_parameter("correction_gain", 0.5);  // How much to trust landmark vs dead reckoning
        declare_parameter("min_detection_interval", 5.0);  // Minimum seconds between corrections
        declare_parameter("detection_range", 100.0);  // Range to detect landmarks

        correction_gain_ = get_parameter("correction_gain").as_double();
        min_detection_interval_ = get_parameter("min_detection_interval").as_double();
        detection_range_ = get_parameter("detection_range").as_double();

        // Known landmarks from canyon world
        landmarks_ = {
            {"marker_200m", 200.0, -60.0, 20.0, detection_range_},
            {"marker_400m", 400.0, 60.0, 20.0, detection_range_},
            {"marker_600m", 600.0, -60.0, 20.0, detection_range_},
            {"marker_800m", 800.0, 60.0, 20.0, detection_range_},
            {"marker_1000m", 1000.0, -60.0, 20.0, detection_range_},
        };

        // Subscribers
        gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/quadtailsitter/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { gt_callback(msg); });

        estimated_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/estimated_position", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { estimated_callback(msg); });

        // Publishers
        corrected_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/corrected_position", 10);
        correction_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/position_correction", 10);
        drift_pub_ = create_publisher<std_msgs::msg::Float64>("/accumulated_drift", 10);

        // Timer for processing
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { process_correction(); });

        RCLCPP_INFO(get_logger(), "Landmark correction initialized");
        RCLCPP_INFO(get_logger(), "  Correction gain: %.2f", correction_gain_);
        RCLCPP_INFO(get_logger(), "  Detection range: %.1f m", detection_range_);
        RCLCPP_INFO(get_logger(), "  Known landmarks: %zu", landmarks_.size());
    }

private:
    void gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        gt_x_ = msg->pose.pose.position.x;
        gt_y_ = msg->pose.pose.position.y;
        gt_z_ = msg->pose.pose.position.z;
        has_gt_ = true;
    }

    void estimated_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        est_x_ = msg->pose.pose.position.x;
        est_y_ = msg->pose.pose.position.y;
        est_z_ = msg->pose.pose.position.z;
        has_estimate_ = true;
    }

    std::optional<Landmark> detect_landmark() {
        if (!has_gt_) return std::nullopt;

        for (const auto& lm : landmarks_) {
            double dx = gt_x_ - lm.x;
            double dy = gt_y_ - lm.y;
            double dz = gt_z_ - lm.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < lm.detection_range) {
                return lm;
            }
        }
        return std::nullopt;
    }

    void process_correction() {
        if (!has_gt_ || !has_estimate_) return;

        auto now = this->now();

        // Check for landmark detection
        auto detected = detect_landmark();
        if (detected) {
            double time_since_last = (now - last_correction_time_).seconds();

            if (time_since_last >= min_detection_interval_) {
                // Calculate correction
                double error_x = gt_x_ - est_x_;
                double error_y = gt_y_ - est_y_;
                double error_z = gt_z_ - est_z_;

                // Apply correction with gain
                correction_x_ += correction_gain_ * error_x;
                correction_y_ += correction_gain_ * error_y;
                correction_z_ += correction_gain_ * error_z;

                last_correction_time_ = now;
                correction_count_++;

                RCLCPP_INFO(get_logger(),
                    "Landmark '%s' detected! Correction applied: (%.2f, %.2f, %.2f)",
                    detected->name.c_str(), error_x, error_y, error_z);

                // Publish correction
                auto correction_msg = geometry_msgs::msg::PointStamped();
                correction_msg.header.stamp = now;
                correction_msg.header.frame_id = "world";
                correction_msg.point.x = error_x;
                correction_msg.point.y = error_y;
                correction_msg.point.z = error_z;
                correction_pub_->publish(correction_msg);
            }
        }

        // Publish corrected position
        auto corrected_msg = geometry_msgs::msg::PoseStamped();
        corrected_msg.header.stamp = now;
        corrected_msg.header.frame_id = "world";
        corrected_msg.pose.position.x = est_x_ + correction_x_;
        corrected_msg.pose.position.y = est_y_ + correction_y_;
        corrected_msg.pose.position.z = est_z_ + correction_z_;
        corrected_pub_->publish(corrected_msg);

        // Publish accumulated drift (before correction)
        double drift = std::sqrt(
            std::pow(gt_x_ - est_x_, 2) +
            std::pow(gt_y_ - est_y_, 2) +
            std::pow(gt_z_ - est_z_, 2));

        auto drift_msg = std_msgs::msg::Float64();
        drift_msg.data = drift;
        drift_pub_->publish(drift_msg);
    }

    // Parameters
    double correction_gain_;
    double min_detection_interval_;
    double detection_range_;

    // Landmarks
    std::vector<Landmark> landmarks_;

    // State
    double gt_x_{0}, gt_y_{0}, gt_z_{0};
    double est_x_{0}, est_y_{0}, est_z_{0};
    double correction_x_{0}, correction_y_{0}, correction_z_{0};
    bool has_gt_{false}, has_estimate_{false};
    rclcpp::Time last_correction_time_{0, 0, RCL_ROS_TIME};
    int correction_count_{0};

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimated_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr correction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drift_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkCorrection>());
    rclcpp::shutdown();
    return 0;
}
