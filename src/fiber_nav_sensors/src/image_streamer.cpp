/**
 * @file image_streamer.cpp
 * @brief Streams camera images to a web server for visualization
 *
 * This node subscribes to camera images and makes them available via HTTP
 * for web-based visualization without X11.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace fiber_nav_sensors {

class ImageStreamer : public rclcpp::Node {
public:
    ImageStreamer() : Node("image_streamer") {
        // Parameters
        declare_parameter("image_topic", "/model/quadtailsitter/link/base_link/sensor/camera/image");
        declare_parameter("output_topic", "/camera/image_compressed");
        declare_parameter("quality", 80);
        declare_parameter("scale", 1.0);

        std::string image_topic = get_parameter("image_topic").as_string();
        std::string output_topic = get_parameter("output_topic").as_string();
        quality_ = get_parameter("quality").as_int();
        scale_ = get_parameter("scale").as_double();

        // Subscriber
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ImageStreamer::image_callback, this, std::placeholders::_1));

        // Publisher for compressed image (for Foxglove)
        compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic, 10);

        RCLCPP_INFO(get_logger(), "Image streamer started");
        RCLCPP_INFO(get_logger(), "  Input: %s", image_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Output: %s", output_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Quality: %d, Scale: %.2f", quality_, scale_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Scale if needed
            cv::Mat output;
            if (scale_ != 1.0) {
                cv::resize(cv_ptr->image, output, cv::Size(), scale_, scale_);
            } else {
                output = cv_ptr->image;
            }

            // Compress to JPEG
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality_};
            cv::imencode(".jpg", output, buffer, params);

            // Publish compressed image
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header = msg->header;
            compressed_msg->format = "jpeg";
            compressed_msg->data = buffer;

            compressed_pub_->publish(std::move(compressed_msg));
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

    int quality_;
    double scale_;
};

}  // namespace fiber_nav_sensors

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiber_nav_sensors::ImageStreamer>());
    rclcpp::shutdown();
    return 0;
}
