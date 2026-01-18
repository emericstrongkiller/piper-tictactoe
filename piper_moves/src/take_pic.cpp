#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <string>

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&CameraSubscriber::listener_callback, this,
                  std::placeholders::_1));
  }

private:
  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Receiving image");

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const std::string dir = "/home/user/ros2_ws/src/helper_scripts/pics";
    const std::string save_path =
        (std::filesystem::path(dir) / "captured_image.jpg").string();

    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    if (!cv::imwrite(save_path, cv_ptr->image)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write image to %s",
                   save_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Image saved to %s", save_path.c_str());

    // Equivalent to rclpy.shutdown(): stop spinning after first image
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}