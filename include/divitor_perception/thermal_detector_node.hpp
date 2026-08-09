#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "divitor_perception/thermal_detector.hpp"

namespace divitor_perception {

class ThermalDetectorNode : public rclcpp::Node {
  public:
    explicit ThermalDetectorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void
    ProcessThermalFrame(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    const cv::Size UPSCALE_SIZE = cv::Size(320, 240);

    std::unique_ptr<ThermalDetector> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
};

} // namespace divitor_perception
