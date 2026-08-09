#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "yolos/yolos.hpp"

namespace divitor_perception {

class YoloDetectorNode : public rclcpp::Node {
  public:
    explicit YoloDetectorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void DetectAndPublish(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    std::unique_ptr<yolos::det::YOLODetector> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
};

} // namespace divitor_perception
