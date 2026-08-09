#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace divitor_driver {

class IMX219CameraNode : public rclcpp::Node {
  public:
    explicit IMX219CameraNode(const rclcpp::NodeOptions &options =
                                  rclcpp::NodeOptions());

  private:
    void PublishImage();

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace divitor_driver