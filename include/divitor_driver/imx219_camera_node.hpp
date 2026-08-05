#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class IMX219CameraNode : public rclcpp::Node {
  public:
    IMX219CameraNode();

  private:
    void PublishImage();

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
