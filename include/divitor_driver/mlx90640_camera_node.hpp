#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "divitor_driver/mlx90640_camera.hpp"

class MLX90640CameraNode : public rclcpp::Node {
  public:
    MLX90640CameraNode();

    void CaptureAndPublishFrame();

  private:
    MLX90640Camera camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
