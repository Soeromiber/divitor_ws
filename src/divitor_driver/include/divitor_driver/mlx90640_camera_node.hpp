#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "divitor_driver/mlx90640_camera.hpp"

namespace divitor_driver {

class MLX90640CameraNode : public rclcpp::Node {
  public:
    explicit MLX90640CameraNode(const rclcpp::NodeOptions &options =
                                    rclcpp::NodeOptions());

    void CaptureAndPublishFrame();

  private:
    MLX90640Camera camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace divitor_driver
