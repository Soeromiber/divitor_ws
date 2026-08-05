#include "divitor_driver/mlx90640_camera_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MLX90640CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
