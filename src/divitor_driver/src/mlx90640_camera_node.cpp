#include "divitor_driver/mlx90640_camera_node.hpp"

#include <chrono>

#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_driver {

MLX90640CameraNode::MLX90640CameraNode(const rclcpp::NodeOptions &options)
    : Node("mlx90640_camera", options) {
    declare_parameter("i2c_address", MLX_I2C_ADDR);
    declare_parameter("power_of_2_refresh_rate", 0x05);

    int i2c_address = get_parameter("i2c_address").as_int();
    int refresh_rate = get_parameter("power_of_2_refresh_rate").as_int();

    if (!camera_.Open(i2c_address, refresh_rate)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open MLX90640 camera. Please check the "
                     "connection and ensure the I2C address is correct.");
        return;
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "thermal_camera/image_raw", 1);

    int true_refresh_rate = 1 << refresh_rate;
    timer_ =
        create_wall_timer(std::chrono::milliseconds(1000 / true_refresh_rate),
                          [this]() { CaptureAndPublishFrame(); });

    RCLCPP_INFO(this->get_logger(), "MLX90640 Camera Node Initialized at %d Hz",
                true_refresh_rate);
}

void MLX90640CameraNode::CaptureAndPublishFrame() {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "mlx90640_camera";

    cv_bridge::CvImage cv_image(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_32FC1,
        cv::Mat(camera_.HEIGHT, camera_.WIDTH, CV_32FC1));

    if (!camera_.ReadFrame(cv_image.image.ptr<float>())) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to read thermal frame from MLX90640 camera.");
        return;
    }

    image_pub_->publish(*cv_image.toImageMsg());
}

} // namespace divitor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(divitor_driver::MLX90640CameraNode)
