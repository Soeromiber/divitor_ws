#define DEBUG

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define MLX_I2C_ADDR 0x33

class MLX90640Camera {
  public:
    const int WIDTH = 32;
    const int HEIGHT = 24;

    bool Open(uint8_t i2c_address, uint8_t refresh_rate,
              float emissivity = 0.95f, float reflected_temperature = 23.0f) {
        i2c_address_ = i2c_address;
        emissivity_ = emissivity;
        reflected_temperature_ = reflected_temperature;

        MLX90640_I2CInit();

        std::cout << "Reading sensor EEPROM calibration data..." << std::endl;
        if (MLX90640_DumpEE(i2c_address_, eeprom_.data()) != 0) {
            return false;
        }

        MLX90640_ExtractParameters(eeprom_.data(), &params_);

        if (MLX90640_SetRefreshRate(i2c_address_, refresh_rate) != 0) {
            return false;
        }

        opened_ = true;
        return true;
    }

    bool ReadFrame(float *temperatures) {
        if (MLX90640_GetFrameData(i2c_address_, frame_.data()) < 0) {
            return false;
        }

        MLX90640_CalculateTo(frame_.data(), &params_, emissivity_,
                             reflected_temperature_, temperatures);
        return true;
    }

    bool opened() const { return opened_; }

  private:
    bool opened_ = false;
    uint8_t i2c_address_;
    float emissivity_;
    float reflected_temperature_;
    std::array<uint16_t, 832> eeprom_{};
    std::array<uint16_t, 834> frame_{};
    paramsMLX90640 params_{};
};

class MLX90640CameraNode : public rclcpp::Node {
  public:
    MLX90640CameraNode() : Node("mlx90640_camera") {
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

        image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("thermal_camera/image_raw", 1);

        int true_refresh_rate =
            1 << refresh_rate; // Convert power-of-2 to actual Hz
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / true_refresh_rate),
            [this]() { CaptureAndPublishFrame(); });

        RCLCPP_INFO(this->get_logger(),
                    "MLX90640 Camera Node Initialized at %d Hz",
                    true_refresh_rate);
    }

    void CaptureAndPublishFrame() {
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

  private:
    MLX90640Camera camera_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MLX90640CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}