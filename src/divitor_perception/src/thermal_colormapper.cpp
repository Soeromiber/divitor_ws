#include "divitor_perception/thermal_colormapper.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace divitor_perception {

ThermalColormapperNode::ThermalColormapperNode(
    const rclcpp::NodeOptions &options)
    : Node("thermal_colormapper", options) {
    declareAndFetchParameters();

    // Dynamic Parameter Callback setup
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : params) {
                if (param.get_name() == "min_temp")
                    min_temp_ = param.as_double();
                else if (param.get_name() == "max_temp")
                    max_temp_ = param.as_double();
            }
            return result;
        });

    // Setup Publisher using SensorDataQoS (Best Effort, Volatile)
    pub_mapped_ = this->create_publisher<sensor_msgs::msg::Image>(
        "perception/thermal/image_colormapped", 10);

    // Setup Subscriber
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "perception/thermal/image_preprocessed", rclcpp::SensorDataQoS(),
        std::bind(&ThermalColormapperNode::imageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Thermal Colormapper Component initialized (min temp: %f, max "
                "temp: %f)",
                min_temp_, max_temp_);
}

void ThermalColormapperNode::declareAndFetchParameters() {
    min_temp_ = declare_parameter<double>("min_temp", 5.0);
    max_temp_ = declare_parameter<double>("max_temp", 45.0);
}

void ThermalColormapperNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat &raw = cv_ptr->image;
    if (raw.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty thermal image frame.");
        return;
    }

    double scale = 255.0 / (max_temp_ - min_temp_);
    double delta = -min_temp_ * scale;
    cv::Mat norm_8u;
    raw.convertTo(norm_8u, CV_8UC1, scale, delta);

    cv::Mat color_mapped;
    cv::cvtColor(norm_8u, color_mapped, cv::COLOR_GRAY2BGR);

    auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
    out_msg->header = msg->header;

    cv_bridge::CvImage cv_out(msg->header, sensor_msgs::image_encodings::BGR8,
                              color_mapped);
    cv_out.toImageMsg(*out_msg);

    pub_mapped_->publish(std::move(out_msg));
}

} // namespace divitor_perception

// Register Component with ROS 2 Package Manager
RCLCPP_COMPONENTS_REGISTER_NODE(divitor_perception::ThermalColormapperNode)
