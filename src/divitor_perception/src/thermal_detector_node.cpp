#include "divitor_perception/thermal_detector_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_perception {

ThermalDetectorNode::ThermalDetectorNode(const rclcpp::NodeOptions &options)
    : Node("thermal_detector_node", options) {
    // Declare parameters for thermal detection thresholds and filters
    declare_parameter("min_human_temp", 31.0f);
    declare_parameter("max_human_temp", 38.0f);
    declare_parameter("min_contour_area", 150.0);
    declare_parameter("max_contour_area", 12000.0);
    declare_parameter("min_aspect_ratio", 0.25);
    declare_parameter("max_aspect_ratio", 0.80);

    // Retrieve parameters
    float minTemp = get_parameter("min_human_temp").as_double();
    float maxTemp = get_parameter("max_human_temp").as_double();
    double minArea = get_parameter("min_contour_area").as_double();
    double maxArea = get_parameter("max_contour_area").as_double();
    double minAR = get_parameter("min_aspect_ratio").as_double();
    double maxAR = get_parameter("max_aspect_ratio").as_double();

    // Initialize the ThermalDetector with the retrieved parameters
    detector_ = std::make_unique<ThermalDetector>(minTemp, maxTemp, minArea,
                                                  maxArea, minAR, maxAR);

    rclcpp::SensorDataQoS qos;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "sensors/camera/thermal/image_raw", qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            ProcessThermalFrame(msg);
        });

    visualization_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/thermal/visualization", 10);
}

void ThermalDetectorNode::ProcessThermalFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    sensor_msgs::msg::Image::UniquePtr vis_msg =
        std::make_unique<sensor_msgs::msg::Image>();
    vis_msg->header = msg->header;
    vis_msg->height = UPSCALE_SIZE.height;
    vis_msg->width = UPSCALE_SIZE.width;
    vis_msg->encoding = "bgr8";
    vis_msg->is_bigendian = false;
    vis_msg->step = vis_msg->width * 3;
    vis_msg->data.resize(vis_msg->height * vis_msg->step);

    cv::Mat visualization(UPSCALE_SIZE.height, UPSCALE_SIZE.width, CV_8UC3,
                          vis_msg->data.data());
    cv::Size src_size(msg->width, msg->height);
    std::vector<HumanDetection> detections =
        detector_->Detect(reinterpret_cast<const float *>(msg->data.data()),
                          src_size, UPSCALE_SIZE, visualization);

    visualization_pub_->publish(std::move(vis_msg));
}

} // namespace divitor_perception

RCLCPP_COMPONENTS_REGISTER_NODE(divitor_perception::ThermalDetectorNode)