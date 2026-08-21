#include "divitor_perception/yolo_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_perception {

YoloDetectorNode::YoloDetectorNode(const rclcpp::NodeOptions &options)
    : Node("yolo_detector_node", options) {
    declare_parameter("model_path", "yolo26n.onnx");
    declare_parameter("labels_path", "coco.names");

    std::string model_path = get_parameter("model_path").as_string();
    std::string labels_path = get_parameter("labels_path").as_string();

    RCLCPP_INFO(get_logger(), "Loading YOLO model from: %s",
                model_path.c_str());
    RCLCPP_INFO(get_logger(), "Loading labels from: %s", labels_path.c_str());
    detector_ =
        std::make_unique<yolos::det::YOLO26Detector>(model_path, labels_path);

    rclcpp::SensorDataQoS sensor_qos;
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensors/camera/rgb/image_raw", sensor_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            DetectAndPublish(msg);
        });

    visualization_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "perception/rgb/visualization", 10);

    RCLCPP_INFO(get_logger(), "YOLO Detector Node initialized successfully.");
}

void YoloDetectorNode::DetectAndPublish(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    auto detections = detector_->detect(frame);
    detector_->drawDetections(frame, detections);

    auto visualization_msg =
        cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    visualization_pub_->publish(*visualization_msg);
}

} // namespace divitor_perception

RCLCPP_COMPONENTS_REGISTER_NODE(divitor_perception::YoloDetectorNode)