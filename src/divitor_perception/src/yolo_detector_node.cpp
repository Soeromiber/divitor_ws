#include "divitor_perception/yolo_detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_perception {

YoloDetectorNode::YoloDetectorNode(const rclcpp::NodeOptions &options)
    : Node("yolo_detector_node", options) {
    declare_parameter(
        "model_path",
        ament_index_cpp::get_package_share_directory("divitor_perception") +
            "/models/yolo26n.onnx");
    declare_parameter(
        "labels_path",
        ament_index_cpp::get_package_share_directory("divitor_perception") +
            "/models/coco.names");

    std::string model_path = get_parameter("model_path").as_string();
    std::string labels_path = get_parameter("labels_path").as_string();

    detector_ =
        std::make_unique<yolos::det::YOLODetector>(model_path, labels_path);

    rclcpp::SensorDataQoS sensor_qos;
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensors/camera/rgb/image_raw", sensor_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            DetectAndPublish(msg);
        });

    visualization_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "perception/rgb/visualization", 10);
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