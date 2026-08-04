#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <opencv2/core/check.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "yolos/yolos.hpp"

class YoloDetectorNode : public rclcpp::Node
{
  public:
    YoloDetectorNode() : Node("yolo_detector_node")
    {
        declare_parameter("model_path",
                          ament_index_cpp::get_package_share_directory("divitor_perception") + "/models/yolo26n.onnx");
        declare_parameter("labels_path",
                          ament_index_cpp::get_package_share_directory("divitor_perception") + "/models/coco.names");

        std::string model_path = get_parameter("model_path").as_string();
        std::string labels_path = get_parameter("labels_path").as_string();

        detector_ = std::make_unique<yolos::det::YOLODetector>(model_path, labels_path);

        rclcpp::SensorDataQoS sensor_qos;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image_raw", sensor_qos,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { DetectAndPublish(msg); });

        visualization_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rgb_detection/visualization", 10);
    }

    void DetectAndPublish(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        auto detections = detector_->detect(frame);
        detector_->drawDetections(frame, detections);

        auto visualization_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        visualization_pub_->publish(*visualization_msg);
    }

  private:
    std::unique_ptr<yolos::det::YOLODetector> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}