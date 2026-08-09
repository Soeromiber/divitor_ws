#include "divitor_driver/imx219_camera_node.hpp"

#include <chrono>
#include <sstream>

#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_driver {

IMX219CameraNode::IMX219CameraNode(const rclcpp::NodeOptions &options)
    : Node("imx219_camera", options) {
    declare_parameter("width", 640);
    declare_parameter("height", 480);

    cv::Size frame_size(get_parameter("width").as_int(),
                        get_parameter("height").as_int());
    int framerate = 30;

    std::stringstream pipeline_ss;
    pipeline_ss << "libcamerasrc ! "
                << "video/x-raw, format=NV12, width=" << frame_size.width
                << ", height=" << frame_size.height
                << ", framerate=" << framerate << "/1 ! " << "videoconvert ! "
                << "video/x-raw, format=BGR ! " << "appsink";
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "GStreamer pipeline: " << pipeline_ss.str());

    capture_.open(pipeline_ss.str(), cv::CAP_GSTREAMER);
    if (!capture_.isOpened()) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Failed to open camera stream with GStreamer!");
        return;
    }

    image_pub_ =
        create_publisher<sensor_msgs::msg::Image>("/rgb_camera/image_raw", 1);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / framerate),
                               [this]() { PublishImage(); });
}

void IMX219CameraNode::PublishImage() {
    cv::Mat frame;
    capture_ >> frame;
    if (frame.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Latest frame is empty!");
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = now();
    msg->header.frame_id = "imx219_camera";
    msg->height = frame.rows;
    msg->width = frame.cols;
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    msg->data.assign(frame.data,
                     frame.data + (frame.total() * frame.elemSize()));

    image_pub_->publish(std::move(msg));
}

} // namespace divitor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(divitor_driver::IMX219CameraNode)
