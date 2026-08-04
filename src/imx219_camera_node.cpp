#include <chrono>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class IMX219Camera : public rclcpp::Node {
  public:
    IMX219Camera() : Node("imx219_camera") {
        declare_parameter("width", 640);
        declare_parameter("height", 480);

        cv::Size frame_size(get_parameter("width").as_int(),
                            get_parameter("height").as_int());
        int framerate = 30;

        std::stringstream pipeline_ss;
        pipeline_ss << "libcamerasrc ! "
                    << "video/x-raw, format=NV12, width=" << frame_size.width
                    << ", height=" << frame_size.height
                    << ", framerate=" << framerate << "/1 ! "
                    << "videoconvert ! " << "video/x-raw, format=BGR ! "
                    << "appsink";
        RCLCPP_DEBUG_STREAM(get_logger(),
                            "GStreamer pipeline: " << pipeline_ss.str());

        capture_.open(pipeline_ss.str(), cv::CAP_GSTREAMER);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR_STREAM(get_logger(),
                                "Failed to open camera stream with GStreamer!");
            return;
        }

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / framerate),
                                   [this]() { PublishImage(); });
    }

  private:
    void PublishImage() {
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
        msg->step =
            static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        msg->data.assign(frame.data,
                         frame.data + (frame.total() * frame.elemSize()));

        image_pub_->publish(std::move(msg));
    }

    cv::VideoCapture capture_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMX219Camera>());
    rclcpp::shutdown();
    return 0;
}