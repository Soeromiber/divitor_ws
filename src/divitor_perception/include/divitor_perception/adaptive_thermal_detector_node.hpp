#ifndef DIVITOR_PERCEPTION__ADAPTIVE_THERMAL_DETECTOR_NODE_HPP_
#define DIVITOR_PERCEPTION__ADAPTIVE_THERMAL_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace divitor_perception {

class AdaptiveThermalDetectorNode : public rclcpp::Node {
  public:
    explicit AdaptiveThermalDetectorNode(const rclcpp::NodeOptions &options);

  private:
    void TemperatureCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void declareAndFetchParameters();

    cv::Mat applyNightThresholding(const cv::Mat &temp_frame);
    cv::Mat applyMiddayThresholding(const cv::Mat &temp_frame);

    // Subscriptions & Publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_preprocessed_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
        pub_detections_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_mask_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Configurable Parameters
    std::string detection_mode_; // "NIGHT" or "MIDDAY"
    double k_sigma_;
    int neighborhood_kernel_;
    double temp_min_;
    double temp_max_;
    double solar_reject_temp_;
    int min_blob_area_;
    int max_blob_area_;
    double min_aspect_ratio_;
    double max_aspect_ratio_;
};

} // namespace divitor_perception

#endif // DIVITOR_PERCEPTION__ADAPTIVE_THERMAL_DETECTOR_NODE_HPP_