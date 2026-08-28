#ifndef DIVITOR_PERCEPTION__THERMAL_PREPROCESSOR_NODE_HPP_
#define DIVITOR_PERCEPTION__THERMAL_PREPROCESSOR_NODE_HPP_

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace divitor_perception {

class ThermalPreprocessorNode : public rclcpp::Node {
  public:
    explicit ThermalPreprocessorNode(const rclcpp::NodeOptions &options);

  private:
    void rawImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void declareAndFetchParameters();
    void replaceBadPixels(cv::Mat &frame) const;

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_preprocessed_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Processing Parameters
    int target_width_;
    int target_height_;
    int gaussian_kernel_size_;
    double gaussian_sigma_;
    bool enable_temporal_filter_;
    double ema_alpha_;
    bool enable_bad_pixel_correction_;
    double bad_pixel_threshold_; // std-dev multiplier
    int bad_pixel_neighborhood_; // median filter kernel size (odd)

    // State buffers for temporal filtering
    cv::Mat prev_frame_32f_;
    bool is_first_frame_{true};
};

} // namespace divitor_perception

#endif // DIVITOR_PERCEPTION__THERMAL_PREPROCESSOR_NODE_HPP_