#include "divitor_perception/thermal_preprocessor_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_perception {

ThermalPreprocessorNode::ThermalPreprocessorNode(
    const rclcpp::NodeOptions &options)
    : Node("thermal_preprocessor", options) {
    declareAndFetchParameters();

    // Dynamic Parameter Callback setup
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : params) {
                if (param.get_name() == "target_width")
                    target_width_ = param.as_int();
                else if (param.get_name() == "target_height")
                    target_height_ = param.as_int();
                else if (param.get_name() == "gaussian_kernel_size")
                    gaussian_kernel_size_ = param.as_int();
                else if (param.get_name() == "gaussian_sigma")
                    gaussian_sigma_ = param.as_double();
                else if (param.get_name() == "enable_temporal_filter")
                    enable_temporal_filter_ = param.as_bool();
                else if (param.get_name() == "ema_alpha")
                    ema_alpha_ = param.as_double();
                else if (param.get_name() == "enable_bad_pixel_correction")
                    enable_bad_pixel_correction_ = param.as_bool();
                else if (param.get_name() == "bad_pixel_threshold")
                    bad_pixel_threshold_ = param.as_double();
                else if (param.get_name() == "bad_pixel_neighborhood") {
                    int k = static_cast<int>(param.as_int());
                    bad_pixel_neighborhood_ = (k % 2 == 0) ? k + 1 : k;
                }
            }
            return result;
        });

    // Setup Publisher using SensorDataQoS (Best Effort, Volatile)
    pub_preprocessed_ = this->create_publisher<sensor_msgs::msg::Image>(
        "perception/thermal/image_preprocessed", 10);

    // Setup Subscriber
    sub_raw_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensors/camera/thermal/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ThermalPreprocessorNode::rawImageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Thermal Preprocessor Component initialized (%dx%d output).",
                target_width_, target_height_);
}

void ThermalPreprocessorNode::declareAndFetchParameters() {
    target_width_ = declare_parameter<int>("target_width", 128);
    target_height_ = declare_parameter<int>("target_height", 96);
    gaussian_kernel_size_ = declare_parameter<int>("gaussian_kernel_size", 5);
    gaussian_sigma_ = declare_parameter<double>("gaussian_sigma", 1.0);
    enable_temporal_filter_ =
        declare_parameter<bool>("enable_temporal_filter", true);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.6);
    enable_bad_pixel_correction_ =
        declare_parameter<bool>("enable_bad_pixel_correction", true);
    // Pixels deviating more than this multiple of the global std-dev from their
    // local median are considered bad and replaced by that median.
    bad_pixel_threshold_ =
        declare_parameter<double>("bad_pixel_threshold", 3.0);
    // Neighborhood kernel for local median (must be odd, >= 3)
    int k = declare_parameter<int>("bad_pixel_neighborhood", 3);
    bad_pixel_neighborhood_ = (k % 2 == 0) ? k + 1 : k;
}

void ThermalPreprocessorNode::rawImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // 1. Convert ROS image message to cv::Mat pointer (32FC1 expected)
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat &current_raw = cv_ptr->image;
    if (current_raw.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty thermal image frame.");
        return;
    }

    // 2. Bad Pixel Detection & Replacement (at native sensor resolution)
    cv::Mat corrected;
    current_raw.copyTo(corrected);
    if (enable_bad_pixel_correction_) {
        replaceBadPixels(corrected);
    }

    // 3. Temporal Exponential Moving Average (EMA) Filtering (at native
    // resolution)
    cv::Mat temporally_filtered;
    if (enable_temporal_filter_) {
        if (is_first_frame_ || prev_frame_32f_.size() != corrected.size()) {
            corrected.copyTo(prev_frame_32f_);
            temporally_filtered = corrected;
            is_first_frame_ = false;
        } else {
            // F_t = alpha * I_t + (1 - alpha) * F_{t-1}
            cv::addWeighted(corrected, ema_alpha_, prev_frame_32f_,
                            1.0 - ema_alpha_, 0.0, temporally_filtered);
            temporally_filtered.copyTo(prev_frame_32f_);
        }
    } else {
        temporally_filtered = corrected;
    }

    // 3. Spatial Upsampling using Bicubic Interpolation
    cv::Mat upsampled;
    cv::resize(temporally_filtered, upsampled,
               cv::Size(target_width_, target_height_), 0.0, 0.0,
               cv::INTER_CUBIC);

    // 4. Spatial Noise Reduction (Gaussian Blur)
    cv::Mat processed_frame;
    if (gaussian_kernel_size_ > 1 && (gaussian_kernel_size_ % 2 != 0)) {
        cv::GaussianBlur(upsampled, processed_frame,
                         cv::Size(gaussian_kernel_size_, gaussian_kernel_size_),
                         gaussian_sigma_, gaussian_sigma_);
    } else {
        processed_frame = upsampled;
    }

    // 5. Construct output ROS Message using std::unique_ptr for Zero-Copy
    // Intra-Process Transport
    auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
    out_msg->header = msg->header; // Preserve timing and frame_id

    cv_bridge::CvImage cv_out(
        msg->header, sensor_msgs::image_encodings::TYPE_32FC1, processed_frame);
    cv_out.toImageMsg(*out_msg);

    // Transfer ownership directly to intra-process buffer
    pub_preprocessed_->publish(std::move(out_msg));
}

// Bad pixel correction: replace pixels whose deviation from their local median
// exceeds `bad_pixel_threshold_` × the frame's global standard deviation.
void ThermalPreprocessorNode::replaceBadPixels(cv::Mat &frame) const {
    // --- Compute local median via medianBlur ---
    // cv::medianBlur supports 32FC1 for ksize 3 or 5.
    cv::Mat median_frame;
    cv::medianBlur(frame, median_frame, bad_pixel_neighborhood_);

    // --- Compute global std-dev for adaptive threshold ---
    cv::Scalar mean_val, stddev_val;
    cv::meanStdDev(frame, mean_val, stddev_val);
    const float threshold = static_cast<float>(bad_pixel_threshold_) *
                            static_cast<float>(stddev_val[0]);

    // --- Identify bad pixels and replace with local median ---
    cv::Mat diff;
    cv::absdiff(frame, median_frame, diff);
    cv::Mat bad_pixel_mask;
    cv::threshold(diff, bad_pixel_mask, threshold, 1.0f, cv::THRESH_BINARY);
    // Where mask == 1, copy median value; leave good pixels unchanged.
    median_frame.copyTo(frame, bad_pixel_mask);
}

} // namespace divitor_perception

// Register Component with ROS 2 Package Manager
RCLCPP_COMPONENTS_REGISTER_NODE(divitor_perception::ThermalPreprocessorNode)