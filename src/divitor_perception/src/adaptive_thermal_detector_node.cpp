#include "divitor_perception/adaptive_thermal_detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace divitor_perception {

AdaptiveThermalDetectorNode::AdaptiveThermalDetectorNode(
    const rclcpp::NodeOptions &options)
    : Node("adaptive_thermal_detector_node", options) {
    declareAndFetchParameters();

    // Dynamic Parameter Callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : params) {
                if (param.get_name() == "detection_mode")
                    detection_mode_ = param.as_string();
                else if (param.get_name() == "k_sigma")
                    k_sigma_ = param.as_double();
                else if (param.get_name() == "neighborhood_kernel")
                    neighborhood_kernel_ = param.as_int();
                else if (param.get_name() == "temp_min")
                    temp_min_ = param.as_double();
                else if (param.get_name() == "temp_max")
                    temp_max_ = param.as_double();
                else if (param.get_name() == "solar_reject_temp")
                    solar_reject_temp_ = param.as_double();
                else if (param.get_name() == "min_blob_area")
                    min_blob_area_ = param.as_int();
                else if (param.get_name() == "max_blob_area")
                    max_blob_area_ = param.as_int();
                else if (param.get_name() == "min_aspect_ratio")
                    min_aspect_ratio_ = param.as_double();
                else if (param.get_name() == "max_aspect_ratio")
                    max_aspect_ratio_ = param.as_double();
            }
            return result;
        });

    // Publishers
    pub_detections_ =
        this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/detections", 1);

    pub_debug_mask_ =
        this->create_publisher<sensor_msgs::msg::Image>("/debug_mask", 1);

    // Subscriber
    sub_preprocessed_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/raw_temperature", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&AdaptiveThermalDetectorNode::TemperatureCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Adaptive Thermal Detector Node initialized in [%s] mode.",
                detection_mode_.c_str());
}

void AdaptiveThermalDetectorNode::declareAndFetchParameters() {
    detection_mode_ = declare_parameter<std::string>("detection_mode", "NIGHT");
    k_sigma_ = declare_parameter<double>("k_sigma", 1.5);
    neighborhood_kernel_ = declare_parameter<int>("neighborhood_kernel", 15);
    temp_min_ = declare_parameter<double>("temp_min", 24.0);
    temp_max_ = declare_parameter<double>("temp_max", 38.0);
    solar_reject_temp_ = declare_parameter<double>("solar_reject_temp", 38.0);
    min_blob_area_ = declare_parameter<int>("min_blob_area", 20);
    max_blob_area_ = declare_parameter<int>("max_blob_area", 1200);
    min_aspect_ratio_ = declare_parameter<double>("min_aspect_ratio", 0.4);
    max_aspect_ratio_ = declare_parameter<double>("max_aspect_ratio", 1.2);
}

cv::Mat
AdaptiveThermalDetectorNode::applyNightThresholding(const cv::Mat &temp_frame) {
    // Compute global image statistics
    cv::Scalar mean_val, stddev_val;
    cv::meanStdDev(temp_frame, mean_val, stddev_val);

    double dynamic_thresh = mean_val[0] + k_sigma_ * stddev_val[0];
    dynamic_thresh = std::max(dynamic_thresh, temp_min_);

    cv::Mat mask;
    // Combine dynamic statistical thresholding with explicit physiological
    // temperature bounds
    cv::inRange(temp_frame, cv::Scalar(dynamic_thresh), cv::Scalar(temp_max_),
                mask);
    return mask;
}

cv::Mat AdaptiveThermalDetectorNode::applyMiddayThresholding(
    const cv::Mat &temp_frame) {
    int ksize = (neighborhood_kernel_ % 2 == 0) ? neighborhood_kernel_ + 1
                                                : neighborhood_kernel_;

    // Compute local neighborhood mean using box filter
    cv::Mat local_mean;
    cv::boxFilter(temp_frame, local_mean, CV_32F, cv::Size(ksize, ksize));

    // Compute local variance/stddev: E[X^2] - (E[X])^2
    cv::Mat temp_sq, local_mean_sq;
    cv::multiply(temp_frame, temp_frame, temp_sq);
    cv::boxFilter(temp_sq, local_mean_sq, CV_32F, cv::Size(ksize, ksize));

    cv::Mat local_var, local_stddev;
    cv::subtract(local_mean_sq, local_mean.mul(local_mean), local_var);
    cv::max(local_var, 0.0, local_var); // Clamp precision errors
    cv::sqrt(local_var, local_stddev);

    // Absolute difference deviation: |T(x,y) - local_mean| > k * local_stddev
    cv::Mat abs_diff;
    cv::absdiff(temp_frame, local_mean, abs_diff);

    cv::Mat threshold_map;
    cv::multiply(local_stddev, cv::Scalar(k_sigma_), threshold_map);

    cv::Mat deviation_mask;
    cv::compare(abs_diff, threshold_map, deviation_mask, cv::CMP_GT);

    // Solar heat rejection mask (Exclude sun-baked ground/metal > 38°C)
    cv::Mat valid_temp_mask;
    cv::inRange(temp_frame, cv::Scalar(18.0), cv::Scalar(solar_reject_temp_),
                valid_temp_mask);

    cv::Mat final_mask;
    cv::bitwise_and(deviation_mask, valid_temp_mask, final_mask);

    return final_mask;
}

void AdaptiveThermalDetectorNode::TemperatureCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat &temp_frame = cv_ptr->image;
    if (temp_frame.empty())
        return;

    // 1. Choose thresholding algorithm
    cv::Mat binary_mask;
    if (detection_mode_ == "MIDDAY") {
        binary_mask = applyMiddayThresholding(temp_frame);
    } else { // Default to NIGHT mode
        binary_mask = applyNightThresholding(temp_frame);
    }

    // 2. Morphological filtering (Opening -> Closing)
    cv::Mat morph_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat cleaned_mask;
    cv::morphologyEx(binary_mask, cleaned_mask, cv::MORPH_OPEN, morph_kernel,
                     cv::Point(-1, -1), 1);
    cv::morphologyEx(cleaned_mask, cleaned_mask, cv::MORPH_CLOSE, morph_kernel,
                     cv::Point(-1, -1), 2);

    // 3. Find connected contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned_mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // 4. Construct Detection2DArray message
    auto detections_msg =
        std::make_unique<vision_msgs::msg::Detection2DArray>();
    detections_msg->header = msg->header;

    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_blob_area_ || area > max_blob_area_) {
            continue;
        }

        cv::Rect bbox = cv::boundingRect(contour);
        double aspect_ratio =
            static_cast<double>(bbox.width) / static_cast<double>(bbox.height);

        // Filter using expected aerial human aspect ratios
        if (aspect_ratio >= min_aspect_ratio_ &&
            aspect_ratio <= max_aspect_ratio_) {
            vision_msgs::msg::Detection2D detection;
            detection.header = msg->header;

            // Center offset
            detection.bbox.center.position.x = bbox.x + (bbox.width / 2.0);
            detection.bbox.center.position.y = bbox.y + (bbox.height / 2.0);
            detection.bbox.size_x = bbox.width;
            detection.bbox.size_y = bbox.height;

            // Detection confidence score based on area density
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = "human";
            hypothesis.hypothesis.score =
                std::min(1.0, area / (bbox.width * bbox.height));
            detection.results.push_back(hypothesis);

            detections_msg->detections.push_back(detection);
        }
    }

    // 5. Publish Detections (Zero-copy transfer)
    pub_detections_->publish(std::move(detections_msg));

    // 6. Publish Debug Binary Mask (if subscribed)
    if (pub_debug_mask_->get_subscription_count() > 0) {
        auto debug_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage cv_debug(
            msg->header, sensor_msgs::image_encodings::MONO8, cleaned_mask);
        cv_debug.toImageMsg(*debug_msg);
        pub_debug_mask_->publish(std::move(debug_msg));
    }
}

} // namespace divitor_perception

RCLCPP_COMPONENTS_REGISTER_NODE(divitor_perception::AdaptiveThermalDetectorNode)