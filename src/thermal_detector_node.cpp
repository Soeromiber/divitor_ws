#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

// Structure to store detected target telemetry
struct HumanDetection
{
    cv::Rect bbox;
    double area;
    double aspectRatio;
    double maxTemperature; // Peak temperature within the target region (°C)
};

class ThermalDetector
{
  private:
    float minHumanTemp;    // Lower temperature threshold (°C)
    float maxHumanTemp;    // Upper temperature threshold (°C)
    double minContourArea; // Minimum pixel area filter
    double maxContourArea; // Maximum pixel area filter
    double minAspectRatio; // Min Width/Height ratio (upright/crouching human)
    double maxAspectRatio; // Max Width/Height ratio

  public:
    // Constructor with default parameters for human detection
    ThermalDetector(float tMin = 31.0f, float tMax = 38.0f, double minArea = 150.0, double maxArea = 12000.0,
                    double minAR = 0.25, double maxAR = 0.80)
        : minHumanTemp(tMin), maxHumanTemp(tMax), minContourArea(minArea), maxContourArea(maxArea),
          minAspectRatio(minAR), maxAspectRatio(maxAR)
    {
    }

    /**
     * Process raw 1D thermal temperature matrix into bounding boxes.
     * @param rawMatrix Pointer to float array containing temperature values in °C
     * (e.g., 768 elements)
     * @param src_size  Size of the source thermal image
     * @param upscale_size Size to which the image should be upscaled
     * @param outputVis Output cv::Mat for colorized visualization
     */
    std::vector<HumanDetection> Detect(const float *rawMatrix, cv::Size src_size, cv::Size upscale_size,
                                       cv::Mat &outputVis)
    {
        // Step 1: Wrap raw float array into a single-channel 32FC1 OpenCV Matrix
        cv::Mat rawTempMat(src_size.height, src_size.width, CV_32FC1, const_cast<float *>(rawMatrix));

        // Step 2: Spatial Upscaling via Bicubic Interpolation (32x24 -> 320x240)
        cv::Mat thermalUpscaled;
        cv::resize(rawTempMat, thermalUpscaled, upscale_size, 0, 0, cv::INTER_CUBIC);

        // Step 3: Absolute Temperature Thresholding (Binary Mask creation)
        cv::Mat binaryMask;
        cv::inRange(thermalUpscaled, cv::Scalar(minHumanTemp), cv::Scalar(maxHumanTemp), binaryMask);

        // Step 4: Morphological Operations (Noise Removal and Gap Bridging)
        cv::Mat cleanedMask;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        // Morphological Opening (Erosion followed by Dilation) eliminates tiny
        // specs
        cv::morphologyEx(binaryMask, cleanedMask, cv::MORPH_OPEN, kernel);
        // Morphological Closing (Dilation followed by Erosion) bridges gaps in body
        // heat
        cv::morphologyEx(cleanedMask, cleanedMask, cv::MORPH_CLOSE, kernel);

        // Step 5: Contour Extraction
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(cleanedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Prepare Visualization: Normalize float matrix to 8-bit and apply thermal
        // colormap
        cv::Mat normMat;
        cv::normalize(thermalUpscaled, normMat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(normMat, outputVis, cv::COLORMAP_INFERNO);

        std::vector<HumanDetection> detections;

        // Step 6: Geometric Rule Filtering
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);

            // Filter Rule 1: Area threshold check
            if (area < minContourArea || area > maxContourArea)
            {
                continue;
            }

            cv::Rect bbox = cv::boundingRect(contour);
            double aspectRatio = static_cast<double>(bbox.width) / bbox.height;

            // Filter Rule 2: Geometric Aspect Ratio check (W / H profile)
            if (aspectRatio >= minAspectRatio && aspectRatio <= maxAspectRatio)
            {
                // Extract maximum temperature value inside the bounding box
                cv::Mat roi = thermalUpscaled(bbox);
                double minVal, maxVal;
                cv::minMaxLoc(roi, &minVal, &maxVal);

                HumanDetection det;
                det.bbox = bbox;
                det.area = area;
                det.aspectRatio = aspectRatio;
                det.maxTemperature = maxVal;
                detections.push_back(det);

                // Draw bounding box and telemetry text on visual display
                cv::rectangle(outputVis, bbox, cv::Scalar(0, 255, 0), 2);
                std::string label = cv::format("Human: %.1f deg C", maxVal);
                cv::putText(outputVis, label, cv::Point(bbox.x, std::max(bbox.y - 5, 15)), cv::FONT_HERSHEY_SIMPLEX,
                            0.45, cv::Scalar(255, 255, 255), 1);
            }
        }

        return detections;
    }
};

class ThermalDetectorNode : public rclcpp::Node
{
  public:
    const cv::Size UPSCALE_SIZE = cv::Size(320, 240); // Upscaled image size for detection and visualization

    ThermalDetectorNode() : Node("thermal_detector_node")
    {
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
        detector_ = std::make_unique<ThermalDetector>(minTemp, maxTemp, minArea, maxArea, minAR, maxAR);

        rclcpp::SensorDataQoS qos;
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "thermal_camera/raw", qos,
            [this](sensor_msgs::msg::Image::UniquePtr msg) { ProcessThermalFrame(std::move(msg)); });

        visualization_pub_ = create_publisher<sensor_msgs::msg::Image>("thermal_camera/visualization", 10);
    }

    void ProcessThermalFrame(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        sensor_msgs::msg::Image::UniquePtr vis_msg = std::make_unique<sensor_msgs::msg::Image>();
        vis_msg->header = msg->header;
        vis_msg->height = UPSCALE_SIZE.height;
        vis_msg->width = UPSCALE_SIZE.width;
        vis_msg->encoding = "bgr8";
        vis_msg->is_bigendian = false;
        vis_msg->step = vis_msg->width * 3;
        vis_msg->data.resize(vis_msg->height * vis_msg->step);

        cv::Mat visualization(UPSCALE_SIZE.height, UPSCALE_SIZE.width, CV_8UC3, vis_msg->data.data());
        cv::Size src_size(msg->width, msg->height);
        std::vector<HumanDetection> detections =
            detector_->Detect(reinterpret_cast<const float *>(msg->data.data()), src_size, UPSCALE_SIZE, visualization);

        visualization_pub_->publish(std::move(vis_msg));
    }

  private:
    std::unique_ptr<ThermalDetector> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThermalDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}