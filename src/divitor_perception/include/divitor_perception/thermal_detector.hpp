#pragma once

#include <opencv2/opencv.hpp>

namespace divitor_perception {

// Structure to store detected target telemetry
struct HumanDetection {
    cv::Rect bbox;
    double area;
    double aspectRatio;
    double maxTemperature; // Peak temperature within the target region (°C)
};

class ThermalDetector {
  private:
    float minHumanTemp;    // Lower temperature threshold (°C)
    float maxHumanTemp;    // Upper temperature threshold (°C)
    double minContourArea; // Minimum pixel area filter
    double maxContourArea; // Maximum pixel area filter
    double minAspectRatio; // Min Width/Height ratio (upright/crouching human)
    double maxAspectRatio; // Max Width/Height ratio

  public:
    // Constructor with default parameters for human detection
    ThermalDetector(float tMin = 31.0f, float tMax = 38.0f,
                    double minArea = 150.0, double maxArea = 12000.0,
                    double minAR = 0.25, double maxAR = 0.80);

    /**
     * Process raw 1D thermal temperature matrix into bounding boxes.
     * @param rawMatrix Pointer to float array containing temperature values in
     * °C (e.g., 768 elements)
     * @param src_size  Size of the source thermal image
     * @param upscale_size Size to which the image should be upscaled
     * @param outputVis Output cv::Mat for colorized visualization
     */
    std::vector<HumanDetection> Detect(const float *rawMatrix,
                                       cv::Size src_size, cv::Size upscale_size,
                                       cv::Mat &outputVis);
};

} // namespace divitor_perception
