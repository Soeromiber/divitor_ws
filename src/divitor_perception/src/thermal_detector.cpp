#include "divitor_perception/thermal_detector.hpp"

namespace divitor_perception {

ThermalDetector::ThermalDetector(float tMin, float tMax, double minArea,
                                 double maxArea, double minAR, double maxAR)
    : minHumanTemp(tMin), maxHumanTemp(tMax), minContourArea(minArea),
      maxContourArea(maxArea), minAspectRatio(minAR), maxAspectRatio(maxAR) {}

std::vector<HumanDetection> ThermalDetector::Detect(const float *rawMatrix,
                                                    cv::Size src_size,
                                                    cv::Size upscale_size,
                                                    cv::Mat &outputVis) {
    // Step 1: Wrap raw float array into a single-channel 32FC1 OpenCV
    // Matrix
    cv::Mat rawTempMat(src_size.height, src_size.width, CV_32FC1,
                       const_cast<float *>(rawMatrix));

    // Step 2: Spatial Upscaling via Bicubic Interpolation (32x24 ->
    // 320x240)
    cv::Mat thermalUpscaled;
    cv::resize(rawTempMat, thermalUpscaled, upscale_size, 0, 0,
               cv::INTER_CUBIC);

    // Step 3: Absolute Temperature Thresholding (Binary Mask creation)
    cv::Mat binaryMask;
    cv::inRange(thermalUpscaled, cv::Scalar(minHumanTemp),
                cv::Scalar(maxHumanTemp), binaryMask);

    // Step 4: Morphological Operations (Noise Removal and Gap Bridging)
    cv::Mat cleanedMask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // Morphological Opening (Erosion followed by Dilation) eliminates tiny
    // specs
    cv::morphologyEx(binaryMask, cleanedMask, cv::MORPH_OPEN, kernel);
    // Morphological Closing (Dilation followed by Erosion) bridges gaps in
    // body heat
    cv::morphologyEx(cleanedMask, cleanedMask, cv::MORPH_CLOSE, kernel);

    // Step 5: Contour Extraction
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleanedMask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // Prepare Visualization: Normalize float matrix to 8-bit and apply
    // thermal colormap
    cv::Mat normMat;
    cv::normalize(thermalUpscaled, normMat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(normMat, outputVis, cv::COLORMAP_INFERNO);

    std::vector<HumanDetection> detections;

    // Step 6: Geometric Rule Filtering
    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);

        // Filter Rule 1: Area threshold check
        if (area < minContourArea || area > maxContourArea) {
            continue;
        }

        cv::Rect bbox = cv::boundingRect(contour);
        double aspectRatio = static_cast<double>(bbox.width) / bbox.height;

        // Filter Rule 2: Geometric Aspect Ratio check (W / H profile)
        if (aspectRatio >= minAspectRatio && aspectRatio <= maxAspectRatio) {
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
            cv::putText(
                outputVis, label, cv::Point(bbox.x, std::max(bbox.y - 5, 15)),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        }
    }

    return detections;
}

} // namespace divitor_perception
