#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

namespace dartvision {

// HSV使用OpenCV的8位范围：H为0至179，S、V为0至255。
struct DetectorConfig {
    int hue_min = 35;
    int hue_max = 95;
    int saturation_min = 45;
    int green_value_min = 140;
    int core_value_min = 240;
    int core_saturation_max = 255;
    int median_kernel = 3;
    int support_radius = 7;
    double area_min = 80.0;
    double area_max_fraction = 0.08;
    double aspect_ratio_max = 2.4;
    double circularity_min = 0.35;
    int green_pixels_min = 20;
    double green_fraction_min = 0.08;
    static DetectorConfig load(const std::string& path);
    void validate() const;
};

struct Detection {
    cv::Point2f center;
    float radius = 0.0F;
    cv::Rect box;
    double area = 0.0;
    double circularity = 0.0;
    double aspect_ratio = 0.0;
    double green_fraction = 0.0;
    double score = 0.0;
};

// 无目标时返回空值，不用上一帧坐标冒充当前帧的检测结果。
std::optional<Detection> detectGreenLight(const cv::Mat& frame,
                                         const DetectorConfig& config,
                                         cv::Mat* green_mask = nullptr,
                                         cv::Mat* core_mask = nullptr);
void annotate(cv::Mat& frame, const std::optional<Detection>& detection,
              int frame_index, double time_s);

} // namespace dartvision
