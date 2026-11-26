#include "detector.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace dartvision {

DetectorConfig DetectorConfig::load(const std::string& path) {
    DetectorConfig config;
    if (!path.empty()) {
        cv::FileStorage file(path, cv::FileStorage::READ);
        if (!file.isOpened()) throw std::runtime_error("Cannot open config: " + path);
        const auto read = [&file](const char* name, auto& value) {
            if (!file[name].empty()) file[name] >> value;
        };
        read("hue_min", config.hue_min);
        read("hue_max", config.hue_max);
        read("saturation_min", config.saturation_min);
        read("green_value_min", config.green_value_min);
        read("core_value_min", config.core_value_min);
        read("core_saturation_max", config.core_saturation_max);
        read("median_kernel", config.median_kernel);
        read("support_radius", config.support_radius);
        read("area_min", config.area_min);
        read("area_max_fraction", config.area_max_fraction);
        read("aspect_ratio_max", config.aspect_ratio_max);
        read("circularity_min", config.circularity_min);
        read("green_pixels_min", config.green_pixels_min);
        read("green_fraction_min", config.green_fraction_min);
    }
    config.validate();
    return config;
}

void DetectorConfig::validate() const {
    if (hue_min < 0 || hue_max > 179 || hue_min > hue_max ||
        saturation_min < 0 || saturation_min > 255 ||
        green_value_min < 0 || green_value_min > 255 ||
        core_value_min < 0 || core_value_min > 255 ||
        core_saturation_max < 0 || core_saturation_max > 255 ||
        median_kernel < 1 || median_kernel > 15 || median_kernel % 2 != 1 ||
        support_radius < 1 || support_radius > 50 ||
        !std::isfinite(area_min) || area_min <= 0.0 ||
        !std::isfinite(area_max_fraction) || area_max_fraction <= 0.0 || area_max_fraction > 1.0 ||
        !std::isfinite(aspect_ratio_max) || aspect_ratio_max < 1.0 ||
        !std::isfinite(circularity_min) || circularity_min < 0.0 || circularity_min > 1.0 ||
        green_pixels_min < 1 || !std::isfinite(green_fraction_min) ||
        green_fraction_min < 0.0 || green_fraction_min > 1.0) {
        throw std::runtime_error("Invalid detector thresholds in config");
    }
}

std::optional<Detection> detectGreenLight(const cv::Mat& frame,
                                         const DetectorConfig& config,
                                         cv::Mat* green_mask,
                                         cv::Mat* core_mask) {
    if (frame.empty() || frame.type() != CV_8UC3)
        throw std::invalid_argument("Expected a nonempty BGR 8-bit frame");
    cv::Mat hsv, green, core;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(config.hue_min, config.saturation_min, config.green_value_min),
                cv::Scalar(config.hue_max, 255, 255), green);
    // 灯芯不限制色相。主灯过曝发白时，可限制饱和度，把偏绿的高亮反射从灯芯轮廓里分离。
    cv::inRange(hsv, cv::Scalar(0, 0, config.core_value_min),
                cv::Scalar(179, config.core_saturation_max, 255), core);
    if (config.median_kernel > 1) {
        cv::medianBlur(green, green, config.median_kernel);
        cv::medianBlur(core, core, config.median_kernel);
    }
    if (green_mask) *green_mask = green.clone();
    if (core_mask) *core_mask = core.clone();

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(core, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const auto support_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(config.support_radius * 2 + 1, config.support_radius * 2 + 1));
    std::optional<Detection> best;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < config.area_min || area > config.area_max_fraction * frame.total()) continue;
        const auto box = cv::boundingRect(contour);
        const double aspect = static_cast<double>(std::max(box.width, box.height)) /
                              std::min(box.width, box.height);
        if (aspect > config.aspect_ratio_max) continue;
        const double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 0.0) continue;
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < config.circularity_min) continue;

        // 高亮区域周围必须有足够绿色光晕，排除白色日光灯和普通白色物体。
        cv::Mat component = cv::Mat::zeros(frame.size(), CV_8U), support, green_support;
        std::vector<std::vector<cv::Point>> one_contour{contour};
        cv::drawContours(component, one_contour, 0, cv::Scalar(255), cv::FILLED);
        cv::dilate(component, support, support_kernel);
        cv::bitwise_and(green, support, green_support);
        const int green_count = cv::countNonZero(green_support);
        const double green_fraction = static_cast<double>(green_count) / cv::countNonZero(support);
        if (green_count < config.green_pixels_min || green_fraction < config.green_fraction_min) continue;

        const auto moments = cv::moments(contour);
        if (moments.m00 <= 0.0) continue;
        Detection candidate;
        // 取高亮连通区域的几何质心；不是最大外接圆中心，也不是IMU坐标。
        candidate.center = cv::Point2f(static_cast<float>(moments.m10 / moments.m00),
                                       static_cast<float>(moments.m01 / moments.m00));
        float radius = 0.0F;
        for (const auto& point : contour)
            radius = std::max(radius, static_cast<float>(cv::norm(cv::Point2f(point) - candidate.center)));
        candidate.radius = radius + 3.0F;
        candidate.box = box;
        candidate.area = area;
        candidate.circularity = circularity;
        candidate.aspect_ratio = aspect;
        candidate.green_fraction = green_fraction;
        // 本任务优先选主要发光体，避免把较小的绿色镜头反光点当主灯。
        candidate.score = area * (0.5 + green_fraction) * (0.5 + circularity);
        if (!best || candidate.score > best->score) best = candidate;
    }
    return best;
}

void annotate(cv::Mat& frame, const std::optional<Detection>& detection,
              int frame_index, double time_s) {
    const std::string frame_text = cv::format("frame %d  t=%.3fs", frame_index, time_s);
    cv::putText(frame, frame_text, cv::Point(5, 14), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(frame, frame_text, cv::Point(5, 14), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (detection) {
        const cv::Point center(cvRound(detection->center.x), cvRound(detection->center.y));
        cv::circle(frame, center, cvRound(detection->radius), cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        cv::drawMarker(frame, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 11, 1, cv::LINE_AA);
        cv::circle(frame, center, 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        const std::string label = cv::format("(%.1f, %.1f)", detection->center.x, detection->center.y);
        int baseline = 0;
        const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
        const cv::Point origin(std::clamp(center.x - size.width / 2, 0, std::max(0, frame.cols - size.width)),
                               std::clamp(center.y - cvRound(detection->radius) - 5, 30, frame.rows - 5));
        cv::putText(frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    } else {
        cv::putText(frame, "GREEN LIGHT: NOT FOUND", cv::Point(5, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
}
} // namespace dartvision
