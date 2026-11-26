#include "detector.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void lamp(cv::Mat& image, cv::Point center, int radius) {
    cv::circle(image, center, radius + 5, cv::Scalar(20, 210, 30), cv::FILLED);
    cv::circle(image, center, radius, cv::Scalar(255, 255, 255), cv::FILLED);
}
}

int main() {
    try {
        const auto config = dartvision::DetectorConfig::load("");
        cv::Mat blank(240, 320, CV_8UC3, cv::Scalar(30, 30, 30));
        require(!dartvision::detectGreenLight(blank, config), "Empty scene should not detect");
        cv::Mat white = blank.clone();
        cv::circle(white, cv::Point(160, 120), 14, cv::Scalar(255, 255, 255), cv::FILLED);
        require(!dartvision::detectGreenLight(white, config), "White light without green halo should not detect");
        cv::Mat scene = blank.clone();
        lamp(scene, cv::Point(160, 120), 14);
        lamp(scene, cv::Point(55, 35), 3); // 模拟较小的绿色反光，不能抢占主灯。
        cv::rectangle(scene, cv::Rect(5, 180, 90, 4), cv::Scalar(255, 255, 255), cv::FILLED);
        const auto detection = dartvision::detectGreenLight(scene, config);
        require(detection && cv::norm(detection->center - cv::Point2f(160, 120)) < 0.6,
                "Overexposed green lamp center should be recovered");
        cv::Mat touching_glint = blank.clone();
        lamp(touching_glint, cv::Point(160, 120), 14);
        // G达到255，使绿色反光通过仅亮度的门槛；反光与主灯高亮区相接。
        cv::circle(touching_glint, cv::Point(180, 120), 8, cv::Scalar(130, 255, 130), cv::FILLED);
        const auto merged = dartvision::detectGreenLight(touching_glint, config);
        require(merged && merged->center.x > 162.0F, "Touching bright ghost should bias a brightness-only center");
        auto white_core = config;
        white_core.core_value_min = 252;
        white_core.core_saturation_max = 35;
        white_core.area_min = 280.0;
        const auto isolated = dartvision::detectGreenLight(touching_glint, white_core);
        require(isolated && cv::norm(isolated->center - cv::Point2f(160, 120)) < 1.0 &&
                isolated->radius + 5.0F < merged->radius,
                "Low-saturation core should suppress the attached green ghost");
        cv::Mat long_light = blank.clone();
        cv::rectangle(long_light, cv::Rect(20, 60, 140, 18), cv::Scalar(20, 210, 30), cv::FILLED);
        cv::rectangle(long_light, cv::Rect(25, 65, 130, 8), cv::Scalar(255, 255, 255), cv::FILLED);
        require(!dartvision::detectGreenLight(long_light, config), "Long bright strip should fail shape filter");
        cv::Mat noise = blank.clone();
        for (int i = 0; i < 50; ++i) noise.at<cv::Vec3b>((i * 23) % 240, (i * 17) % 320) = cv::Vec3b(0, 255, 0);
        require(!dartvision::detectGreenLight(noise, config), "Isolated green noise should not detect");
        cv::Mat border = blank.clone();
        lamp(border, cv::Point(160, 237), 14);
        require(dartvision::detectGreenLight(border, config).has_value(), "Partly clipped lamp should remain detectable");
        auto invalid = config;
        invalid.median_kernel = 4;
        bool rejected = false;
        try { invalid.validate(); } catch (const std::runtime_error&) { rejected = true; }
        require(rejected, "Even median kernel must be rejected");
        std::cout << "PASS: blank, white lamp, saturated green lamp, reflection, strip, noise, border, config\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
