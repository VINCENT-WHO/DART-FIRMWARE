#include "detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#endif

namespace fs = std::filesystem;

namespace {
// OpenCV部分Windows构建的imwrite不接受UTF-8文件名，编码后用宽路径写入。
void savePng(const fs::path& path, const cv::Mat& image) {
    std::vector<unsigned char> bytes;
    if (!cv::imencode(".png", image, bytes)) throw std::runtime_error("Cannot encode preview PNG");
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    output.close();
    if (!output) throw std::runtime_error("Cannot save preview PNG");
}
}

int main(int argc, char** argv) {
    try {
#ifdef _WIN32
        // Windows的普通main参数可能是本地代码页，中文路径需从UTF-16命令行取回。
        int wide_count = 0;
        LPWSTR* wide_args = CommandLineToArgvW(GetCommandLineW(), &wide_count);
        if (!wide_args) throw std::runtime_error("Cannot read Unicode command line");
        std::vector<std::string> utf8_args;
        for (int i = 0; i < wide_count; ++i) {
            const int length = WideCharToMultiByte(CP_UTF8, 0, wide_args[i], -1, nullptr, 0, nullptr, nullptr);
            std::string value(static_cast<std::size_t>(length), '\0');
            WideCharToMultiByte(CP_UTF8, 0, wide_args[i], -1, value.data(), length, nullptr, nullptr);
            value.pop_back();
            utf8_args.push_back(std::move(value));
        }
        LocalFree(wide_args);
        std::vector<char*> utf8_argv;
        for (auto& value : utf8_args) utf8_argv.push_back(value.data());
        argc = wide_count;
        argv = utf8_argv.data();
#endif
        const cv::String keys =
            "{help h ? | | Print usage}"
            "{input | | Input video path}"
            "{output | | Output directory (new or empty)}"
            "{config | | Optional OpenCV YAML configuration}"
            "{debug |false| Save green/core masks as a video}";
        cv::CommandLineParser parser(argc, argv, keys);
        if (parser.has("help") || argc == 1) { parser.printMessage(); return 0; }
        const auto input = parser.get<std::string>("input");
        const auto output = parser.get<std::string>("output");
        const auto config_path = parser.get<std::string>("config");
        const bool debug = parser.get<bool>("debug");
        if (!parser.check()) { parser.printErrors(); return 1; }
        if (input.empty() || output.empty()) throw std::runtime_error("--input and --output are required");
        const auto input_path = fs::u8path(input);
        const auto output_path = fs::u8path(output);
        if (!fs::is_regular_file(input_path)) throw std::runtime_error("Input video does not exist");
        if (fs::exists(output_path) && (!fs::is_directory(output_path) || !fs::is_empty(output_path)))
            throw std::runtime_error("Output directory must be new or empty; existing files will not be overwritten");
        const auto config = dartvision::DetectorConfig::load(config_path);
        cv::VideoCapture capture(input, cv::CAP_ANY);
        if (!capture.isOpened()) throw std::runtime_error("Cannot decode input video; check OpenCV runtime dependencies");
        const double fps = capture.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(fps) || fps <= 0.0) throw std::runtime_error("Video has no valid nominal FPS");
        cv::Mat frame;
        if (!capture.read(frame)) throw std::runtime_error("Video contains no decodable frame");
        const cv::Size video_size = frame.size();
        fs::create_directories(output_path);
        const int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        cv::VideoWriter writer((output_path / "annotated.avi").u8string(), codec, fps, frame.size());
        if (!writer.isOpened()) throw std::runtime_error("Cannot create MJPEG output video");
        cv::VideoWriter debug_writer;
        if (debug) {
            debug_writer.open((output_path / "masks.avi").u8string(), codec, fps,
                               cv::Size(frame.cols * 2, frame.rows));
            if (!debug_writer.isOpened()) throw std::runtime_error("Cannot create debug video");
        }
        std::ofstream csv(output_path / "centers.csv", std::ios::binary);
        if (!csv) throw std::runtime_error("Cannot create centers.csv");
        csv << "\xEF\xBB\xBF";
        csv << "帧序号,视频相对时间_s,检测到目标,中心X_px,中心Y_px,标注半径_px,"
               "包围框X_px,包围框Y_px,包围框宽_px,包围框高_px,灯芯面积_px2,圆度,长宽比,绿色支持比例\n";
        csv << std::fixed << std::setprecision(6);
        int frame_index = 0, detected_count = 0, first_detected = -1, last_detected = -1;
        const auto begin = std::chrono::steady_clock::now();
        do {
            cv::Mat green_mask, core_mask;
            const auto detection = dartvision::detectGreenLight(frame, config, &green_mask, &core_mask);
            const double time_s = frame_index / fps;
            csv << frame_index << ',' << time_s << ',' << (detection ? 1 : 0);
            if (detection) {
                const auto& d = *detection;
                csv << ',' << d.center.x << ',' << d.center.y << ',' << d.radius
                    << ',' << d.box.x << ',' << d.box.y << ',' << d.box.width << ',' << d.box.height
                    << ',' << d.area << ',' << d.circularity << ',' << d.aspect_ratio << ',' << d.green_fraction;
                if (first_detected < 0) first_detected = frame_index;
                last_detected = frame_index;
                ++detected_count;
            } else csv << ",,,,,,,,,,,"; // 无目标坐标留空，不填0，也不沿用上一帧。
            csv << '\n';
            dartvision::annotate(frame, detection, frame_index, time_s);
            writer.write(frame);
            if (frame_index % 25 == 0 || (detection && detected_count == 1)) {
                savePng(output_path / ("frame_" + std::to_string(frame_index) + ".png"), frame);
            }
            if (debug) {
                cv::Mat masks, masks_bgr;
                cv::hconcat(green_mask, core_mask, masks);
                cv::cvtColor(masks, masks_bgr, cv::COLOR_GRAY2BGR);
                debug_writer.write(masks_bgr);
            }
            ++frame_index;
        } while (capture.read(frame));
        writer.release();
        debug_writer.release();
        csv.close();
        if (!csv) throw std::runtime_error("Writing centers.csv failed");
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        std::ofstream summary(output_path / "summary.json");
        summary << std::setprecision(10)
                << "{\n  \"opencv_version\": \"" << CV_VERSION << "\",\n"
                << "  \"decoded_frames\": " << frame_index << ",\n"
                << "  \"detected_frames\": " << detected_count << ",\n"
                << "  \"not_found_frames\": " << frame_index - detected_count << ",\n"
                << "  \"first_detected_frame\": " << first_detected << ",\n"
                << "  \"last_detected_frame\": " << last_detected << ",\n"
                << "  \"nominal_fps\": " << fps << ",\n"
                << "  \"width\": " << video_size.width << ",\n  \"height\": " << video_size.height << ",\n"
                << "  \"processing_seconds\": " << elapsed << "\n}\n";
        summary.close();
        if (!summary) throw std::runtime_error("Cannot save summary.json");
        std::cout << "Decoded " << frame_index << " frames; detected " << detected_count
                  << "; first=" << first_detected << "; last=" << last_detected
                  << "; elapsed=" << elapsed << " s\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
