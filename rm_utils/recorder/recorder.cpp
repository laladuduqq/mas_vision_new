/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-21 21:05:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 21:19:36
 * @FilePath: /mas_vision_new/rm_utils/recorder/recorder.cpp
 * @Description: 
 */
#include "recorder.hpp"
#include <fmt/chrono.h>
#include <filesystem>
#include <string>
#include "ulog.hpp"

namespace rm_utils {



Recorder::Recorder(double fps)
    : init_(false)
    , fps_(fps)
    , output_dir_("records")
{
    start_time_ = std::chrono::steady_clock::now();
    last_time_ = start_time_;
    ULOG_INFO_TAG("Recorder", "Recorder created with FPS: %f", fps);
}

Recorder::~Recorder()
{
    if (!init_) 
        return;
        
    video_writer_.release();
    ULOG_INFO_TAG("Recorder", "Recorder released");
}

bool Recorder::init(const cv::Mat& img)
{
    if (img.empty()) {
        ULOG_ERROR_TAG("Recorder", "Cannot initialize with empty image");
        return false;
    }
        
    // 创建输出目录
    std::filesystem::create_directories(output_dir_);
    ULOG_DEBUG_TAG("Recorder", "Output directory ensured: %s", output_dir_.c_str());
    
    // 生成基于时间戳的文件名
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    video_path_ = fmt::format("{}/{}.avi", output_dir_, file_name);
    ULOG_INFO_TAG("Recorder", "Creating video file: %s", video_path_.c_str());

    auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
    init_ = video_writer_.isOpened();
    
    if (init_) {
        ULOG_INFO_TAG("Recorder", "Recorder initialized successfully. Size: %dx%d", 
                     img.cols, img.rows);
    } else {
        ULOG_ERROR_TAG("Recorder", "Failed to initialize video writer");
    }
    
    return init_;
}

void Recorder::record(const cv::Mat& img)
{
    if (img.empty()) {
        ULOG_WARNING_TAG("Recorder", "Skipping empty frame");
        return;
    }
        
    if (!init_) {
        if (!init(img)) {
            ULOG_ERROR_TAG("Recorder", "Failed to initialize recorder during recording");
            return;
        }
    }

    auto timestamp = std::chrono::steady_clock::now();
    auto since_last = std::chrono::duration<double>(timestamp - last_time_).count();
    if (since_last < 1.0 / fps_) {
        ULOG_DEBUG_TAG("Recorder", "Skipping frame due to FPS limit");
        return;
    }

    last_time_ = timestamp;
    
    if (video_writer_.isOpened()) {
        video_writer_.write(img);
        ULOG_DEBUG_TAG("Recorder", "Frame written to video");
    } else {
        ULOG_ERROR_TAG("Recorder", "Video writer is not opened");
    }
}

} // namespace rm_utils