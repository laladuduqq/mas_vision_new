/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-31 13:30:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-31 13:33:08
 * @FilePath: /mas_vision_new/auto_aim/auto_aim_info/auto_aim_info.cpp
 * @Description: 自动瞄准线程信息统计类实现
 */
#include "auto_aim_info.hpp"
#include <algorithm>

namespace auto_aim {

AutoAimInfo::AutoAimInfo() 
    : fps_(0.0), frame_count_(0) {
    last_fps_time_ = std::chrono::high_resolution_clock::now();
}

void AutoAimInfo::startTimer(const std::string& timer_name) {
    auto& timer = timers_[timer_name];
    timer.start_time = std::chrono::high_resolution_clock::now();
    timer.is_running = true;
}

void AutoAimInfo::stopTimer(const std::string& timer_name) {
    auto it = timers_.find(timer_name);
    if (it != timers_.end() && it->second.is_running) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - it->second.start_time);
        it->second.elapsed_time_ms = duration.count() / 1000.0; // 转换为毫秒
        it->second.is_running = false;
    }
}

void AutoAimInfo::updateFPS() {
    frame_count_++;
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_fps_time_);
    
    // 每秒更新一次FPS
    if (duration.count() >= 1000) {
        fps_ = frame_count_ * 1000.0 / duration.count();
        frame_count_ = 0;
        last_fps_time_ = current_time;
    }
}

void AutoAimInfo::drawInfo(cv::Mat& image, const std::string& timer_name) const {
    // 更新FPS显示
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps_));
    
    // 查找指定的计时器
    std::string time_text = timer_name + ": -- ms";
    auto it = timers_.find(timer_name);
    if (it != timers_.end()) {
        time_text = timer_name + ": " + std::to_string(it->second.elapsed_time_ms) + " ms";
    }
    
    // 在图像右上角绘制信息
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    
    // 获取文本尺寸
    int baseline = 0;
    cv::Size fps_size = cv::getTextSize(fps_text, font_face, font_scale, thickness, &baseline);
    cv::Size time_size = cv::getTextSize(time_text, font_face, font_scale, thickness, &baseline);
    
    // 选择较大的宽度作为文本框宽度
    int text_width = std::max(fps_size.width, time_size.width);
    int text_height = fps_size.height + time_size.height + 10; // 10为间距
    
    // 确保绘制区域不超出图像边界
    int x_offset = std::min(image.cols - text_width - 20, image.cols - 10);
    int y_offset = 10;
    int box_width = std::min(text_width + 10, image.cols - x_offset - 5);
    int box_height = std::min(text_height, image.rows - y_offset - 5);
    
    if (box_width > 0 && box_height > 0) {
        // 绘制半透明背景
        cv::Rect text_box(x_offset, y_offset, box_width, box_height);
        cv::Mat roi = image(text_box);
        cv::Mat color_roi(roi.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::addWeighted(roi, 0.5, color_roi, 0.5, 0, roi);
        
        // 绘制文本
        drawTextWithBackground(image, fps_text, 
                              cv::Point(x_offset + 5, y_offset + 20),
                              font_scale, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 0));
        drawTextWithBackground(image, time_text,
                              cv::Point(x_offset + 5, y_offset + 40),
                              font_scale, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 0));
    }
}

void AutoAimInfo::drawTextWithBackground(cv::Mat& image, const std::string& text,
                                        const cv::Point& origin,
                                        double font_scale,
                                        const cv::Scalar& text_color,
                                        const cv::Scalar& bg_color) const {
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    int thickness = 1;
    
    // 获取文本尺寸
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    
    // 确保文本绘制位置在图像范围内
    cv::Point text_origin = origin;
    if (text_origin.x + text_size.width > image.cols) {
        text_origin.x = std::max(0, image.cols - text_size.width);
    }
    if (text_origin.y - text_size.height < 0) {
        text_origin.y = text_size.height;
    }
    if (text_origin.y > image.rows) {
        text_origin.y = image.rows;
    }
    
    // 绘制文本
    cv::putText(image, text, text_origin, font_face, font_scale, text_color, thickness);
}

} // namespace auto_aim