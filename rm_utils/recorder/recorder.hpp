/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-21 21:05:25
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 21:15:49
 * @FilePath: /mas_vision_new/rm_utils/recorder/recorder.hpp
 * @Description: 
 */
#ifndef RM_UTILS_RECORDER_HPP
#define RM_UTILS_RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

namespace rm_utils {

class Recorder
{
public:
    Recorder(double fps = 30);
    ~Recorder();
    
    bool init(const cv::Mat& img);
    void record(const cv::Mat& img);
    
    bool isInitialized() const { return init_; }
    
    void setOutputDirectory(const std::string& dir) { output_dir_ = dir; }
    void setFPS(double fps) { fps_ = fps; }

private:
    bool init_;
    double fps_;
    std::string output_dir_;
    std::string text_path_;
    std::string video_path_;
    cv::VideoWriter video_writer_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;
};

} // namespace rm_utils

#endif // RM_UTILS_RECORDER_HPP