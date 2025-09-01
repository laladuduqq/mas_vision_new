/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-31 13:30:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-31 13:32:47
 * @FilePath: /mas_vision_new/auto_aim/auto_aim_info/auto_aim_info.hpp
 * @Description: 自动瞄准线程信息统计类
 */
#ifndef _AUTO_AIM_INFO_H_
#define _AUTO_AIM_INFO_H_

#include <opencv2/opencv.hpp>
#include <chrono>
#include <map>
#include <string>

namespace auto_aim {

class AutoAimInfo {
public:
    AutoAimInfo();
    ~AutoAimInfo() = default;

    // 计时器操作
    void startTimer(const std::string& timer_name);
    void stopTimer(const std::string& timer_name);
    
    // FPS计算
    void updateFPS();
    
    // 在图像上绘制信息
    void drawInfo(cv::Mat& image, const std::string& timer_name) const;
    
    // 获取FPS值
    double getFPS() const { return fps_; }

private:
    struct TimerInfo {
        std::chrono::high_resolution_clock::time_point start_time;
        double elapsed_time_ms;  // 毫秒
        bool is_running;
    };
    
    std::map<std::string, TimerInfo> timers_;
    
    // FPS相关
    double fps_;
    int frame_count_;
    std::chrono::high_resolution_clock::time_point last_fps_time_;
    
    // 辅助函数
    void drawTextWithBackground(cv::Mat& image, const std::string& text, 
                               const cv::Point& origin, 
                               double font_scale = 0.5,
                               const cv::Scalar& text_color = cv::Scalar(255, 255, 255),
                               const cv::Scalar& bg_color = cv::Scalar(0, 0, 0)) const;
};

} // namespace auto_aim

#endif // _AUTO_AIM_INFO_H_