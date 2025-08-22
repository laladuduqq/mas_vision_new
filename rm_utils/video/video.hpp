/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 10:53:50
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-22 11:05:13
 * @FilePath: /mas_vision_new/rm_utils/video/video.hpp
 * @Description: 
 */
#ifndef _VIDEO_H_
#define _VIDEO_H_

#include <opencv2/opencv.hpp>
#include <string>

namespace rm_utils {

class VideoPlayer {
public:
    VideoPlayer();
    ~VideoPlayer();
    
    bool open(const std::string& videoPath);
    bool read(cv::Mat& frame);
    void close();
    
    bool isOpened() const;
    double getFPS() const;
    cv::Size getFrameSize() const;
    
    // 添加FPS控制功能
    void setTargetFPS(double fps);
    double getTargetFPS() const;
    void enableFPSSync(bool enable);

private:
    cv::VideoCapture videoCapture;
    bool isVideoOpened;
    
    // FPS控制相关成员
    double targetFPS;
    bool syncFPS;
    std::chrono::high_resolution_clock::time_point lastFrameTime;
};

} // namespace rm_utils

#endif // _VIDEO_H_