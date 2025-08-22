/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 10:53:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-22 10:54:30
 * @FilePath: /mas_vision_new/rm_utils/video/video.cpp
 * @Description: 
 */
#include "video.hpp"
#include "ulog.hpp"
#include <chrono>
#include <thread>

namespace rm_utils {

VideoPlayer::VideoPlayer() : isVideoOpened(false), targetFPS(30.0), syncFPS(true) {
    ULOG_DEBUG_TAG("VideoPlayer", "VideoPlayer created");
}

VideoPlayer::~VideoPlayer() {
    close();
    ULOG_DEBUG_TAG("VideoPlayer", "VideoPlayer destroyed");
}

bool VideoPlayer::open(const std::string& videoPath) {
    close(); // 关闭任何已打开的视频
    
    videoCapture.open(videoPath);
    
    if (!videoCapture.isOpened()) {
        ULOG_ERROR_TAG("VideoPlayer", "无法打开视频文件: %s", videoPath.c_str());
        isVideoOpened = false;
        return false;
    }
    
    isVideoOpened = true;
    lastFrameTime = std::chrono::high_resolution_clock::now();
    ULOG_INFO_TAG("VideoPlayer", "成功打开视频文件: %s", videoPath.c_str());
    return true;
}

bool VideoPlayer::read(cv::Mat& frame) {
    if (!isVideoOpened || !videoCapture.isOpened()) {
        return false;
    }
    
    bool success = videoCapture.read(frame);
    
    // 如果读取失败，可能是视频结束了，尝试重新定位到开头
    if (!success) {
        videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
        success = videoCapture.read(frame);
        
        if (success) {
            ULOG_INFO_TAG("VideoPlayer", "视频循环播放");
        } else {
            ULOG_WARNING_TAG("VideoPlayer", "无法重新读取视频");
        }
    }
    
    // FPS同步控制
    if (success && syncFPS && targetFPS > 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(now - lastFrameTime).count();
        auto targetDuration = 1.0 / targetFPS;
        
        if (elapsed < targetDuration) {
            auto sleepTime = targetDuration - elapsed;
            std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }
        
        lastFrameTime = std::chrono::high_resolution_clock::now();
    }
    
    return success;
}

void VideoPlayer::close() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
    isVideoOpened = false;
    ULOG_DEBUG_TAG("VideoPlayer", "视频播放器已关闭");
}

bool VideoPlayer::isOpened() const {
    return isVideoOpened && videoCapture.isOpened();
}

double VideoPlayer::getFPS() const {
    if (isOpened()) {
        return videoCapture.get(cv::CAP_PROP_FPS);
    }
    return 0.0;
}

cv::Size VideoPlayer::getFrameSize() const {
    if (isOpened()) {
        int width = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
        return cv::Size(width, height);
    }
    return cv::Size(0, 0);
}

void VideoPlayer::setTargetFPS(double fps) {
    targetFPS = fps;
    ULOG_DEBUG_TAG("VideoPlayer", "设置目标FPS: %f", fps);
}

double VideoPlayer::getTargetFPS() const {
    return targetFPS;
}

void VideoPlayer::enableFPSSync(bool enable) {
    syncFPS = enable;
    ULOG_DEBUG_TAG("VideoPlayer", "FPS同步 %s", enable ? "启用" : "禁用");
}

} // namespace rm_utils