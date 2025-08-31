/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 23:15:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-31 11:28:04
 * @FilePath: /mas_vision_new/auto_aim/thread/auto_aim_thread.cpp
 * @Description: 自动瞄准线程实现
 */
#include "armor_detector.hpp"
#include "armor_track.hpp"
#include "topicqueue.hpp"
#include "ulog.hpp"
#include "serial_types.hpp"
#include "HikCamera.h"

#include <string>
#include <thread>
#include <atomic>
#include <chrono>

extern std::atomic<bool> running;
extern rm_utils::TopicQueue<CameraFrame> image_queue;
extern rm_utils::TopicQueue<ReceivedDataMsg> serial_queue;

static std::thread auto_aim_thread;
static std::atomic<bool> auto_aim_thread_running(false);
static std::unique_ptr<auto_aim::ArmorDetector> armor_detector = nullptr;
static std::unique_ptr<auto_aim::Tracker> armor_tracker = nullptr;

// 自动瞄准线程函数
void autoAimThreadFunc() {
    ULOG_INFO_TAG("AutoAim", "Auto aim thread started");
    
    // 初始化装甲板检测器
    try {
        armor_detector = std::make_unique<auto_aim::ArmorDetector>();
        ULOG_INFO_TAG("AutoAim", "Armor detector initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("AutoAim", "Failed to initialize armor detector: %s", e.what());
        return;
    }

    // 初始化装甲板跟踪器
    try {
        armor_tracker = std::make_unique<auto_aim::Tracker>("config/auto_aim.yaml");
        ULOG_INFO_TAG("AutoAim", "Armor tracker initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("AutoAim", "Failed to initialize armor tracker: %s", e.what());
        return;
    }
    
    
    ULOG_INFO_TAG("AutoAim", "Subscribed to camera and serial data");
    
    // 主处理循环
    while (running.load() && auto_aim_thread_running.load()) {
        // IMU数据处理
        ReceivedDataMsg msg;
        if (serial_queue.pop("serial/data", msg)) {
            Eigen::Quaterniond q = getSerialDataAt(msg.timestamp);
            armor_tracker->set_R_gimbal2world(q);
        }
        CameraFrame frame;
        if (image_queue.pop("image/camera", frame)) {
            if (armor_detector && !frame.frame.empty()) {
                auto armors = armor_detector->ArmorDetect(frame.frame);
                cv::Mat armor_debug_img = frame.frame.clone();
                armor_detector->showResult(armor_debug_img);
                auto timestamp = std::chrono::steady_clock::now();
                auto tracked_targets = armor_tracker->track(armors, timestamp);
                cv::Mat track_debug_img = frame.frame.clone();
                armor_tracker->drawDebug(track_debug_img);
            }
        }
    }
    cv::destroyAllWindows();
    // 清理资源
    armor_detector.reset();
    
    ULOG_INFO_TAG("AutoAim", "Auto aim thread stopped");
}

// 启动自动瞄准线程
void startAutoAimThread() {
    auto_aim_thread_running = true;
    auto_aim_thread = std::thread(autoAimThreadFunc);
}

// 停止自动瞄准线程
void stopAutoAimThread() {
    if (!auto_aim_thread_running) {
        ULOG_WARNING_TAG("AutoAim", "Auto aim thread not running");
        return;
    }
    auto_aim_thread_running = false;
    // 等待自动瞄准线程退出
    if (auto_aim_thread.joinable()) {
        auto_aim_thread.join();
    }
}