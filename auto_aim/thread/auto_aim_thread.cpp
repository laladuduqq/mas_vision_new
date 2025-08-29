/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 23:15:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-29 18:25:55
 * @FilePath: /mas_vision_new/auto_aim/thread/auto_aim_thread.cpp
 * @Description: 自动瞄准线程实现
 */
#include "armor_detector.hpp"
#include "armor_track.hpp"
#include "pubsub.hpp"
#include "performance_monitor.hpp"
#include "ulog.hpp"
#include "serial_types.hpp"
#include "HikCamera.h"

#include <string>
#include <thread>
#include <atomic>
#include <chrono>

extern std::atomic<bool> running;
extern mas_utils::PerformanceMonitor perfMonitor;

static std::atomic<bool> auto_aim_thread_running(false);
static std::atomic<bool> auto_aim_thread_finished(true);
static std::unique_ptr<auto_aim::ArmorDetector> armor_detector = nullptr;
static std::unique_ptr<auto_aim::Tracker> armor_tracker = nullptr;

// 自动瞄准线程函数
void autoAimThreadFunc() {
    auto_aim_thread_finished = false;
    // 注册性能监控
    perfMonitor.addThread("AutoAimThread", perfMonitor.getThreadsId());
    
    ULOG_INFO_TAG("AutoAim", "Auto aim thread started");
    
    // 初始化装甲板检测器
    try {
        armor_detector = std::make_unique<auto_aim::ArmorDetector>();
        ULOG_INFO_TAG("AutoAim", "Armor detector initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("AutoAim", "Failed to initialize armor detector: %s", e.what());
        auto_aim_thread_finished = true;
        return;
    }

    // 初始化装甲板跟踪器
    try {
        armor_tracker = std::make_unique<auto_aim::Tracker>("config/auto_aim.yaml");
        ULOG_INFO_TAG("AutoAim", "Armor tracker initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("AutoAim", "Failed to initialize armor tracker: %s", e.what());
        auto_aim_thread_finished = true;
        return;
    }
    
    // 创建订阅者
    Subscriber subscriber;
    
    // 用于存储最新接收到的帧和IMU数据
    std::atomic<bool> frame_ready(false);
    std::atomic<bool> imu_ready(false);
    CameraFrame latest_frame;
    ReceivedDataMsg latest_imu_data;
    
    // 订阅相机图像
    subscriber.subscribe<CameraFrame>("camera/image", 
        [&latest_frame, &frame_ready](const CameraFrame& frame) {
            latest_frame = frame;
            frame_ready = true;
        });
    
    // 订阅串口数据
    subscriber.subscribe<ReceivedDataMsg>("serial/data",
        [&latest_imu_data, &imu_ready](const ReceivedDataMsg& data) {
            latest_imu_data = data;
            imu_ready = true;
        });
    
    ULOG_INFO_TAG("AutoAim", "Subscribed to camera and serial data");
    
    // 主处理循环
    while (running.load() && auto_aim_thread_running.load()) {
        // 如果有IMU数据，可以在这里处理
        if (imu_ready.load()) {
            // 重置IMU就绪标志
            imu_ready = false;
            Eigen::Quaterniond q = getSerialDataAt(latest_imu_data.timestamp);
            armor_tracker->set_R_gimbal2world(q);
        }
        // 检查是否有新的图像数据
        if (frame_ready.load()) {
            // 重置帧就绪标志
            frame_ready = false;
            // 执行装甲板检测
            if (armor_detector && !latest_frame.frame.empty()) {
                auto processStartTime = std::chrono::steady_clock::now();
                // 检测装甲板
                auto armors = armor_detector->ArmorDetect(latest_frame.frame);
                armor_detector->showResult(latest_frame.frame);
                // 跟踪装甲板
                auto timestamp = std::chrono::steady_clock::now();
                auto tracked_targets = armor_tracker->track(armors, timestamp);
                // 显示跟踪信息
                armor_tracker->drawDebug(latest_frame.frame);
            }
        }
    }
    
    cv::destroyAllWindows();
    // 清理资源
    armor_detector.reset();
    
    auto_aim_thread_finished = true;
    ULOG_INFO_TAG("AutoAim", "Auto aim thread stopped");
}

// 启动自动瞄准线程
void startAutoAimThread() {
    auto_aim_thread_running = true;
    static std::thread auto_aim_thread(autoAimThreadFunc);
    // 分离线程，让它独立运行
    auto_aim_thread.detach();
    ULOG_INFO_TAG("AutoAim", "Auto aim thread launched");
}

// 停止自动瞄准线程
void stopAutoAimThread() {
    auto_aim_thread_running = false;
    // 等待自动瞄准线程完全退出
    while (!auto_aim_thread_finished.load());
    ULOG_INFO_TAG("AutoAim", "Auto aim thread stopped");
}