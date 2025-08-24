/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 23:15:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-24 23:15:59
 * @FilePath: /mas_vision_new/auto_aim/thread/auto_aim_thread.cpp
 * @Description: 自动瞄准线程实现
 */
#include "armor_detector/armor_detector.hpp"
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

    // FPS计算相关变量
    auto lastTime = std::chrono::steady_clock::now();
    int frameCount = 0;
    double fps = 0.0;
    // 处理时间计算
    double processTime = 0.0;
    
    // 主处理循环
    while (running.load() && auto_aim_thread_running.load()) {
        // 检查是否有新的图像数据
        if (frame_ready.load()) {
            // 重置帧就绪标志
            frame_ready = false;
            
            // 执行装甲板检测
            if (armor_detector && !latest_frame.frame.empty()) {
                auto processStartTime = std::chrono::steady_clock::now();
                // 检测装甲板
                auto armors = armor_detector->ArmorDetect(latest_frame.frame);
                cv::Mat display = armor_detector->showResult(latest_frame.frame);
                // 计算处理时间
                auto processEndTime = std::chrono::steady_clock::now();
                processTime = std::chrono::duration<double, std::milli>(processEndTime - processStartTime).count();
                // 计算FPS
                frameCount++;
                auto currentTime = std::chrono::steady_clock::now();
                double elapsedTime = std::chrono::duration<double>(currentTime - lastTime).count();
                if (elapsedTime >= 1.0) {
                    fps = frameCount / elapsedTime;
                    frameCount = 0;
                    lastTime = currentTime;
                }
                std::string fps_info = "FPS: " + std::to_string(static_cast<int>(fps));
                cv::putText(display,fps_info,cv::Point(10, 60),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0, 255, 0),1);
                std::string process_info = "Process Time: " + std::to_string(processTime) + " ms";
                cv::putText(display,process_info,cv::Point(10, 35),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0, 255, 0),1);
                cv::imshow("Armor Detection Result", display);
            }
        }
        
        // 如果有IMU数据，可以在这里处理
        if (imu_ready.load()) {
            // 重置IMU就绪标志
            imu_ready = false;
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