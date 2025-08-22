/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-22 10:42:57
 * @FilePath: /mas_vision_new/hikcamera/thread/cam_thread.cpp
 * @Description:
 */
#include "HikCamera.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>
#include "pubsub.hpp"
#include "performance_monitor.hpp"
#include "ulog.hpp"
#include "recorder.hpp"
#include "yaml-cpp/yaml.h"

extern std::atomic<bool> running;
extern mas_utils::PerformanceMonitor perfMonitor;

static bool displayEnabled = false;
static std::atomic<bool> camera_thread_running(false);
static std::atomic<bool> camera_thread_finished(true); // 标记相机线程是否已完成
static bool recordEnabled = false;
static double recordFps = 30.0;
static std::unique_ptr<rm_utils::Recorder> recorder = nullptr;

// 相机线程函数
void cameraThreadFunc() {
    camera_thread_finished = false; // 标记线程开始运行
    // 注册性能监控
    perfMonitor.addThread("CameraThread", perfMonitor.getThreadsId());
    
    // 默认参数
    float exposure_time = 5000.0f;
    float gain = 10.0f;
    // 在相机线程中读取YAML配置文件
    try {
        YAML::Node config = YAML::LoadFile("config/camera_set.yaml");
        if (config["camera"]) {
            exposure_time = config["camera"]["exposuretime"].as<float>(5000.0f);
            gain = config["camera"]["gain"].as<float>(10.0f);
            ULOG_INFO_TAG("Camera","已加载相机参数");
        }
        
        // 读取显示配置
        if (config["display"]) {
            displayEnabled = config["display"].as<bool>(false);
        }
        
        // 读取录制配置
        if (config["record"]) {
            recordEnabled = config["record"]["enabled"].as<bool>(false);
            recordFps = config["record"]["fps"].as<double>(30.0);
        }
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("Camera", "Failed to load camera config: %s", e.what());
    };

    // 在相机线程内部创建相机对象，传入读取到的参数
    hikcamera::HikCamera cam(exposure_time, gain);
    
    // 初始化相机
    if (!cam.openCamera()) {
        ULOG_ERROR_TAG("Camera","Failed to initialize camera");
        running = false;
        camera_thread_finished = true; // 标记线程完成
        return;
    }
    
    ULOG_INFO_TAG("Camera","Camera initialized successfully");

    // 如果启用了录制，创建Recorder实例
    if (recordEnabled) {
        recorder = std::make_unique<rm_utils::Recorder>(recordFps);
        ULOG_INFO_TAG("Camera", "Recorder initialized with FPS: %f", recordFps);
    }

    // 创建图像发布者
    Publisher<CameraFrame> imagePublisher("camera/image");
    
    while (running.load() && camera_thread_running.load()) {
        cv::Mat frame;
        // 获取帧
        if (cam.grabImage(frame)) {
            auto timestamp = std::chrono::steady_clock::now();
            // 创建带时间戳的帧
            CameraFrame timeCameraFrame{frame, timestamp};
            // 发布图像消息到PubSub系统
            imagePublisher.publish(timeCameraFrame);
            
            // 如果启用了录制，进行录制
            if (recordEnabled && recorder) {
                recorder->record(frame);
            }
            
            if (displayEnabled)
            {
                // 显示图像
                cv::Mat resizedDrawingFrame;
                cv::resize(frame, resizedDrawingFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
                cv::imshow("Camera", resizedDrawingFrame);
                cv::waitKey(1);
            }
        } 
    }

    // 重置recorder指针，确保正确析构
    recorder.reset();
    cam.closeCamera();
    if (displayEnabled)
    {
        // 关闭所有OpenCV窗口
        cv::destroyAllWindows();
    }
    
    camera_thread_finished = true; // 标记线程完成
}

// 启动相机线程
void startCameraThread() {
    camera_thread_running = true;
    static std::thread camera_thread(cameraThreadFunc);
    // 分离线程，让它独立运行
    camera_thread.detach();
}

// 停止相机线程
void stopCameraThread() {
    camera_thread_running = false;
    // 等待相机线程完全退出
    while (!camera_thread_finished.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ULOG_INFO_TAG("Camera", "Camera thread stopped");
}