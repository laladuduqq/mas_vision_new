/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 22:47:18
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
    
    // 在相机线程内部创建相机对象
    hikcamera::HikCamera cam;
    try {
        cv::FileStorage fs("config/camera_set.json", cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["display"] >> displayEnabled;
            
            // 读取录制配置
            cv::FileNode recordNode = fs["record"];
            if (!recordNode.empty()) {
                recordNode["enabled"] >> recordEnabled;
                recordNode["fps"] >> recordFps;
            }
            
            fs.release();
        }
    } catch (const cv::Exception& e) {
        ULOG_ERROR_TAG("Camera", "Failed to load camera config: %s", e.what());
    };
    
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