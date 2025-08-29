/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-29 19:12:27
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
#include "video.hpp"  
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
    std::string videoPath = "";
    std::string mode = "camera"; // 默认为相机模式
    double videoFps = 30.0; // 视频播放FPS
    
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
        
        // 读取模式配置
        if (config["mode"]) {
            mode = config["mode"].as<std::string>("camera");
        }
        
        // 读取视频路径和FPS（如果模式是video）
        if (config["video"]) {
            if (config["video"]["path"]) {
                videoPath = config["video"]["path"].as<std::string>("");
            }
            if (config["video"]["fps"]) {
                videoFps = config["video"]["fps"].as<double>(30.0);
            }
        }
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("Camera", "Failed to load camera config: %s", e.what());
    };
    
    // 创建图像发布者
    Publisher<CameraFrame> imagePublisher("camera/image");
    
    bool initSuccess = false;
    
    // 根据配置选择模式
    if (mode == "video" && !videoPath.empty()) {
        // 视频模式
        ULOG_INFO_TAG("Camera", "使用视频模式，视频路径: %s, FPS: %f", videoPath.c_str(), videoFps);
        rm_utils::VideoPlayer videoPlayer;
        // 设置视频播放FPS
        videoPlayer.setTargetFPS(videoFps);
        initSuccess = videoPlayer.open(videoPath);
        
        if (initSuccess) {
            ULOG_INFO_TAG("Camera", "视频初始化成功");
            
            // 如果启用了录制，创建Recorder实例
            if (recordEnabled) {
                recorder = std::make_unique<rm_utils::Recorder>(recordFps);
                ULOG_INFO_TAG("Camera", "Recorder initialized with FPS: %f", recordFps);
            }
            
            while (running.load() && camera_thread_running.load()) {
                cv::Mat frame;
                if (videoPlayer.read(frame)) {
                    auto timestamp = std::chrono::steady_clock::now();
                    // 创建带时间戳的帧
                    CameraFrame timeCameraFrame{frame, timestamp};
                    // 发布图像消息到PubSub系统
                    imagePublisher.publish(timeCameraFrame);
                    
                    // 如果启用了录制，进行录制
                    if (recordEnabled && recorder) {
                        recorder->record(frame);
                    }
                    
                    if (displayEnabled) {
                        // 显示图像
                        cv::Mat resizedDrawingFrame;
                        cv::resize(frame, resizedDrawingFrame, cv::Size(frame.cols/2, frame.rows/2), 0, 0, cv::INTER_LINEAR);
                        cv::imshow("Camera", resizedDrawingFrame);
                    }
                } else {
                    // 视频读取失败
                    ULOG_WARNING_TAG("Camera", "无法读取视频帧");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
    } else {
        // 相机模式（默认）
        ULOG_INFO_TAG("Camera", "使用相机模式");
        
        // 在相机线程内部创建相机对象，传入读取到的参数
        hikcamera::HikCamera cam(exposure_time, gain);
        initSuccess = cam.openCamera();
        
        // 初始化相机
        if (initSuccess) {
            ULOG_INFO_TAG("Camera","相机初始化成功");

            // 如果启用了录制，创建Recorder实例
            if (recordEnabled) {
                recorder = std::make_unique<rm_utils::Recorder>(recordFps);
                ULOG_INFO_TAG("Camera", "Recorder initialized with FPS: %f", recordFps);
            }

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
                    
                    if (displayEnabled) {
                        // 显示图像
                        cv::Mat resizedDrawingFrame;
                        cv::resize(frame, resizedDrawingFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
                        cv::imshow("Camera", resizedDrawingFrame);
                        cv::waitKey(1);
                    }
                }
            }
            
            cam.closeCamera();
        }
    }
    
    if (!initSuccess) {
        ULOG_ERROR_TAG("Camera","Failed to initialize camera or open video");
        running = false;
        camera_thread_finished = true; // 标记线程完成
        return;
    }
    
    // 重置recorder指针，确保正确析构
    recorder.reset();
    
    if (displayEnabled) {
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
    while (!camera_thread_finished.load());
    ULOG_INFO_TAG("Camera", "Camera thread stopped");
}