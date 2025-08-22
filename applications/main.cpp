/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 16:17:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-22 21:00:33
 * @FilePath: /mas_vision_new/applications/main.cpp
 * @Description: 
 */
#include "performance_monitor.hpp"
#include "pubsub.hpp"
#include "ulog.hpp"
#include <thread>
#include <atomic>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>



// 声明线程函数
// camera
void startCameraThread();
void stopCameraThread();
// pubsub
void startPubSubThread();
void stopPubSubThread();
// serial
void startSerialThread();
void stopSerialThread();

// 声明校准函数
int runCalibration();

std::atomic<bool> running(true);
// 性能监控器实例
mas_utils::PerformanceMonitor perfMonitor;

// 信号处理函数，用于退出
void signalHandler(int signum) {
    ULOG_WARNING("Interrupt signal (%d) received.", signum);
    running = false;
}

int main(int argc, char* argv[])
{
    // 检查命令行参数，如果提供了calibrate参数，则运行校准模式
    if (argc > 1 && std::string(argv[1]) == "calibrate") {
        return runCalibration();
    }

    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 初始化ulog日志系统
    ULOG_INIT_CONSOLE_AND_FILE(ULOG_INFO_LEVEL, ULOG_TRACE_LEVEL);

    // 配置性能监控器
    perfMonitor.addThread("Main Thread", perfMonitor.getThreadsId());
    
    // 启动性能监控
    perfMonitor.startMonitoring();
    
    // 启动PubSub消息中心线程
    startPubSubThread();

    // 启动相机线程
    startCameraThread();

    // 启动串口线程
    startSerialThread();

    ULOG_INFO_TAG("main","Application started.");

    // 主循环
    while (running.load()) {
        // 显示性能监控窗口
        perfMonitor.showPerformanceWindow();
        
        // 主循环可以处理其他任务
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // 停止串口线程
    stopSerialThread();

    // 停止相机线程
    stopCameraThread();

    // 停止PubSub消息中心
    stopPubSubThread();


    // 停止性能监控
    perfMonitor.stopMonitoring();


    ULOG_INFO_TAG("main","Application exiting");

    // 反初始化ulog，关闭日志文件
    ULOG_DEINIT();
    return 0;
}