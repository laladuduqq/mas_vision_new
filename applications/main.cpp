/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 16:17:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-30 23:05:24
 * @FilePath: /mas_vision_new/applications/main.cpp
 * @Description: 
 */
#include "ulog.hpp"
#include <thread>
#include <atomic>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <string> 


// 声明线程函数
// camera
void startCameraThread();
void stopCameraThread();
// serial
void startSerialThread();
void stopSerialThread();
// auto aim
void startAutoAimThread();
void stopAutoAimThread();

// 声明校准函数
int runCalibration();
int runHandeyeCalibration();
int runWorldHandEyeCalibration();

std::atomic<bool> running(true);

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
    else if (argc > 1 && std::string(argv[1]) == "handeye") {
        return runHandeyeCalibration();
    }
    else if (argc > 1 && std::string(argv[1]) == "worldhandeye") {
        return runWorldHandEyeCalibration();
    }

    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 初始化ulog日志系统
    ULOG_INIT_CONSOLE_AND_FILE(ULOG_INFO_LEVEL, ULOG_TRACE_LEVEL);

    // 启动相机线程
    startCameraThread();

    // 启动串口线程
    startSerialThread();

    // 启动自动瞄准线程
    startAutoAimThread();

    ULOG_INFO_TAG("main","Application started.");

    // 主循环
    while (running.load()) {
        // 主循环可以处理其他任务
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    // 停止自动瞄准线程
    stopAutoAimThread();

    // 停止串口线程
    stopSerialThread();

    // 停止相机线程
    stopCameraThread();

    ULOG_INFO_TAG("main","Application exiting");

    // 反初始化ulog，关闭日志文件
    ULOG_DEINIT();
    return 0;
}