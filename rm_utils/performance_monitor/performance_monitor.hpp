/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 20:25:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 22:25:05
 * @FilePath: /mas_vision_new/rm_utils/performance_monitor/performance_monitor.hpp
 * @Description: 
 */
#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <mutex>
#include <map>

#ifdef __linux__
#include <fstream>
#endif

namespace mas_utils {

struct ThreadInfo {
    std::string name;
    int threadId;
    float cpuUsage;
    ThreadInfo(const std::string& n, int tid = 0) : name(n), threadId(tid), cpuUsage(0.0f) {}
};

struct CoreInfo {
    int coreId;
    float usage;
    CoreInfo(int id) : coreId(id), usage(0.0f) {}
};

class PerformanceMonitor {
public:
    PerformanceMonitor();
    ~PerformanceMonitor();

    // 启动监控线程
    void startMonitoring();
    
    // 停止监控线程
    void stopMonitoring();
    
    // 添加要监控的线程
    void addThread(const std::string& threadName, int threadId = 0);
    
    // 显示性能监控窗口
    void showPerformanceWindow();

    long getThreadsId();

private:
    std::atomic<bool> monitoring_;
    std::thread monitorThread_;
    std::mutex threadsMutex_;
    
    // 线程信息
    std::vector<ThreadInfo> threads_;
    
    // CPU核心信息
    std::vector<CoreInfo> cores_;
    
    // 系统资源使用情况
    float cpuUsage_;
    float memoryUsage_;
    float diskUsage_;
    
    // 进程内存使用情况 (MB)
    float processMemory_;
    
    // OpenCV窗口相关
    cv::Mat perfFrame_;
    bool showWindow_;
    
#ifdef __linux__
    // Linux特定的辅助函数
    std::vector<std::string> readLinesFromFile(const std::string& filename);
    void updateSystemUsage();
    void updateThreadUsage();
    void updateCoreUsage();
#endif

    // 监控线程主函数
    void monitorLoop();
    
    // 绘制性能图表
    void drawPerformanceChart();
};

} // namespace mas_utils

#endif // PERFORMANCE_MONITOR_H