#include "performance_monitor.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <sys/syscall.h>
#include "ulog.hpp"

#ifdef __linux__
#include <unistd.h>
#include <sys/statvfs.h>
#include <dirent.h>
#include <cstring>
#endif

mas_utils::PerformanceMonitor::PerformanceMonitor() 
    : monitoring_(false)
    , cpuUsage_(0.0f)
    , memoryUsage_(0.0f)
    , diskUsage_(0.0f)
    , processMemory_(0.0f)
    , perfFrame_(600, 800, CV_8UC3, cv::Scalar(0, 0, 0))
    , showWindow_(true) {
    ULOG_DEBUG_TAG("PerformanceMonitor", "PerformanceMonitor created");
}

mas_utils::PerformanceMonitor::~PerformanceMonitor() {
    ULOG_DEBUG_TAG("PerformanceMonitor", "PerformanceMonitor destroyed");
    stopMonitoring();
}

void mas_utils::PerformanceMonitor::startMonitoring() {
    if (!monitoring_) {
        monitoring_ = true;
        monitorThread_ = std::thread(&PerformanceMonitor::monitorLoop, this);
        ULOG_INFO_TAG("PerformanceMonitor", "Performance monitoring started");
    } else {
        ULOG_WARNING_TAG("PerformanceMonitor", "Performance monitoring already running");
    }
}

void mas_utils::PerformanceMonitor::stopMonitoring() {
    if (monitoring_) {
        ULOG_INFO_TAG("PerformanceMonitor", "Stopping performance monitoring");
        monitoring_ = false;
        if (monitorThread_.joinable()) {
            monitorThread_.join();
            ULOG_DEBUG_TAG("PerformanceMonitor", "Performance monitoring thread joined");
        }
    } else {
        ULOG_WARNING_TAG("PerformanceMonitor", "Performance monitoring not running");
    }
}

void mas_utils::PerformanceMonitor::addThread(const std::string& threadName, int threadId) {
    std::lock_guard<std::mutex> lock(threadsMutex_);
    threads_.emplace_back(threadName, threadId);
    ULOG_DEBUG_TAG("PerformanceMonitor", "Added thread to monitor: %s (ID: %d)", threadName.c_str(), threadId);
}

long mas_utils::PerformanceMonitor::getThreadsId() { 
    long tid = syscall(SYS_gettid);
    ULOG_DEBUG_TAG("PerformanceMonitor", "Getting thread ID: %ld", tid);
    return tid;
}

void mas_utils::PerformanceMonitor::showPerformanceWindow() {
    if (showWindow_) {
        cv::imshow("Performance Monitor", perfFrame_);
        // 如果按下ESC键，关闭窗口
        int key = cv::waitKey(1);
        if (key == 27) {
            ULOG_INFO_TAG("PerformanceMonitor", "Performance window closed by user (ESC key)");
            showWindow_ = false;
            cv::destroyWindow("Performance Monitor");
        }
    } else {
        // 如果窗口被关闭，按任意键重新打开
        int key = cv::waitKey(1);
        if (key != -1) {
            ULOG_INFO_TAG("PerformanceMonitor", "Performance window reopened by user");
            showWindow_ = true;
            cv::namedWindow("Performance Monitor");
        }
    }
}

void mas_utils::PerformanceMonitor::monitorLoop() {
    ULOG_INFO_TAG("PerformanceMonitor", "Performance monitor loop started");
    while (monitoring_) {
#ifdef __linux__
        updateSystemUsage();
        updateCoreUsage();
        updateThreadUsage();
        drawPerformanceChart();
#endif
        // 每200毫秒更新一次
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    ULOG_INFO_TAG("PerformanceMonitor", "Performance monitor loop stopped");
}

#ifdef __linux__
std::vector<std::string> mas_utils::PerformanceMonitor::readLinesFromFile(const std::string& filename) {
    std::vector<std::string> lines;
    std::ifstream file(filename);
    std::string line;
    
    if (file.is_open()) {
        while (std::getline(file, line)) {
            lines.push_back(line);
        }
        file.close();
    } else {
        ULOG_WARNING_TAG("PerformanceMonitor", "Failed to open file: %s", filename.c_str());
    }
    
    return lines;
}

void mas_utils::PerformanceMonitor::updateSystemUsage() {
    // 更新CPU使用率
    static unsigned long long lastTotal = 0;
    static unsigned long long lastIdle = 0;
    
    std::vector<std::string> lines = readLinesFromFile("/proc/stat");
    if (!lines.empty()) {
        std::istringstream iss(lines[0]);
        std::string cpuLabel;
        unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
        
        iss >> cpuLabel >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
        
        unsigned long long currentIdle = idle + iowait;
        unsigned long long currentTotal = user + nice + system + idle + iowait + irq + softirq + steal;
        
        unsigned long long totalDiff = currentTotal - lastTotal;
        unsigned long long idleDiff = currentIdle - lastIdle;
        
        if (totalDiff > 0) {
            cpuUsage_ = 100.0f * (totalDiff - idleDiff) / totalDiff;
        }
        
        lastTotal = currentTotal;
        lastIdle = currentIdle;
    }
    
    // 更新内存使用率
    std::vector<std::string> memLines = readLinesFromFile("/proc/meminfo");
    unsigned long memTotal = 0;
    unsigned long memFree = 0;
    unsigned long memBuffers = 0;
    unsigned long memCached = 0;
    unsigned long memAvailable = 0;
    
    for (const std::string& line : memLines) {
        std::istringstream iss(line);
        std::string key;
        unsigned long value;
        
        iss >> key >> value;
        
        if (key == "MemTotal:") {
            memTotal = value;
        } else if (key == "MemFree:") {
            memFree = value;
        } else if (key == "MemAvailable:") {
            memAvailable = value;
        } else if (key == "Buffers:") {
            memBuffers = value;
        } else if (key == "Cached:") {
            memCached = value;
        }
    }
    
    if (memTotal > 0) {
        unsigned long actualFree = memFree + memBuffers + memCached;
        memoryUsage_ = 100.0f * (memTotal - memAvailable) / memTotal;
    }
    
    // 更新磁盘使用率
    struct statvfs buf;
    if (statvfs(".", &buf) == 0) {
        unsigned long long totalBlocks = buf.f_blocks;
        unsigned long long freeBlocks = buf.f_bfree;
        
        if (totalBlocks > 0) {
            diskUsage_ = 100.0f * (totalBlocks - freeBlocks) / totalBlocks;
        }
    }
    
    // 更新进程内存使用情况
    std::string statusFile = "/proc/self/status";
    std::vector<std::string> statusLines = readLinesFromFile(statusFile);
    for (const std::string& line : statusLines) {
        if (line.substr(0, 6) == "VmRSS:") {
            std::istringstream iss(line);
            std::string key;
            unsigned long value;
            iss >> key >> value;
            processMemory_ = value / 1024.0f; // 转换为MB
            break;
        }
    }
    
    ULOG_DEBUG_TAG("PerformanceMonitor", "System usage - CPU: %.1f%%, Memory: %.1f%%, Disk: %.1f%%, Process Memory: %.1f MB", 
                   cpuUsage_, memoryUsage_, diskUsage_, processMemory_);
}

void mas_utils::PerformanceMonitor::updateCoreUsage() {
    static std::map<int, unsigned long long> lastCoreTotal;
    static std::map<int, unsigned long long> lastCoreIdle;
    
    std::vector<std::string> lines = readLinesFromFile("/proc/stat");
    
    // 确保cores_大小正确
    int cpuCoreCount = std::thread::hardware_concurrency();
    if (cores_.size() != cpuCoreCount) {
        cores_.clear();
        for (int i = 0; i < cpuCoreCount; ++i) {
            cores_.emplace_back(i);
        }
        ULOG_DEBUG_TAG("PerformanceMonitor", "Adjusted core count to %d", cpuCoreCount);
    }
    
    // 处理每个CPU核心行（跳过第一行总CPU）
    for (size_t i = 1; i < lines.size(); ++i) {
        if (lines[i].substr(0, 3) != "cpu") break;
        
        std::istringstream iss(lines[i]);
        std::string cpuLabel;
        unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
        
        iss >> cpuLabel >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
        
        // 提取核心编号
        int coreId = std::stoi(cpuLabel.substr(3));
        if (coreId >= (int)cores_.size()) continue;
        
        unsigned long long currentIdle = idle + iowait;
        unsigned long long currentTotal = user + nice + system + idle + iowait + irq + softirq + steal;
        
        auto lastTotalIt = lastCoreTotal.find(coreId);
        auto lastIdleIt = lastCoreIdle.find(coreId);
        
        if (lastTotalIt != lastCoreTotal.end() && lastIdleIt != lastCoreIdle.end()) {
            unsigned long long totalDiff = currentTotal - lastTotalIt->second;
            unsigned long long idleDiff = currentIdle - lastIdleIt->second;
            
            if (totalDiff > 0) {
                cores_[coreId].usage = 100.0f * (totalDiff - idleDiff) / totalDiff;
            }
        }
        
        lastCoreTotal[coreId] = currentTotal;
        lastCoreIdle[coreId] = currentIdle;
    }
}

void mas_utils::PerformanceMonitor::updateThreadUsage() {
    static std::map<int, unsigned long long> lastThreadTimes;
    static std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
    
    auto now = std::chrono::steady_clock::now();
    auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate).count();
    lastUpdate = now;
    
    if (timeDiff == 0) timeDiff = 200; // 默认更新间隔为200毫秒
    
    // 获取当前进程的所有线程ID
    DIR* dir = opendir("/proc/self/task");
    if (!dir) {
        ULOG_ERROR_TAG("PerformanceMonitor", "Failed to open /proc/self/task directory");
        return;
    }
    
    struct dirent* entry;
    std::set<int> currentThreadIds;
    
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] != '.') {
            try {
                int tid = std::stoi(entry->d_name);
                currentThreadIds.insert(tid);
            } catch (...) {
                // 忽略无法转换的条目
            }
        }
    }
    closedir(dir);
    
    // 更新线程信息
    std::lock_guard<std::mutex> lock(threadsMutex_);
    for (auto& thread : threads_) {
        // 如果没有指定线程ID，则尝试匹配
        int actualTid = thread.threadId;
        if (actualTid == 0) {
            // 如果未设置线程ID，尝试使用当前存在的线程ID
            if (!currentThreadIds.empty()) {
                actualTid = *currentThreadIds.begin();
                thread.threadId = actualTid;
            }
        }
        
        // 检查线程ID是否有效
        if (actualTid <= 0 || currentThreadIds.find(actualTid) == currentThreadIds.end()) {
            thread.cpuUsage = 0.0f;
            continue;
        }
        
        // 为每个线程读取stat文件获取CPU时间
        std::string statPath = "/proc/self/task/" + std::to_string(actualTid) + "/stat";
        std::vector<std::string> statLines = readLinesFromFile(statPath);
        
        if (!statLines.empty()) {
            try {
                // 解析stat文件获取utime和stime
                std::istringstream iss(statLines[0]);
                std::vector<std::string> tokens;
                std::string token;
                
                while (iss >> token) {
                    tokens.push_back(token);
                }
                
                // utime是第14个字段，stime是第15个字段（从0开始索引则是13和14）
                if (tokens.size() > 15) {
                    unsigned long utime = std::stoul(tokens[13]);
                    unsigned long stime = std::stoul(tokens[14]);
                    unsigned long long totalTime = utime + stime;
                    
                    // 计算CPU使用率
                    auto lastTimeIt = lastThreadTimes.find(actualTid);
                    if (lastTimeIt != lastThreadTimes.end()) {
                        unsigned long long timeDiffTicks = totalTime - lastTimeIt->second;
                        // sysconf(_SC_CLK_TCK)通常是100，表示每秒100个时钟周期
                        long clockTicksPerSec = sysconf(_SC_CLK_TCK);
                        // CPU使用率 = (时间差/时钟周期) / (时间间隔/1000) * 100%
                        thread.cpuUsage = (timeDiffTicks * 1000.0f / clockTicksPerSec) / timeDiff * 100.0f;
                        // 限制最大值为100%
                        thread.cpuUsage = std::min(thread.cpuUsage, 100.0f);
                    }
                    
                    lastThreadTimes[actualTid] = totalTime;
                } else {
                    thread.cpuUsage = 0.0f;
                }
            } catch (...) {
                ULOG_WARNING_TAG("PerformanceMonitor", "Failed to parse stat file for thread %d", actualTid);
                thread.cpuUsage = 0.0f;
            }
        } else {
            thread.cpuUsage = 0.0f;
        }
    }
}
#endif

void mas_utils::PerformanceMonitor::drawPerformanceChart() {
    // 计算需要的窗口高度
    int baseHeight = 400; // 基础高度（系统信息、CPU核心信息等）
    int coreHeight = 0;
    int threadHeight = 0;
    
    // 计算CPU核心信息所需高度
    if (!cores_.empty()) {
        int coreRows = (cores_.size() + 3) / 4; // 每行显示4个核心
        coreHeight = coreRows * 30 + 60; // 60是标题和间距
    }
    
    // 计算线程信息所需高度
    {
        std::lock_guard<std::mutex> lock(threadsMutex_);
        threadHeight = 60 + (int)threads_.size() * 25; // 60是标题和间距，每个线程25像素
    }
    
    // 计算总高度（至少600像素）
    int totalHeight = std::max(600, baseHeight + coreHeight + threadHeight);
    
    // 如果窗口大小不够，重新创建
    if (perfFrame_.rows < totalHeight) {
        perfFrame_ = cv::Mat(totalHeight, 800, CV_8UC3, cv::Scalar(30, 30, 30));
        ULOG_DEBUG_TAG("PerformanceMonitor", "Resized performance frame to %d x 800", totalHeight);
    } else {
        perfFrame_ = cv::Scalar(30, 30, 30); // 深灰色背景
    }
    
    // 绘制标题
    cv::putText(perfFrame_, "System Performance Monitor", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
    
    // 绘制系统资源使用情况
    int startY = 60;
    int barHeight = 25;
    int barWidth = 300;
    
    // CPU使用率
    cv::putText(perfFrame_, "Total CPU Usage:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + barWidth, startY + barHeight/2), 
                  cv::Scalar(80, 80, 80), -1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + (int)(barWidth * cpuUsage_ / 100.0), startY + barHeight/2), 
                  cv::Scalar(0, 0, 255), -1);
    cv::putText(perfFrame_, cv::format("%.1f%%", cpuUsage_), 
                cv::Point(210 + barWidth, startY + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // 内存使用率
    startY += 40;
    cv::putText(perfFrame_, "Memory Usage:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + barWidth, startY + barHeight/2), 
                  cv::Scalar(80, 80, 80), -1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + (int)(barWidth * memoryUsage_ / 100.0), startY + barHeight/2), 
                  cv::Scalar(0, 255, 0), -1);
    cv::putText(perfFrame_, cv::format("%.1f%%", memoryUsage_), 
                cv::Point(210 + barWidth, startY + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // 磁盘使用率
    startY += 40;
    cv::putText(perfFrame_, "Disk Usage:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + barWidth, startY + barHeight/2), 
                  cv::Scalar(80, 80, 80), -1);
    cv::rectangle(perfFrame_, 
                  cv::Point(200, startY - barHeight/2), 
                  cv::Point(200 + (int)(barWidth * diskUsage_ / 100.0), startY + barHeight/2), 
                  cv::Scalar(255, 255, 0), -1);
    cv::putText(perfFrame_, cv::format("%.1f%%", diskUsage_), 
                cv::Point(210 + barWidth, startY + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // 进程内存使用情况
    startY += 40;
    cv::putText(perfFrame_, "Process Memory:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    cv::putText(perfFrame_, cv::format("%.1f MB", processMemory_), 
                cv::Point(210, startY + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // 绘制CPU核心信息标题
    startY += 60;
    cv::putText(perfFrame_, "CPU Cores Usage:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 255), 1);
    
    // 绘制CPU核心使用情况
    startY += 30;
    int coresPerRow = 4;
    int coreSpacing = 180;
    for (size_t i = 0; i < cores_.size(); ++i) {
        int row = i / coresPerRow;
        int col = i % coresPerRow;
        
        int x = 20 + col * coreSpacing;
        int y = startY + row * 30;
    
        
        cv::putText(perfFrame_, cv::format("Core %d:", cores_[i].coreId), cv::Point(x, y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // 绘制核心CPU使用率条
        cv::rectangle(perfFrame_, 
                      cv::Point(x + 70, y - barHeight/4), 
                      cv::Point(x + 70 + 80, y + barHeight/4), 
                      cv::Scalar(80, 80, 80), -1);
        cv::rectangle(perfFrame_, 
                      cv::Point(x + 70, y - barHeight/4), 
                      cv::Point(x + 70 + (int)(80 * cores_[i].usage / 100.0), y + barHeight/4), 
                      cv::Scalar(255, 100, 0), -1);
        cv::putText(perfFrame_, cv::format("%.0f%%", cores_[i].usage), 
                    cv::Point(x + 160, y + 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }
    
    // 更新起始Y位置
    int coreRows = (cores_.size() + coresPerRow - 1) / coresPerRow;
    startY += coreRows * 30 + 30;
    
    // 绘制线程信息标题
    cv::putText(perfFrame_, "Thread Information:", cv::Point(10, startY), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 255), 1);
    
    // 绘制线程列表
    {
        std::lock_guard<std::mutex> lock(threadsMutex_);
        startY += 30;
        for (const auto& thread : threads_) {
            // 移除了防止超出画面的检查，因为我们已经调整了窗口大小
            
            cv::putText(perfFrame_, thread.name, cv::Point(20, startY), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
            // 绘制线程CPU使用率条
            cv::rectangle(perfFrame_, 
                          cv::Point(150, startY - barHeight/4), 
                          cv::Point(150 + 100, startY + barHeight/4), 
                          cv::Scalar(80, 80, 80), -1);
            cv::rectangle(perfFrame_, 
                          cv::Point(150, startY - barHeight/4), 
                          cv::Point(150 + (int)(100 * thread.cpuUsage / 100.0), startY + barHeight/4), 
                          cv::Scalar(255, 100, 0), -1);
            cv::putText(perfFrame_, cv::format("%.1f%%", thread.cpuUsage), 
                        cv::Point(260, startY + 5), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            startY += 25;
        }
    }
    
    // 绘制帮助信息
    cv::putText(perfFrame_, "Press ESC to close window", cv::Point(10, perfFrame_.rows - 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
    cv::putText(perfFrame_, "Press any key to reopen", cv::Point(10, perfFrame_.rows - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
}
