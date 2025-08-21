/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 22:13:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 08:50:56
 * @FilePath: /mas_vision/rm_utils/PubSub/thread/pubsub_thread.cpp
 * @Description: 消息中心线程实现
 */

#include "pubsub.hpp"
#include <thread>
#include "performance_monitor.hpp"


// 性能监控器实例
extern mas_utils::PerformanceMonitor perfMonitor;
// 运行标志
extern std::atomic<bool> running;

// 消息中心线程函数
void runMessageCenter() {
    perfMonitor.addThread("MessageCenter", perfMonitor.getThreadsId());
    MessageCenter& center = MessageCenter::getInstance();
    center.start();
    
    // 持续处理消息直到停止
    while (running.load() && center.isRunning()) {
        center.processMessages();
    }
}

// 启动消息中心线程
void startPubSubThread() {
    static std::thread message_center_thread(runMessageCenter);
    // 分离线程，让它独立运行
    message_center_thread.detach();
}

// 停止消息中心
void stopPubSubThread() {
    MessageCenter::getInstance().stop();
}