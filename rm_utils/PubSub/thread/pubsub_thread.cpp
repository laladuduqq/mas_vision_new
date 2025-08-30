/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 22:13:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-30 21:46:54
 * @FilePath: /mas_vision_new/rm_utils/PubSub/thread/pubsub_thread.cpp
 * @Description: 消息中心线程实现
 */

#include "pubsub.hpp"
#include <thread>
#include "ulog.hpp"

// 运行标志
extern std::atomic<bool> running;

// 消息中心线程函数
void runMessageCenter() {
    MessageCenter& center = MessageCenter::getInstance();
    center.start();
    
    ULOG_INFO_TAG("PubSub", "Message center thread started");
    
    // 持续处理消息直到停止
    while (running.load() && center.isRunning()) {
        center.processMessages();
    }
    
    ULOG_INFO_TAG("PubSub", "Message center thread stopped");
}

// 启动消息中心线程
void startPubSubThread() {
    static std::thread message_center_thread(runMessageCenter);
    // 分离线程，让它独立运行
    message_center_thread.detach();
    ULOG_INFO_TAG("PubSub", "Message center thread launched");
}

// 停止消息中心
void stopPubSubThread() {
    MessageCenter::getInstance().stop();
    ULOG_INFO_TAG("PubSub", "Message center stopped");
}