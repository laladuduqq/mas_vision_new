/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 22:13:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 21:41:48
 * @FilePath: /mas_vision_new/rm_utils/PubSub/pubsub.cpp
 * @Description: 发布-订阅消息机制实现
 */

#include "pubsub.hpp"
#include <cstring>
#include "ulog.hpp"

// MessageCenter实现
MessageCenter& MessageCenter::getInstance() {
    static MessageCenter instance;
    return instance;
}

MessageCenter::MessageCenter() 
    : running_(false), next_subscriber_id_(1), messages_published_(0), messages_processed_(0),
      batch_size_(10), queue_size_limit_(100) {
    ULOG_DEBUG_TAG("PubSub", "MessageCenter created");
}

MessageCenter::~MessageCenter() {
    ULOG_DEBUG_TAG("PubSub", "MessageCenter destroyed");
    stop();
}

void MessageCenter::start() {
    if (running_) {
        ULOG_WARNING_TAG("PubSub", "MessageCenter already running");
        return;
    }
    
    running_ = true;
    messages_published_ = 0;
    messages_processed_ = 0;
    ULOG_INFO_TAG("PubSub", "MessageCenter started");
}

void MessageCenter::stop() {
    if (!running_) {
        ULOG_WARNING_TAG("PubSub", "MessageCenter not running");
        return;
    }
    
    // 先处理队列中剩余的所有消息
    ULOG_INFO_TAG("PubSub", "Processing remaining messages before stopping");
    while (!message_queue_.empty()) {
        processMessages();
    }
    
    // 然后停止消息中心
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        running_ = false;
        ULOG_INFO_TAG("PubSub", "MessageCenter stopped");
    }
    condition_.notify_all();
}

SubscriberID MessageCenter::generateSubscriberID() {
    return next_subscriber_id_++;
}

void MessageCenter::publish(const Topic& topic, const void* data, size_t size) {
    if (!running_) {
        ULOG_WARNING_TAG("PubSub", "MessageCenter not running, cannot publish to topic '%s'", topic.c_str());
        return;
    }
    
    // 增加发布计数
    messages_published_++;
    
    // 获取该主题的所有订阅者
    std::vector<std::pair<SubscriberID, SubscriberInfo>> topic_subscribers;
    {
        std::lock_guard<std::mutex> lock(subscribers_mutex_);
        auto it = subscribers_.find(topic);
        if (it != subscribers_.end()) {
            for (const auto& sub_pair : it->second) {
                // 检查数据大小是否匹配
                if (sub_pair.second.data_size == size) {
                    topic_subscribers.push_back(sub_pair);
                }
            }
        }
    }
    
    // 如果没有订阅者，直接返回
    if (topic_subscribers.empty()) {
        ULOG_DEBUG_TAG("PubSub", "No subscribers for topic '%s'", topic.c_str());
        return;
    }
    
    // 检查队列大小限制
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (message_queue_.size() >= queue_size_limit_) {
            // 队列满时丢弃最旧的消息
            message_queue_.pop();
            ULOG_WARNING_TAG("PubSub", "Message queue full, dropping oldest message for topic '%s'", topic.c_str());
        }
        message_queue_.emplace(topic, data, size, topic_subscribers);
    }
    
    ULOG_DEBUG_TAG("PubSub", "Published message to topic '%s', subscriber count: %lu", topic.c_str(), topic_subscribers.size());
    
    // 通知处理线程
    condition_.notify_one();
}

SubscriberID MessageCenter::subscribe(const Topic& topic, size_t data_size, const MessageCallback& callback, DeliveryMode mode) {
    SubscriberID id = generateSubscriberID();
    
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_[topic][id] = {callback, mode, data_size};
    
    ULOG_INFO_TAG("PubSub", "New subscription to topic '%s' with ID %lu, data size: %lu", topic.c_str(), id, data_size);
    
    return id;
}

void MessageCenter::unsubscribe(const Topic& topic, SubscriberID subscriber_id) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    auto it = subscribers_.find(topic);
    if (it != subscribers_.end()) {
        size_t erased = it->second.erase(subscriber_id);
        if (erased > 0) {
            ULOG_INFO_TAG("PubSub", "Unsubscribed from topic '%s' with ID %lu", topic.c_str(), subscriber_id);
        }
        
        if (it->second.empty()) {
            subscribers_.erase(it);
            ULOG_DEBUG_TAG("PubSub", "No more subscribers for topic '%s', removing topic", topic.c_str());
        }
    }
}

void MessageCenter::processMessages() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // 等待有消息或停止信号
    condition_.wait(lock, [this] { 
        return !message_queue_.empty() || !running_; 
    });
    
    // 批处理消息以提高性能
    size_t processed_count = 0;
    while (!message_queue_.empty() && processed_count < batch_size_) {
        QueueItem item = std::move(message_queue_.front());
        message_queue_.pop();
        processed_count++;

        lock.unlock(); // 解锁以允许其他线程添加消息
        
        // 处理每个订阅者
        for (const auto& sub_pair : item.subscribers) {
            const SubscriberInfo& info = sub_pair.second;
            
            switch (info.mode) {
                case DeliveryMode::POINTER:
                    // 指针模式：直接传递数据包
                    {
                        DataPacket packet(const_cast<char*>(item.data.data()), item.data.size());
                        info.callback(packet);
                    }
                    break;
                    
                case DeliveryMode::COPY:
                    // 复制模式：创建数据副本
                    {
                        std::vector<char> copied_data(item.data);
                        DataPacket packet(copied_data.data(), copied_data.size());
                        info.callback(packet);
                    }
                    break;
            }
        }
        
        messages_processed_++;
        ULOG_DEBUG_TAG("PubSub", "Processed message for topic '%s'", item.topic.c_str());
        lock.lock(); // 重新锁定以检查队列
    }
}

// Subscriber实现
Subscriber::Subscriber() {
    ULOG_DEBUG_TAG("PubSub", "Subscriber created");
}

Subscriber::~Subscriber() {
    // 取消所有订阅
    for (const auto& pair : subscription_ids_) {
        MessageCenter::getInstance().unsubscribe(pair.first, pair.second);
    }
    ULOG_DEBUG_TAG("PubSub", "Subscriber destroyed");
}

void Subscriber::unsubscribe(const Topic& topic) {
    auto it = subscription_ids_.find(topic);
    if (it != subscription_ids_.end()) {
        MessageCenter::getInstance().unsubscribe(topic, it->second);
        subscription_ids_.erase(it);
        ULOG_INFO_TAG("PubSub", "Unsubscribed from topic '%s'", topic.c_str());
    } else {
        ULOG_WARNING_TAG("PubSub", "Not subscribed to topic '%s'", topic.c_str());
    }
}