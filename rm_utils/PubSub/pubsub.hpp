/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 21:58:01
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 00:06:37
 * @FilePath: /mas_vision/rm_utils/PubSub/pubsub.hpp
 * @Description: 
 */
#ifndef PUBSUB_HPP
#define PUBSUB_HPP

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <memory>
#include <atomic>
#include <chrono>

// 消息传输模式
enum class DeliveryMode {
    POINTER,  // 指针模式（引用计数）
    COPY      // 复制模式
};

// 消息主题
using Topic = std::string;

// 订阅者ID
using SubscriberID = uint64_t;

// 通用数据结构
struct DataPacket {
    void* data;
    size_t size;
    
    DataPacket(void* d, size_t s) : data(d), size(s) {}
};

// 消息处理回调函数
using MessageCallback = std::function<void(const DataPacket&)>;

// 订阅者信息
struct SubscriberInfo {
    MessageCallback callback;
    DeliveryMode mode;
    size_t data_size;
};

// 消息中心类
class MessageCenter {
public:
    static MessageCenter& getInstance();
    
    // 禁止拷贝
    MessageCenter(const MessageCenter&) = delete;
    MessageCenter& operator=(const MessageCenter&) = delete;
    
    // 启动和停止消息中心
    void start();
    void stop();
    
    // 检查是否正在运行
    bool isRunning() const { return running_; }
    
    // 发布消息
    void publish(const Topic& topic, const void* data, size_t size);
    
    // 订阅主题
    SubscriberID subscribe(const Topic& topic, size_t data_size, const MessageCallback& callback, DeliveryMode mode = DeliveryMode::COPY);
    
    // 取消订阅
    void unsubscribe(const Topic& topic, SubscriberID subscriber_id);
    
    // 处理消息队列
    void processMessages();
    
    // 批处理大小设置
    void setBatchSize(size_t size) { batch_size_ = size; }
    
    // 队列大小限制设置
    void setQueueSizeLimit(size_t limit) { queue_size_limit_ = limit; }

private:
    MessageCenter();
    ~MessageCenter();
    
    // 生成订阅者ID
    SubscriberID generateSubscriberID();
    
    // 消息队列项
    struct QueueItem {
        Topic topic;
        std::vector<char> data;
        std::vector<std::pair<SubscriberID, SubscriberInfo>> subscribers;
        
        QueueItem(const Topic& t, const void* d, size_t s,
                  const std::vector<std::pair<SubscriberID, SubscriberInfo>>& subs)
            : topic(t), data(static_cast<const char*>(d), static_cast<const char*>(d) + s), subscribers(subs) {}
    };
    
    std::unordered_map<Topic, std::unordered_map<SubscriberID, SubscriberInfo>> subscribers_;
    std::queue<QueueItem> message_queue_;
    std::mutex queue_mutex_;
    std::mutex subscribers_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> running_;
    std::atomic<SubscriberID> next_subscriber_id_;
    
    // 性能优化参数
    size_t batch_size_;
    size_t queue_size_limit_;
    
    // 性能监控
    std::atomic<uint64_t> messages_published_;
    std::atomic<uint64_t> messages_processed_;
};

// 发布者模板类
template<typename T>
class Publisher {
public:
    Publisher(const Topic& topic) : topic_(topic) {}
    
    void publish(const T& data) {
        MessageCenter::getInstance().publish(topic_, &data, sizeof(T));
    }
    
    void publish(const T* data) {
        MessageCenter::getInstance().publish(topic_, data, sizeof(T));
    }
    
private:
    Topic topic_;
};

// 订阅者类
class Subscriber {
public:
    Subscriber();
    ~Subscriber();
    
    // 订阅主题
    template<typename T>
    void subscribe(const Topic& topic, const std::function<void(const T&)>& callback, 
                   DeliveryMode mode = DeliveryMode::COPY) {
        auto wrapper_callback = [callback](const DataPacket& packet) {
            if (packet.size == sizeof(T)) {
                callback(*static_cast<const T*>(packet.data));
            }
        };
        
        SubscriberID id = MessageCenter::getInstance().subscribe(topic, sizeof(T), wrapper_callback, mode);
        subscription_ids_[topic] = id;
    }
    
    // 取消订阅
    void unsubscribe(const Topic& topic);

private:
    std::unordered_map<Topic, SubscriberID> subscription_ids_;
};

// PubSub线程相关函数
void runMessageCenter();
void startPubSubThread();
void stopPubSubThread();

#endif // PUBSUB_HPP