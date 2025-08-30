#ifndef RM_UTILS_TOPIC_QUEUE_HPP
#define RM_UTILS_TOPIC_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <atomic>
#include "ulog.hpp"

namespace rm_utils {

template<typename T>
class TopicQueue {
public:
    explicit TopicQueue(size_t max_size = 100) 
        : max_size_(max_size), running_(true) {}

    void push(const std::string& topic, const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!running_) {
            ULOG_WARNING_TAG("Queue", "Queue is stopped");
            return;
        }
        
        auto& queue = queues_[topic];
        if (queue.size() >= max_size_) {
            queue.pop();  // 如果队列满了，删除最旧的数据
            ULOG_DEBUG_TAG("Queue", "Queue for topic '%s' full, dropping oldest message", topic.c_str());
        }
        queue.push(value);
        lock.unlock();
        not_empty_.notify_all();
    }

    bool pop(const std::string& topic, T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        auto it = queues_.find(topic);
        if (it == queues_.end()) {
            return false;
        }
        
        auto& queue = it->second;
        while (queue.empty() && running_) {
            not_empty_.wait(lock);
            // 重新检查topic是否存在，因为可能在等待期间被删除
            it = queues_.find(topic);
            if (it == queues_.end()) {
                return false;
            }
            queue = it->second;
        }
        
        if (queue.empty()) {
            return false;
        }
        
        value = std::move(queue.front());
        queue.pop();
        return true;
    }

    bool try_pop(const std::string& topic, T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = queues_.find(topic);
        if (it == queues_.end() || it->second.empty()) {
            return false;
        }
        value = std::move(it->second.front());
        it->second.pop();
        return true;
    }

    bool empty(const std::string& topic) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = queues_.find(topic);
        return it == queues_.end() || it->second.empty();
    }

    size_t size(const std::string& topic) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = queues_.find(topic);
        return it == queues_.end() ? 0 : it->second.size();
    }

    void clear(const std::string& topic) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = queues_.find(topic);
        if (it != queues_.end()) {
            std::queue<T> empty;
            std::swap(it->second, empty);
        }
    }

    void clear_all() {
        std::lock_guard<std::mutex> lock(mutex_);
        queues_.clear();
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        not_empty_.notify_all();
    }

    void start() {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = true;
    }

private:
    std::unordered_map<std::string, std::queue<T>> queues_;
    const size_t max_size_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::atomic<bool> running_;
};

} // namespace rm_utils

#endif // RM_UTILS_TOPIC_QUEUE_HPP