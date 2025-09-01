/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-29 21:00:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-01 11:59:24
 * @FilePath: /mas_vision_new/rm_utils/udp/udp_comm.cpp
 * @Description: 基于asio的UDP通信工具类实现
 */
#include "udp_comm.hpp"
#include "ulog.hpp"
#include <vector>
#include <chrono>

namespace rm_utils {

UDPClient::UDPClient(const std::string& target_ip, int image_port, int message_port)
    : target_ip_(target_ip)
    , image_port_(image_port)
    , message_port_(message_port)
    , initialized_(false)
    , shutdown_(false) {
    ULOG_DEBUG_TAG("UDPClient", "UDPClient created with target_ip: %s, image_port: %d, message_port: %d", 
                   target_ip.c_str(), image_port, message_port);
}

UDPClient::~UDPClient() {
    shutdown();
    ULOG_DEBUG_TAG("UDPClient", "UDPClient destroyed");
}

bool UDPClient::init() {
    try {
        // 创建io_context
        io_context_ = std::make_unique<asio::io_context>();
        
        // 创建图像传输套接字
        image_socket_ = std::make_unique<asio::ip::udp::socket>(*io_context_);
        image_socket_->open(asio::ip::udp::v4());
        
        // 创建消息传输套接字
        message_socket_ = std::make_unique<asio::ip::udp::socket>(*io_context_);
        message_socket_->open(asio::ip::udp::v4());
        
        // 设置目标端点
        asio::ip::address target_address = asio::ip::make_address(target_ip_);
        image_endpoint_ = asio::ip::udp::endpoint(target_address, image_port_);
        message_endpoint_ = asio::ip::udp::endpoint(target_address, message_port_);
        
        initialized_ = true;
        shutdown_ = false;
        ULOG_INFO_TAG("UDPClient", "UDP sockets initialized successfully");
        return true;
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("UDPClient", "Failed to initialize UDP sockets: %s", e.what());
        close();
        return false;
    }
}

void UDPClient::close() {
    try {
        if (image_socket_ && image_socket_->is_open()) {
            image_socket_->close();
        }
        
        if (message_socket_ && message_socket_->is_open()) {
            message_socket_->close();
        }
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("UDPClient", "Error closing UDP sockets: %s", e.what());
    }
    
    image_socket_.reset();
    message_socket_.reset();
    io_context_.reset();
    initialized_ = false;
    ULOG_INFO_TAG("UDPClient", "UDP sockets closed");
}

void UDPClient::shutdown() {
    if (shutdown_) {
        return; // 已经关闭了
    }
    
    shutdown_ = true;
    
    // 发送退出消息通知接收端
    if (initialized_) {
        std::string exit_message = "exit:Client shutting down";
        try {
            // 使用互斥锁保护消息套接字操作
            std::lock_guard<std::mutex> lock(message_socket_mutex_);
            if (message_socket_ && message_socket_->is_open()) {
                message_socket_->send_to(asio::buffer(exit_message), message_endpoint_);
            }
        } catch (const std::exception& e) {
            ULOG_ERROR_TAG("UDPClient", "Failed to send exit message: %s", e.what());
        }
    }
    
    close();
    ULOG_INFO_TAG("UDPClient", "UDP client shutdown completed");
}

std::vector<char> UDPClient::createPacket(const std::string& header, const std::vector<char>& data) {
    // 创建带有头部的数据包
    // 格式: [header_length(4字节)][header][data]
    uint32_t header_length = static_cast<uint32_t>(header.length());
    
    std::vector<char> packet(sizeof(uint32_t) + header_length + data.size());
    char* packet_ptr = packet.data();
    
    // 复制头部长度
    memcpy(packet_ptr, &header_length, sizeof(uint32_t));
    packet_ptr += sizeof(uint32_t);
    
    // 复制头部
    if (header_length > 0) {
        memcpy(packet_ptr, header.c_str(), header_length);
        packet_ptr += header_length;
    }
    
    // 复制数据
    if (!data.empty()) {
        memcpy(packet_ptr, data.data(), data.size());
    }
    
    return packet;
}

bool UDPClient::canSendImage(const std::string& header) const {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    auto it = last_send_times_.find(header);
    
    if (it == last_send_times_.end()) {
        // 第一次发送该header的图像，允许发送
        return true;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);
    
    // 检查是否超过了最小帧间隔时间
    return elapsed >= MIN_FRAME_TIME;
}

void UDPClient::updateSendTime(const std::string& header) const {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    last_send_times_[header] = std::chrono::steady_clock::now();
}

bool UDPClient::sendImage(const cv::Mat& image, const std::string& header, int quality) {
    if (!initialized_ || image.empty() || shutdown_) {
        return false;
    }

    // 检查发送频率限制
    if (!canSendImage(header)) {
        return false; // 跳过发送以维持频率限制
    }

    try {
        // 将图像编码为JPEG格式以减小数据量
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};

        cv::imencode(".jpg", image, buffer, params);

        // 检查数据包大小是否超过UDP限制(通常为65507字节)
        uint32_t header_length = static_cast<uint32_t>(header.length());
        if (buffer.size() > 65507 - sizeof(uint32_t) - header_length) {
            ULOG_WARNING_TAG("UDPClient", "Image data size %d bytes exceeds UDP limit, consider reducing quality", 
                           static_cast<int>(buffer.size()));
            return false;
        }

        // 转换为char向量
        std::vector<char> image_data(buffer.begin(), buffer.end());
        
        // 创建带头部的数据包
        std::vector<char> packet = createPacket(header, image_data);

        // 使用互斥锁保护图像套接字操作
        std::lock_guard<std::mutex> lock(image_socket_mutex_);
        if (shutdown_) return false;
        
        // 设置发送超时时间，避免无限期阻塞
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(image_socket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        
        // 使用带错误码的发送方式，避免抛出异常导致线程无法退出
        asio::error_code error;
        image_socket_->send_to(asio::buffer(packet), image_endpoint_, 0, error);
        
        if (error) {
            ULOG_ERROR_TAG("UDPClient", "Failed to send image data: %s", error.message().c_str());
            return false;
        }
        
        // 更新发送时间以进行频率控制
        updateSendTime(header);
        
        ULOG_DEBUG_TAG("UDPClient", "Sent image data: %d bytes (with header: %s)", static_cast<int>(packet.size()), header.c_str());
        return true;
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("UDPClient", "Failed to send image data: %s", e.what());
        return false;
    }
}

bool UDPClient::sendMessage(const std::string& message, const std::string& header) {
    if (!initialized_ || message.empty() || shutdown_) {
        return false;
    }

    try {
        // 将消息转换为char向量
        std::vector<char> message_data(message.begin(), message.end());
        
        // 创建带头部的数据包
        std::vector<char> packet = createPacket(header, message_data);

        // 使用互斥锁保护消息套接字操作
        std::lock_guard<std::mutex> lock(message_socket_mutex_);
        if (shutdown_) return false;
        
        // 设置发送超时时间，避免无限期阻塞
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(message_socket_->native_handle(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        
        // 使用带错误码的发送方式，避免抛出异常导致线程无法退出
        asio::error_code error;
        message_socket_->send_to(asio::buffer(packet), message_endpoint_, 0, error);
        
        if (error) {
            ULOG_ERROR_TAG("UDPClient", "Failed to send message data: %s", error.message().c_str());
            return false;
        }
        
        ULOG_DEBUG_TAG("UDPClient", "Sent message with header '%s': %s", header.c_str(), message.c_str());
        return true;
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("UDPClient", "Failed to send message data: %s", e.what());
        return false;
    }
}

} // namespace rm_utils