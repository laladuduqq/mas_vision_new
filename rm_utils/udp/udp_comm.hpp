/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-29 21:00:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-31 11:38:44
 * @FilePath: /mas_vision_new/rm_utils/udp/udp_comm.hpp
 * @Description: 基于asio的UDP通信工具类
 */
#ifndef RM_UTILS_UDP_COMM_HPP
#define RM_UTILS_UDP_COMM_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <asio.hpp>
#include <memory>
#include <mutex>
#include <atomic>
#include <map>
#include <chrono>

namespace rm_utils {

class UDPClient {
public:
    /**
     * @brief 构造函数
     * @param target_ip 目标IP地址
     * @param image_port 图像传输端口
     * @param message_port 消息传输端口
     */
    UDPClient(const std::string& target_ip = "127.0.0.1", 
              int image_port = 9993, 
              int message_port = 9994);
    
    /**
     * @brief 析构函数
     */
    ~UDPClient();
    
    /**
     * @brief 初始化UDP套接字
     * @return 初始化是否成功
     */
    bool init();
    
    /**
     * @brief 关闭UDP套接字
     */
    void close();
    
    /**
     * @brief 退出UDP客户端
     */
    void shutdown();
    
    /**
     * @brief 发送图像数据
     * @param image 要发送的图像
     * @param header 数据包头部标识符
     * @param quality JPEG压缩质量(1-100)
     * @return 发送是否成功
     */
    bool sendImage(const cv::Mat& image, const std::string& header = "image", int quality = 80);
    
    /**
     * @brief 发送消息数据
     * @param message 要发送的消息
     * @param header 数据包头部标识符
     * @return 发送是否成功
     */
    bool sendMessage(const std::string& message, const std::string& header = "message");
    
    /**
     * @brief 检查UDP套接字是否已初始化
     * @return 是否已初始化
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief 检查UDP客户端是否已关闭
     * @return 是否已关闭
     */
    bool isShutdown() const { return shutdown_; }

private:
    std::string target_ip_;
    int image_port_;
    int message_port_;
    
    bool initialized_;
    std::atomic<bool> shutdown_;
    
    // 修改命名空间为asio而非boost::asio
    std::unique_ptr<asio::io_context> io_context_;
    std::unique_ptr<asio::ip::udp::socket> image_socket_;
    std::unique_ptr<asio::ip::udp::socket> message_socket_;
    asio::ip::udp::endpoint image_endpoint_;
    asio::ip::udp::endpoint message_endpoint_;
    
    // 添加互斥锁以确保线程安全
    mutable std::mutex image_socket_mutex_;
    mutable std::mutex message_socket_mutex_;
    
    // 图像发送频率控制
    mutable std::mutex timing_mutex_;
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_send_times_;
    static constexpr double MAX_IMAGE_FPS = 30.0;
    static constexpr std::chrono::milliseconds MIN_FRAME_TIME{static_cast<int>(1000.0 / MAX_IMAGE_FPS)};
    
    // 辅助方法：创建带头部的数据包
    std::vector<char> createPacket(const std::string& header, const std::vector<char>& data);
    
    // 辅助方法：检查是否可以发送图像（频率控制）
    bool canSendImage(const std::string& header) const;
    
    // 辅助方法：更新发送时间
    void updateSendTime(const std::string& header) const;
};

} // namespace rm_utils

#endif // RM_UTILS_UDP_COMM_HPP