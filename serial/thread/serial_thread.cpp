#include "serial.h"
#include <thread>
#include <atomic>
#include <chrono>
#include "ulog.hpp"
#include "pubsub.hpp"
#include "serial_types.hpp"
#include "yaml-cpp/yaml.h"
#include <mutex>

// 全局变量
extern std::atomic<bool> running;


static std::thread serial_thread;
static std::unique_ptr<serial::Serial> serial_port = nullptr;
static std::atomic<bool> serial_thread_running(false);


// 串口数据缓存类
class SerialDataBuffer {
public:
    struct DataPoint {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
        double yaw, pitch, roll;
        int mode;
        
        DataPoint() : q(Eigen::Quaterniond::Identity()), timestamp(std::chrono::steady_clock::now()), 
                      yaw(0), pitch(0), roll(0), mode(0) {}
                      
        DataPoint(const ReceivedDataMsg& msg) 
            : q(eulerToQuaternion(msg.yaw, msg.pitch, msg.roll)), 
              timestamp(msg.timestamp),
              yaw(msg.yaw), pitch(msg.pitch), roll(msg.roll),
              mode(msg.mode) {}
        
    private:
        Eigen::Quaterniond eulerToQuaternion(double yaw, double pitch, double roll) {
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
            return q.normalized();
        }
    };

private:
    std::deque<DataPoint> buffer_;
    static constexpr size_t MAX_BUFFER_SIZE = 100;
    mutable std::mutex buffer_mutex_;

public:
    void addData(const ReceivedDataMsg& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        DataPoint point(msg);
        buffer_.push_back(point);
        
        // 保持缓冲区大小在限制范围内
        if (buffer_.size() > MAX_BUFFER_SIZE) {
            buffer_.pop_front();
        }
    }
    
    Eigen::Quaterniond getDataAt(std::chrono::steady_clock::time_point timestamp) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        if (buffer_.empty()) {
            return Eigen::Quaterniond::Identity();
        }
        
        // 如果时间戳早于第一个数据点，返回第一个数据点的姿态
        if (timestamp <= buffer_.front().timestamp) {
            return buffer_.front().q;
        }
        
        // 如果时间戳晚于最后一个数据点，返回最后一个数据点的姿态
        if (timestamp >= buffer_.back().timestamp) {
            return buffer_.back().q;
        }
        
        // 查找时间戳附近的两个数据点
        DataPoint data_ahead, data_behind;
        bool found = false;
        
        for (const auto& data : buffer_) {
            if (data.timestamp >= timestamp) {
                data_behind = data;
                found = true;
                break;
            }
            data_ahead = data;
        }
        
        // 如果没有找到合适的数据点，返回单位四元数
        if (!found) {
            return Eigen::Quaterniond::Identity();
        }
        
        // 四元数球面线性插值(Slerp)
        Eigen::Quaterniond q_a = data_ahead.q.normalized();
        Eigen::Quaterniond q_b = data_behind.q.normalized();
        auto t_a = data_ahead.timestamp;
        auto t_b = data_behind.timestamp;
        
        // 避免除零错误
        if (t_b <= t_a) {
            return q_b;
        }
        
        std::chrono::duration<double> t_ab = t_b - t_a;
        std::chrono::duration<double> t_ac = timestamp - t_a;
        
        // 插值参数
        double k = t_ac.count() / t_ab.count();
        k = std::max(0.0, std::min(1.0, k)); // 限制在[0,1]范围内
        
        Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
        return q_c;
    }
    
    bool isEmpty() const {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return buffer_.empty();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return buffer_.size();
    }
};

// 串口数据缓存实例
static SerialDataBuffer serial_data_buffer;

Eigen::Quaterniond getSerialDataAt(std::chrono::steady_clock::time_point timestamp) {
    return serial_data_buffer.getDataAt(timestamp);
}

// 虚拟串口参数
struct VirtualSerialParams {
    int update_rate;           // Hz
    float yaw;       // 度
    float pitch;     // 度
    float roll;      // 度
    int mode;       // 模式
};

// 串口线程函数 - 物理串口模式
void serialThreadFunc() {
    // 读取YAML配置
    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 115200;
    int timeout = 1000;
    serial::bytesize_t bytesize = serial::eightbits;
    serial::parity_t parity = serial::parity_none;
    serial::stopbits_t stopbits = serial::stopbits_one;
    serial::flowcontrol_t flowcontrol = serial::flowcontrol_none;
    size_t packet_size = sizeof(ReceivePacket);
    
    try {
        YAML::Node config = YAML::LoadFile("config/serial_config.yaml");
        if (config["serial"]) {
            if (config["serial"]["port"]) 
                port = config["serial"]["port"].as<std::string>("/dev/ttyUSB0");
            if (config["serial"]["baudrate"]) 
                baudrate = config["serial"]["baudrate"].as<uint32_t>(115200);
            if (config["serial"]["timeout"]) 
                timeout = config["serial"]["timeout"].as<int>(1000);
            if (config["serial"]["bytesize"]) {
                int bs = config["serial"]["bytesize"].as<int>(8);
                switch(bs) {
                    case 5: bytesize = serial::fivebits; break;
                    case 6: bytesize = serial::sixbits; break;
                    case 7: bytesize = serial::sevenbits; break;
                    case 8: bytesize = serial::eightbits; break;
                    default: bytesize = serial::eightbits; break;
                }
            }
            if (config["serial"]["parity"]) {
                std::string p = config["serial"]["parity"].as<std::string>("none");
                if (p == "none") parity = serial::parity_none;
                else if (p == "odd") parity = serial::parity_odd;
                else if (p == "even") parity = serial::parity_even;
                else parity = serial::parity_none;
            }
            if (config["serial"]["stopbits"]) {
                int sb = config["serial"]["stopbits"].as<int>(1);
                if (sb == 1) stopbits = serial::stopbits_one;
                else if (sb == 2) stopbits = serial::stopbits_two;
                else stopbits = serial::stopbits_one;
            }
            if (config["serial"]["flowcontrol"]) {
                std::string fc = config["serial"]["flowcontrol"].as<std::string>("none");
                if (fc == "none") flowcontrol = serial::flowcontrol_none;
                else if (fc == "software") flowcontrol = serial::flowcontrol_software;
                else if (fc == "hardware") flowcontrol = serial::flowcontrol_hardware;
                else flowcontrol = serial::flowcontrol_none;
            }
        }
    } catch (const std::exception& e) {
        ULOG_WARNING_TAG("Serial", "Failed to load serial config, using defaults: %s", e.what());
    }
    
    // 初始化串口
    try {
        serial_port = std::make_unique<serial::Serial>(
            port, baudrate, serial::Timeout::simpleTimeout(timeout), 
            bytesize, parity, stopbits, flowcontrol);
        
        if (serial_port->isOpen()) {
            ULOG_INFO_TAG("Serial", "Serial port opened successfully: %s @ %d", 
                         port.c_str(), baudrate);
        } else {
            ULOG_ERROR_TAG("Serial", "Failed to open serial port: %s", port.c_str());
            return;
        }
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("Serial", "Exception while opening serial port: %s", e.what());
        return;
    }
    
    // 创建发布者
    Publisher<ReceivedDataMsg> serialDataPublisher("serial/data");
    
    // 读取数据循环
    std::vector<uint8_t> buffer(packet_size);
    
    while (running.load() && serial_thread_running.load() && serial_port->isOpen()) {
        try {
            // 读取数据包
            size_t bytes_read = serial_port->read(buffer, packet_size);
            
            // 检查数据包大小
            if (bytes_read == packet_size) {
                // 检查数据包头和尾
                ReceivePacket* packet = reinterpret_cast<ReceivePacket*>(buffer.data());
                
                if (packet->header == 0xAA && packet->tail == 0x5A) {
                    // 创建带时间戳的消息
                    ReceivedDataMsg msg;
                    msg.yaw = static_cast<double>(packet->yaw)     * M_PI / 180.0;
                    msg.pitch = static_cast<double>(packet->pitch) * M_PI / 180.0;
                    msg.roll = static_cast<double>(packet->roll)   * M_PI / 180.0;
                    msg.mode = packet->mode;
                    msg.timestamp = std::chrono::steady_clock::now();

                    // 添加到数据缓存
                    serial_data_buffer.addData(msg);
                    
                    // 发布消息
                    serialDataPublisher.publish(msg);
                    
                    ULOG_DEBUG_TAG("Serial", "Received valid packet - Yaw: %.2f, Pitch: %.2f, Roll: %.2f, Mode: %d", 
                                  msg.yaw, msg.pitch, msg.roll, msg.mode);
                } else {
                    ULOG_WARNING_TAG("Serial", "Invalid packet header or tail. Header: 0x%02X, Tail: 0x%02X", 
                                    packet->header, packet->tail);
                }
            } else if (bytes_read > 0) {
                ULOG_WARNING_TAG("Serial", "Incomplete packet received: %d bytes, expected %d", 
                                bytes_read, packet_size);
            }
        } catch (const std::exception& e) {
            ULOG_ERROR_TAG("Serial", "Exception while reading from serial port: %s", e.what());
            // 等待一点时间再继续
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // 关闭串口
    if (serial_port && serial_port->isOpen()) {
        serial_port->close();
        ULOG_INFO_TAG("Serial", "Serial port closed");
    }
}

// 虚拟串口线程函数
void virtualSerialThreadFunc() {    
    // 读取虚拟串口参数
    VirtualSerialParams params;
    params.update_rate = 100;
    params.yaw = 0.0f;
    params.pitch = 0.0f;
    params.roll= 0.0f;
    params.mode = 0;
    
    try {
        YAML::Node config = YAML::LoadFile("config/serial_config.yaml");
        if (config["serial"] && config["serial"]["virtual_params"]) {
            auto virtual_params = config["serial"]["virtual_params"];
            if (virtual_params["update_rate"])
                params.update_rate = virtual_params["update_rate"].as<int>(100);
            if (virtual_params["yaw"])
                params.yaw = virtual_params["yaw"].as<float>(0.0f);
            if (virtual_params["pitch"])
                params.pitch= virtual_params["pitch"].as<float>(0.0f);
            if (virtual_params["roll"])
                params.roll= virtual_params["roll"].as<float>(0.0f);
            if (virtual_params["mode"])
                params.mode= virtual_params["mode"].as<int>(0);
        }
    } catch (const std::exception& e) {
        ULOG_WARNING_TAG("Serial", "Failed to load virtual serial config, using defaults: %s", e.what());
    }
    
    ULOG_INFO_TAG("Serial", "Virtual serial mode enabled with update rate: %d Hz", params.update_rate);
    
    // 创建发布者
    Publisher<ReceivedDataMsg> serialDataPublisher("serial/data");
    
    auto start_time = std::chrono::steady_clock::now();
    int loop_count = 0;
    
    // 生成虚拟数据循环
    while (running.load() && serial_thread_running.load()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();
        
        
        // 生成虚拟数据
        float time_sec = static_cast<float>(loop_count) / params.update_rate;
        
        ReceivedDataMsg msg;
        msg.yaw = params.yaw     * M_PI / 180.0;
        msg.pitch = params.pitch * M_PI / 180.0;
        msg.roll = params.roll   * M_PI / 180.0;
        msg.mode = params.mode;
        msg.timestamp = std::chrono::steady_clock::now();

        // 添加到数据缓存
        serial_data_buffer.addData(msg);
        
        // 发布消息
        serialDataPublisher.publish(msg);
        
        loop_count++;
        
        // 控制更新频率
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1000 / params.update_rate));
    }
}

// 启动串口线程
void startSerialThread() {
    serial_thread_running = true;
    
    // 检查是否启用虚拟串口模式
    bool virtual_mode = false;
    try {
        YAML::Node config = YAML::LoadFile("config/serial_config.yaml");
        if (config["serial"] && config["serial"]["virtual"]) {
            virtual_mode = config["serial"]["virtual"].as<bool>(false);
        }
    } catch (const std::exception& e) {
        ULOG_WARNING_TAG("Serial", "Failed to read virtual mode config, using physical serial: %s", e.what());
        virtual_mode = false;
    }
    
    // 根据模式启动相应的线程

    if (virtual_mode) {
        ULOG_INFO_TAG("Serial", "Starting virtual serial thread");
        serial_thread = std::thread(virtualSerialThreadFunc);
    } else {
        ULOG_INFO_TAG("Serial", "Starting physical serial thread");
        serial_thread = std::thread(serialThreadFunc);
    }
    ULOG_INFO_TAG("Serial", "Serial thread started");
}

// 停止串口线程
void stopSerialThread() {
    if (!serial_thread_running) {
        ULOG_WARNING_TAG("Serial", "Serial thread not running");
        return;
    }
    serial_thread_running = false;
    // 等待串口线程退出
    if (serial_thread.joinable()) {
        serial_thread.join();
    }
    ULOG_INFO_TAG("Serial", "Serial thread stopped");
}