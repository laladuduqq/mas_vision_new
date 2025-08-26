/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-26 17:09:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-26 21:26:34
 * @FilePath: /mas_vision_new/applications/calibration/calibrate_worldhandeye.cpp
 * @Description: 
 */

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <atomic>
#include <thread>
#include <chrono>
#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp之前包含
#include <opencv2/core/eigen.hpp>

#include "HikCamera.h"
#include "performance_monitor.hpp"
#include "ulog.hpp"
#include "pubsub.hpp"
#include "serial.h"
#include "rmmath.hpp"
#include "serial_types.hpp"

// 声明外部变量
extern std::atomic<bool> running;
extern mas_utils::PerformanceMonitor perfMonitor;

// 声明线程函数
void startPubSubThread();
void stopPubSubThread();
void startCameraThread();
void stopCameraThread();
void startSerialThread();
void stopSerialThread();

// 定义内部变量
static CameraFrame SubImage;
static std::atomic<bool> imagesubReady(false);
static std::atomic<bool> pattern_found(false);

struct GimbalData {
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    std::chrono::steady_clock::time_point timestamp;
};

static GimbalData current_gimbal_data;
static std::atomic<bool> gimbal_data_ready(false);

static void processWorldHandeyeImage(const CameraFrame& frame) {
    if (!frame.frame.empty())
    {
        SubImage.frame = frame.frame.clone();
        SubImage.timestamp = frame.timestamp;
        imagesubReady = true;
    }
}

// 串口数据处理回调函数
static void worldhandeyeGimbalDataCallback(const ReceivedDataMsg& data) {
    current_gimbal_data.yaw = data.yaw;
    current_gimbal_data.pitch = data.pitch;
    current_gimbal_data.roll = data.roll;
    current_gimbal_data.timestamp = data.timestamp;
    gimbal_data_ready = true;
}

class WorldHandEyeCalibrator {
public:
    WorldHandEyeCalibrator(const std::string& config_path) 
        : config_path_(config_path), capturing_(false), image_count_(0) {
        // 读取配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 获取标定板参数
        pattern_size_.width = config["calibration"]["board_width"].as<int>(9);
        pattern_size_.height = config["calibration"]["board_height"].as<int>(6);
        square_size_ = config["calibration"]["square_size"].as<float>(1.0);
        
        min_images_ = config["calibration"]["min_images"].as<int>(10);
        max_images_ = config["calibration"]["max_images"].as<int>(250);
        
        // 创建保存图像的目录
        image_save_path_ = "worldhandeye_calibration_images";
        std::filesystem::create_directories(image_save_path_);
        
        ULOG_INFO_TAG("WorldHandEyeCalibrator", "WorldHandEyeCalibrator initialized with board size %dx%d", 
                     pattern_size_.width, pattern_size_.height);
    }
    
    void runInteractiveCalibration() {
        std::cout << "世界手眼标定程序\n";
        std::cout << "================\n";
        std::cout << "1. 开始捕获标定图像\n";
        std::cout << "2. 执行世界手眼标定\n";
        std::cout << "3. 退出\n";
        
        int choice;
        while (true) {
            std::cout << "\n请选择操作: ";
            std::cin >> choice;
            
            switch (choice) {
                case 1:
                    captureImages();
                    break;
                case 2:
                    if (calibrate()) {
                        std::cout << "标定完成！结果已保存到配置文件。\n";
                    } else {
                        std::cout << "标定失败，请检查图像质量和云台数据。\n";
                    }
                    std::cout << "\n按任意键返回主菜单...";
                    cv::destroyAllWindows();
                    cv::waitKey(0); // 等待用户按键
                    break;
                case 3:
                    return;
                default:
                    std::cout << "无效选择，请重新输入。\n";
            }
        }
    }

private:
    void captureImages() {
        std::cout << "开始捕获标定图像...\n";
        std::cout << "请将棋盘格放置在相机视野内的不同位置和角度\n";
        std::cout << "确保云台处于不同姿态\n";
        std::cout << "按回车键继续...\n";
        std::cin.get(); // 等待用户输入

        // 启动PubSub消息中心线程
        startPubSubThread();

        // 启动相机线程
        startCameraThread();
        
        // 启动串口线程
        startSerialThread();
        
        capturing_ = true;
        image_count_ = 0;
        pattern_found = false;
        gimbal_data_ready = false;
        
        // 创建订阅者
        Subscriber subscriber;

        // 订阅相机图像
        subscriber.subscribe<CameraFrame>("camera/image", [](const CameraFrame& frame) {
            processWorldHandeyeImage(frame);
        }, DeliveryMode::COPY);
        
        // 订阅云台数据
        subscriber.subscribe<ReceivedDataMsg>("serial/data", worldhandeyeGimbalDataCallback, DeliveryMode::COPY);
        
        ULOG_INFO_TAG("WorldHandEyeCalibrator", "Started capturing chessboard images");
        std::cout << "正在捕获图像。在显示窗口中按 'c' 捕获图像，按 'q' 停止捕获\n";

        int key = 0;
        cv::Mat last_frame;
        GimbalData last_gimbal_data;
        
        // 处理帧的循环
        while (capturing_ && image_count_ < max_images_ && key != 27 && key != 'q' && key != 'Q') {
            if (imagesubReady.load()) {
                // 检测棋盘格
                std::vector<cv::Point2f> corners;
                pattern_found = cv::findChessboardCorners(
                    SubImage.frame, pattern_size_, corners, 
                    cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
                
                // 保存当前帧用于可能的捕获操作
                last_frame = SubImage.frame.clone();
                
                processFrame(SubImage.frame);
                imagesubReady = false;
            }
            
            // 更新云台数据
            if (gimbal_data_ready.load()) {
                last_gimbal_data = current_gimbal_data;
            }
            
            // 处理按键事件
            key = cv::waitKey(1) & 0xFF;
            if (key == 'c' || key == 'C') {
                // 检查是否检测到棋盘格并且还有空间存储图像
                if (pattern_found && image_count_ < max_images_ && gimbal_data_ready) {
                    // 保存图像
                    std::string filename = fmt::format("{}/image_{:03d}.jpg", image_save_path_, static_cast<int>(image_count_));
                    cv::imwrite(filename, last_frame);
                    
                    // 保存云台姿态数据
                    std::string data_filename = fmt::format("{}/gimbal_{:03d}.txt", image_save_path_, static_cast<int>(image_count_));
                    saveGimbalData(data_filename, last_gimbal_data);
                    
                    image_count_++;
                    ULOG_INFO_TAG("WorldHandEyeCalibrator", "Captured image %d/%d with gimbal data", static_cast<int>(image_count_), max_images_);
                } else if (!pattern_found) {
                    ULOG_INFO_TAG("WorldHandEyeCalibrator", "未检测到棋盘格，无法捕获图像");
                } else if (!gimbal_data_ready) {
                    ULOG_INFO_TAG("WorldHandEyeCalibrator", "未收到云台数据，无法捕获图像");
                }
            }
        }
        
        capturing_ = false;
        cv::destroyAllWindows();
        
        // 停止各线程
        stopCameraThread();
        stopSerialThread();
        stopPubSubThread();

        ULOG_INFO_TAG("WorldHandEyeCalibrator", "图像捕获完成，共捕获 %d张图像", static_cast<int>(image_count_));
    }
    
    void saveGimbalData(const std::string& filename, const GimbalData& data) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << data.yaw << " " << data.pitch << " " << data.roll << "\n";
            file.close();
        }
    }
    
    GimbalData readGimbalData(const std::string& filename) {
        GimbalData data;
        std::ifstream file(filename);
        if (file.is_open()) {
            file >> data.yaw >> data.pitch >> data.roll;
            file.close();
        }
        return data;
    }
    
    void processFrame(const cv::Mat& frame) {
        if (!capturing_ || frame.empty()) return;
        
        // 检测棋盘格
        std::vector<cv::Point2f> corners;
        bool pattern_found_local = cv::findChessboardCorners(
            frame, pattern_size_, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        
        // 绘制检测结果
        cv::Mat display_frame = frame.clone();
        cv::drawChessboardCorners(display_frame, pattern_size_, corners, pattern_found_local);
        
        // 显示状态
        std::string status = pattern_found_local ? "Pattern Found" : "Pattern Not Found";
        cv::putText(display_frame, status, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1, 
                   pattern_found_local ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        
        std::string gimbal_status = gimbal_data_ready ? "Gimbal Data Ready" : "Gimbal Data Not Ready";
        cv::putText(display_frame, gimbal_status, cv::Point(10, 70), 
                   cv::FONT_HERSHEY_SIMPLEX, 1, 
                   gimbal_data_ready ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        
        cv::putText(display_frame, fmt::format("Images: {}/{}", static_cast<int>(image_count_), min_images_), 
                   cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        cv::putText(display_frame, "Press 'c' to capture, 'q' to quit", 
                   cv::Point(10, display_frame.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        
        cv::imshow("World HandEye Calibration", display_frame);
    }
    
    std::vector<cv::Point3f> generateObjectPoints() {
        std::vector<cv::Point3f> obj;
        for (int i = 0; i < pattern_size_.height; i++) {
            for (int j = 0; j < pattern_size_.width; j++) {
                // 以中心点为原点的坐标系
                float x = 0;
                float y = (-j + 0.5 * pattern_size_.width) * square_size_;
                float z = (-i + 0.5 * pattern_size_.height) * square_size_;
                obj.emplace_back(x, y, z);
            }
        }
        return obj;
    }
    
    bool calibrate() {
        ULOG_INFO_TAG("WorldHandEyeCalibrator", "开始执行世界手眼标定...");
        
        // 收集图像和角点
        std::vector<std::vector<cv::Point2f>> image_points;
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<GimbalData> gimbal_data_list;
        cv::Size image_size;
        
        // 生成对象点（棋盘格在世界坐标系中的坐标）
        std::vector<cv::Point3f> obj = generateObjectPoints();
        
        // 读取保存的图像
        int valid_image_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(image_save_path_)) {
            if (entry.path().extension() == ".jpg") {
                std::string image_filename = entry.path().string();
                std::string index_str = entry.path().stem().string().substr(6); // 去掉"image_"前缀
                std::string data_filename = image_save_path_ + "/gimbal_" + index_str + ".txt";
                
                cv::Mat image = cv::imread(image_filename);
                if (image.empty()) continue;
                
                if (valid_image_count == 0) {
                    image_size = image.size();
                }
                
                std::vector<cv::Point2f> corners;
                bool found = cv::findChessboardCorners(image, pattern_size_, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
                
                // 检查是否有对应的云台数据文件
                std::ifstream data_file(data_filename);
                bool data_exists = data_file.good();
                data_file.close();
                
                if (found && data_exists) {
                    // 精细化角点位置
                    cv::Mat gray;
                    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                    
                    image_points.push_back(corners);
                    object_points.push_back(obj);
                    gimbal_data_list.push_back(readGimbalData(data_filename));
                    valid_image_count++;
                    ULOG_INFO_TAG("WorldHandEyeCalibrator", "Processed image %s with gimbal data", 
                                 entry.path().filename().c_str());
                }
            }
        }
        
        if (image_points.size() < 4) {
            ULOG_ERROR_TAG("WorldHandEyeCalibrator", "Not enough valid images for calibration (%d < 4)", 
                          static_cast<int>(image_points.size()));
            return false;
        }
        
        ULOG_INFO_TAG("WorldHandEyeCalibrator", "使用 %d 张图像进行标定", static_cast<int>(image_points.size()));
        
        // 读取相机参数
        YAML::Node config = YAML::LoadFile(config_path_);
        std::vector<double> camera_matrix_data = config["calibration"]["camera_matrix"].as<std::vector<double>>();
        std::vector<double> distort_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
        
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
        cv::Mat dist_coeffs = cv::Mat(distort_coeffs_data.size(), 1, CV_64F, distort_coeffs_data.data()).clone();
        
        // 执行PnP计算每张图像的位姿
        std::vector<cv::Mat> rvecs, tvecs;
        
        for (size_t i = 0; i < image_points.size(); i++) {
            cv::Mat rvec, tvec;
            cv::solvePnP(object_points[i], image_points[i], camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
            rvecs.push_back(rvec);
            tvecs.push_back(tvec);
        }
        
        // 计算云台相对于世界坐标系的位姿
        std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
        
        // 读取云台到IMU本体的旋转矩阵
        std::vector<double> R_gimbal2imubody_data = config["calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
        Eigen::Matrix3d R_gimbal2imubody = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_gimbal2imubody_data.data());
        
        for (size_t i = 0; i < gimbal_data_list.size(); i++) {
            // 将欧拉角转换为四元数
            Eigen::Vector3d ypr(gimbal_data_list[i].yaw, gimbal_data_list[i].pitch, gimbal_data_list[i].roll);
            Eigen::Matrix3d R_gimbal = rm_utils::rotation_matrix(ypr);
            
            // 计算云台的旋转矩阵（相对于世界坐标系）
            Eigen::Matrix3d R_gimbal2world = R_gimbal;
            Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
            
            // 转换为OpenCV格式
            cv::Mat R_world2gimbal_cv;
            cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
            
            R_world2gimbal_list.push_back(R_world2gimbal_cv);
            t_world2gimbal_list.push_back((cv::Mat_<double>(3, 1) << 0, 0, 0)); // 平移设为0
        }
        
        // 执行世界手眼标定
        cv::Mat R_world2board, t_world2board;
        cv::Mat R_gimbal2camera, t_gimbal2camera;
        cv::calibrateRobotWorldHandEye(
            rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, 
            R_world2board, t_world2board, R_gimbal2camera, t_gimbal2camera);
        
        t_gimbal2camera /= 1e3;  // mm to m
        t_world2board /= 1e3;    // mm to m
        
        // 计算相机相对于云台的变换
        cv::Mat R_camera2gimbal, t_camera2gimbal;
        cv::transpose(R_gimbal2camera, R_camera2gimbal);
        t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
        
        // 计算相机同理想情况的偏角
        Eigen::Matrix3d R_camera2gimbal_eigen;
        cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
        Eigen::Matrix3d R_gimbal2ideal;
        R_gimbal2ideal << 0, -1, 0, 0, 0, -1, 1, 0, 0;
        Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
        Eigen::Vector3d ypr = rm_utils::eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree
        
        // 计算标定板到世界坐标系原点的水平距离
        auto x = t_world2board.at<double>(0);
        auto y = t_world2board.at<double>(1);
        auto distance = std::sqrt(x * x + y * y);
        
        // 计算标定板同竖直摆放时的偏角
        Eigen::Matrix3d R_board2world_eigen;
        cv::cv2eigen(R_world2board.t(), R_board2world_eigen); // 转置得到board2world
        Eigen::Vector3d board_ypr = rm_utils::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;  // degree
        
        // 保存标定结果到配置文件
        saveCalibrationResults(R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, ypr, distance, board_ypr);
        
        return true;
    }
    
    void saveCalibrationResults(
        const std::vector<double>& R_gimbal2imubody_data, 
        const cv::Mat& R_camera2gimbal, 
        const cv::Mat& t_camera2gimbal,
        const Eigen::Vector3d& camera_ypr,
        double distance,
        const Eigen::Vector3d& board_ypr) {
        
        // 读取现有配置
        YAML::Node config = YAML::LoadFile(config_path_);
        
        // 准备相机到云台的变换数据
        std::vector<double> R_camera2gimbal_data(
            R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
        std::vector<double> t_camera2gimbal_data(
            t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());
        
        // 更新配置，使用Flow格式确保带方括号
        YAML::Node R_gimbal2imubody_node(R_gimbal2imubody_data);
        YAML::Node R_camera2gimbal_node(R_camera2gimbal_data);
        YAML::Node t_camera2gimbal_node(t_camera2gimbal_data);
        
        R_gimbal2imubody_node.SetStyle(YAML::EmitterStyle::Flow);
        R_camera2gimbal_node.SetStyle(YAML::EmitterStyle::Flow);
        t_camera2gimbal_node.SetStyle(YAML::EmitterStyle::Flow);
        
        config["calibration"]["R_gimbal2imubody"] = R_gimbal2imubody_node;
        config["calibration"]["R_camera2gimbal"] = R_camera2gimbal_node;
        config["calibration"]["t_camera2gimbal"] = t_camera2gimbal_node;
        
        // 写入文件
        std::ofstream fout(config_path_);
        fout << config;
        fout.close();
        
        ULOG_INFO_TAG("WorldHandEyeCalibrator", "Calibration results saved to %s", config_path_.c_str());
        
        // 打印结果
        std::cout << "\n云台到IMU本体旋转矩阵:\n";
        for (size_t i = 0; i < R_gimbal2imubody_data.size(); i += 3) {
            std::cout << "[ ";
            for (int j = 0; j < 3; j++) {
                std::cout << std::fixed << std::setprecision(6) << std::setw(10) 
                         << R_gimbal2imubody_data[i + j] << " ";
            }
            std::cout << "]\n";
        }
        
        std::cout << "\n相机到云台旋转矩阵:\n";
        for (int i = 0; i < R_camera2gimbal.rows; i++) {
            std::cout << "[ ";
            for (int j = 0; j < R_camera2gimbal.cols; j++) {
                std::cout << std::fixed << std::setprecision(6) << std::setw(10) 
                         << R_camera2gimbal.at<double>(i, j) << " ";
            }
            std::cout << "]\n";
        }
        
        std::cout << "\n相机到云台平移向量 (米):\n[ ";
        for (int i = 0; i < t_camera2gimbal.rows; i++) {
            std::cout << std::fixed << std::setprecision(6) << std::setw(10) 
                     << t_camera2gimbal.at<double>(i, 0) << " ";
        }
        std::cout << "]\n";
        
        std::cout << "\n相机同理想情况的偏角 (度):\n";
        std::cout << "yaw: " << std::fixed << std::setprecision(2) << camera_ypr[0] << "°\n";
        std::cout << "pitch: " << std::fixed << std::setprecision(2) << camera_ypr[1] << "°\n";
        std::cout << "roll: " << std::fixed << std::setprecision(2) << camera_ypr[2] << "°\n";
        
        std::cout << "\n标定板到世界坐标系原点的水平距离: " << std::fixed << std::setprecision(2) << distance << " m\n";
        
        std::cout << "\n标定板同竖直摆放时的偏角 (度):\n";
        std::cout << "yaw: " << std::fixed << std::setprecision(2) << board_ypr[0] << "°\n";
        std::cout << "pitch: " << std::fixed << std::setprecision(2) << board_ypr[1] << "°\n";
        std::cout << "roll: " << std::fixed << std::setprecision(2) << board_ypr[2] << "°\n";
    }
    
    std::string config_path_;
    cv::Size pattern_size_;
    float square_size_;
    int min_images_;
    int max_images_;
    std::string image_save_path_;
    
    std::atomic<bool> capturing_;
    std::atomic<int> image_count_;
};

int runWorldHandEyeCalibration() {
    running = true; // 确保running标志设置为true
    ULOG_INIT_CONSOLE_AND_FILE(ULOG_INFO_LEVEL, ULOG_TRACE_LEVEL);
    ULOG_INFO_TAG("main", "World HandEye calibration started");
    
    try {
        WorldHandEyeCalibrator calibrator("config/camera_set.yaml");
        calibrator.runInteractiveCalibration();
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("main", "Exception occurred: %s", e.what());
        std::cout << "发生错误: " << e.what() << "\n";
        return -1;
    }
    
    ULOG_INFO_TAG("main", "World HandEye calibration program finished");
    ULOG_DEINIT();
    return 0;
}