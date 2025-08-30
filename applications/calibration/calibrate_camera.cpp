#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <atomic>

#include "HikCamera.h"
#include "topicqueue.hpp"
#include "ulog.hpp"

// 声明外部变量
extern std::atomic<bool> running;

// 声明线程函数
void startCameraThread();
void stopCameraThread();

// 定义内部变量
extern rm_utils::TopicQueue<CameraFrame> image_queue; 
static CameraFrame SubImage;
static std::atomic<bool> imagesubReady(false);
static std::atomic<bool> pattern_found(false); // 添加pattern_found变量

static void processImage(const CameraFrame& frame) {
    if (!frame.frame.empty())
    {
        SubImage.frame = frame.frame.clone();
        SubImage.timestamp = frame.timestamp;
        imagesubReady = true;
    }
}

class CameraCalibrator {
public:
    CameraCalibrator(const std::string& config_path) : config_path_(config_path), capturing_(false), image_count_(0) {
        // 读取配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 获取标定板参数
        pattern_size_.width = config["calibration"]["board_width"].as<int>(9);
        pattern_size_.height = config["calibration"]["board_height"].as<int>(6);
        square_size_ = config["calibration"]["square_size"].as<float>(1.0);
        
        min_images_ = config["calibration"]["min_images"].as<int>(10);
        max_images_ = config["calibration"]["max_images"].as<int>(250);
        
        // 创建保存图像的目录
        image_save_path_ = "calibration_images";
        std::filesystem::create_directories(image_save_path_);
        
        ULOG_INFO_TAG("Calibrator", "CameraCalibrator initialized with board size %dx%d", 
                     pattern_size_.width, pattern_size_.height);
    }
    
    void runInteractiveCalibration() {
        std::cout << "相机标定程序\n";
        std::cout << "============\n";
        std::cout << "1. 开始捕获标定图像\n";
        std::cout << "2. 执行相机标定\n";
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
                        std::cout << "标定失败，请检查图像质量。\n";
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

        // 启动相机线程
        startCameraThread();
        
        capturing_ = true;
        image_count_ = 0;
        pattern_found = false; // 初始化pattern_found

        CameraFrame frame;
        // 检查是否有新的图像数据
        if (image_queue.pop("image/camera", frame)){
            processImage(frame);
        }
        
        ULOG_INFO_TAG("Calibrator", "Started capturing chessboard images");
        std::cout << "正在捕获图像。在显示窗口中按 'c' 捕获图像，按 'q' 停止捕获\n";

        int key = 0;
        cv::Mat last_frame; // 保存最后一帧用于捕获
        
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
            
            // 处理按键事件
            key = cv::waitKey(1) & 0xFF;
            if (key == 'c' || key == 'C') {
                // 检查是否检测到棋盘格并且还有空间存储图像
                if (pattern_found && image_count_ < max_images_) {
                    // 保存图像
                    std::string filename = fmt::format("{}/image_{:03d}.jpg", image_save_path_, static_cast<int>(image_count_));
                    cv::imwrite(filename, last_frame);
                    image_count_++;
                    ULOG_INFO_TAG("Calibrator", "Captured image %d/%d", static_cast<int>(image_count_), max_images_);
                } else if (!pattern_found) {
                    ULOG_INFO_TAG("Calibrator", "未检测到棋盘格，无法捕获图像");
                }
            }
        }
        
        capturing_ = false;
        cv::destroyAllWindows();
        // 停止相机线程
        stopCameraThread();

        ULOG_INFO_TAG("Calibrator", "图像捕获完成，共捕获 %d张图像", static_cast<int>(image_count_));
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
        
        cv::putText(display_frame, fmt::format("Images: {}/{}", static_cast<int>(image_count_), min_images_), 
                   cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        cv::putText(display_frame, "Press 'c' to capture, 'q' to quit", 
                   cv::Point(10, display_frame.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        
        cv::imshow("Camera Calibration", display_frame);
    }
    
    bool calibrate() {
        ULOG_INFO_TAG ("Calibrator","开始执行相机标定...");
        
        // 收集图像和角点
        std::vector<std::vector<cv::Point2f>> image_points;
        std::vector<std::vector<cv::Point3f>> object_points;
        cv::Size image_size;
        
        // 生成对象点（棋盘格在世界坐标系中的坐标）
        std::vector<cv::Point3f> obj;
        for (int i = 0; i < pattern_size_.height; i++) {
            for (int j = 0; j < pattern_size_.width; j++) {
                obj.emplace_back(j * square_size_, i * square_size_, 0.0f);
            }
        }
        
        // 读取保存的图像
        int valid_image_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(image_save_path_)) {
            if (entry.path().extension() == ".jpg") {
                cv::Mat image = cv::imread(entry.path().string());
                if (image.empty()) continue;
                
                if (valid_image_count == 0) {
                    image_size = image.size();
                }
                
                std::vector<cv::Point2f> corners;
                bool found = cv::findChessboardCorners(image, pattern_size_, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
                
                if (found) {
                    // 精细化角点位置
                    cv::Mat gray;
                    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                    
                    image_points.push_back(corners);
                    object_points.push_back(obj);
                    valid_image_count++;
                    ULOG_INFO_TAG("Calibrator", "Processed image %s", entry.path().filename().c_str());
                }
            }
        }
        
        if (image_points.size() < 4) {
            ULOG_ERROR_TAG("Calibrator", "Not enough valid images for calibration (%d < 4)", 
                          static_cast<int>(image_points.size()));
            return false;
        }
        
        ULOG_INFO_TAG("Calibrator", "使用 %d 张图像进行标",image_points.size());
        
        // 执行相机标定
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        
        double rms = cv::calibrateCamera(object_points, image_points, image_size, 
                                       camera_matrix, dist_coeffs, rvecs, tvecs);
        
        ULOG_INFO_TAG("Calibrator", "标定完成，RMS误差: %f", rms);
        
        // 计算重投影误差
        double total_error = 0;
        size_t total_points = 0;
        for (size_t i = 0; i < object_points.size(); i++) {
            std::vector<cv::Point2f> reprojected_points;
            cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, reprojected_points);
            
            double error = cv::norm(image_points[i], reprojected_points, cv::NORM_L2);
            total_error += error;
            total_points += object_points[i].size();
        }
        
        double mean_error = total_error / total_points;
        ULOG_INFO_TAG("Calibrator", "平均重投影误差: %f 像素", mean_error);
        
        // 保存标定结果到配置文件
        saveCalibrationResults(camera_matrix, dist_coeffs, mean_error);
        
        return true;
    }
    
    void saveCalibrationResults(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, double reprojection_error) {
        // 读取现有配置
        YAML::Node config = YAML::LoadFile(config_path_);
        
        // 准备相机矩阵数据
        std::vector<double> camera_matrix_data;
        for (int i = 0; i < camera_matrix.rows; i++) {
            for (int j = 0; j < camera_matrix.cols; j++) {
                camera_matrix_data.push_back(camera_matrix.at<double>(i, j));
            }
        }
        
        // 准备畸变系数数据
        std::vector<double> dist_coeffs_data;
        for (int i = 0; i < dist_coeffs.rows; i++) {
            dist_coeffs_data.push_back(dist_coeffs.at<double>(i, 0));
        }
        
        // 更新配置，使用Flow格式确保带方括号
        YAML::Node camera_matrix_node(camera_matrix_data);
        YAML::Node dist_coeffs_node(dist_coeffs_data);
        camera_matrix_node.SetStyle(YAML::EmitterStyle::Flow);
        dist_coeffs_node.SetStyle(YAML::EmitterStyle::Flow);
        
        config["calibration"]["camera_matrix"] = camera_matrix_node;
        config["calibration"]["distort_coeffs"] = dist_coeffs_node;
        config["calibration"]["reprojection_error"] = reprojection_error;
        
        // 写入文件
        std::ofstream fout(config_path_);
        fout << config;
        fout.close();
        
        ULOG_INFO_TAG("Calibrator", "Calibration results saved to %s", config_path_.c_str());
        
        // 打印结果
        std::cout << "\n相机内参矩阵:\n";
        for (int i = 0; i < camera_matrix.rows; i++) {
            for (int j = 0; j < camera_matrix.cols; j++) {
                std::cout << std::fixed << std::setprecision(4) << std::setw(10) 
                         << camera_matrix.at<double>(i, j) << " ";
            }
            std::cout << "\n";
        }
        
        std::cout << "\n畸变系数:\n";
        for (int i = 0; i < dist_coeffs.rows && i < 5; i++) {
            std::cout << "k" << i << ": " << std::fixed << std::setprecision(6) 
                     << dist_coeffs.at<double>(i, 0) << "\n";
        }
        
        std::cout << "\n重投影误差: " << std::fixed << std::setprecision(4) 
                 << reprojection_error << " 像素\n";
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

int runCalibration() {
    running = true; // 确保running标志设置为true
    ULOG_INIT_CONSOLE_AND_FILE(ULOG_INFO_LEVEL, ULOG_TRACE_LEVEL);
    ULOG_INFO_TAG("main", "Camera calibration started");
    
    try {
        CameraCalibrator calibrator("config/camera_set.yaml");
        calibrator.runInteractiveCalibration();
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("main", "Exception occurred: %s", e.what());
        std::cout << "发生错误: " << e.what() << "\n";
        return -1;
    }
    
    ULOG_INFO_TAG("main", "Camera calibration program finished");
    ULOG_DEINIT();
    return 0;
}