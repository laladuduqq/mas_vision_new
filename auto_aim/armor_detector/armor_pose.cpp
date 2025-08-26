/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-26 21:46:40
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-26 21:55:27
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/armor_pose.cpp
 * @Description: 装甲板姿态估计模块实现
 */
#include "armor_pose.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include "ulog.hpp"

namespace auto_aim {

constexpr double LIGHTBAR_LENGTH = 0.056;      // 灯条长度，单位：米
constexpr double SMALL_ARMOR_WIDTH = 0.135;    // 小装甲板宽度，单位：米
constexpr double LARGE_ARMOR_WIDTH = 0.230;    // 大装甲板宽度，单位：米

ArmorPoseEstimator::ArmorPoseEstimator(const std::string& config_path) {
    try {
        // 读取配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 读取相机内参
        if (config["calibration"]) {
            auto camera_matrix_data = config["calibration"]["camera_matrix"].as<std::vector<double>>();
            auto dist_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
            
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
            dist_coeffs_ = cv::Mat(dist_coeffs_data.size(), 1, CV_64F, dist_coeffs_data.data()).clone();
            
            ULOG_INFO_TAG("ArmorPoseEstimator", "Camera parameters loaded successfully");
        }
        
        // 读取坐标变换参数
        if (config["calibration"]["R_gimbal2imubody"]) {
            auto R_gimbal2imubody_data = config["calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
            R_gimbal2imubody_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_gimbal2imubody_data.data());
        } else {
            R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
        }
        
        if (config["calibration"]["R_camera2gimbal"]) {
            auto R_camera2gimbal_data = config["calibration"]["R_camera2gimbal"].as<std::vector<double>>();
            R_camera2gimbal_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_camera2gimbal_data.data());
        } else {
            R_camera2gimbal_ = Eigen::Matrix3d::Identity();
        }
        
        if (config["calibration"]["t_camera2gimbal"]) {
            auto t_camera2gimbal_data = config["calibration"]["t_camera2gimbal"].as<std::vector<double>>();
            t_camera2gimbal_ = Eigen::Map<Eigen::Vector3d>(t_camera2gimbal_data.data());
        } else {
            t_camera2gimbal_ = Eigen::Vector3d::Zero();
        }
        
        // 初始化装甲板3D点
        initArmorPoints();
        
        ULOG_INFO_TAG("ArmorPoseEstimator", "ArmorPoseEstimator initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("ArmorPoseEstimator", "Failed to initialize ArmorPoseEstimator: %s", e.what());
        // 使用默认参数初始化
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
        R_camera2gimbal_ = Eigen::Matrix3d::Identity();
        t_camera2gimbal_ = Eigen::Vector3d::Zero();
        initArmorPoints();
    }
}

void ArmorPoseEstimator::initArmorPoints() {
    // 小装甲板3D点定义（以装甲板中心为原点）
    small_armor_points_ = {
        {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
        {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
        {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
        {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
    };
    
    // 大装甲板3D点定义（以装甲板中心为原点）
    large_armor_points_ = {
        {0, LARGE_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
        {0, -LARGE_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
        {0, -LARGE_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
        {0, LARGE_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
    };
    
    ULOG_INFO_TAG("ArmorPoseEstimator", "Armor 3D points initialized");
}

const std::vector<cv::Point3f>& ArmorPoseEstimator::getArmorPoints(ArmorType type) const {
    if (type == ArmorType::BIG) {
        return large_armor_points_;
    } else {
        return small_armor_points_;
    }
}

bool ArmorPoseEstimator::estimatePose(Armor& armor) {
    try {
        // 获取对应装甲板类型的3D点
        const std::vector<cv::Point3f>& object_points = getArmorPoints(armor.type);
        
        // 检查点数是否匹配
        if (armor.points.size() != object_points.size()) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "Point count mismatch: image points(%d) vs object points(%d)", 
                           static_cast<int>(armor.points.size()), static_cast<int>(object_points.size()));
            return false;
        }
        
        // 使用solvePnP计算姿态
        cv::Vec3d rvec, tvec;
        bool success = cv::solvePnP(object_points, armor.points, camera_matrix_, dist_coeffs_, 
                                   rvec, tvec, false, cv::SOLVEPNP_IPPE);
        
        if (!success) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "solvePnP failed for armor");
            return false;
        }
        
        // 转换为Eigen矩阵
        Eigen::Vector3d t_camera;
        cv::cv2eigen(tvec, t_camera);
        
        // 从相机坐标系转换到云台坐标系
        Eigen::Vector3d t_gimbal = R_camera2gimbal_ * t_camera + t_camera2gimbal_;
        
        // 存储结果
        armor.xyz_in_gimbal[0] = t_gimbal(0);
        armor.xyz_in_gimbal[1] = t_gimbal(1);
        armor.xyz_in_gimbal[2] = t_gimbal(2);
        
        // 计算旋转矩阵
        cv::Mat R_matrix;
        cv::Rodrigues(rvec, R_matrix);
        
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(R_matrix, R_armor2camera);
        
        // 从装甲板到相机的旋转转换到装甲板到云台的旋转
        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
        
        // 计算欧拉角 (ZYX顺序)
        double yaw = std::atan2(R_armor2gimbal(1, 0), R_armor2gimbal(0, 0));
        double pitch = std::atan2(-R_armor2gimbal(2, 0), 
                                 std::sqrt(R_armor2gimbal(2, 1) * R_armor2gimbal(2, 1) + 
                                          R_armor2gimbal(2, 2) * R_armor2gimbal(2, 2)));
        double roll = std::atan2(R_armor2gimbal(2, 1), R_armor2gimbal(2, 2));
        
        armor.ypr_in_gimbal[0] = yaw;
        armor.ypr_in_gimbal[1] = pitch;
        armor.ypr_in_gimbal[2] = roll;
        
        ULOG_DEBUG_TAG("ArmorPoseEstimator", "Pose estimated successfully for armor %s", armor.name.c_str());
        return true;
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("ArmorPoseEstimator", "Error in pose estimation: %s", e.what());
        return false;
    }
}

} // namespace auto_aim