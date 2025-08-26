/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-26 21:46:40
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-26 21:46:44
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/armor_pose.hpp
 * @Description: 装甲板姿态估计模块
 */
#ifndef _ARMOR_POSE_H_
#define _ARMOR_POSE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

#include "armor_types.hpp"

namespace auto_aim {

class ArmorPoseEstimator {
public:
    /**
     * @brief 构造函数，初始化姿态估计器
     * @param config_path 配置文件路径
     */
    explicit ArmorPoseEstimator(const std::string& config_path);

    /**
     * @brief 计算装甲板的姿态
     * @param armor 装甲板对象，包含关键点等信息
     * @return 是否成功计算姿态
     */
    bool estimatePose(Armor& armor);

    /**
     * @brief 获取相机内参矩阵
     * @return 相机内参矩阵
     */
    cv::Mat getCameraMatrix() const { return camera_matrix_; }

    /**
     * @brief 获取畸变系数
     * @return 畸变系数向量
     */
    cv::Mat getDistCoeffs() const { return dist_coeffs_; }

private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // 坐标变换参数
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;

    // 装甲板3D模型点
    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;

    /**
     * @brief 初始化装甲板3D模型点
     */
    void initArmorPoints();

    /**
     * @brief 根据装甲板类型获取对应的3D点
     * @param type 装甲板类型
     * @return 对应的3D点集
     */
    const std::vector<cv::Point3f>& getArmorPoints(ArmorType type) const;
};

} // namespace auto_aim

#endif // _ARMOR_POSE_H_