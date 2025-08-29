/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-26 21:46:40
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-28 23:34:05
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
     * @brief 计算装甲板的姿态，对外统一接口
     * @param armor 装甲板对象，包含关键点等信息
     * @return 是否成功计算姿态
     */
    bool solve(Armor& armor);

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

    /**
     * @brief 获取云台到世界坐标系的旋转矩阵
     * @return 旋转矩阵
     */
    Eigen::Matrix3d R_gimbal2world() const;

    /**
     * @brief 设置云台到世界坐标系的旋转矩阵
     * @param q 旋转矩阵
     */
    void set_R_gimbal2world(const Eigen::Quaterniond & q);
    /**
     * @brief 投影装甲板
     * @param xyz_in_world 装甲板在世界坐标系下的3D点
     * @param yaw 装甲板偏航角
     * @param type 装甲板类型
     * @param name 装甲板名称
     * @return 投影后的装甲板点集
     */
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, std::string armor_name) const;

private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // 坐标变换参数
    Eigen::Matrix3d R_gimbal2imubody_;  // 云台->IMU坐标系
    Eigen::Matrix3d R_camera2gimbal_;   // 相机->云台坐标系
    Eigen::Vector3d t_camera2gimbal_;   // 相机->IMU坐标系
    Eigen::Matrix3d R_gimbal2world_;   // 云台->世界坐标系（通过读取imu姿态更新）

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
    /**
     * @brief 优化装甲板偏航角
     * @param armor 装甲板对象
     */
    void optimize_yaw(Armor & armor) const;
    /**
     * @brief 计算装甲板投影误差
     * @param armor 装甲板对象
     * @param yaw 装甲板偏航角
     * @param inclined 倾斜角
     * @return 投影误差
     */
    double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
    /**
     * @brief 投影误差计算
     * @param cv_refs 投影参考点集
     * @param cv_pts 投影点集
     * @param inclined 倾斜角
     * @return 投影误差
     */
    double SJTU_cost(const std::vector<cv::Point2f> & cv_refs, 
    const std::vector<cv::Point2f> & cv_pts,const double & inclined) const;

    /**
     * @brief 投影装甲板
     * @param worldPoints 装甲板在世界坐标系下的3D点
     * @return 投影后的装甲板点集
     */
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

    /**
     * @brief 内部姿态估计实现
     * @param armor 装甲板对象
     * @return 是否成功计算姿态
     */
    bool estimatePose(Armor& armor);
};

} // namespace auto_aim

#endif // _ARMOR_POSE_H_