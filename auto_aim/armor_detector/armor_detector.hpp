/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 13:47:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-27 10:05:20
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/armor_detector.hpp
 * @Description: 
 */
#ifndef _ARMOR_DETECTOR_H_
#define _ARMOR_DETECTOR_H_

#include <opencv2/core/mat.hpp>

#include "armor_pose.hpp"
#include "armor_types.hpp"
#include "number_classifier.hpp"

namespace auto_aim {
class ArmorDetector 
{ 
    public:
    ArmorDetector();
    /**
     * @brief 识别装甲板
     * @param bgr_img 输入图像
     * @return 识别到的装甲板
     */
    std::vector<Armor> ArmorDetect(const cv::Mat & bgr_img);
    /**
     * @brief 显示识别结果
     * @param bgr_img 输入图像
     * @return 显示结果
     */
    cv::Mat showResult(const cv::Mat& bgr_img) const;
        /**
     * @brief 获取云台到世界坐标系的旋转矩阵
     * @return 旋转矩阵
     */
    Eigen::Matrix3d R_gimbal2world() const;

    /**
     * @brief 设置云台到世界坐标系的旋转矩阵
     * @param q 四元数
     */
    void set_R_gimbal2world(const Eigen::Quaterniond & q);

    private:
    //识别参数
    double threshold_;         // 二值化阈值
    EnemyColor detect_color_; // 要检测的装甲板颜色
    bool debug_;             // 调试开关
    std::vector<LightBar> lights_;
    std::vector<Armor> armors_;
    // 灯条参数
    // width / height
    double lightbar_min_ratio_;
    double lightbar_max_ratio_;
    // vertical angle
    double lightbar_max_angle_;
    // area condition
    double lightbar_min_fill_ratio_;
    double color_diff_thresh_;
    // 装甲板参数
    double armor_min_light_ratio_;
    // light pairs distance
    double armor_min_small_center_distance_;
    double armor_max_small_center_distance_;
    double armor_min_large_center_distance_;
    double armor_max_large_center_distance_;
    // horizontal angle
    double armor_max_angle_;
    // 数字识别器
    std::unique_ptr<NumberClassifier> classifier_;
    double confidence;
    // 姿态估计器
    std::unique_ptr<ArmorPoseEstimator> pose_estimator_;

    // 辅助函数
    void lightbar_points_corrector(LightBar & lightbar, const cv::Mat & gray_img) const;
    std::vector<LightBar> findLights(const cv::Mat& bin_img, const cv::Mat& src_img) const;
    bool isLight(const LightBar& light) const;
    std::vector<Armor> findArmors(const std::vector<LightBar> & lightbars,const cv::Mat & bgr_img);
    ArmorType isArmor(const LightBar & light_1, const LightBar & light_2) const;
    bool containLight(const LightBar & light_1, const LightBar & light_2, const std::vector<LightBar> & lights) const;
    cv::Mat getAllNumbersImage() const ;
};

}

#endif // _ARMOR_DETECTOR_H_