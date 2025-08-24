/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 21:54:23
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-24 17:45:19
 * @FilePath: /mas_vision_new/auto_aim/armor_types.hpp
 * @Description: 
 */
#ifndef _ARMOR_TYPES_H_
#define _ARMOR_TYPES_H_

#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense> 


namespace auto_aim
{
    // Armor size, Unit: m
    constexpr double SMALL_ARMOR_WIDTH = 133.0 / 1000.0; // 135
    constexpr double SMALL_ARMOR_HEIGHT = 50.0 / 1000.0; // 55
    constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0;
    constexpr double LARGE_ARMOR_HEIGHT = 50.0 / 1000.0; // 55

    enum  EnemyColor
    {
        RED,
        BLUE,
        WHITE
    };
    const std::vector<std::string> COLORS = {"RED", "BLUE", "WHITE"};

    enum ArmorType
    {
        BIG,
        SMALL,
        UNKNOWN
    };
    const std::vector<std::string> ARMOR_TYPES = {"BIG", "SMALL","UNKNOWN"};

    enum ArmorName
    {
        one,
        two,
        three,
        four,
        five,
        sentry,
        outpost,
        base,
        negative
    };
    const std::vector<std::string> ARMOR_NAMES = {"1",    "2",     "3", "4",     "5",
                                                "sentry", "outpost", "base",  "negative"};


    struct LightBar : public cv::Rect
    {
        EnemyColor color;
        cv::Point2f center, top, bottom;
        double length, width, ratio;
        float tilt_angle;
        LightBar() {};
        explicit LightBar(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
        : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
        {
            length = cv::norm(top - bottom);
            width = area / length;
            center = (top + bottom) / 2;
            ratio = width / length;
        }
    };

    struct Armor
    {
        // lightbar部分
        EnemyColor color;
        LightBar left_light, right_light;     
        cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
        // 装甲板类型
        ArmorType type;
        // 数字识别部分
        cv::Mat number_img;
        std::string name;
        double confidence;


        Eigen::Vector3d xyz_in_gimbal;  // 单位：m
        Eigen::Vector3d xyz_in_world;   // 单位：m
        Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
        Eigen::Vector3d ypr_in_world;   // 单位：rad
        Eigen::Vector3d ypd_in_world;   // 球坐标系

        double yaw_raw;  // rad
        double roll_raw;
        Armor() = default;
        Armor(const LightBar & left, const LightBar & right)
        : left_light(left), right_light(right)
        {
            color = left.color;
            center = (left.center + right.center) / 2;

            auto left2right = right.center - left.center;
            roll_raw = std::atan2(left2right.y, left2right.x);
        }
    };

}  // namespace auto_aim



#endif // _ARMOR_TYPES_H_