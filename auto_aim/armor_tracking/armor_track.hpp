/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-28 12:52:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-29 19:52:30
 * @FilePath: /mas_vision_new/auto_aim/armor_tracking/armor_track.hpp
 * @Description: 
 */
#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>
#include <vector>

#include "armor_types.hpp"
#include "armor_tracking/armor_target.hpp"
#include "armor_detector/armor_pose.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path);

  std::string state() const;

  std::list<Target> track(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t);
    
  // 添加调试绘制函数
  void drawDebug(const cv::Mat& bgr_img);

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
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;

  // 姿态估计器
  std::unique_ptr<ArmorPoseEstimator> pose_estimator_;

  void state_machine(bool found);

  bool set_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP