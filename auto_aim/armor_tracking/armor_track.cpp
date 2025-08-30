/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-28 12:52:18
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-30 23:07:09
 * @FilePath: /mas_vision_new/auto_aim/armor_tracking/armor_track.cpp
 * @Description: 
 */
#include "armor_track.hpp"

#include <yaml-cpp/yaml.h>
#include <numeric>

#include "ulog.hpp"
#include "rmmath.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path)
: detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["auto_aim"] && yaml["auto_aim"]["tracker"]) {
    auto tracker_config = yaml["auto_aim"]["tracker"];
    min_detect_count_ = tracker_config["min_detect_count"].as<int>(5);
    max_temp_lost_count_ = tracker_config["max_temp_lost_count"].as<int>(10);
    outpost_max_temp_lost_count_ = tracker_config["outpost_max_temp_lost_count"].as<int>(30);
  } else {
    // 默认参数
    min_detect_count_ = 5;
    max_temp_lost_count_ = 10;
    outpost_max_temp_lost_count_ = 30;
  }
  normal_temp_lost_count_ = max_temp_lost_count_;
  
  // 初始化姿态估计器
  try {
      pose_estimator_ = std::make_unique<ArmorPoseEstimator>("config/camera_set.yaml");
      ULOG_INFO_TAG("tracker", "ArmorPoseEstimator initialized successfully");
  } catch (const std::exception& e) {
      ULOG_ERROR_TAG("tracker", "Failed to initialize ArmorPoseEstimator: %s", e.what());
      pose_estimator_ = nullptr;
  }
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  auto dt = rm_utils::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    ULOG_WARNING_TAG("Tracker", "[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  
  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  std::sort(armors.begin(), armors.end(), 
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }
  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    ULOG_DEBUG_TAG("Tracker", "[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (target_.ekf().recent_nis_failures.size() > 0 &&
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    ULOG_DEBUG_TAG("Tracker", "[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

void Tracker::drawDebug(const cv::Mat& bgr_img)
{
  // 检查输入图像是否有效
  if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0) {
    ULOG_WARNING_TAG("tracker", "Invalid input image for drawDebug");
    return;
  }
  
  // 绘制跟踪状态
  cv::putText(bgr_img, "Tracker State: " + state_, cv::Point(10, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
              
    // 绘制预测的装甲板位置（仅在tracking或temp_lost状态）
    if (state_ == "tracking" || state_ == "temp_lost") {
      // 显示目标信息
      if (!target_.name.empty()) {
        // 绘制目标名称
        cv::putText(bgr_img, "Target: " + target_.name, cv::Point(10, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 绘制目标装甲板类型
        cv::putText(bgr_img, "Type: " + ARMOR_TYPES[target_.armor_type], cv::Point(10, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 绘制目标优先级
        cv::putText(bgr_img, "Priority: " + std::to_string(target_.priority), cv::Point(10, 120), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 绘制是否收敛
        std::string converge_status = target_.convergened() ? "Converged" : "Not Converged";
        cv::putText(bgr_img, "Status: " + converge_status, cv::Point(10, 180), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                    target_.convergened() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        
        // 绘制是否发生跳跃
        std::string jump_status = target_.jumped ? "Jumped" : "Not Jumped";
        cv::putText(bgr_img, "Jump: " + jump_status, cv::Point(10, 210), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
      std::vector<Eigen::Vector4d> armor_xyza_list = target_.armor_xyza_list();
      for (size_t i = 0; i < armor_xyza_list.size(); ++i) {
        const Eigen::Vector4d& xyza = armor_xyza_list[i];
        // 重投影装甲板到图像平面
        if (pose_estimator_) {
          auto image_points = pose_estimator_->reproject_armor(
            xyza.head(3), xyza[3], target_.armor_type, target_.name);
          
          for (size_t j = 0; j < image_points.size(); ++j) {
            cv::line(bgr_img, image_points[j], image_points[(j+1) % image_points.size()], 
            cv::Scalar(0, 255, 0), 2);
          }
          
          // 标记装甲板ID
          if (!image_points.empty()) {
            cv::putText(bgr_img, std::to_string(i), image_points[0], 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
          }
        }
      }
      
      // 显示EKF状态信息
      if (target_.ekf_x().size() >= 11) {
        const Eigen::VectorXd& x = target_.ekf_x();
        cv::putText(bgr_img, cv::format("Pos: (%.2f, %.2f, %.2f)", x[0], x[2], x[4]), 
                    cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        cv::putText(bgr_img, cv::format("Vel: (%.2f, %.2f, %.2f)", x[1], x[3], x[5]), 
                    cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        cv::putText(bgr_img, cv::format("Angle: %.2f rad", x[6]), 
                    cv::Point(10, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        cv::putText(bgr_img, cv::format("AngVel: %.2f", x[7]), 
                    cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        cv::putText(bgr_img, cv::format("Radius: %.3f", x[8]), 
                    cv::Point(10, 380), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
      }
      
      // 显示目标装甲板的位姿信息
      // 获取最新的装甲板位姿
      if (!armor_xyza_list.empty()) {
        const Eigen::Vector4d& latest_armor = armor_xyza_list.front();
        cv::putText(bgr_img, cv::format("Armor Pos: (%.2f, %.2f, %.2f)", 
                                    latest_armor[0], latest_armor[1], latest_armor[2]), 
                    cv::Point(10, 420), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
        cv::putText(bgr_img, cv::format("Armor Yaw: %.2f rad (%.1f deg)", 
                                    latest_armor[3], latest_armor[3] * 180.0 / CV_PI), 
                    cv::Point(10, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
      }
    }
  } else if (state_ != "lost") {
    // 当没有目标但不是lost状态时，显示提示信息
    cv::putText(bgr_img, "Searching target...", cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
  } else {
    // 当没有追踪目标时，显示提示信息
    cv::putText(bgr_img, "No target tracked", cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
  }
  // 创建缩小版图像用于显示
  cv::Mat resized_img;
  cv::resize(bgr_img, resized_img, cv::Size(bgr_img.cols/2, bgr_img.rows/2));
  cv::imshow("Tracker Debug", resized_img);
  cv::waitKey(1);
}
void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }
  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }
  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }
  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == "outpost")
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  // 装甲板姿态在Tracker中进行解算
  if (pose_estimator_) {
      pose_estimator_->solve(armor);
  }

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::BIG) &&
                    (armor.name == "three" || armor.name == "four" ||
                     armor.name == "five");

  if (is_balance) {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }
  else if (armor.name == "outpost") {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }
  else if (armor.name == "base") {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }
  else {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;

    // 装甲板姿态在Tracker中进行解算
    if (pose_estimator_) {
        pose_estimator_->solve(armor);
    }
    target_.update(armor);
  }

  return true;
}


Eigen::Matrix3d Tracker::R_gimbal2world() const {
    if (pose_estimator_) {
        return pose_estimator_->R_gimbal2world();
    }
    return Eigen::Matrix3d::Identity();
}

void Tracker::set_R_gimbal2world(const Eigen::Quaterniond & q) {
    if (pose_estimator_) {
        pose_estimator_->set_R_gimbal2world(q);
    }
}

}  // namespace auto_aim