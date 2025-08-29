/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-26 21:46:40
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-29 19:26:49
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/armor_pose.cpp
 * @Description: 装甲板姿态估计模块实现
 */
#include "armor_pose.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include "rmmath.hpp"
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

Eigen::Matrix3d ArmorPoseEstimator::R_gimbal2world() const { return R_gimbal2world_; }

void ArmorPoseEstimator::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool ArmorPoseEstimator::solve(Armor& armor) {
    return estimatePose(armor);
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
        
        // 添加对输入参数的详细检查
        // 检查object_points是否有效
        if (object_points.empty()) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "Object points are empty");
            return false;
        }
        
        // 检查armor.points是否有效
        if (armor.points.empty()) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "Image points are empty");
            return false;
        }
        
        // 检查相机内参矩阵是否有效
        if (camera_matrix_.empty() || camera_matrix_.cols != 3 || camera_matrix_.rows != 3) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "Invalid camera matrix");
            return false;
        }
        
        // 检查畸变系数是否有效
        if (dist_coeffs_.empty()) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "Invalid distortion coefficients");
            return false;
        }
        
        // 检查所有点坐标是否有效（非无穷大、非NaN）
        for (const auto& point : object_points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                ULOG_WARNING_TAG("ArmorPoseEstimator", "Invalid object point coordinates");
                return false;
            }
        }
        
        for (const auto& point : armor.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                ULOG_WARNING_TAG("ArmorPoseEstimator", "Invalid image point coordinates");
                return false;
            }
        }
        
        // 使用solvePnP计算姿态
        cv::Vec3d rvec, tvec;
        bool success = cv::solvePnP(object_points, armor.points, camera_matrix_, dist_coeffs_, 
                                   rvec, tvec, false, cv::SOLVEPNP_IPPE);
        
        if (!success) {
            ULOG_WARNING_TAG("ArmorPoseEstimator", "solvePnP failed for armor");
            return false;
        }
        
        // 3. 坐标变换：相机系 → 云台系 → 世界系
        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);
        armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
        armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

        // 4. 计算欧拉角（俯仰、滚转、偏航）
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(rmat, R_armor2camera);
        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
        Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
        armor.ypr_in_gimbal = rm_utils::eulers(R_armor2gimbal, 2, 1, 0);  // 云台系欧拉角
        armor.ypr_in_world  = rm_utils::eulers(R_armor2world, 2, 1, 0);   // 世界系欧拉角
        armor.ypd_in_world  = rm_utils::xyz2ypd(armor.xyz_in_world);
        
        optimize_yaw(armor);
        
        ULOG_DEBUG_TAG("ArmorPoseEstimator", "Pose estimated successfully for armor %s", armor.name.c_str());
        return true;
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("ArmorPoseEstimator", "Error in pose estimation: %s", e.what());
        return false;
    }
}

std::vector<cv::Point2f> ArmorPoseEstimator::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, std::string armor_name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (armor_name == "outpost") ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::BIG) ? large_armor_points_ : small_armor_points_;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
  return image_points;
}

void ArmorPoseEstimator::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = rm_utils::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = rm_utils::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = rm_utils::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double ArmorPoseEstimator::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * rm_utils::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      rm_utils::square(pixel_dis * std::sin(inclined)) +
      rm_utils::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double ArmorPoseEstimator::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> ArmorPoseEstimator::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, dist_coeffs_, pixelPoints);
  return pixelPoints;
}

} // namespace auto_aim