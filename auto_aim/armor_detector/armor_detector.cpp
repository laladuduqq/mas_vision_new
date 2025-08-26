/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 13:47:12
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-26 22:15:10
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/armor_detector.cpp
 * @Description: 
 */
#include "armor_detector.hpp"
#include "armor_types.hpp"
#include "ulog.hpp"
#include "yaml-cpp/yaml.h"
#include <fmt/format.h>
#include <numeric>
#include <opencv2/core/mat.hpp>

namespace auto_aim {

ArmorDetector::ArmorDetector()
{
    // yaml 读取参数
    try {
        YAML::Node config = YAML::LoadFile("config/auto_aim.yaml");
        if (config["auto_aim"] && config["auto_aim"]["armor_detector"]) {
            YAML::Node armor_detector = config["auto_aim"]["armor_detector"]; 
            // 读取基础参数
            debug_ = armor_detector["debug"].as<bool>(false);
            // 读取二值化参数
            threshold_ = armor_detector["binary_thres"].as<int>(90);
            // 读取颜色参数
            std::string color = armor_detector["detect_color"].as<std::string>("RED");
            // 根据COLORS向量中的定义匹配颜色
            if (color == "blue" || color == "BLUE") {
                detect_color_ = auto_aim::EnemyColor::BLUE;
            } else if (color == "red" || color == "RED") {
                detect_color_ = auto_aim::EnemyColor::RED;
            } else {
                // 默认为红色
                detect_color_ = auto_aim::EnemyColor::RED;
            }
            // 读取参数
            YAML::Node lights_params = armor_detector["lights_params"];
            lightbar_min_ratio_      = lights_params["min_ratio"].as<double>(0.1),
            lightbar_max_ratio_      = lights_params["max_ratio"].as<double>(0.5),
            lightbar_max_angle_      = lights_params["max_angle"].as<double>(35.0),
            lightbar_min_fill_ratio_ = lights_params["min_fill_ratio"].as<double>(0.8),
            color_diff_thresh_       = lights_params["color_diff_thresh"].as<double>(20.0);
            
            YAML::Node armors_params = armor_detector["armors_params"];
            armor_min_light_ratio_       = armors_params["min_light_ratio"].as<double>(0.7);
            armor_min_small_center_distance_      = armors_params["min_small_center_distance"].as<double>(0.8);
            armor_max_small_center_distance_      = armors_params["max_small_center_distance"].as<double>(3.2);
            armor_min_large_center_distance_      = armors_params["min_large_center_distance"].as<double>(3.2);
            armor_max_large_center_distance_      = armors_params["max_large_center_distance"].as<double>(5.5);
            armor_max_angle_                      = armors_params["max_angle"].as<double>(35.0);            
            //读取数字识别参数
            if (armor_detector["number_params"]) {
                YAML::Node number_params = armor_detector["number_params"];
                std::string model_path = number_params["model_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/lenet.onnx");
                std::string label_path = number_params["label_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/label.txt");
                confidence = number_params["classifier_threshold"].as<double>(0.7);
                
                std::vector<std::string> ignore_classes;
                // 读取忽略类别
                if (number_params["ignore_classes"]) {
                    ignore_classes.clear();
                    for (const auto& cls : number_params["ignore_classes"]) {
                        ignore_classes.push_back(cls.as<std::string>());
                    }
                }
                // 创建数字识别器
                classifier_ = std::make_unique<NumberClassifier>(model_path, label_path, confidence, ignore_classes);
            }
        }
    } catch (const std::exception& e) {
        ULOG_WARNING_TAG("armor_detector", "Failed to load config, using defaults: %s", e.what());
    }
    // 初始化姿态估计器
    try {
        pose_estimator_ = std::make_unique<ArmorPoseEstimator>("config/camera_set.yaml");
        ULOG_INFO_TAG("armor_detector", "ArmorPoseEstimator initialized successfully");
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("armor_detector", "Failed to initialize ArmorPoseEstimator: %s", e.what());
        pose_estimator_ = nullptr;
    }
}


std::vector<Armor> ArmorDetector::ArmorDetect(const cv::Mat & bgr_img)
{   
  // 转换为灰度图
  cv::Mat gray;
  if (bgr_img.channels() == 3) {
      cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
  } else {
      gray = bgr_img;
  }
  cv::Mat binary;
  // 应用阈值进行二值化
  cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY);
  // 寻找灯条
  lights_ = findLights(binary,bgr_img);
  // 寻找装甲板
  armors_ = findArmors(lights_,bgr_img);
  // 对识别出的装甲板中的每个灯条应用角点优化
  for (auto& armor : armors_) {
      lightbar_points_corrector(armor.left_light, gray);
      lightbar_points_corrector(armor.right_light, gray);
      armor.points.emplace_back(armor.left_light.top);
      armor.points.emplace_back(armor.right_light.top);
      armor.points.emplace_back(armor.right_light.bottom);
      armor.points.emplace_back(armor.left_light.bottom);
      // 进行姿态估计
      if (pose_estimator_) {
          pose_estimator_->estimatePose(armor);
      }
  }

  return armors_;
}

void ArmorDetector::lightbar_points_corrector(LightBar & lightbar, const cv::Mat & gray_img) const
{
  // 配置参数
  constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值
  constexpr float ROI_SCALE = 0.07;     // ROI扩展比例
  constexpr float SEARCH_START = 0.4;   // 搜索起始位置比例（原0.8/2）
  constexpr float SEARCH_END = 0.6;     // 搜索结束位置比例（原1.2/2）

  // 扩展并裁剪ROI，使用cv::Rect
  cv::Rect roi_box = cv::Rect(lightbar);
  roi_box.x -= roi_box.width * ROI_SCALE;
  roi_box.y -= roi_box.height * ROI_SCALE;
  roi_box.width += 2 * roi_box.width * ROI_SCALE;
  roi_box.height += 2 * roi_box.height * ROI_SCALE;

  // 边界约束
  roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);

  // 归一化ROI
  cv::Mat roi = gray_img(roi_box);
  const float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 生成稀疏点云（优化性能）
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; ++i) {
    for (int j = 0; j < roi.cols; ++j) {
      const float weight = roi.at<float>(i, j);
      if (weight > 1e-3) {          // 忽略极小值提升性能
        points.emplace_back(j, i);  // 坐标相对于ROI区域
      }
    }
  }

  // 检查是否有足够的点进行PCA分析
  if (points.size() < 2) {
    // 如果点数不足，使用默认方法设置角点
    lightbar.top = cv::Point2f(lightbar.x + lightbar.width / 2.0f, lightbar.y);
    lightbar.bottom = cv::Point2f(lightbar.x + lightbar.width / 2.0f, lightbar.y + lightbar.height);
    return;
  }

  // PCA计算对称轴方向
  cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis /= cv::norm(axis);
  if (axis.y > 0) axis = -axis;  // 统一方向

  const auto find_corner = [&](int direction) -> cv::Point2f {
    const float dx = axis.x * direction;
    const float dy = axis.y * direction;
    const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

    std::vector<cv::Point2f> candidates;

    // 横向采样多个候选线
    const int half_width = (lightbar.width - 2) / 2;
    for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
      // 计算搜索起点
      cv::Point2f start_point(
        lightbar.center.x + lightbar.length * SEARCH_START * dx + i_offset - roi_box.x,
        lightbar.center.y + lightbar.length * SEARCH_START * dy - roi_box.y);

      // 沿轴搜索亮度跳变点
      cv::Point2f corner = start_point;
      float max_diff = 0;
      bool found = false;

      for (float step = 0; step < search_length; ++step) {
        const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

        // 边界检查
        if (
          cur_point.x < 0 || cur_point.x >= roi.cols || cur_point.y < 0 ||
          cur_point.y >= roi.rows) {
          break;
        }

        // 计算亮度差（使用双线性插值提升精度）
        cv::Point2f prev_point(cur_point.x - dx, cur_point.y - dy);
        if (prev_point.x < 0 || prev_point.x >= roi.cols || prev_point.y < 0 || prev_point.y >= roi.rows) {
          continue;
        }

        const auto prev_val = roi.at<float>(prev_point);
        const auto cur_val = roi.at<float>(cur_point);
        const float diff = prev_val - cur_val;

        if (diff > max_diff && prev_val > mean_val) {
          max_diff = diff;
          corner = prev_point;  // 跳变发生在上一位置
          found = true;
        }
      }

      if (found) {
        candidates.push_back(corner);
      }
    }

    // 返回候选点均值或者默认点
    if (candidates.empty()) {
      // 如果没有找到候选点，返回默认位置
      float direction_factor = (direction > 0) ? -0.5f : 0.5f;
      return cv::Point2f(
        lightbar.center.x - roi_box.x,
        lightbar.center.y + direction_factor * lightbar.length - roi_box.y
      );
    } else {
      cv::Point2f sum = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
      return sum / static_cast<float>(candidates.size());
    }
  };

  // 并行检测顶部和底部
  cv::Point2f top_local = find_corner(1);
  cv::Point2f bottom_local = find_corner(-1);
  
  // 将局部坐标转换为全局坐标
  lightbar.top = cv::Point2f(top_local.x + roi_box.x, top_local.y + roi_box.y);
  lightbar.bottom = cv::Point2f(bottom_local.x + roi_box.x, bottom_local.y + roi_box.y);
}



// 辅助函数：判断是否为灯条
bool ArmorDetector::isLight(const LightBar& lightbar) const {
  bool ratio_ok = lightbar_min_ratio_ < lightbar.ratio && lightbar.ratio < lightbar_max_ratio_;
  bool angle_ok = lightbar.tilt_angle < lightbar_max_angle_;
  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

std::vector<LightBar> ArmorDetector::findLights(const cv::Mat& bin_img, const cv::Mat& src_img) const {
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // 寻找轮廓，使用CHAIN_APPROX_SIMPLE减少点数
    cv::findContours(bin_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 创建灯条类
    std::vector<LightBar> lightbars;

    // 遍历所有轮廓
    for (const auto & contour : contours) {
        if (contour.size() < 5) continue;  // 轮廓过短的跳过

        // 先使用边界框进行快速过滤
        auto b_rect = cv::boundingRect(contour);
        
        // 快速检查宽高比，过滤明显不符合的轮廓
        double rect_ratio = static_cast<double>(b_rect.width) / b_rect.height;
        if (rect_ratio > 1.0 || rect_ratio < 0.05) continue; // 灯条应该是竖直且细长的
        
        auto r_rect = cv::minAreaRect(contour);
        // 检查旋转矩形的宽高比
        double r_rect_ratio = r_rect.size.width / r_rect.size.height;
        if (r_rect_ratio > 1.0) r_rect_ratio = 1.0 / r_rect_ratio;
        if (r_rect_ratio < 0.05) continue;

        cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
        std::vector<cv::Point> mask_contour;
        for (const auto & p : contour) {
            mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
        }
        cv::fillPoly(mask, {mask_contour}, 255);
        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);
        
        // 检查填充率
        bool is_fill_rotated_rect = points.size() / (r_rect.size.width * r_rect.size.height) > lightbar_min_fill_ratio_;
        if (!is_fill_rotated_rect) continue;
        
        // 使用最小二乘法拟合直线
        cv::Vec4f line_param;
        cv::fitLine(points, line_param, cv::DIST_L2, 0, 0.01, 0.01);
        
        cv::Point2f top, bottom;
        double angle_k;
        if (int(line_param[0] * 100) == 100 || int(line_param[1] * 100) == 0) {
            // 垂直线情况
            top = cv::Point2f(b_rect.x + b_rect.width / 2.0f, b_rect.y);
            bottom = cv::Point2f(b_rect.x + b_rect.width / 2.0f, b_rect.y + b_rect.height);
            angle_k = 0;
        } else {
            // 一般情况
            auto k = line_param[1] / line_param[0];
            auto b = (line_param[3] + b_rect.y) - k * (line_param[2] + b_rect.x);
            top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
            bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
            angle_k = std::atan(k) / CV_PI * 180 - 90;
            if (angle_k > 90) {
              angle_k = 180 - angle_k;
            }
        }
        
        auto lightbar = LightBar(b_rect, top, bottom, points.size(), angle_k);
        
        // 先进行快速角度检查
        if (lightbar.tilt_angle > lightbar_max_angle_) continue;

        if (isLight(lightbar)) {
            // 优化颜色识别，只采样部分点，并添加边界检查
            int sum_r = 0, sum_b = 0;
            int sample_count = 0;
            int step = std::max(1, static_cast<int>(contour.size() / 20)); // 最多采样20个点
            
            for (size_t i = 0; i < contour.size(); i += step) {
                const auto& point = contour[i];
                // 严格的边界检查
                if (point.x >= 0 && point.x < src_img.cols && point.y >= 0 && point.y < src_img.rows) {
                    // 额外检查确保点在bounding rect内
                    if (point.x >= b_rect.x && point.x < b_rect.x + b_rect.width && 
                        point.y >= b_rect.y && point.y < b_rect.y + b_rect.height) {
                        try {
                            sum_r += src_img.at<cv::Vec3b>(point.y, point.x)[2];
                            sum_b += src_img.at<cv::Vec3b>(point.y, point.x)[0];
                            sample_count++;
                        } catch (const cv::Exception& e) {
                            // 忽略异常点
                            continue;
                        }
                    }
                }
            }
            
            if (sample_count > 0) {
                double avg_color_diff = std::abs(sum_r - sum_b) / static_cast<double>(sample_count);
                if (avg_color_diff > color_diff_thresh_) {
                    lightbar.color = (sum_r > sum_b) ? EnemyColor::RED : EnemyColor::BLUE;
                } else {
                    lightbar.color = EnemyColor::WHITE;
                }
            } else {
                lightbar.color = EnemyColor::WHITE;
            }
            
            lightbars.emplace_back(lightbar);
        }
    } 

    // 将灯条从左到右排序
    std::sort(lightbars.begin(), lightbars.end(), [](const LightBar & a, const LightBar & b) { return a.center.x < b.center.x; });
    
    return lightbars;
}
  
std::vector<Armor> ArmorDetector::findArmors(const std::vector<LightBar> & lightbars,const cv::Mat & bgr_img)
{
  std::vector<Armor> armors;
  
  // Loop all the pairing of lights
  for (auto light_1 = lightbars.begin(); light_1 != lightbars.end(); light_1++) {
    for (auto light_2 = std::next(light_1); light_2 != lightbars.end(); light_2++) {
      if (light_1->color != detect_color_ || light_2->color != detect_color_) continue;
      //if (light_1->color != light_2->color) continue;

      // 快速预筛选，避免不必要的复杂计算
      // 检查x距离是否合理（避免距离过远的灯条匹配）
      float x_distance = std::abs(light_1->center.x - light_2->center.x);
      float avg_length = (light_1->length + light_2->length) / 2;
      if (x_distance > avg_length * 5.0) continue; // 最大中心距离限制
      
      // 检查y距离是否合理（避免垂直距离过大的灯条匹配）
      float y_distance = std::abs(light_1->center.y - light_2->center.y);
      if (y_distance > avg_length * 2.0) continue; // 垂直距离限制
      
      // 检查两个灯条形成的包围矩形内是否包含其他灯条
      if (containLight(*light_1, *light_2, lightbars)) {continue;}
      
      ArmorType type = isArmor(*light_1, *light_2);
      if (type != ArmorType::UNKNOWN)
      {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        // // 数字识别
        classifier_->classify(bgr_img,armor);
        if (armor.name != "negative" && armor.confidence > confidence) {
          armors.emplace_back(armor);
        }
      }
    }
  }                       
  return armors;
}


// Check if there is another light in the boundingRect formed by the 2 lights
bool ArmorDetector::containLight(
  const LightBar & light_1, const LightBar & light_2, const std::vector<LightBar> & lights) const
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType ArmorDetector::isArmor(const LightBar & light_1, const LightBar & light_2) const
{
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                              : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > armor_min_light_ratio_;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (armor_min_small_center_distance_ <= center_distance &&
                              center_distance < armor_max_small_center_distance_) ||
                              (armor_min_large_center_distance_ <= center_distance &&
                              center_distance < armor_max_large_center_distance_);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < armor_max_angle_;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
      type = center_distance > armor_min_large_center_distance_ ? ArmorType::BIG : ArmorType::SMALL;
    } else {
      type = ArmorType::UNKNOWN;
    }

    return type;
}



cv::Mat ArmorDetector::showResult(const cv::Mat& bgr_img) const
{
    if (debug_)
    {
        // 根据输入图像大小自动计算显示尺寸，整体为原图尺寸
        int display_width = bgr_img.cols;
        int display_height = bgr_img.rows;
        
        // 确保最小尺寸
        display_width = std::max(display_width, 640);
        display_height = std::max(display_height, 480);
        
        // 增加宽度以容纳更宽的数字图像区域 (增加50像素)
        int number_img_width = 100; // 原来是 half_width，现在增加宽度
        display_width += 50;
        
        // 创建显示窗口，分为四个区域加一个数字图像区域
        cv::Mat debug_display(display_height, display_width, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // 计算每个子图像的尺寸
        int half_width = (display_width - number_img_width) / 2;
        int half_height = display_height / 2;
        
        // 1. 显示原始图像 (左上角)
        cv::Mat resized_bgr;
        cv::resize(bgr_img, resized_bgr, cv::Size(half_width, half_height), 0, 0, cv::INTER_LINEAR);
        resized_bgr.copyTo(debug_display(cv::Rect(0, 0, half_width, half_height)));
        cv::putText(debug_display, "Original Image", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 2. 显示二值化图像 (右上角)
        cv::Mat resized_binary;
        // 转换为灰度图
        cv::Mat gray;
        if (bgr_img.channels() == 3) {
            cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = bgr_img;
        }
        cv::Mat binary;
        // 应用阈值进行二值化
        cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY);
        cv::resize(binary, resized_binary, cv::Size(half_width, half_height), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(resized_binary, resized_binary, cv::COLOR_GRAY2BGR);
        resized_binary.copyTo(debug_display(cv::Rect(half_width, 0, half_width, half_height)));
        cv::putText(debug_display, "Binary Image", cv::Point(half_width + 10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 3. 显示灯条和角点 (左下角)
        cv::Mat lights_img = bgr_img.clone();
        for (const auto& light : lights_) {
            // 使用线条绘制灯条
            cv::line(lights_img, light.top, light.bottom, cv::Scalar(0, 255, 0), 3);
            cv::circle(lights_img, light.top, 3, cv::Scalar(0, 0, 255), 3);
            cv::circle(lights_img, light.bottom, 3, cv::Scalar(0, 0, 255), 3);
            cv::circle(lights_img, light.center, 3, cv::Scalar(0, 255, 0), 3);
        }
        cv::Mat resized_lights;
        cv::resize(lights_img, resized_lights, cv::Size(half_width, half_height), 0, 0, cv::INTER_LINEAR);
        resized_lights.copyTo(debug_display(cv::Rect(0, half_height, half_width, half_height)));
        cv::putText(debug_display, "Lights and Corners", cv::Point(10, half_height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(debug_display, "Lights Count: " + std::to_string(lights_.size()), cv::Point(10, half_height + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 4. 显示装甲板 (右下角)
        cv::Mat armor_img = bgr_img.clone();
        for (const auto& armor : armors_) {
            // 绘制装甲板轮廓
            std::vector<cv::Point2f> armor_points = {
                armor.left_light.top,
                armor.right_light.top,
                armor.right_light.bottom,
                armor.left_light.bottom
            };
            
            for (size_t i = 0; i < armor_points.size(); i++) {
                cv::line(armor_img, armor_points[i], armor_points[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }
            
            // 显示装甲板信息
            std::string armor_info = armor.name + " (" + std::to_string(armor.confidence).substr(0, 4) + ")";
            cv::putText(armor_img, armor_info, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            // 显示装甲板位姿信息
            std::string position_info = "X:" + std::to_string(armor.xyz_in_gimbal[0]).substr(0, 5) + 
                                      " Y:" + std::to_string(armor.xyz_in_gimbal[1]).substr(0, 5) + 
                                      " Z:" + std::to_string(armor.xyz_in_gimbal[2]).substr(0, 5);
            cv::putText(armor_img, position_info, 
                       cv::Point(armor.center.x, armor.center.y + 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            std::string angle_info = "Y:" + std::to_string(armor.ypr_in_gimbal[0] * 180.0 / CV_PI).substr(0, 5) + 
                                    " P:" + std::to_string(armor.ypr_in_gimbal[1] * 180.0 / CV_PI).substr(0, 5) + 
                                    " R:" + std::to_string(armor.ypr_in_gimbal[2] * 180.0 / CV_PI).substr(0, 5);
            cv::putText(armor_img, angle_info, 
                       cv::Point(armor.center.x, armor.center.y + 80), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::Mat resized_armor;
        cv::resize(armor_img, resized_armor, cv::Size(half_width, half_height), 0, 0, cv::INTER_LINEAR);
        resized_armor.copyTo(debug_display(cv::Rect(half_width, half_height, half_width, half_height)));
        cv::putText(debug_display, "Armors and Numbers", cv::Point(half_width + 10, half_height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 5. 显示所有数字图像 (整体右侧)
        if (!armors_.empty()) {
            cv::Mat all_numbers_img = getAllNumbersImage();
            if (!all_numbers_img.empty()) {
                cv::Mat resized_numbers_img;
                cv::Mat colored_numbers_img;
                
                // 确保数字图像是三通道的
                if (all_numbers_img.channels() == 1) {
                    cv::cvtColor(all_numbers_img, colored_numbers_img, cv::COLOR_GRAY2BGR);
                } else {
                    colored_numbers_img = all_numbers_img;
                }
                
                // 计算数字图像区域的尺寸 (右侧垂直条)
                cv::Rect number_area(half_width * 2, 0, number_img_width, display_height);
                // 缩放数字图像以适应显示区域
                cv::resize(colored_numbers_img, resized_numbers_img, cv::Size(number_img_width, display_height), 0, 0, cv::INTER_LINEAR);
                // 将缩放后的数字图像复制到显示区域
                resized_numbers_img.copyTo(debug_display(number_area));
                cv::putText(debug_display, "Number Images", cv::Point(half_width * 2 + 10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
        }
        
        return debug_display;
    }
    else
    {
        cv::Mat result_img = bgr_img.clone();
        // 绘制装甲板
        for (const auto& armor : armors_) {
            std::vector<cv::Point2f> armor_points = {
                armor.left_light.top,
                armor.right_light.top,
                armor.right_light.bottom,
                armor.left_light.bottom
            };
            
            for (size_t i = 0; i < armor_points.size(); i++) {
                cv::line(result_img, armor_points[i], armor_points[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }
            // 显示装甲板信息
            std::string armor_info = armor.name + " (" + std::to_string(armor.confidence).substr(0, 4) + ")";
            cv::putText(result_img, armor_info, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            // 显示装甲板位姿信息
            std::string position_info = "X:" + std::to_string(armor.xyz_in_gimbal[0]).substr(0, 5) + 
                                      " Y:" + std::to_string(armor.xyz_in_gimbal[1]).substr(0, 5) + 
                                      " Z:" + std::to_string(armor.xyz_in_gimbal[2]).substr(0, 5);
            cv::putText(result_img, position_info, 
                       cv::Point(armor.center.x, armor.center.y + 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            std::string angle_info = "Y:" + std::to_string(armor.ypr_in_gimbal[0] * 180.0 / CV_PI).substr(0, 5) + 
                                    " P:" + std::to_string(armor.ypr_in_gimbal[1] * 180.0 / CV_PI).substr(0, 5) + 
                                    " R:" + std::to_string(armor.ypr_in_gimbal[2] * 180.0 / CV_PI).substr(0, 5);
            cv::putText(result_img, angle_info, 
                       cv::Point(armor.center.x, armor.center.y + 80), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
        
        // 缩放到640*480
        cv::Mat resized_result;
        cv::resize(result_img, resized_result, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
     
        return resized_result;
    }
}

cv::Mat ArmorDetector::getAllNumbersImage() const
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

}

