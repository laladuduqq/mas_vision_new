/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-23 10:20:58
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-29 19:23:56
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/number_classifier.cpp
 * @Description: 
 */
#include "number_classifier.hpp"
#include <fstream>
#include <algorithm>
#include <iostream>
#include "armor_types.hpp"
#include "ulog.hpp"

namespace auto_aim {
NumberClassifier::NumberClassifier(const std::string &model_path,
                                   const std::string &label_path,
                                   const double thre,const std::vector<std::string> &ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
    // Load the DNN model
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);

        // Check if the model was loaded successfully
        if (net_.empty()) {
            ULOG_ERROR_TAG("NumberClassifier","模型文件加载失败 %s",model_path.c_str());
            return;
        }

        ULOG_INFO_TAG("NumberClassifier","成功加载模型文件: %s",model_path.c_str());
    } catch (const cv::Exception& e) {
        ULOG_ERROR_TAG("NumberClassifier","模型文件加载失败 %s",model_path.c_str());
        return;
    }

    // Load class names
    try {
        std::ifstream label_file(label_path);
        if (!label_file.is_open()) {
            ULOG_ERROR_TAG("NumberClassifier","无法打开标签文件: %s",label_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(label_file, line)) {
            class_names_.push_back(line);
        }

        ULOG_INFO_TAG("NumberClassifier","成功加载标签文件: %s",label_path.c_str());
        ULOG_INFO_TAG("NumberClassifier","标签数量: %d",class_names_.size());
    } catch (const std::exception& e) {
        ULOG_ERROR_TAG("NumberClassifier","加载标签文件时发生异常: %s",e.what());
        return;
    }
    ULOG_INFO_TAG("NumberClassifier", "数字识别器初始化完成");
}


cv::Mat NumberClassifier::extractNumber(const cv::Mat &src,Armor &armor) const {
    // Light length in image
    static const int light_length = 12;
    // Image size after warp
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // Number ROI size
    static const cv::Size roi_size(20, 28);
    static const cv::Size input_size(28, 28);

    // Check if src image is valid
    if (src.empty() || src.cols <= 0 || src.rows <= 0) {
        ULOG_WARNING_TAG("NumberClassifier", "Invalid source image");
        return cv::Mat();
    }

    // Check if armor light points are valid
    if (armor.left_light.center.x < 0 || armor.left_light.center.y < 0 ||
        armor.right_light.center.x < 0 || armor.right_light.center.y < 0 ||
        armor.left_light.top.x < 0 || armor.left_light.top.y < 0 ||
        armor.left_light.bottom.x < 0 || armor.left_light.bottom.y < 0 ||
        armor.right_light.top.x < 0 || armor.right_light.top.y < 0 ||
        armor.right_light.bottom.x < 0 || armor.right_light.bottom.y < 0) {
        ULOG_WARNING_TAG("NumberClassifier", "Invalid armor light points");
        return cv::Mat();
    }

    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
        armor.left_light.bottom, armor.left_light.top, armor.right_light.top, armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    
    // Check if warp_width is valid
    if (warp_width <= 0) {
        ULOG_WARNING_TAG("NumberClassifier", "Invalid warp width");
        return cv::Mat();
    }
    
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    
    // Check if the points are valid before computing perspective transform
    for (int i = 0; i < 4; i++) {
        if (!std::isfinite(lights_vertices[i].x) || !std::isfinite(lights_vertices[i].y) ||
            !std::isfinite(target_vertices[i].x) || !std::isfinite(target_vertices[i].y)) {
            ULOG_WARNING_TAG("NumberClassifier", "Invalid perspective transform points");
            return cv::Mat();
        }
    }
    
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    
    // Check if rotation_matrix is valid
    if (rotation_matrix.empty()) {
        ULOG_WARNING_TAG("NumberClassifier", "Failed to compute perspective transform matrix");
        return cv::Mat();
    }
    
    try {
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
    } catch (const cv::Exception& e) {
        ULOG_WARNING_TAG("NumberClassifier", "Exception in warpPerspective: %s", e.what());
        return cv::Mat();
    }

    // Get ROI
    try {
        number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
    } catch (const cv::Exception& e) {
        ULOG_WARNING_TAG("NumberClassifier", "Exception in ROI extraction: %s", e.what());
        return cv::Mat();
    }

    // Binarize
    try {
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::resize(number_image, number_image, input_size);
    } catch (const cv::Exception& e) {
        ULOG_WARNING_TAG("NumberClassifier", "Exception in image processing: %s", e.what());
        return cv::Mat();
    }
    
    return number_image;
}

void NumberClassifier::classify(const cv::Mat &src, Armor &armor) {
    // Check if the network is loaded
    if (net_.empty()) {
        armor.confidence = 0.0;
        armor.name = ArmorName::negative;
        return;
    }

    try {
        // Extract number image
        cv::Mat number_image = extractNumber(src, armor);
        
        // Check if number image is empty
        if (number_image.empty()) {
            armor.confidence = 0.0;
            armor.name = ArmorName::negative;
            return;
        }

        // Normalize
        cv::Mat input = number_image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);

        // Set the input blob for the neural network
        std::lock_guard<std::mutex> lock(mutex_);
        net_.setInput(blob);

        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward().clone();

        // Decode the output
        double confidence;
        cv::Point class_id_point;
        cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        // Check if label_id is valid
        if (label_id >= 0 && label_id < static_cast<int>(class_names_.size())) {
            armor.confidence = confidence;
            armor.name = class_names_[label_id];
            armor.number_img = number_image.clone();
        } else {
            armor.confidence = 0.0;
            armor.name = ArmorName::negative;
        }
        
        // Check if should ignore this armor after classification
        // Check if confidence is below threshold
        if (armor.confidence < threshold) {
            armor.confidence = 0.0;
            armor.name = ArmorName::negative;
            return;
        }
        
        // Check for armor type mismatch
        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::BIG) {
            mismatch_armor_type = armor.name == "outpost" || armor.name == "2" || armor.name == "sentry" || armor.name == "5" || armor.name == "4";
        } else if (armor.type == ArmorType::SMALL) {
            mismatch_armor_type = armor.name == "1" || armor.name == "base";
        }
        
        if (mismatch_armor_type) {
            armor.confidence = 0.0;
            armor.name = ArmorName::negative;
            return;
        }
    } catch (const cv::Exception& e) {
        ULOG_ERROR_TAG("NumberClassifier","数字分类时发生OpenCV异常: %s",e.what());
        armor.confidence = 0.0;
        armor.name = ArmorName::negative;
    }
}
}
