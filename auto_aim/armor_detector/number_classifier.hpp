/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-23 10:21:23
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-24 14:45:51
 * @FilePath: /mas_vision_new/auto_aim/armor_detector/number_classifier.hpp
 * @Description: 
 */
#ifndef _NUMBER_CLASSIFIER_H_
#define _NUMBER_CLASSIFIER_H_

#include "armor_types.hpp"
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
namespace auto_aim {
    class NumberClassifier {
        public:
            NumberClassifier(const std::string &model_path,const std::string &label_path,
                            const double threshold,const std::vector<std::string> &ignore_classes);
            void classify(const cv::Mat &src, Armor &armor);
            cv::Mat extractNumber(const cv::Mat &src,Armor &armor) const;
            void eraseIgnoreClasses(std::vector<Armor> &armors);
        private:
            double threshold;
            std::mutex mutex_;
            cv::dnn::Net net_;
            std::vector<std::string> class_names_;
            std::vector<std::string> ignore_classes_;

    };

}

#endif // _NUMBER_CLASSIFIER_H_