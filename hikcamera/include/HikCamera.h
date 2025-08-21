/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-27 17:37:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 20:05:25
 * @FilePath: /mas_vision/hikcamera/include/HikCamera.h
 * @Description: 
 */
#ifndef HIKCAMERA_H
#define HIKCAMERA_H

#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

namespace hikcamera {
    class HikCamera {
    public:
        HikCamera();
        ~HikCamera();

        bool openCamera();
        void closeCamera();
        bool grabImage(cv::Mat& outImg);

        // 相机校准相关函数
        bool calibrateCamera(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                            const std::vector<std::vector<cv::Point2f>>& imagePoints,
                            const cv::Size& imageSize,
                            cv::Mat& cameraMatrix,
                            cv::Mat& distCoeffs);
        bool saveCalibration(const std::string& filename,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs,
                            const cv::Size& imageSize);

    private:
        void* handle;           // Camera handle
        bool isConnected;       // Connection status
        char serialNumber[64];  // Device serial number
        unsigned int g_nPayloadSize; // Payload size for image buffer
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
        int fail_conut_ = 0;
    };
}

#endif // HIKCAMERA_H