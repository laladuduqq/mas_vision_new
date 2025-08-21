/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-27 17:37:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-21 21:29:18
 * @FilePath: /mas_vision_new/hikcamera/include/HikCamera.h
 * @Description: 
 */
#ifndef HIKCAMERA_H
#define HIKCAMERA_H

#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

struct CameraFrame {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
};

namespace hikcamera {
    class HikCamera {
    public:
        HikCamera();
        ~HikCamera();

        bool openCamera();
        void closeCamera();
        bool grabImage(cv::Mat& outImg);

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