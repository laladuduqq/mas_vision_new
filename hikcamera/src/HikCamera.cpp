#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include "HikCamera.h"
#include "MvCameraControl.h"
#include "ulog.hpp"

hikcamera::HikCamera::HikCamera() : handle(NULL), isConnected(false) {
    memset(serialNumber, 0, sizeof(serialNumber));
}

hikcamera::HikCamera::~HikCamera() {
    closeCamera();
}

bool hikcamera::HikCamera::openCamera() {
    int nRet = MV_OK;

    // Initialize SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        ULOG_ERROR_TAG("hikcamera","Initialize SDK fail! nRet [0x%x]", nRet);
        return false;
    }


    // ch:枚举设备 | en:Enum device
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Enum Devices fail! nRet [0x%x]", nRet);
        return false;
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            ULOG_INFO_TAG("hikcamera","[device %d]:", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo){break;} 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        ULOG_ERROR_TAG("hikcamera","Find No Devices!");
        return false;
    }

    // ch:选择设备并创建句柄,默认选择第一个设备 | en:Select device and create handle, default to select the first device
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Create Handle fail! nRet [0x%x]", nRet);
        return false;
    }

    // ch:打开设备 | en:Open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Open Device fail! nRet [0x%x]", nRet);
        return false;
    }

    // 默认参数
    float exposure_time = 5000.0f;
    float gain = 10.0f;

    try {
        cv::FileStorage fs("config/camera_set.json", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            ULOG_ERROR_TAG("hikcamera","无法打开 JSON 文件，使用默认值");
        } else {
            cv::FileNode cam = fs["camera"];
            if (!cam.empty()) {
                exposure_time = (float)cam["exposuretime"];  // 支持 int/float 自动转换
                gain          = (float)cam["gain"];
            }
            fs.release();
            ULOG_INFO_TAG("hikcamera","已加载相机参数");
        }
    } catch (const cv::Exception& e) {
        ULOG_ERROR_TAG("hikcamera","OpenCV 异常%s",e.what());

    }

    // Exposure time
    // 获取并设置曝光
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    if (MV_OK == nRet) {
        ULOG_DEBUG_TAG("hikcamera","exposure time current value:%f", stExposureTime.fCurValue);
        if (exposure_time >= stExposureTime.fMin && exposure_time <= stExposureTime.fMax) {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time);
            if (MV_OK == nRet) ULOG_INFO_TAG("hikcamera","set exposure time:%f OK!",exposure_time);
            else ULOG_ERROR_TAG("hikcamera","set exposure time failed! nRet [%x]\n", nRet);
        } else {
            ULOG_ERROR_TAG("hikcamera","exposure value invalid!");
        }
    } else {
        ULOG_ERROR_TAG("hikcamera","Get ExposureTime failed! nRet [0x%x]", nRet);
        return false;
    }

    // Gain
    // 获取并设置gain
    MVCC_FLOATVALUE stGain = {0};
    nRet = MV_CC_GetFloatValue(handle, "Gain", &stGain);
    if (MV_OK == nRet) {
        ULOG_DEBUG_TAG("hikcamera","gain current value:%f", stGain.fCurValue);
        if (gain >= stGain.fMin && gain <= stGain.fMax) {
            nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
            if (MV_OK == nRet) ULOG_INFO_TAG("hikcamera","set gain:%f OK!", gain);
            else ULOG_ERROR_TAG("hikcamera","set gain failed! nRet [%x]", nRet);
        } else {
            ULOG_ERROR_TAG("hikcamera","gain value invalid!");
        }
    } else {
        ULOG_ERROR_TAG("hikcamera","Get Gain failed! nRet [0x%x]", nRet);
        return false;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValueEx(handle,"GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK)
            {
                ULOG_ERROR_TAG("hikcamera","Set Packet Size fail nRet [0x%x]!", nRet);
            }
        }
        else  
        {
            ULOG_WARNING_TAG("hikcamera","Get Packet Size fail nRet [0x%x]!", nPacketSize);
        }
    }


    // ch:设置触发模式为off | en:Set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Set Trigger Mode fail! nRet [0x%x]", nRet);
        return false;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Get PayloadSize fail! nRet [0x%x]", nRet);
        return false;
    }
    g_nPayloadSize = stParam.nCurValue;

    // ch:开始取流 | en:Start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        ULOG_ERROR_TAG("hikcamera","Start Grabbing fail! nRet [0x%x]", nRet);
        return false;
    }
    
    isConnected = true;
    
    return true;
}

void hikcamera::HikCamera::closeCamera() {
    if (isConnected) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = NULL;
        isConnected = false;
        MV_CC_Finalize();
    }
}

bool hikcamera::HikCamera::grabImage(cv::Mat& outImg) {
    if (!isConnected) {
        return false;
    }

    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

    int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
    if (nRet == MV_OK && stOutFrame.pBufAddr) {
        // 优先用海康SDK转换为RGB8
        if (stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8) {

            // 设置插值方法为均衡
            MV_CC_SetBayerCvtQuality(handle, 1);

            int width = stOutFrame.stFrameInfo.nWidth;
            int height = stOutFrame.stFrameInfo.nHeight;
            int outBufSize = width * height * 3 + 2048;
            std::vector<unsigned char> rgbBuf(outBufSize);

            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            stConvertParam.nWidth = width;
            stConvertParam.nHeight = height;
            stConvertParam.pSrcData = (unsigned char*)stOutFrame.pBufAddr;
            stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLenEx;
            stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = rgbBuf.data();
            stConvertParam.nDstBufferSize = outBufSize;

            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK == nRet) {
                outImg = cv::Mat(height, width, CV_8UC3, rgbBuf.data()).clone();
                cv::cvtColor(outImg, outImg, cv::COLOR_RGB2BGR); // 关键：RGB转BGR
            } else {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                MV_CC_FreeImageBuffer(handle, &stOutFrame);
                return false;
            }
        } else if (stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
            outImg = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3, stOutFrame.pBufAddr).clone();
        } else {
            cv::Mat imgMono(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC1, stOutFrame.pBufAddr);
            cv::cvtColor(imgMono, outImg, cv::COLOR_GRAY2BGR);
        }

        MV_CC_FreeImageBuffer(handle, &stOutFrame);
        fail_conut_ =0;
        
        return true;
    } else {
        ULOG_ERROR_TAG("hikcamera","Get image buffer failed! nRet [0x%x]\n", nRet);
        MV_CC_StopGrabbing(handle);
        MV_CC_StartGrabbing(handle);
        fail_conut_++;
        if (fail_conut_ > 5)
        {
            closeCamera();
        }
    }
    return false;
}



bool hikcamera::HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        ULOG_ERROR_TAG("hikcamera","The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        ULOG_INFO_TAG("hikcamera","CurrentIp: %d.%d.%d.%d" , nIp1, nIp2, nIp3, nIp4);
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        ULOG_INFO_TAG("hikcamera","Serial Number: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        ULOG_INFO_TAG("hikcamera","Device Number: %d", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        ULOG_INFO_TAG("hikcamera","Serial Number: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        ULOG_INFO_TAG("hikcamera","Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        ULOG_INFO_TAG("hikcamera","Serial Number: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        ULOG_INFO_TAG("hikcamera","Model Name: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        ULOG_INFO_TAG("hikcamera","Serial Number: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        ULOG_INFO_TAG("hikcamera","Model Name: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        ULOG_INFO_TAG("hikcamera","UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        ULOG_INFO_TAG("hikcamera","Serial Number: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        ULOG_INFO_TAG("hikcamera","Model Name: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
    }
    else
    {
        ULOG_ERROR_TAG("hikcamera","Not support.");
    }

    return true;
}