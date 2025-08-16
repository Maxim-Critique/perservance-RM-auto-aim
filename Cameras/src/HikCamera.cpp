#include "HikCamera.h"
std::mutex imageMutex;
namespace Camera
{
    void imageCallback(unsigned char *data, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser)
    {
        auto target =static_cast<HikCamera*>(pUser);
        if (pFrameInfo && data)
        {   
            std::lock_guard<std::mutex> lock(imageMutex);
            target->mLastTime = std::chrono::steady_clock::now();

            uint64_t devTimeStamp = (static_cast<uint64_t>(pFrameInfo->nDevTimeStampHigh) << 32) | pFrameInfo->nDevTimeStampLow;
            target->mLastTimestampMs = devTimeStamp;
            // 确保 pFrameInfo 和 data 都有效
            target->mRawImage = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, data);
            
            // std::cout <<pFrameInfo->nHeight<< "Received valid image frame." <<pFrameInfo->nWidth<< std::endl;
        }
        else
        {
            std::cout << "Invalid frame data or pFrameInfo is null" << std::endl;
        }

    }
}
cv::Mat Camera::HikCamera::convertToBGR(const cv::Mat image)
{
    cv::Mat result;
    if(!image.empty()){ cv::cvtColor(image,result,cv::COLOR_BayerBG2BGR);}
#ifdef DEBUG
    else{std::cout<<"image empty"<<std::endl;}
#endif
    return result;
}



void Camera::HikCamera::open()
{
    MV_CC_DEVICE_INFO_LIST device_list;

    while (1)
    {
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        // 尝试枚举设备
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (nRet != MV_OK || device_list.nDeviceNum == 0)
        {
            continue;
        }
        else
        {
            break;
            //std::cout << "Device found, proceeding to open." << std::endl;
        }
    }

    // 创建设备句柄
    int nRet = MV_CC_CreateHandle(&mHandle, device_list.pDeviceInfo[mIndex]);
    if (nRet != MV_OK)
    {
        std::cerr << "Failed to create device handle." << std::endl;
        return;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(mHandle);
    if (nRet != MV_OK)
    {
        std::cerr << "Failed to open device." << std::endl;
        return;
    }

    // 注册图像回调函数
    MV_CC_RegisterImageCallBackEx(mHandle, imageCallback, this);

    // 设置曝光模式为自动
    MV_CC_SetEnumValue(mHandle, "ExposureMode", 1);

    // 开始抓取图像
    MV_CC_StartGrabbing(mHandle);
}

void Camera::HikCamera::close()
{
    MV_CC_StopGrabbing(mHandle);
    MV_CC_CloseDevice(mHandle);
    MV_CC_DestroyHandle(mHandle);
}

//fmt

void Camera::HikCamera::setExposureTime(float exposure_time)
{
    MV_CC_SetFloatValue(mHandle,"ExposureTime",exposure_time);
}


void Camera::HikCamera::setWhiteBalance(int channel, float ratio)
{
    switch (channel)
    {
        case 0:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",0);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        case 1:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",1);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        case 2:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",2);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        default:
            break;
    }
}

void Camera::HikCamera::setGain(float value)
{
    //MV_CC_SetFloatValue(mHandle, "Gain", value);
    MV_CC_SetEnumValue(mHandle, "GainAuto", 2);
}

void Camera::HikCamera::setGamma(float value)
{
    // 1. 打开伽马校正使能
    MV_CC_SetBoolValue(mHandle, "GammaEnable", true);
    
    // 2. 设置gamma类型，user：1，sRGB：2
    MV_CC_SetEnumValue(mHandle, "GammaSelector", 1); // 1 代表 user
    
    // 3. 设置gamma值，推荐范围0.5-2，1为线性拉伸
    MV_CC_SetFloatValue(mHandle, "Gamma", value); // 设置 Gamma 值为 1.0

}

float Camera::HikCamera::getFPS()
{
    return mFPS;
}

cv::Mat Camera::HikCamera::getImage()
{
    cv::Mat image;
    int attempts = 5; // 限制重试次数
    while (attempts > 0)
    {
        {
            std::lock_guard<std::mutex> lock(imageMutex);
            if (!mRawImage.empty())
            {
                image = mRawImage.clone(); // 复制图像数据
                break; // 成功获取图像后退出循环
            }
        }
#ifdef DEBUG
        std::cout << "Image is empty, retrying..." << std::endl;
#endif
        attempts--;
    }
    return convertToBGR(image);
}

uint64_t Camera::HikCamera::getLastTimestamp() const {
    std::lock_guard<std::mutex> lock(imageMutex);
    return mLastTimestampMs;
}

std::chrono::steady_clock::time_point Camera::HikCamera::GetTime(){
    return mLastTime;
}