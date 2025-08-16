#pragma once
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include<chrono>
#include <mutex>
namespace Camera
{
    class HikCamera {
        friend  void imageCallback(unsigned char *data, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    private:
        unsigned int mIndex=0;
        void* mHandle;
        uint64_t mLastTimestampMs = 0;
    public:
        int mCount;
        float mFPS;
        std::chrono::steady_clock::time_point mLastTime;
        cv::Mat mRawImage;
        uint64_t getLastTimestamp() const;

        cv::Mat convertToBGR(cv::Mat image);
        void open();
        void close();
        /**
         * @details 单位us
         * @param exposure_time
         */
        void setExposureTime(float exposure_time);
        /**
         * @param channel 0 Red 1 Green 2 Blue
         * @param ratio
         */
        void setWhiteBalance(int channel,float ratio);

        /**
         *
         * @param value 0~16
         */
        void setGain(float value);
        float getFPS();
        cv::Mat getImage();
        void setGamma(float value);

        std::chrono::steady_clock::time_point GetTime();
    };

}

