/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "Base.h"
using namespace cv;
using namespace std;
extern atomic<int> CameraMode;//1 是外触发
extern Mat ExternalImage;//外触发时用的图像
extern atomic<double> GALAXY_EXPOSURE_TIME;
static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

// ---------------------------- galaxy ----------------------------
class CameraDevice
{
public:
    CameraDevice();
    ~CameraDevice();
    int init();
    void getImage(Mat &img);
     GX_DEV_HANDLE hDevice;
    uint64_t getFrameNumber();    
    void CameraModeSwitch();//切换触发模式和曝光时间
private:
    double Curr_EXPOSURE_TIME; //目前的曝光时间
    int CameraCurrMode;//目前相机的触发模式 1 是外触发，如果CameraMode!= CameraCurrMode，修改触发模式
    GX_STATUS status;
    // GX_DEV_HANDLE hDevice;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    Mat src;
    uint64_t nFrameNum;};

//----------------------------- v4l2 ------------------------------
class CaptureVideo
{
public:
    CaptureVideo(const char *device, unsigned int in_size_buffer = 1);
    ~CaptureVideo();
    bool startStream();
    bool closeStream();
    char *video_name(char *format = ".avi");

    // setting

    bool setExposureTime(bool in_exp, int in_t);
    bool setVideoFormat(int in_width, int in_height, bool in_mjpg = 1);
    bool setVideoFPS(int in_fps);
    bool setBufferSize(int in_buffer_size);

    // restarting
    bool changeVideoFormat(int in_width, int in_height, bool in_mjpg = 1);
    void restartCapture();

    // getting
    void imread(Mat &image);
    bool getVideoSize(int &width, int &height);
    int getFrameCount()
    {
        return current_frame;
    }
    int getFD()
    {
        return fd;
    }
    void info();

    CaptureVideo &operator>>(Mat &image);

private:
    void cvtRaw2Mat(const void *data, Mat &image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int in_fd, int in_request, void *arg);

    struct MapBuffer
    {
        void *ptr;
        unsigned int size;
    };
    unsigned int capture_width;
    unsigned int capture_height;
    unsigned int format;
    int fd;
    unsigned int buffer_size;
    unsigned int buffer_index;
    unsigned int current_frame;
    MapBuffer *mb;
    const char *video_path;
};
