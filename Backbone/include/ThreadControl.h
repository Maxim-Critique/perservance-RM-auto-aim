#ifndef THREAD_H
#define THREAD_H

#pragma once
#include "IAmFuture.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "thread"
#include "unistd.h"
#include "chrono"
#include "fstream"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include "Camera.h"
#include "Base.h"
#include "HikCamera.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string>
#include <time.h> 
#include "SerialPort.h"
#include <csignal>  
#include <atomic>
#include <stdlib.h>
#include <systemd/sd-daemon.h>
#include "Buff_Task.h"
#include <mutex>

using namespace cv;
using namespace std;

/**
 * @brief 线程管理类
 * 负责图像生成、图像处理、串口数据接收
 */
class ThreadControl
{
public:
    ThreadControl();          // 线程管理构造函数，用于线程中变量初始化
    void ImageProduce();      // 短焦摄像头获取图像线程
    void ImageProcess();      // 图像处理线程，用于自瞄，能量机关识别
    void GetSTM32();
    void PredictProcess();
    
private:
    bool buff_or_armor;
    YAML::Node readYAML(const std::string& filename);
    double exposure_time_aim;
    double exposure_time_buff;
    bool video_write_enable = false; 
    std::unique_ptr<cv::VideoWriter> video_writer;
    std::mutex video_mutex;
    bool show;
    
    // 新增：共享帧和互斥锁
    cv::Mat shared_frame;
    std::mutex frame_mutex;
    std::atomic<bool> frame_updated{false};
};

/**
 * @brief 图像信息，用于线程之间的图像传输
 */
struct ImageData { 
    cv::Mat img;
    uint64_t timestamp;
};

#endif // THREAD_H

//在画面中展示Yaw,Pitch,Roll
void ShowYawPitchRoll(Mat &image);