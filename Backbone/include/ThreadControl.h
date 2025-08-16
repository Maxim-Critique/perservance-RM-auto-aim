#ifndef THREAD_H
#define THREAD_H

#pragma once
#include "IAmFuture.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "thread"
#include "unistd.h"               //UNIX系统服务的函数原型
#include "chrono"                 //是标准模板库中与时间有关的头文
#include "fstream"                //输入输出到指定文件
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
};

/**
 * @brief 图像信息，用于线程之间的图像传输,and num
 */
struct ImageData { 
    cv::Mat img;
    uint64_t timestamp; // 新增时间戳字段
};

#endif // THREAD_H



double now();
bool shootable();//发射间隔
//在画面中展示装甲板参数，框装甲板
void ConnectPoints(Armor& armor, cv::Mat& img);
//移除小装甲板
void RemoveSmallArmors(Armors& armorss);
//防倒影函数
void ProcessArmors(Armors& armorss);
//在画面中展示Yaw,Pitch
void ShowYawPitchRoll(Mat &image);
//防甩头
bool Protection(Armors armors,string x,double yaw);
//获得ROI
Rect MakeROI(Point2d found_armor, int x, int y, Mat img);
//发ID
int FoundID(string x);
//选择ds
void ChooseDataSave (const std::chrono::steady_clock::time_point &camera_time);