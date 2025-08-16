#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include "Camera.h"
#include "Buff_Detect.h"
#include <chrono>
#include <thread>
#include <condition_variable>
#include "CAlphaEKF.h"
#include "Buff_Predict.h"
using namespace cv;
using namespace std;

class Buff_Task
{
public:
    Buff_Task();                                            // 初始化
    ~Buff_Task();                                           // 析构函数
    bool SeizeGlory(Mat &src, u_int64_t time);               // 函数主接口
    
private:
    // void PredictionThread();                                // 预测线程函数
    // Point2d GetLatestPrediction();             // 获取最新预测结果
    void Transmit(Point3d Aim);
    void GetThetaPhi();
    // std::unique_ptr<BigNewtonGauss>Gauss;
    std::unique_ptr<Buff_Discover> Buff_discover;           // 识别类指针
    // std::unique_ptr<CAlphaEKF>EKF;                  
    double phi;
    double theta;
    // std::mutex queue_mutex;                                 // 队列互斥锁
    // std::mutex prediction_mutex;                            // 预测结果互斥锁
    std::condition_variable cv;                             // 条件变量
    bool if_tuili;
    double compensate_pitch;                                // pitch补偿
    double compensate_yaw;                                  // yaw补偿
    double last_time;
    Point3d last_center;
    bool init;
    int show;
    int last_angle = 0;
    int rotatedirection = 0;
    int rotatedirection_number = 0;
};