#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H
#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Base.h"
#include <deque>
#include <chrono>

using namespace std;
using namespace cv;



struct Data_Save {
    double camera_yaw_angle;
    double camera_pit_angle;  
    double camera_roll_angle;
    int Buff_or_Armor = 2;       // 自瞄或大符
    bool buff_mode;          // 大符或小符
    bool choose_aim;         // 操作手是否开启自瞄
    float PitchCompensation; // 大符Pitch补偿
    float YawCompensation;   // 大符Yaw补偿
    int color;               // 敌方颜色
    int spinning_top;        // 是否进入小陀螺
};

struct Data_Before{
    double yaw;
    double pitch;
    std::chrono::steady_clock::time_point get_time;
};
    

extern Data_Save ds;
extern bool bool_serial;
struct Data_Get
{
    int size;
    u_char raw_data[21];
    u_char MAI_raw_data[21];

    void Vision2Control(int32_t x, int32_t y,int32_t ui_x, int32_t ui_y, uint8_t found, uint8_t fire);
    // void Vision2Control();
};

extern Data_Get dg;

class SerialPort
{
public:
    SerialPort(const char *filename);

    void restart_serial(void); // 尝试重连的函数
    int read_data();
    void send_data(const struct Data_Get &data);
    
    int fd;
    int last_fd;
    bool success_;

private:
    const char *file_name_;
    int buadrate_;
    float last_bullet_speed;
    
};
/*--------------- serial data transmission ----------------*/
static SerialPort serial_(SERIAL_PATH);
/*--------------- serial data transmission ----------------*/
#endif