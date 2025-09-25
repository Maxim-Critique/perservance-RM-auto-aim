#ifndef BASE_H
#define BASE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#pragma once 

using namespace std;
using namespace cv;
extern int keyboard_key;
//-------------------------------------------------------------------
// 功能状态开关
//-------------------------------------------------------------------
#define PI 3.1415926
#define NOT_FRAME_DECIMATION       ///*非视频抽帧*///
//#define IS_FRAME_DECIMATION      ///*视频抽帧开关*///
#define VIDEO                      ///*读取视频开关*///
// #define CAMERA                     ///*读取摄像头开关海康*///
// #define CAMERA1                 ///*读取摄像头开关大恒*///             
// #define WRITE                   ///*视频写入开关*///
//#define PIC                      ///*debug开关*///
#define SHORT                      ///*短焦距开关*///
#define CapturingPhoto             ///*拍照开关*///
#define VIDEO_ADD "/home/killer_machine/WRITE/videos/5.avi"
//waitkey的值
#define DEBUG_SERIAL 0

#define ONE 1
#define ONE_SIZE ONE + 1
// 串口
#define SERIAL_VMIN 16
#define SERIAL_VTIME 0
#define READ_SIZE 24
#endif // BASE_H
// serial port paths 串口路径
#define SERIAL_PATH "/dev/ttyUSB0"

// 弹速
#define bullet_speed 24

#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))

extern std::string YamlPath;
