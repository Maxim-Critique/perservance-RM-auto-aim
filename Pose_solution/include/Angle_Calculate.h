#pragma once
#include <queue>
#include <Base.h>
#include "SerialPort.h"
#include <fstream>
#include "find_armor.h"
#include "Coordinate.h"
#include <yaml-cpp/yaml.h>
using namespace std;
using namespace cv;

class PNP
{
public:
    // use solvePnP and add compensation
    PNP(Data_Save ds);

    void BuffPnP(vector<Point2f> Buff_points, Point3d &pose, Point3d &Rpose, Point3d &Aimpose, double &roll, double &yaw);

    // 解算主接口
    void Solve(Armor &armor);
private:
    cv::Mat rvec, tvec;                                                        // 旋转与平移向量  
    vector<Point3f>world_points;                                               // 装甲板3维坐标
    static const vector<Point3f> world_points_big;                             // 大装甲板3维坐标    
    static const vector<Point3f> world_points_small;                           // 小装甲板3维坐标  
    static const Mat Intrinsic_Matrix;                                         // 相机内参
    static const Mat Distortion;                                               // 畸变函数
    static const Eigen::Vector3d camera_gimbal;                                // pnp系下的相机相对云台位置
    double sys_yaw;                                                            // 云台yaw角
    Eigen::Matrix3d R_armor2world = Eigen::Matrix<double, 3, 3>::Zero();       // 相机->云台坐标系旋转矩阵
    Eigen::Vector3d t_armor2world;                                             // pnp系下的绝对系坐标
    void readConfig(Armor armor);  
    vector<Point2f> Reprojection(double yaw);                                  // 重投影函数
    double distance_x;
};

