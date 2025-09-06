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
    PNP();
    
    void Armorpnp(Armor &armor, Mat &src_img);
    void BuffPnP(vector<Point2f> Buff_points, Point3d &pose, Point3d &Rpose, Point3d &Aimpose, double &roll, double &yaw);
    // 世界坐标定义PNP
    static const vector<Point3f> world_points_big;                                 
    static const vector<Point3f> world_points_small;
    // 相机内置参数， 畸变函数定义
    static Mat Intrinsic_Matrix;
    static Mat Distortion;
    // 解算主接口
    void Solve(Armor &armor, Mat &img, Gimbal_Angle &initial_abs);
    // 可视化
    void show(Armor &armor,double yaw,std::vector<cv::Point2f> &image_points);
    double sys_yaw;                                                         // 云台yaw角
    Eigen::Matrix3d R_armor2world = Eigen::Matrix<double, 3, 3>::Zero();    // 相机-》云台坐标系旋转矩阵
    Point2d ProjectTo2D(Point3d point3D);
    void readConfig(Armor armor);  
    double distance_x;
};

