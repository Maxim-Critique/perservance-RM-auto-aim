#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;
struct Gimbal_Angle
{
    double target_yaw;        // 目标yaw 
    double target_pitch;      // 目标pitch
    double manage_yaw;        // 控制yaw
    double yaw_vel;           // 控制yaw的速度
};

class Coordinate_converter
{
public:
    Coordinate_converter();
    cv::Point3d cam2abs(cv::Point3d position);                       // 相机转绝对
    cv::Point3d abs2cam(cv::Point3d position);                       // 绝对转相机
    cv::Point3f changeCoor(cv::Point3f point);                       // 相机转枪管
    Gimbal_Angle GetAngle(cv::Point3f position);                     // 计算云台角度
    void GetTFMat(const double &x, const double &y, const double &z);// 获取相机转绝对的矩阵
    Matrix4d GetTrichMat(double yaw,Point3d point);                  // 获取三分法转坐标系矩阵
    Matrix3d tf_mat;                                                 // 相机转绝对的矩阵
    Eigen::Vector3d camera_gimbal;
};

// 旋转矩阵生成函数
Matrix3d GetRotMatX(double x);
Matrix3d GetRotMatY(double y);
Matrix3d GetRotMatZ(double z);
Matrix3d GetRotMatXYZ(double x, double y, double z);

// 工具函数
double R2D(double rad);
double D2R(double degree);
double GetDistance(cv::Point3d p);
double GetDistance(cv::Point2d p,cv::Point2d q);
extern Coordinate_converter C_C;

class Paint
{
public:
    void Curve2(double value1, double value2);
    void KF_Curve(double real_data, double pridect_data);
    void Speed_Curve(double real_data);
    vector<double> vec_real;
    vector<double> vec_pridect;
    vector<double> vec_speed;
};

static Paint painting;