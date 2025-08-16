#pragma once
#include <queue>
#include <Base.h>
#include "SerialPort.h"
#include <fstream>
#include "find_armor.h"
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
    //世界坐标定义
    vector<Point3f> world_points;

    //相机内置参数， 畸变函数定义
    Mat Intrinsic_Matrix;
    Mat Distortion;

    //平移向量， 旋转矩阵定义
    Mat rvec;
    Mat tvec;

    Point2d ProjectTo2D(Point3d point3D);
    void readConfig(Armor armor);  
    double distance_x;
};

static PNP pnp_armor;
