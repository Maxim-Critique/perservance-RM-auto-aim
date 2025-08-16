#include <opencv2/opencv.hpp>
#include <iostream>
#pragma once
using namespace cv;
using namespace std;
struct Contour_RotatedRect {
    RotatedRect rect;
    Mat image;
    vector<Point>contour;
    std::vector<Point2f> ordered_pnp_points; // 有序顶点
    Point3d pose;
    double roll;
    double yaw;
};

class Buff_Discover
{
public:
    Buff_Discover();
    Mat Buff_img_deal(Mat src);                          // 预处理      
    bool FoundBuff(Mat &src,Mat &img);                   // 轮廓查找，识别部分的主接口
    Contour_RotatedRect sl;                              // 待击打的流水灯
    Point3d Rpose,Aimpose;
private:
    void ExtractCirRegions(cv::Mat src,Mat &draw_img);   // 找靶环
    void ExtractRectRegions(cv::Mat src,Mat draw_img);   // 找流水灯 
    Contour_RotatedRect ProcessBH();                     // mlp验证靶环
    Contour_RotatedRect ProcessSL(Point2f center_bh);    // 流水灯验证，利用靶环
    std::vector<std::vector<cv::Point>> contours;        // 存放第一次二值化轮廓
    vector<Contour_RotatedRect>contour_rect_bh;          // 靶环的结构体
    vector<Contour_RotatedRect>contour_rect_sl;          // 流水灯结构体
    Point2d bh_center;
    int show;
    Point3d last_Rpose;
    Point3d last_Aimpose;
    void NOError();
};
