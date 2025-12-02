#ifndef FIND_ARMOR_H
#define FIND_ARMOR_H
#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <future>
#include "Base.h"
#include <algorithm>  // std::max 和 std::min
#include <cmath>      // fmax
#include <yaml-cpp/yaml.h>
#include "SerialPort.h"
#include <vector>
using namespace cv;
using namespace std;



/******************* 灯条类定义 ***********************/
class LightBlob
{
public:
    cv::RotatedRect rect; // 灯条位置
    double area_ratio;    // 与旋转矩形的占比
    double length;        // 灯条长度
    int blob_color;   // 灯条颜色

    cv::Point2f top, bottom, center;
    LightBlob(cv::RotatedRect &r, double ratio, int color)
        : rect(r), area_ratio(ratio), blob_color(color)
    {
        length = max(rect.size.height, rect.size.width);

        cv::Point2f p[4];
        r.points(p);
        std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b){ return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;
        center = (top + bottom) / 2;
    };
    LightBlob() = default;
};

typedef std::vector<LightBlob> LightBlobs;

/******************* 装甲板ID类定义　**********************/

enum class ArmorType
{
    SMALL,
    LARGE,
    INVALID
};
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};


struct Armor
{
    Armor() = default;
    Armor(const std::vector<cv::Point2f>& points, ArmorType type)
    {
        this->points = points;
        this->type = type;
    }

    cv::Point2f center;
    double yaw;
    double predict_yaw;
    // Number part
    cv::Mat number_img;
    std::string number;
    float confidence;
    std::string classfication_result;
    vector<Point2f> points;
    vector<Point2f> re_points;
    ArmorType type;
    Eigen::Vector3d cam_pose;
    Eigen::Vector3d abs_pose;
    Eigen::Vector3d abs_ypd;
};
typedef vector<Armor> Armors;

class NumberClassifier
{
public:
    NumberClassifier(
        const std::string &model_path, const std::string &label_path, const double threshold,
        const std::vector<std::string> &ignore_classes = {});

    void extractNumbers(const cv::Mat &src, std::vector<Armor> &armors);

    void classify(std::vector<Armor> &armors);

    double threshold;

private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

#define ENEMY_BLUE 0
#define ENEMY_RED 1

#define BLOB_RED ENEMY_RED
#define BLOB_BLUE ENEMY_BLUE

extern NumberClassifier classifier;
class Tradition{
public:

    Tradition();
    // 找灯条
    LightBlobs FindLight(Mat &src,int enemy_color);
    // 找装甲板
    Armors FindArmor(Mat &src, LightBlobs &light_blobs);

private:
    bool is_get_from_ds_;                  // 是否使用电控颜色
    int enemy_color_;                      // 敌方颜色
    int light_threshold_;                  // 二值化高阈值
    int dim_threshold_;                    // 二值化低阈值
};

#endif 


