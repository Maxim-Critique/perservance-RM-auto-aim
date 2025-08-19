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
using namespace cv;
using namespace std;



/******************* 灯条类定义 ***********************/
class LightBlob
{
public:
    cv::RotatedRect rect; // 灯条位置
    double area_ratio;
    double length;      // 灯条长度
    uint8_t blob_color; // 灯条颜色
    int light_area;

    cv::Point2f top, bottom, center;
    LightBlob(cv::RotatedRect &r, double ratio, uint8_t color, int area)
        : rect(r), area_ratio(ratio), blob_color(color), light_area(area)
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
static LightBlobs light_blobs;

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
    Armor(const std::vector<cv::Point2f>& points, int index, ArmorType type)
    {
        this->points = points;
        this->type = type;
        this->center = (points[0]+points[2])/2;  
    }

    // Light pairs part
    LightBlob left_light, right_light;
    cv::Point2f center;
    double yaw;
    double pridict_yaw;
    // Number part
    cv::Mat number_img;
    std::string number;
    float confidence;
    std::string classfication_result;
    vector<Point2f> points;
    ArmorType type;
    Point3d cam_pose;
    Point3d abs_pose;
    float area;
};
typedef vector<Armor> Armors;

/******************* 装甲板类定义　**********************/
class ArmorBox
{
public:
    cv::Rect2d rect;
    LightBlobs light_blobs;
    int box_color;
    ArmorType type;
    explicit ArmorBox(const LightBlobs &blobs = LightBlobs(), int color = 0, const ArmorType &type = ArmorType());
};
typedef std::vector<ArmorBox> ArmorBoxes;

static ArmorBoxes armor_boxes;

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
    Armors Find_light(Mat &src,const cv::Rect& roi);
    // 找装甲板
    Armors Find_armor(cv::Mat &src, LightBlobs &light_blobs);

    YAML::Node readYAML(const std::string& filename);
private:
    bool is_get_from_ds;
    int enemy_color;
};



#endif // FIND_ARMOR_H


