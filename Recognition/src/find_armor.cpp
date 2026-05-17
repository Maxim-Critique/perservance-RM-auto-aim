#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "find_armor.h"

#include <fstream>      // 文件流操作
#include <sys/stat.h>   // 文件状态
#include <unistd.h>     // 系统调用（getcwd）
#include <iostream>     // 输入输出
#include <string>
// 判断装甲板倾斜度
static bool AngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    cv::Point2f direction = light_blob_i.center - light_blob_j.center;
    float angle = std::atan2(std::abs(direction.y), std::abs(direction.x));
    return angle<tools::D2R(45);
}
// 判断装甲板的长宽比
static bool LengthWidthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    // 计算两个灯条中心点的距离
    double side_length = cv::norm(light_blob_i.rect.center - light_blob_j.rect.center);
    // 获取两个灯条中较长的长度
    double max_length = std::max(light_blob_i.length, light_blob_j.length);
    // 检查距离与灯条长度的比例是否在合理范围内
    return (side_length / max_length < 5.0 && side_length / max_length > 1.0);
}
// 判断两个灯条的长度比
static bool LengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    // 获取较长和较短的灯条长度
    double longer_length = std::max(light_blob_i.length, light_blob_j.length);
    double shorter_length = std::min(light_blob_i.length, light_blob_j.length);
    
    // 判断较长灯条与较短灯条的比值是否在合理范围内
    return (longer_length / shorter_length < 2.5);
}
// 两个灯条中是否夹了灯条
static bool ContainLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, const LightBlobs &lights)
{
    auto points = std::vector<cv::Point2f>{light_blob_i.top, light_blob_i.bottom, light_blob_j.top, light_blob_j.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto &test_light : lights)
    {
        if (test_light.center == light_blob_i.center || test_light.center == light_blob_j.center)
            continue;

        if (
            bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

// 判断两个灯条是否可以匹配
static bool IsCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, int enemy_color)
{
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           LengthRatioJudge(light_blob_i, light_blob_j) &&
           LengthWidthJudge(light_blob_i, light_blob_j) &&
           AngleJudge(light_blob_i, light_blob_j);
}

void Judge_Type(const LightBlob &light_blob_i, const LightBlob &light_blob_j, ArmorType &type)
{
    // 两个灯条的间距
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    // 灯条的高度
    double light_height;
    light_height = (light_blob_i.length + light_blob_j.length) / 2;
    if ((side_length / light_height) > 2.8)
    {
        type = ArmorType::LARGE;
    }
    else
    {
        type = ArmorType::SMALL;
    }
}

static cv::Point2f GreyLevelCentroid(const cv::Point2f& point, double radius, const cv::Mat &src) {
    
    cv::Rect roi_rect(cvRound(point.x - radius), cvRound(point.y - radius), 
                     cvRound(2 * radius), cvRound(2 * radius));  
    
    // 边界检查
    roi_rect &= cv::Rect(0, 0, src.cols, src.rows);
    if (roi_rect.width <= 0 || roi_rect.height <= 0) {
        return point;
    }
    
    cv::Mat roi = src(roi_rect); 
    // 转换为灰度
    cv::Mat gray_roi;
    cv::cvtColor(roi, gray_roi, cv::COLOR_BGR2GRAY);
    
    // 创建圆形掩码并直接应用
    cv::Mat mask = cv::Mat::zeros(gray_roi.size(), CV_8U);
    cv::circle(mask, cv::Point(gray_roi.cols/2, gray_roi.rows/2), cvRound(radius), 255, -1);
    
    // 应用掩码：将掩码外的像素设为0
    cv::bitwise_and(gray_roi, gray_roi, gray_roi, mask);
    
    // 计算矩
    cv::Moments m = cv::moments(gray_roi, false);
    
    if (m.m00 <= 0) {
        return point;
    }
    
    // 计算重心坐标
    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;
    return cv::Point2f(point.x - radius + cx, point.y - radius + cy);
}

std::vector<cv::Point2f> GetPoints(const LightBlob &blob_i, const LightBlob &blob_j, const cv::Mat &src) {
    std::vector<cv::Point2f> points;
    // 确定左右灯条

    const LightBlob* left_blob = &blob_i;
    const LightBlob* right_blob = &blob_j;
    if (blob_i.center.x > blob_j.center.x) {
        left_blob = &blob_j;
        right_blob = &blob_i;
    }
    
    // 对每个灯条的四个角点进行灰度重心精修
    cv::Point2f left_bottom_refined = GreyLevelCentroid(left_blob->bottom, left_blob->length*0.1, src);
    cv::Point2f left_top_refined = GreyLevelCentroid(left_blob->top, left_blob->length*0.1, src);
    cv::Point2f right_top_refined = GreyLevelCentroid(right_blob->top, right_blob->length*0.1, src);
    cv::Point2f right_bottom_refined = GreyLevelCentroid(right_blob->bottom, right_blob->length*0.1, src);
    
    /*
        1-----2
        |     |
        0-----3
    */
    // 按顺时针顺序返回四个角点：左下、左上、右上、右下
    points.push_back(left_bottom_refined);
    points.push_back(left_top_refined);
    points.push_back(right_top_refined);
    points.push_back(right_bottom_refined);
    
    return points;
}


// 匹配所有灯条，得出装甲板候选区
Armors Tradition::FindArmor(cv::Mat &src, LightBlobs &light_blobs)
{
    Armors armors;
    ArmorType type;
    for (size_t i = 0; i < light_blobs.size() - 1; ++i)
    {
        const auto& blob_i = light_blobs[i];
        for (size_t j = i + 1; j < light_blobs.size(); ++j)
        {
            const auto& blob_j = light_blobs[j];
            if (!IsCoupleLight(blob_i, blob_j, enemy_color_))
            {
                continue;
            }
           
            if (ContainLight(blob_i, blob_j, light_blobs))
            {
                continue;
            }
            Judge_Type(blob_i, blob_j, type);            
            std::vector<cv::Point2f> points = GetPoints(blob_i,blob_j,src);
            armors.emplace_back(points,type);

        }
    }
    return armors;
}

Tradition::Tradition(){
    try {
        YAML::Node config = YAML::LoadFile(YamlPath);
        threshold_ = config["Tradition"]["threshold"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()) + "Tradition");
    }
    
}