#include <opencv2/opencv.hpp>
#include <iostream>
#include "BuffType.h"
#pragma once
using namespace cv;
using namespace std;


namespace auto_buff{
class BuffDetector
{
public:
    BuffDetector();
    std::optional<PowerRune> Detect(cv::Mat & bgr_img);

private:
    // 预处理
    void HandleImg();             
    // 进行圆拟合，获得待机击打区域
    void DetectCircles(vector<FanBlade>& fanblades);
    // 进行矩形拟合
    void DetectRect(std::vector<std::pair<float,std::vector<cv::Point2f>>> &rects);
    // 获得流水灯
    void GetLightBars(vector<FanBlade>& fanblades,std::vector<std::pair<float,std::vector<cv::Point2f>>> &rects);
    // 获取R标
    cv::Point2f GetR(std::vector<FanBlade> & fanblades);
    // 验证待击打区域
    bool Verify(cv::Point2f points[4]);
    // 是否丢失
    void Lose();
    // 可视化
    void Show(vector<FanBlade>& fanblades, PowerRune &P);

    cv::dnn::Net net_;
    std::vector<std::vector<cv::Point>> contours;        // 存放二值化轮廓
    Track_status status_;                                // 状态
    int lose_;                                           // 丢失的次数
    double lastlen_;
    bool debug_ = true;                                  // debug
    bool show_ = true;                                   // 可视化
    bool save_samples_ = false;                          // 是否保存样本

    cv::Mat img_;                                        // 原始图像
    cv::Mat gray_img_;                                   // 二值化后的图
    std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
