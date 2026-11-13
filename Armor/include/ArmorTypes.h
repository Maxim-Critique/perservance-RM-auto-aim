#ifndef ARMOR_TYPES_H
#define ARMOR_TYPES_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include "math_tools.hpp"
using namespace cv;
using namespace std;

/**
 * @file ArmorTypes.h
 * @brief 检测、解算及历史运动模型之间共用的数据结构。
 * 当前构建由 ArmorDetector、Slover 和 sentry/adapter 引入；全局 Armor 是桥接前的数据类型，
 * sentry_aim::Armor 是桥接后的追踪观测类型，需通过适配函数转换。
 *
 * 像素坐标左上为原点，x 向右、y 向下；位置单位 m，内部角度 rad。
 * Detector 只填图像/分类字段，Slover 再填位置/朝向；历史 GetArmors 只填 abs_pose/predict_yaw。
 * 同一个 Armor 在不同阶段并不是每个字段都有值，不能把默认构造当作全零有效观测。
 */

/**
 * @brief 一根候选灯条的二维几何与颜色信息，不包含三维姿态。
 * top/bottom 是按图像 y 排序后的两端中点，区别于旋转矩形自身的 angle 字段。
 */
class LightBlob
{
public:
    cv::RotatedRect rect; ///< 包围灯条的最小旋转矩形，center/size 为像素，OpenCV rect.angle 为度。
    double length;        ///< max(rect.size.height, rect.size.width)，灯条长边像素长度，不是实际物理长度。
    int blob_color;       ///< 灯条颜色编码，使用 BLOB_BLUE/RED；并非 RGB 三元颜色值。

    /// 依次为图像上端中点、下端中点和两者中点，单位像素，top.y 通常小于 bottom.y。
    cv::Point2f top, bottom, center;
    /**
     * @brief 从旋转矩形与已判定的颜色构造候选灯条。
     * 复制矩形后将其四个顶点按 y 从小到大排序，前两点平均为 top，后两点平均为 bottom。
     * @param r 输入旋转矩形；签名为引用，但实现不修改原矩形。
     * @param color 颜色编码，含义与敌色常量一致。
     */
    LightBlob(cv::RotatedRect &r, int color)
        : rect(r), blob_color(color)
    {
        length = max(rect.size.height, rect.size.width);

        cv::Point2f p[4];
        r.points(p);
        std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b){ return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;
        center = (top + bottom) / 2;
    };
    /// 默认构造不运行上述端点计算；length/blob_color 等需在读取前由使用者填入。
    LightBlob() = default;
};

/// 一帧候选灯条列表；顺序本身不代表左到右或目标优先级。
typedef std::vector<LightBlob> LightBlobs;

/// 根据两灯条几何关系区分的装甲板尺寸类型，不是敌方机器人类别。
enum class ArmorType
{
    SMALL,   ///< 小板模型，数值 0；PnP 使用对应小板物点。
    LARGE,   ///< 大板模型，数值 1；PnP 使用对应大板物点。
    INVALID  ///< 无效/不采用的板，数值 2；解算器直接拒绝。
};
/// 与 ArmorType 枚举数值一一对应的英文名称，索引前应保证枚举有效。
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

/**
 * @brief 一块装甲板在检测、位姿解算或模型预测阶段的数据载体。
 *
 * points 的约定：0 左下、1 左上、2 右上、3 右下，供透视变换和 PnP 共用。
 * yaw 描述板朝向，abs_ypd[0] 描述从参考原点看板中心的方向，两者通常不相等。
 * abs_pose 由当前姿态旋转后的云台参考系表达，未包含导航定位的全局平移。
 */
struct Armor
{
    /// 默认构造不会显式初始化所有数值字段，使用前须确认它们由哪个阶段写入。
    Armor() = default;
    /**
     * @brief 保存四角点与类型，并按四点平均生成图像中心。
     * @param points 预期恰有四个且遵循上述顺序的像素角点；当前实现固定乘 1/4。
     * @param type 由几何配对决定的大小板类型。
     * 本构造不计算标签、置信度、三维位置或朝向；这些由后续阶段分别填写。
     */
    Armor(const std::vector<cv::Point2f>& points, ArmorType type)
    {
        this->points = points;
        this->type = type;
        this->center = cv::Point2f(0.f, 0.f);
        if (!this->points.empty()) {
            for (const auto &point : this->points) {
                this->center += point;
            }
            this->center *= 1.0f / 4.0f;
        }
    }

    cv::Point2f center;        ///< 四图像角点的均值，像素；用于画面中心距离和目标关联。
    double yaw;               ///< 解算板朝向，rad；Slover 当前写 sys_yaw+相对最佳yaw+pi，不保证已归一化。
    double predict_yaw;       ///< 阶段相关的参考朝向，rad；Slover 写 sys_yaw+pi，历史模型写该板预测径向相位。
    cv::Mat number_img;       ///< 分类用 28x28 单通道二值数字 ROI，不是原图完整装甲板截图。
    std::string number;       ///< 模型标签，例如 1/3/guard/outpost；不是唯一目标编号，也不一定可转成整数。
    float confidence;         ///< 分类最大 softmax 概率，通常 [0,1]；不等同于 PnP 精度或跟踪置信度。
    vector<Point2f> points;   ///< 检测精修的四个灯条端点，像素，顺序固定为左下/左上/右上/右下。
    vector<Point2f> re_points;///< 根据最佳相对 yaw 重投影得到的四点，像素；用于观察拟合误差。
    ArmorType type;           ///< 大小板几何类型，决定数字展开宽度、类别规则和 PnP 物点。
    Eigen::Vector3d gim_pose; ///< 板中心在云台系的位置 (x,y,z)，m；由相机外参转换获得。
    Eigen::Vector3d abs_pose; ///< 板中心在姿态补偿世界参考系的位置 (x,y,z)，m；历史预测模型也写此字段。
    Eigen::Vector3d abs_ypd;  ///< abs_pose 转球坐标：(atan2(y,x), atan2(z,hypot(x,y)), 三维距离)，单位 rad/rad/m。
};
/// 装甲板集合；检测列表、已解算列表和预测列表虽共用此类型，但有效字段范围不同。
typedef vector<Armor> Armors;

#define ENEMY_BLUE 0 ///< 敌方蓝色编码，传入检测器；具体消息含义仍需与电控约定一致。
#define ENEMY_RED 1  ///< 敌方红色编码，不是颜色通道索引。

#define BLOB_RED ENEMY_RED   ///< 灯条红色编码，与敌色红常量复用。
#define BLOB_BLUE ENEMY_BLUE ///< 灯条蓝色编码，与敌色蓝常量复用。

#endif
