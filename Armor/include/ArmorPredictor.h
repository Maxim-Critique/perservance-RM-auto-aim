#ifndef ARMOR_PREDICTOR_H
#define ARMOR_PREDICTOR_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "ArmorTypes.h"
#include <algorithm>
#include "math_tools.hpp"

/**
 * @file ArmorPredictor.h
 * @brief 历史版整车运动学模型：把十维状态展开为各装甲板的未来位置。
 *
 * 当前 CMake 已排除 ArmorPredictor.cpp；现行 AutoArmor 使用 Armor/sentry 中的模型。
 * 本文件的状态顺序只适用于历史 Aim / ExtendedKalmanFilter，不可直接套到 sentry 状态上。
 * 位置和半径单位为 m，平移速度为 m/s，角度为 rad，角速度为 rad/s，预测时间为 s。
 */

/**
 * @brief 普通四装甲板目标和三装甲板前哨站共用的模型接口。
 *
 * 模型假设旋转中心做平面匀速运动、相位做匀角速度运动，高度和半径在预测期间不变。
 * GetArmors() 根据这些成员计算位置，不修改模型自身的状态或推进内部时间。
 */
class KinematicModel
{
public:
    /// 清空高度和半径数组；基类构造本身不为全部标量成员赋值。
    KinematicModel();
    /// 当前声明不是虚析构函数；不应通过裸基类指针 delete 派生对象。
    ~KinematicModel();

    /**
     * @brief 获取相对当前模型时刻 predict_time 秒后的全部装甲板。
     * @param predict_time 相对时间偏移；0 表示当前状态，历史规划器也会传负值作反向外推。
     * @return 长度为 number 的装甲板列表；实现只写 abs_pose 和 predict_yaw。
     * 图像角点、类别、置信度、球坐标等字段不由本接口生成，不能当作完整检测结果使用。
     */
    virtual Armors GetArmors(double predict_time = 0) = 0;

public:
    cv::Point2d center;             ///< 当前旋转中心的世界系平面位置 (xc, yc)，m。
    cv::Point2d velocity;           ///< 旋转中心的平面速度 (vx, vy)，m/s。
    std::vector<double> height;     ///< 板中心高度，m；普通目标按 i%2 取值，前哨按 i%3 取值。
    std::vector<double> radius;     ///< 板中心到旋转轴的水平距离，m；普通目标交替两种半径，前哨共用 radius[0]。
    double phase;                   ///< 索引 0 装甲板当前径向相位，rad；点位采用 center + r*(cos,sin)。
    double palstance;               ///< 相位变化率，rad/s；沿用原变量名，正负遵循世界系 atan2(y,x)。

    int number;                     ///< 展开的装甲板数量：StandardModel 为 4，OutpostModel 为 3。
};

/**
 * @brief 两组半径/高度交替排列、相邻相位相差 pi/2 的四装甲板模型。
 * 状态顺序：[xc, vx, yc, vy, z1, z2, r1, r2, phase, omega]。
 * i=0/2 共用 z1、r1，i=1/3 共用 z2、r2；高度没有独立速度状态。
 */
class StandardModel : public KinematicModel
{
public:
    /// 委托给全零十维状态构造；此时半径为 0，仅是占位模型。
    StandardModel();
    /// 将上述顺序的十维状态 X 拆入基类成员；本构造不额外校验半径或状态合理性。
    StandardModel(const Eigen::Matrix<double, 10, 1> &X);
    /// 释放该模型持有的数组等成员。
    ~StandardModel();

    /**
     * @brief 按匀速中心与匀速旋转外推四块板的位置。
     * @param predict_time 相对模型时刻的秒数。
     * @return 第 i 块板位于 center + velocity*t + radius[i%2]*(cos(a),sin(a))，
     * 其中 a=phase+palstance*t+i*pi/2；z 取 height[i%2]，只填 abs_pose / predict_yaw。
     */
    Armors GetArmors(double predict_time = 0);

};

/**
 * @brief 三块板相差 2*pi/3、共用半径但各自保留高度的前哨站模型。
 * 状态顺序：[xc, vx, yc, vy, z1, z2, z3, r, phase, omega]。
 * 与 StandardModel 的第 6、7 号元素含义不同，创建模型时必须匹配目标类别。
 */
class OutpostModel : public KinematicModel
{
public:
    /// 委托给全零十维状态构造；半径为 0 的占位模型。
    OutpostModel();
    /// 根据前哨站状态顺序初始化中心、速度、三种高度、单一半径和旋转状态。
    OutpostModel(const Eigen::Matrix<double, 10, 1> &X);
    /// 释放模型自身成员。
    ~OutpostModel();

    /**
     * @brief 按当前状态展开三块前哨装甲板。
     * @param predict_time 相对时间偏移，s；此函数本身仍按 velocity 对中心做匀速外推。
     * @return 每块板使用 radius[0] 和 height[i%3]，相位增加 i*2*pi/3；
     * 只填 abs_pose / predict_yaw，不负责检测、滤波或实际云台控制。
     */
    Armors GetArmors(double predict_time = 0);


};
#endif
