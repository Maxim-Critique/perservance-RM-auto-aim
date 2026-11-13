#ifndef ARMOR_PLANNER_H
#define ARMOR_PLANNER_H

#include <Eigen/Dense>
#include <iostream>
#include "ArmorPredictor.h"
#include "trajectory.hpp"
#include "Base.h"

/**
 * @file ArmorPlanner.h
 * @brief 历史版瞄准轨迹规划：运动模型预测、无阻力弹道与 yaw 五次多项式平滑。
 * 当前 CMake 已排除 ArmorPlanner.cpp，现行瞄准由 Armor/sentry/aimer 实现。
 * 本接口的时间为 s，角度为 rad，角速度为 rad/s，角加速度为 rad/s^2。
 */

/**
 * @brief 基于历史 KinematicModel 生成一次云台角度建议，不负责串口和开火。
 *
 * 每次调用先估算飞行时间，再在命中预测时刻附近采样 yaw 路径，寻找极值并尝试平滑。
 * 成员会被每次规划覆盖，不能把同一 Planner 对象视作可并发调用的无状态函数。
 */
class Planner
{
public:
    /// 从 YamlPath 读取 Transmit.predict_time 和 Planer.max_yaw_acc；配置异常向外传播。
    Planner();
    /**
     * @brief 执行“估算飞行时间 -> 采样 -> 找极值 -> 平滑/直接取点 -> 输出角度”。
     * @param model 非空运动模型，须能生成有效且在实现搜索距离内的装甲板位置。
     * @return 写入 target_yaw、target_pitch、manage_yaw、yaw_vel；当前实现未填写 yaw_acc。
     * target_yaw 是预测点原始方向，manage_yaw 可能经过平滑，两者用途不能混淆。
     */
    Gimbal_Angle TrajectoryPlanning(std::shared_ptr<KinematicModel> model);
private:

    /// 历史预留声明；当前 ArmorPlanner.cpp 未提供定义，也没有调用。
    void GetTurn();
    /**
     * @brief 生成 101 个采样点：预测时刻为 hit_time + [-0.5,0.5]，间隔 0.01 s。
     * path[i].time 另记为 i*0.01，即 [0,1]；中心索引 50 对应 hit_time，而非 0 s。
     * 角速度/角加速度由 yaw 差分得到，当前实现没有为差分单独处理跨 pi 的跳变。
     */
    void GetPath();
    /// 历史预留声明；当前实现中未定义，实际拟合入口是 GetPolynomial。
    void GetFPolynomial();
    /**
     * @brief 优先取覆盖路径中心且离中心更近的极值平滑多项式，缺省使用中心采样值。
     * target_pitch 通过 hit_time 时刻的目标位置重新求弹道；不向电控发送结果。
     */
    Gimbal_Angle ComputeGimbalAngle();
    /// 在相对 time 秒的所有预测板中选世界系欧氏距离最小者，返回 atan2(y,x)，rad。
    double GetGimbalYaw(double time);
    /**
     * @brief 围绕指定极值逐步扩展拟合区间，尝试满足加速度条件并覆盖路径中心。
     * @param extremum path 中的极值索引；-1 表示未找到。
     * @return 六维多项式系数；全零向量作为“未找到可用平滑段”的哨兵值。
     */
    Eigen::VectorXd GetPolynomial(int extremum);
    /**
     * @brief 解六个端点约束，求 theta(t)=a0+a1*t+...+a5*t^5。
     * @param start 起点时间、角度、角速度及角加速度，时间采用 path.time 的局部坐标。
     * @param end 终点约束，时间应与 start 不同。
     * @return [a0,a1,a2,a3,a4,a5]，按原实现使用 QR 求解，不额外报告病态程度。
     */
    Eigen::VectorXd ComputeQuinticPoly(const Path_Point& start, const Path_Point& end);
    /// 在同一局部时间坐标 t 处计算 theta、一次导数、二次导数，封装为 Path_Point 返回。
    Path_Point EvalPolyAtT(const Eigen::VectorXd& poly, double t);
    /// 按现有实现检查区间端点及求出的内部驻点，判断角加速度绝对值是否超过 max_yaw_acc。
    bool CheckAccelConstraint(const Eigen::VectorXd& poly, double t0, double t1);
    /**
     * @brief 选择 time 秒后的最近预测板，用水平距离和高度调用 tools::Trajectory。
     * 输入位置单位 m，使用 Base.h 的 bullet_speed（m/s），返回飞行时间及抬头为正的 pitch。
     * 调用方应保证存在有效板；本函数没有独立处理空列表，也没有在此检查 unsolvable。
     */
    tools::Trajectory GetTrajectory(double time);
    /// 从路径中心分别向左、向右找首个严格局部极大/极小值，结果写入两个极值索引。
    void FindExtremumPoints();
    double hit_time;                          ///< 当前弹道飞行时间 + predict_time，是相对模型时刻的预测偏移，s。
    double predict_time;                      ///< YAML Transmit.predict_time：手调的额外预测延迟，s。
    double max_yaw_acc;                       ///< YAML Planer.max_yaw_acc：平滑段允许的 yaw 角加速度绝对值，rad/s^2。
    int left_extremum;                        ///< 中心左侧（含中心）首个极值的 path 索引；-1 表示没有。
    int right_extremum;                       ///< 中心右侧（含中心）首个极值的 path 索引；-1 表示没有。
    std::shared_ptr<KinematicModel> m_status; ///< 当前规划使用的运动模型，由 TrajectoryPlanning 接收并保留共享所有权。
    std::vector<Path_Point> path;             ///< 本次采样路径；各点含局部时间、yaw、角速度、角加速度。

};
#endif
