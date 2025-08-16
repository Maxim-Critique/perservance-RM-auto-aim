#pragma once
#include <Eigen/Dense>
#include "Base.h"
#include "SerialPort.h"
// 状态量维度。取2或4，代表pitch和yaw上的位置，或位置与速度
#define _D_STATE_ 2
// 控制量维度。取2即可，代表pitch和yaw的速度
#define _D_CTRL_ 2

#if (_D_STATE_ != 2 && _D_STATE_ != 4) || _D_CTRL_ != 2
#error                                                             \
[MPC] The state dimention has not been set to a presupposed value, \
please check your params.
#endif



/**
 * @brief 模型预测控制器
 * @author 王铭远
 */
class MPC
{
public:
    MPC();
    ~MPC();

    /**
     * @brief 模块主接口。获取当前的云台控制速度
     * @param cur_pose 当前位姿
     * @param status 当前状态
     * @return 控制速度
     */
    Point2d getGimbalSpeed(vector<Point2d> P_Y);

private:
    /**
     * @brief 设定MPC计算过程所需的矩阵
     */
    void setMPCMatrix();

    /**
     * @brief 获取预测区间内的所有控制目标
     */
    Eigen::Matrix<double, _D_STATE_,6> getPredictControlTarget(vector<Point2d>P_Y);

    /**
     * @brief 求解无约束二次规划问题形如 1/2 x^T H x + f^T x = 0
     * @param H 二次项系数
     * @param f 一次项系数
     */
    Eigen::MatrixXd solveQuadProg(Eigen::MatrixXd H, Eigen::MatrixXd f);

private:
    // 标志位参数
    bool m_debug;

    // 逻辑参数
    int m_step;                                 // 预测步长
    double m_dt;

    // 损失函数权重参数
    Eigen::Matrix<double, _D_STATE_, 1> m_process_error_weight;     // 过程误差权重
    Eigen::Matrix<double, _D_CTRL_,  1> m_process_control_weight;   // 过程控制权重
    Eigen::Matrix<double, _D_STATE_, 1> m_target_error_weight;      // 目标误差权重

    // MPC系统状态矩阵
    Eigen::Matrix<double, _D_STATE_, _D_STATE_> m_A;                // 状态转移矩阵
    Eigen::Matrix<double, _D_STATE_, _D_CTRL_>  m_B;                // 控制输入矩阵
    Eigen::Matrix<double, _D_STATE_, _D_STATE_> m_Q;                // 过程误差权重矩阵
    Eigen::Matrix<double, _D_CTRL_,  _D_CTRL_>  m_R;                // 过程控制权重矩阵
    Eigen::Matrix<double, _D_STATE_, _D_STATE_> m_F;                // 目标误差权重矩阵

    // MPC中间计算矩阵
    Eigen::MatrixXd m_H;        // 二次项系数
    Eigen::MatrixXd m_E;        // 一次项系数
    Eigen::MatrixXd m_Ed;       // 一次项系数（目标量）

};


