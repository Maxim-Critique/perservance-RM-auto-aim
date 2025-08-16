#ifndef EKF_H_
#define EKF_H_

#include <algorithm> 
#include <deque>
#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include "Base.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>
#include <filesystem>
class ExtendedKalmanFilter {
public:
    
    // 构造函数
    ExtendedKalmanFilter(const Eigen::VectorXd & X_);

    // 计算预测状态
    Eigen::VectorXd predict( Eigen::VectorXd & x_post);

    // 根据测量更新估计状态
    Eigen::VectorXd update(const Eigen::VectorXd & z,int num1);
    Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    YAML::Node readYAML(const std::string& filename);

    //更新时间差
    void Updatedt(const double &time);
    bool is_stability = false;                 // da转化标志位
private:
    // 非线性函数处理
    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    
    VecVecFunc f;  // 状态转移函数
    VecVecFunc h;  // 测量函数
    VecMatFunc jacobian_f;  // 状态转移雅可比矩阵
    VecMatFunc jacobian_h;  // 测量雅可比矩阵
    VecMatFunc update_Q;  // 更新过程噪声协方差
    VecMatFunc update_R;  // 更新测量噪声协方差
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;  // 状态转移矩阵
    Eigen::MatrixXd H;  // 测量矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差矩阵
    Eigen::MatrixXd R;  // 测量噪声协方差矩阵
    Eigen::MatrixXd P_pri;  // 先验估计协方差矩阵
    Eigen::MatrixXd P_post;  // 后验估计协方差矩阵
    Eigen::MatrixXd K;  // 卡尔曼增益
    int n;  // 状态向量维数
    Eigen::MatrixXd I;  // 单位矩阵
    Eigen::VectorXd x_pri;  // 先验状态向量
    Eigen::VectorXd x_post;  // 后验状态向量

    // 初始化函数
    void initializeFunctions();
    
    // 时间步长
    double dt_;
    double m_gain;
    
    // 雅可比函数
    Eigen::MatrixXd jacobianFunctionF(const Eigen::VectorXd & x);
    Eigen::VectorXd measurementFunction(const Eigen::VectorXd & x , int i);
    Eigen::MatrixXd jacobianFunctionH(const Eigen::VectorXd & x , int i);
    
    // 噪声协方差函数
    Eigen::MatrixXd processNoiseCovariance(const Eigen::VectorXd & x_p);
    Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);
    //限制半径范围
    void Constraint(Eigen::VectorXd & X);

    
    double dd_;                                // 平面距离噪声系数
    double da_;                                // 角度噪声系数
    double dz;                                 // 高度噪声系数
    double dr;                                 // 半径噪声系数
    double rd;
    double rz;
    double ra;
    double da_high;
    double da_low;
    
    int find_times = 0;                        // 找到的帧数
};

#endif // EKF_H_
