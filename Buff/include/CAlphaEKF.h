// #ifndef CALPHA_EKF_H
// #define CALPHA_EKF_H

// #include <Eigen/Dense>

// class CAlphaEKF {
// public:
//     // 构造函数
//     CAlphaEKF(double angle);
    
//     // 预测步骤（包含观测角度赋值）
//     void predict(double dt);
    
//     // 更新步骤
//     Eigen::Vector4d update(double measurement);
    
//     // 获取当前状态
//     Eigen::Vector4d getState();

// private:
//     // 状态向量和协方差矩阵
//     Eigen::Vector4d x_pri_, x_post_;
//     Eigen::Matrix4d P_pri_, P_post_;
    
//     // 系统矩阵
//     Eigen::Matrix4d F_, Q_, I_;
//     Eigen::Matrix<double, 1, 4> H_; 
//     Eigen::Vector4d K_;
//     double R_;
//     double dt_;
    
//     // 角度归一化 (0, 2π)
//     static double normalizeAngle(double angle);
    
//     // 计算雅可比矩阵
//     Eigen::Matrix4d computeJacobianF();
//     Eigen::RowVector4d computeJacobianH();
    
//     // 噪声矩阵计算
//     Eigen::Matrix4d computeProcessNoise();
//     double computeMeasurementNoise();
// };

// #endif // CALPHA_EKF_H