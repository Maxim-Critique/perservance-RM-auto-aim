// #include "CAlphaEKF.h"
// #include <cmath>
// #include "iostream"
// using namespace std;
// // ���캯��ʵ�� - ������ʼ��˳��
// CAlphaEKF::CAlphaEKF(double angle) :
//     x_pri_(Eigen::Vector4d::Zero()),  // �ȳ�ʼ��Ϊ��
//     x_post_(Eigen::Vector4d::Zero()), // �ȳ�ʼ��Ϊ��
//     P_pri_(Eigen::Matrix4d::Identity() * 1e-5),
//     P_post_(Eigen::Matrix4d::Identity() * 1e-5),
//     I_(Eigen::Matrix4d::Identity()),
//     dt_(0.01),
//     R_(10)
// {
//     // �ڹ��캯�����ڳ�ʼ��״̬����
//     x_pri_ << angle, 0, 0, 0;
//     x_post_ << angle, 0, 0, 0;
// }

// // �Ƕȹ�һ��ʵ�� (0, 2��)
// double CAlphaEKF::normalizeAngle(double angle) {
//     angle = fmod(angle, 2*M_PI);
//     if (angle > M_PI) angle -= 2*M_PI;
//     if (angle <= -M_PI) angle += 2*M_PI;
//     return angle;
// }

// // Ԥ�ⲽ��ʵ�� - ��������ƥ��
// void CAlphaEKF::predict(double dt) {
//     dt_ = dt;
//     F_ = computeJacobianF();
//     Q_ = computeProcessNoise();
    
//     x_pri_ = F_ * x_post_;
//     P_pri_ = F_ * P_post_ * F_.transpose() + Q_;
    
//     x_pri_[0] = normalizeAngle(x_pri_[0]);
// }

// double angleDifference(double a, double b) {
//     double diff = a - b;
//     // Normalize to [-��, ��)
//     diff = fmod(diff + M_PI, 2*M_PI);
//     if (diff < 0) diff += 2*M_PI;
//     diff -= M_PI;
//     return diff;
// }

// // ���²���ʵ�� - ������������
// Eigen::Vector4d CAlphaEKF::update(double measurement) {
//     H_ = computeJacobianH();  
//     double S = (H_ * P_pri_ * H_.transpose())(0,0) + R_;
    
//     K_ = P_pri_ * H_.transpose() / S;
    
//     // Use proper angle difference calculation
//     double residual = angleDifference(measurement, x_pri_[0]);
    
//     x_post_ = x_pri_ + K_ * residual;
//     P_post_ = (I_ - K_ * H_) * P_pri_;
//     x_post_[0] = normalizeAngle(x_post_[0]);
//     return x_post_;
// }

// // ����״̬ת���ſɱȾ���ʵ��
// Eigen::Matrix4d CAlphaEKF::computeJacobianF() {
//     Eigen::Matrix4d F;
//     F << 1,   dt_,    0.5*dt_*dt_,   (1.0/6.0)*dt_*dt_*dt_,
//          0,   1,      dt_,           0.5*dt_*dt_,
//          0,   0,      1,             dt_,
//          0,   0,      0,             1;
//     return F;
// }

// // ��������ſɱȾ���ʵ��
// Eigen::RowVector4d CAlphaEKF::computeJacobianH() {
//     return Eigen::RowVector4d(1, 0, 0, 0);
// }

// // �����������ʵ��
// Eigen::Matrix4d CAlphaEKF::computeProcessNoise() {
//     Eigen::Matrix4d Q;
//     double t4 = pow(dt_, 3) / 3;
//     double t3 = pow(dt_, 2) / 2;
//     double t2 = dt_;
//     double q = 100;
//     double q_angle = 0.1*q;
//     double q_vel = 1.0*q;
//     double q_acce = 10*q;
//     double q_alpha = 100*q;
    
//     Q << t4*q_angle, t3*q_angle, 0,        0,
//          t3*q_angle, t2*q_angle, 0,        0,
//          0,         0,          t4*q_vel, t3*q_vel,
//          0,         0,          t3*q_vel, t2*q_vel;
    
//     Q(2,2) += t4*q_acce;
//     Q(3,3) += t2*q_alpha;
    
//     return Q;
// }
