#include "Buff_Predict.h"
#include "camera_to_imu.h"
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

BigNewtonGauss::BigNewtonGauss() {
    fit_timestamps.clear();
    fit_speeds.clear();

    // 初始化参数为允许范围的平均值
    a = (0.780 + 1.045) / 2.0;
    omega = (1.884 + 2.000) / 2.0;
    phase = 0; // 相位
    b = 2.090 - a; // 根据约束 b=2.090-a

    // 保存初始值用于惩罚计算
    a_init = a;
    omega_init = omega;
    phase_init = phase;
    b_init = b;

    // 惩罚权重 (需要根据实际情况调整)
    lambda = 0.01;

    initialized = 0;
    direction = 0.0;

    last_time = 0;
    last_angle = 0;
}

int BigNewtonGauss::fit(double angle,double speed,uint64_t timestamp){
    if(speed>0.7){
        fit_timestamps.push_back(timestamp);
        fit_speeds.push_back(speed);
    }
    last_angle = angle;
    last_time = timestamp;
    if (!initialized && fit_timestamps.size() > min_fit_samples) {
        initialized = true;
    }
    // 没初始化也没滤
    if (!initialized) {
        return false;
    }
    
    while (fit_timestamps.size() > max_fit_samples) {
        fit_timestamps.pop_front();
        fit_speeds.pop_front();
    }
    
    // 旋转方向
    float speed_sum = 0;
    for (float spd: fit_speeds) {
        speed_sum += spd;
    }
    if (speed_sum > 0) {
        direction = 1.0;
    }
    else {
        direction = -1.0;
    }

    // 迭代加权最小二乘法实现L1优化
    const int size = fit_timestamps.size();
    MatrixXd J(size + 4, 4); // 增加4行用于惩罚项
    MatrixXd r(size + 4, 1); // 增加4行用于惩罚项
    MatrixXd W = MatrixXd::Identity(size + 4, size + 4); // 权重矩阵
    MatrixXd p(4, 1);

    // 数据拟合部分
    for (int i = 0; i < size; i ++) {
        float curr_speed = fabs(fit_speeds.at(i));
        float curr_ts = fit_timestamps.at(i) / 1e8;
        float sin_x = sin(omega * curr_ts + phase);
        float cos_x = cos(omega * curr_ts + phase);
        r(i) = curr_speed - (a * sin_x + b);
        J(i, 0) = -sin_x;
        J(i, 1) = -a * curr_ts * cos_x;
        J(i, 2) = -a * cos_x;
        J(i, 3) = -1;
        
        // L1范数权重: 1/|r| (避免除以零)
        W(i, i) = 1.0 / (fabs(r(i)) + 1e-6);
    }
    // 添加惩罚项
    // a的惩罚
    r(size) = lambda * (a - a_init);
    J(size, 0) = lambda;
    J(size, 1) = 0;
    J(size, 2) = 0;
    J(size, 3) = 0;
    W(size, size) = 1.0 / (fabs(r(size)) + 1e-6);
    
    // omega的惩罚
    r(size+1) = lambda * (omega - omega_init);
    J(size+1, 0) = 0;
    J(size+1, 1) = lambda;
    J(size+1, 2) = 0;
    J(size+1, 3) = 0;
    W(size+1, size+1) = 1.0 / (fabs(r(size+1)) + 1e-6);
    
    // phase的惩罚
    r(size+2) = lambda * (phase - phase_init);
    J(size+2, 0) = 0;
    J(size+2, 1) = 0;
    J(size+2, 2) = lambda;
    J(size+2, 3) = 0;
    W(size+2, size+2) = 1.0 / (fabs(r(size+2)) + 1e-6);
    
    // b的惩罚 (同时考虑约束 b=2.090-a)
    r(size+3) = lambda * (b - b_init) + lambda * (a + b - 2.090);
    J(size+3, 0) = lambda;
    J(size+3, 1) = 0;
    J(size+3, 2) = 0;
    J(size+3, 3) = lambda;
    W(size+3, size+3) = 1.0 / (fabs(r(size+3)) + 1e-6);

    p << a, omega, phase, b;
    Eigen::Matrix<double, 4, 1> dp = (J.transpose() * W * J).inverse() * J.transpose() * W * r;
    p -= 0.05 * dp;
    
    // 更新参数
    a = p(0);
    omega = p(1);
    phase = p(2);
    b = p(3);
    
    // 确保b满足约束条件
    b = 2.090 - a;
    return true;
}

double BigNewtonGauss::getAngle(uint64_t timestamp) {
    double dt = (timestamp - last_time) / 1e8;
    double t = fit_timestamps.back() / 1e8;
    cout << " a "<<a<<" omega "<<omega<<" phase "<<phase<<" b "<<b;
    double f_t = -a / omega * cos(omega * t + phase) + b * t;
    double f_dt = -a / omega * cos(omega * (t + dt) + phase) + b * (t + dt);
    double d_angle = (f_dt - f_t) * direction;
    cout<<"  f_t "<<f_t<<" f_dt "<<f_dt<<"  d_angle "<<R2D(d_angle)<<endl;
    double predict_angle = last_angle + d_angle;
    while (predict_angle > 2*M_PI) predict_angle -= 2*M_PI;
    while (predict_angle < 0) predict_angle += 2*M_PI;
    if(!initialized){
        return last_angle;
    }
    return predict_angle;
    // return a*sin(omega * t + phase)+ b;
}