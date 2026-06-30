// #include "Buff_Predict.h"
// #include "math.hpp"
// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

// BigNewtonGauss::BigNewtonGauss() {
//     fit_timestamps.clear();
//     fit_speeds.clear();

//     // ��ʼ������Ϊ������Χ��ƽ��ֵ
//     a = (0.780 + 1.045) / 2.0;
//     omega = (1.884 + 2.000) / 2.0;
//     phase = 0; // ��λ
//     b = 2.090 - a; // ����Լ�� b=2.090-a

//     // �����ʼֵ���ڳͷ�����
//     a_init = a;
//     omega_init = omega;
//     phase_init = phase;
//     b_init = b;

//     // �ͷ�Ȩ�� (��Ҫ����ʵ���������)
//     lambda = 0.01;

//     initialized = 0;
//     direction = 0.0;

//     last_time = 0;
//     last_angle = 0;
// }

// int BigNewtonGauss::fit(double angle,double speed,time_point timestamp){
//     if(speed>0.7){
//         fit_timestamps.push_back(timestamp);
//         fit_speeds.push_back(speed);
//     }
//     last_angle = angle;
//     last_time = timestamp;
//     if (!initialized && fit_timestamps.size() > min_fit_samples) {
//         initialized = true;
//     }
//     // û��ʼ��Ҳû��
//     if (!initialized) {
//         return false;
//     }
    
//     while (fit_timestamps.size() > max_fit_samples) {
//         fit_timestamps.pop_front();
//         fit_speeds.pop_front();
//     }
    
//     // ��ת����
//     float speed_sum = 0;
//     for (float spd: fit_speeds) {
//         speed_sum += spd;
//     }
//     if (speed_sum > 0) {
//         direction = 1.0;
//     }
//     else {
//         direction = -1.0;
//     }

//     // ������Ȩ��С���˷�ʵ��L1�Ż�
//     const int size = fit_timestamps.size();
//     MatrixXd J(size + 4, 4); // ����4�����ڳͷ���
//     MatrixXd r(size + 4, 1); // ����4�����ڳͷ���
//     MatrixXd W = MatrixXd::Identity(size + 4, size + 4); // Ȩ�ؾ���
//     MatrixXd p(4, 1);

//     // ������ϲ���
//     for (int i = 0; i < size; i ++) {
//         float curr_speed = fabs(fit_speeds.at(i));
//         float curr_ts = fit_timestamps.at(i) / 1e8;
//         float sin_x = sin(omega * curr_ts + phase);
//         float cos_x = cos(omega * curr_ts + phase);
//         r(i) = curr_speed - (a * sin_x + b);
//         J(i, 0) = -sin_x;
//         J(i, 1) = -a * curr_ts * cos_x;
//         J(i, 2) = -a * cos_x;
//         J(i, 3) = -1;
        
//         // L1����Ȩ��: 1/|r| (���������)
//         W(i, i) = 1.0 / (fabs(r(i)) + 1e-6);
//     }
//     // ���ӳͷ���
//     // a�ĳͷ�
//     r(size) = lambda * (a - a_init);
//     J(size, 0) = lambda;
//     J(size, 1) = 0;
//     J(size, 2) = 0;
//     J(size, 3) = 0;
//     W(size, size) = 1.0 / (fabs(r(size)) + 1e-6);
    
//     // omega�ĳͷ�
//     r(size+1) = lambda * (omega - omega_init);
//     J(size+1, 0) = 0;
//     J(size+1, 1) = lambda;
//     J(size+1, 2) = 0;
//     J(size+1, 3) = 0;
//     W(size+1, size+1) = 1.0 / (fabs(r(size+1)) + 1e-6);
    
//     // phase�ĳͷ�
//     r(size+2) = lambda * (phase - phase_init);
//     J(size+2, 0) = 0;
//     J(size+2, 1) = 0;
//     J(size+2, 2) = lambda;
//     J(size+2, 3) = 0;
//     W(size+2, size+2) = 1.0 / (fabs(r(size+2)) + 1e-6);
    
//     // b�ĳͷ� (ͬʱ����Լ�� b=2.090-a)
//     r(size+3) = lambda * (b - b_init) + lambda * (a + b - 2.090);
//     J(size+3, 0) = lambda;
//     J(size+3, 1) = 0;
//     J(size+3, 2) = 0;
//     J(size+3, 3) = lambda;
//     W(size+3, size+3) = 1.0 / (fabs(r(size+3)) + 1e-6);

//     p << a, omega, phase, b;
//     Eigen::Matrix<double, 4, 1> dp = (J.transpose() * W * J).inverse() * J.transpose() * W * r;
//     p -= 0.05 * dp;
    
//     // ���²���
//     a = p(0);
//     omega = p(1);
//     phase = p(2);
//     b = p(3);
    
//     // ȷ��b����Լ������
//     b = 2.090 - a;
//     return true;
// }

// double BigNewtonGauss::getAngle(time_point timestamp) {
//     double dt = (timestamp - last_time) / 1e8;
//     double t = fit_timestamps.back() / 1e8;
//     cout << " a "<<a<<" omega "<<omega<<" phase "<<phase<<" b "<<b;
//     double f_t = -a / omega * cos(omega * t + phase) + b * t;
//     double f_dt = -a / omega * cos(omega * (t + dt) + phase) + b * (t + dt);
//     double d_angle = (f_dt - f_t) * direction;
//     cout<<"  f_t "<<f_t<<" f_dt "<<f_dt<<"  d_angle "<<tools::R2D(d_angle)<<endl;
//     double predict_angle = last_angle + d_angle;
//     while (predict_angle > 2*M_PI) predict_angle -= 2*M_PI;
//     while (predict_angle < 0) predict_angle += 2*M_PI;
//     if(!initialized){
//         return last_angle;
//     }
//     return predict_angle;
//     // return a*sin(omega * t + phase)+ b;
// }