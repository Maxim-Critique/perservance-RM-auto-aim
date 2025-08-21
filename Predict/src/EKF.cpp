#include "EKF.h"
#include "camera_to_imu.h"
ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd & X_)
    : x_pri(X_),x_post(X_),I(Eigen::MatrixXd::Identity(10, 10)),
      P_pri(Eigen::Matrix<double, 10, 10>::Identity() * 1e-5), 
      P_post(Eigen::Matrix<double, 10, 10>::Identity() * 1e-5)
    {
        dt_ = 0.01;
        readYAML(Yaml);
    }


Eigen::MatrixXd ExtendedKalmanFilter::jacobianFunctionF(const Eigen::VectorXd & x) {
    Eigen::MatrixXd F(10, 10);
        F <<   1, dt_,  0, 0,    0, 0, 0, 0, 0, 0,
               0, 1,    0, 0,    0, 0, 0, 0, 0, 0,
               0, 0,    1, dt_,  0, 0, 0, 0, 0, 0,
               0, 0,    0, 1,    0, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    1, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 1, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 1, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 1, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 0, 1, dt_,
               0, 0,    0, 0,    0, 0, 0, 0, 0, 1;

    return F;
}

Eigen::VectorXd ExtendedKalmanFilter::measurementFunction(const Eigen::VectorXd & X,int i) {
    if (X.size() < 10) {
        throw std::runtime_error("Input vector x size is insufficient");
    }
    Eigen::Matrix<double, 4, 1> h;
        h << X(0) + X(6 + i % 2) * cos(X(8) + i * PI / 2),
             X(2) + X(6 + i % 2) * sin(X(8) + i * PI / 2),
             X(4 + i % 2),
             X(8) + i * PI / 2;
        return h;
}

Eigen::MatrixXd ExtendedKalmanFilter::jacobianFunctionH(const Eigen::VectorXd & X , int i) {
    if (X.size() < 10) {
        throw std::runtime_error("Input vector x size is insufficient");
    }
     Eigen::Matrix<double, 4, 10> H;
        //   x  vx y  vy z1           z2                  r1                                   r2                                     yaw                                   
        H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8) + i * PI / 2), (i % 2) * cos(X(8) + i * PI / 2),-X(6 + i % 2, 0) * sin(X(8) + i * PI / 2), 0,
             0, 0, 1, 0, 0,           0,     ((i + 1) % 2) * sin(X(8) + i * PI / 2), (i % 2) * sin(X(8) + i * PI / 2), X(6 + i % 2, 0) * cos(X(8) + i * PI / 2), 0,
             0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0,
             0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0;
        return H;
}

YAML::Node ExtendedKalmanFilter::readYAML(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        dd_ = config["EKF"]["ProcessNoise"]["dd"].as<double>();
        da_high = config["EKF"]["ProcessNoise"]["dahigh"].as<double>();
        da_low = config["EKF"]["ProcessNoise"]["dalow"].as<double>();
        dz  = config["EKF"]["ProcessNoise"]["dz"].as<double>();
        dr  = config["EKF"]["ProcessNoise"]["dr"].as<double>();
        rd  = config["EKF"]["RNoise"]["rd"].as<double>();
        rz  = config["EKF"]["RNoise"]["rz"].as<double>();
        ra  = config["EKF"]["RNoise"]["ra"].as<double>();
        m_gain = config["EKF"]["m_gain"].as<double>();
        da_ = da_high;
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()));
    }
    return YAML::LoadFile(filename);
}

Eigen::MatrixXd ExtendedKalmanFilter::processNoiseCovariance(const Eigen::VectorXd & x_p) {
    Eigen::MatrixXd m_Q(10, 10);
    double t4 = std::pow(dt_, 3) / 3;
    double t3 = std::pow(dt_, 2) / 2;
    double t2 = dt_;
    m_Q <<  
        t4 * dd_, t3 * dd_, 0,       0,       0,  0, 0,  0,  0,       0,
        t3 * dd_, t2 * dd_, 0,       0,       0,  0, 0,  0,  0,       0,
        0,       0,       t4 * dd_, t3 * dd_, 0,  0, 0,  0,  0,       0,
        0,       0,       t3 * dd_, t2 * dd_, 0,  0, 0,  0,  0,       0,
        0,       0,       0,       0,       dz, 0, 0,  0,  0,       0,
        0,       0,       0,       0,       0, dz, 0,  0,  0,       0,
        0,       0,       0,       0,       0,  0, dr, 0,  0,       0,
        0,       0,       0,       0,       0,  0, 0,  dr, 0,       0,
        0,       0,       0,       0,       0,  0, 0,  0,  t4 * da_, t3 * da_,
        0,       0,       0,       0,       0,  0, 0,  0,  t3 * da_, t2 * da_;

    return m_Q;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(Eigen::VectorXd & x_post) {
    // 调用成员函数计算雅可比矩阵 F 和过程噪声协方差矩阵 Q
    F = jacobianFunctionF(x_post);
    Q = processNoiseCovariance(x_post);
    // 预测步骤
    x_pri = F * (x_post); // 调用预测模型函数
    P_pri = F * P_post * F.transpose() + Q;
    Constraint(x_pri);//约束
    return x_pri;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, int num1) {
    // 调用成员函数计算测量模型雅可比矩阵 H 和测量噪声协方差矩阵 R
    H.resize(4,10);
    H = jacobianFunctionH(x_pri, num1);
    V.resize(4, 4);
    V << getMeasureNoisePD(x_pri, num1);
    R.resize(4,4);
    R<<rd,    0,     0,     0,
        0,   rd,     0,     0,
        0,    0,    rz,     0,
        0,    0,     0,    ra;
    R(2, 2) *= m_gain * abs(z(3));
    // 计算卡尔曼增益
    K = P_pri * H.transpose() * ((H * P_pri * H.transpose() + V * R * V.transpose())).inverse();
    // 调用测量函数 h 计算预测的测量值
    Eigen::VectorXd h = measurementFunction(x_pri, num1);
    Eigen::MatrixXd tmp = z - h;
    tmp(3, 0) = _std_radian(tmp(3, 0));
    // 更新状态
    x_post = x_pri + K * (tmp);
    P_post = (I - K * H) * P_pri;
    Constraint(x_post);//约束
    // std::ofstream out("ekf_data.txt", std::ios::app);  // 追加模式
    // out << dt_ << " ";
    // for (size_t i = 0; i < x_post.size(); ++i) {
    //     out << x_post[i] << " ";  // 写入10个状态量（空格分隔）
    // }
    // std::ofstream in("adjust.txt", std::ios::app);  // 追加模式
    // in << dt_ << " ";
    // for (size_t i = 0; i < z.size(); ++i) {
    //     in << z[i] << " ";  
    // }
    // in << "\n";
    return x_post;
}

void ExtendedKalmanFilter::Constraint(Eigen::VectorXd & X_){
    X_[6] = std::clamp(X_[6], 0.18, 0.4);
    X_[7] = std::clamp(X_[7], 0.18, 0.4);
    X_[8] = _std_radian(X_[8]);
    if(abs(X_[9])>25){
        X_[9] = 0;
    }
}

Eigen::MatrixXd ExtendedKalmanFilter::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
{
    Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
    double psi = atan2(h(1, 0), h(0, 0));
    double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
    double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
    Eigen::Matrix<double, 4, 4> V;
    V <<   -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
            d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
            0,                        d * cos(phi),            sin(phi),            0,
            0,                        0,                       0,                   1;
    return V;
}

Eigen::MatrixXd ExtendedKalmanFilter::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
{
    Eigen::Matrix<double, 4, 1> h;
    h <<    X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2),
            X(2, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2),
            X(4 + i % 2, 0),
            X(8, 0) + i * PI / 2;
    return h;
}

void ExtendedKalmanFilter::Updatedt(const double &time){
    dt_ = time;
    if(!is_stability){
        find_times++;
        if(find_times>=80){
            is_stability = true;
            da_ = da_low;
        }
    }
}