#include "EKF.h"
#include <cmath>
#include <algorithm>
#include "Coordinate.h"


double ExtendedKalmanFilter::UpdateStrategy::rd = 0.0;
double ExtendedKalmanFilter::UpdateStrategy::rz = 0.0;
double ExtendedKalmanFilter::UpdateStrategy::ra = 0.0;
double ExtendedKalmanFilter::UpdateStrategy::m_gain = 0.0;
double ExtendedKalmanFilter::UpdateStrategy::rr = 0.0;

Eigen::Vector2d ComputeArmorCenter(const Armor& armor1, const Armor& armor2) {
    Eigen::Vector2d p1(armor1.abs_pose.x, armor1.abs_pose.y);
    Eigen::Vector2d p2(armor2.abs_pose.x, armor2.abs_pose.y);
    
    Eigen::Vector2d d1(std::cos(armor1.yaw), std::sin(armor1.yaw));
    Eigen::Vector2d d2(std::cos(armor2.yaw), std::sin(armor2.yaw));
    
    // 使用叉积计算交点（最优雅的方法）
    // 两条直线的参数方程：p1 + t*d1 和 p2 + s*d2
    // 交点满足：p1 + t*d1 = p2 + s*d2
    // 整理得：t*d1 - s*d2 = p2 - p1
    
    // 使用叉积求解：t = (p2 - p1) × d2 / (d1 × d2)
    double denominator = d1.x() * d2.y() - d1.y() * d2.x();    
    Eigen::Vector2d p2_minus_p1 = p2 - p1;
    double t = (p2_minus_p1.x() * d2.y() - p2_minus_p1.y() * d2.x()) / denominator;
    
    return p1 + t * d1;
}

// 单装甲板策略实现
void ExtendedKalmanFilter::SingleArmorStrategy::configure(
    const Eigen::VectorXd& x_pri, const Data& data) {
    x = x_pri;
    armor = data.armors[0];
    index = data.match_indices[0];
    
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> 
ExtendedKalmanFilter::SingleArmorStrategy::compute() {
    
    // 预测观测
    double angle = _std_radian(x(8) + index * PI/2);
    double radius = x(6 + index%2);
    Eigen::Vector4d h;
    h << x(0) + radius*cos(angle),
         x(2) + radius*sin(angle),
         x(4 + index%2),
         x(8) + index*PI/2;
    // 实际观测
    Eigen::Vector4d z;
    z << armor.abs_pose.x, armor.abs_pose.y, armor.abs_pose.z, _std_radian(armor.yaw);
    // 残差
    Eigen::Vector4d residual = z - h;
    residual(3) = _std_radian(residual(3));
    // 雅可比矩阵
    Eigen::Matrix<double, 4, 10> H;
    H << 1, 0, 0, 0,           0,       0, ((index+1)%2)*cos(angle), (index%2)*cos(angle), -radius*sin(angle), 0,
         0, 0, 1, 0,           0,       0, ((index+1)%2)*sin(angle), (index%2)*sin(angle),  radius*cos(angle), 0,
         0, 0, 0, 0, (index+1)%2, index%2,                        0,                    0,                  0, 0,
         0, 0, 0, 0,           0,       0,                        0,                    0,                  1, 0;
     
    // 噪声矩阵
    Eigen::Matrix<double, 4, 4> V;
    double psi = atan2(h(1, 0), h(0, 0));
    double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
    double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
    V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
          d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
          0,                        d * cos(phi),            sin(phi),            0,
          0,                        0,                       0,                   1;
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    
    R << rd, 0,                  0, 0,
          0,rd,                  0, 0,
          0, 0,                 rz, 0,
          0, 0,                  0, ra;  // 直接使用继承的噪声参数

    return {residual, H, V, R};
}

// 双装甲板策略实现
void ExtendedKalmanFilter::DualArmorStrategy::configure(
    const Eigen::VectorXd& x_pri, const Data& data) {
    x = x_pri;
    armor1 = data.armors[0];
    armor2 = data.armors[1];
    index1 = data.match_indices[0];
    index2 = data.match_indices[1];
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> 
ExtendedKalmanFilter::DualArmorStrategy::compute() {
    // 预测观测
    Eigen::Matrix<double, 8, 1> h;
    
    // 第一块装甲板
    double angle1 = _std_radian(x(8) + index1*PI/2);
    double radius1 = x(6 + index1%2);
    h.segment<4>(0) << x(0) + radius1*cos(angle1),
                       x(2) + radius1*sin(angle1),
                       x(4 + index1%2),
                       x(8) + index1*PI/2;
    
    // 第二块装甲板
    double angle2 = _std_radian(x(8) + index2*PI/2);
    double radius2 = x(6 + index2%2);
    h.segment<4>(4) << x(0) + radius2*cos(angle2),
                       x(2) + radius2*sin(angle2),
                       x(4 + index2%2),
                       x(8) + index2*PI/2;

    // 实际观测
    Eigen::Matrix<double, 8, 1> z;
    z << armor1.abs_pose.x, armor1.abs_pose.y, armor1.abs_pose.z, armor1.yaw,
         armor2.abs_pose.x, armor2.abs_pose.y, armor2.abs_pose.z, armor2.yaw;

    // 残差
    Eigen::Matrix<double, 8, 1> residual = z - h;
    residual(3) = _std_radian(residual(3));
    residual(7) = _std_radian(residual(7));
    
    // 雅可比矩阵
    Eigen::Matrix<double, 8, 10> H;
    
    // 第一块装甲板部分
    H.block<4,10>(0,0) << 
        1, 0, 0, 0, 0, 0, ((index1+1)%2)*cos(angle1), (index1%2)*cos(angle1), -radius1*sin(angle1), 0,
        0, 0, 1, 0, 0, 0, ((index1+1)%2)*sin(angle1), (index1%2)*sin(angle1), radius1*cos(angle1), 0,
        0, 0, 0, 0, (index1+1)%2, index1%2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    
    // 第二块装甲板部分
    H.block<4,10>(4,0) << 
        1, 0, 0, 0, 0, 0, ((index2+1)%2)*cos(angle2), (index2%2)*cos(angle2), -radius2*sin(angle2), 0,
        0, 0, 1, 0, 0, 0, ((index2+1)%2)*sin(angle2), (index2%2)*sin(angle2), radius2*cos(angle2), 0,
        0, 0, 0, 0, (index2+1)%2, index2%2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0;

     // 噪声矩阵 V - 为每块装甲板分别计算球坐标转换
    Eigen::Matrix<double, 8, 8> V = Eigen::Matrix<double, 8, 8>::Zero();
    
    // 第一块装甲板的球坐标转换
    double psi1 = atan2(h(1, 0), h(0, 0));
    double phi1 = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
    double d1 = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
    
    V.block<4,4>(0,0) << 
        -d1 * cos(phi1) * sin(psi1), -d1 * sin(phi1) * cos(psi1), cos(phi1) * cos(psi1), 0,
         d1 * cos(phi1) * cos(psi1), -d1 * sin(phi1) * sin(psi1), cos(phi1) * sin(psi1), 0,
         0,                           d1 * cos(phi1),             sin(phi1),             0,
         0,                           0,                          0,                     1;

    // 第二块装甲板的球坐标转换
    double psi2 = atan2(h(5, 0), h(4, 0));
    double phi2 = atan2(h(6, 0), sqrt(pow(h(4, 0), 2) + pow(h(5, 0), 2)));
    double d2 = sqrt(pow(h(4, 0), 2) + pow(h(5, 0), 2) + pow(h(6, 0), 2));
    
    V.block<4,4>(4,4) << 
        -d2 * cos(phi2) * sin(psi2), -d2 * sin(phi2) * cos(psi2), cos(phi2) * cos(psi2), 0,
         d2 * cos(phi2) * cos(psi2), -d2 * sin(phi2) * sin(psi2), cos(phi2) * sin(psi2), 0,
         0,                           d2 * cos(phi2),             sin(phi2),            0,
         0,                           0,                          0,                   1;
    Eigen::Matrix<double, 8, 8> R = Eigen::Matrix<double, 8, 8>::Zero();
    R.diagonal() << rd, rd, rz, ra,  // 直接使用继承的噪声参数
                    rd, rd, rz, ra;

    return {residual, H, V, R};
}

// ExtendedKalmanFilter 成员函数实现
ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd& initial_state) :
    x_pri(initial_state), x_post(initial_state),
    P_pri(Eigen::MatrixXd::Identity(10, 10) * 1e-5),
    P_post(Eigen::MatrixXd::Identity(10, 10) * 1e-5),
    I(Eigen::MatrixXd::Identity(10, 10)),
    is_stable(false), high_times(0) {
        loadConfig(Yaml);
    }

void ExtendedKalmanFilter::loadConfig(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        dd = config["EKF"]["ProcessNoise"]["dd"].as<double>();
        da_high = config["EKF"]["ProcessNoise"]["dahigh"].as<double>();
        da_low = config["EKF"]["ProcessNoise"]["dalow"].as<double>();
        dz = config["EKF"]["ProcessNoise"]["dz"].as<double>();
        dr = config["EKF"]["ProcessNoise"]["dr"].as<double>();

        // 初始化基类噪声参数（所有策略共享）
        UpdateStrategy::LoadNoiseParams(config["EKF"]["RNoise"]["rd"].as<double>(), 
                                        config["EKF"]["RNoise"]["rz"].as<double>(), 
                                        config["EKF"]["RNoise"]["ra"].as<double>(),
                                        config["EKF"]["m_gain"].as<double>(),
                                        config["EKF"]["RNoise"]["rr"].as<double>());
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()));
    }
}

Eigen::MatrixXd ExtendedKalmanFilter::computeProcessNoise() const {
    Eigen::MatrixXd Q(10, 10);
    double t4 = pow(dt_, 3)/3;
    double t3 = pow(dt_, 2)/2;
    double t2 = dt_;
    double da = is_stable ? da_low : da_high;
    Q << t4*dd, t3*dd, 0,     0,     0,   0, 0,   0, 0,     0,
         t3*dd, t2*dd, 0,     0,     0,   0, 0,   0, 0,     0,
         0,     0,     t4*dd, t3*dd, 0,   0, 0,   0, 0,     0,
         0,     0,     t3*dd, t2*dd, 0,   0, 0,   0, 0,     0,
         0,     0,     0,     0,     dz,  0, 0,   0, 0,     0,
         0,     0,     0,     0,     0,  dz, 0,   0, 0,     0,
         0,     0,     0,     0,     0,   0, dr,  0, 0,     0,
         0,     0,     0,     0,     0,   0, 0,  dr, 0,     0,
         0,     0,     0,     0,     0,   0, 0,   0, t4*da, t3*da,
         0,     0,     0,     0,     0,   0, 0,   0, t3*da, t2*da;

    return Q;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(double dt) {
    dt_ = dt;
    // 状态转移矩阵
    Eigen::MatrixXd F(10, 10);
    F << 1, dt, 0,  0, 0, 0, 0, 0, 0,  0,
         0,  1, 0,  0, 0, 0, 0, 0, 0,  0,
         0,  0, 1, dt, 0, 0, 0, 0, 0,  0,
         0,  0, 0,  1, 0, 0, 0, 0, 0,  0,
         0,  0, 0,  0, 1, 0, 0, 0, 0,  0,
         0,  0, 0,  0, 0, 1, 0, 0, 0,  0,
         0,  0, 0,  0, 0, 0, 1, 0, 0,  0,
         0,  0, 0,  0, 0, 0, 0, 1, 0,  0,
         0,  0, 0,  0, 0, 0, 0, 0, 1, dt,
         0,  0, 0,  0, 0, 0, 0, 0, 0,  1;
    
    // 过程噪声
    Eigen::MatrixXd Q = computeProcessNoise();
    // 预测步骤
    x_pri = F * x_post;
    P_pri = F * P_post * F.transpose() + Q;
    
    return x_pri;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Armors& armors, const std::vector<int>& match_indices) {
    if (match_indices.empty()) return x_pri;
    
    // 准备策略数据
    UpdateStrategy::Data data{armors, match_indices};
    
    // 创建策略
    auto strategy = createStrategy(data);
    if (!strategy) return x_pri;
    
    // 配置策略
    strategy->configure(x_pri, data);
    
    // 计算更新组件
    auto [residual, H, V, R] = strategy->compute();
    
    // 卡尔曼增益
    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + V * R * V.transpose()).inverse();
    // 状态更新
    x_post = x_pri + K * residual;
    P_post = (I - K * H) * P_pri;
    // 状态约束
    constrainState(x_post);
    
    // 检查是否稳定
    if(!is_stable){
        if (high_times > 80) {
            is_stable = true;
        }else{
            high_times++;
        }

    }

    
    return x_post;
}

std::unique_ptr<ExtendedKalmanFilter::UpdateStrategy> 
ExtendedKalmanFilter::createStrategy(const UpdateStrategy::Data& data) {
    size_t n = data.match_indices.size();
    
    if (n == 1) {
        return std::make_unique<SingleArmorStrategy>();
    }
    
    if (n == 2) {
        return std::make_unique<DualArmorStrategy>();
    }
    
    return nullptr;
}

void ExtendedKalmanFilter::constrainState(Eigen::VectorXd& state) {
    // 半径约束
    state(6) = std::clamp(state(6), 0.18, 0.4);
    state(7) = std::clamp(state(7), 0.18, 0.4);
    
    // 角度标准化
    state(8) = _std_radian(state(8));
    
    // 角速度约束
    if (abs(state(9)) > 25) {
        state(9) = 0;
    }
}
