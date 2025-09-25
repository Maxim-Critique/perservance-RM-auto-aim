#include "CA.h"
#include "Coordinate.h"
CAEKF::CAEKF(Eigen::VectorXd & X_) 
    : x_pri(X_), x_post(X_), I(Eigen::MatrixXd::Identity(7,7)) 
{
    // ��ʼ��Э�������
    P_pri = Eigen::MatrixXd::Identity(7,7) * 1e-5;
    P_post = Eigen::MatrixXd::Identity(7,7) * 1e-5;
    dt_ = 0.01;
    
    // ��ȡYAML����
    readYAML();
}

void CAEKF::readYAML() {
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        dd_ = config["CAEKF"]["ProcessNoise"]["dd"].as<double>();
        dz_ = config["CAEKF"]["ProcessNoise"]["dz"].as<double>();
        rd_ = config["CAEKF"]["RNoise"]["rd"].as<double>();
        rz_ = config["CAEKF"]["RNoise"]["rz"].as<double>();
    } catch (YAML::Exception & e) {
        std::cerr << "YAML Error: " << e.what() <<" CAEKF "<< std::endl;
        dd_ = 0.01;
        dz_ = 0.1;
        rd_ = 0.1;
        rz_ = 0.2;
    } 
}

Eigen::MatrixXd CAEKF::jacobianFunctionF() {
    Eigen::MatrixXd F(7,7);
    double dt2 = 0.5 * dt_ * dt_;
    F << 1, dt_, dt2, 0,  0,  0, 0,
         0, 1,   dt_, 0,  0,  0, 0,
         0, 0,   1,   0,  0,  0, 0,
         0, 0,   0,   1, dt_, dt2, 0,
         0, 0,   0,   0, 1,   dt_, 0,
         0, 0,   0,   0, 0,   1, 0,
         0, 0,   0,   0, 0,   0, 1;
    return F;
}

Eigen::VectorXd CAEKF::measurementFunction() {
    Eigen::VectorXd h(3);
    h << x_pri(0), x_pri(3), x_pri(6);
    return h;
}

Eigen::MatrixXd CAEKF::jacobianFunctionH() {
    Eigen::MatrixXd H(3,7);
    H << 1,0,0,0,0,0,0,
         0,0,0,1,0,0,0,
         0,0,0,0,0,0,1;
    return H;
}

Eigen::MatrixXd CAEKF::processNoiseCovariance() {
    Eigen::MatrixXd Q(7,7);
    double d = 100;
    double v = 100;
    double a = 1000;
    double dt = dt_;
    Q << d,       0,     0,    0,       0,       0,    0,
         0,       v,     0,    0,       0,       0,    0,
         0,       0,     a,    0,       0,       0,    0,
         0,       0,     0,    d,       0,       0,    0,
         0,       0,     0,    0,       v,       0,    0,
         0,       0,     0,    0,       0,       a,    0,
         0,       0,     0,    0,       0,       0,    dz_;
    
    return Q;
}

void CAEKF::predict(double time) {
    dt_ = time;
    F = jacobianFunctionF();
    Q = processNoiseCovariance();
    this->x_pri = F * this->x_post;
    P_pri = F * P_post * F.transpose() + Q;
}

void CAEKF::update(Point3d point) {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(3, 1);
    z << point.x,point.y,point.z;
    H = jacobianFunctionH();
    R = Eigen::MatrixXd::Zero(3,3);
    R(0,0) = rd_; R(1,1) = rd_; R(2,2) = rz_;
    
    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    Eigen::VectorXd h = measurementFunction();
    x_post = x_pri + K * (z - h);
    P_post = (I - K * H) * P_pri;
    
    painting.KF_Curve(x_post[0],x_post[1]*80);
}

Point3d CAEKF::GetArmor(double predict_time) {
    double dt = predict_time;
    Point3d pose;
    // ʹ��״̬����ֱ�Ӽ���Ԥ��λ�� (x + v*t + 0.5*a*t?)
    pose = Point3d(
        x_post[0] + x_post[1]*dt + 0.5*x_post[2]*dt*dt,  // x
        x_post[3] + x_post[4]*dt + 0.5*x_post[5]*dt*dt,  // y
        x_post[6]                                       // z
    );
    
    return pose;
}