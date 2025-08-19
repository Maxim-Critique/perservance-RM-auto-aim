#ifndef CAEKF_H
#define CAEKF_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include "find_armor.h"
class CAEKF {
public:
    CAEKF(Eigen::VectorXd & X_);
    void predict(double time);
    void update(Point3d point);
    Point3d GetArmor(double predict_time);
    
private:
    Eigen::MatrixXd jacobianFunctionF();
    Eigen::VectorXd measurementFunction();
    Eigen::MatrixXd jacobianFunctionH();
    Eigen::MatrixXd processNoiseCovariance();
    
    void readYAML();
    
    Eigen::VectorXd x_pri;
    Eigen::VectorXd x_post;
    Eigen::MatrixXd P_pri;
    Eigen::MatrixXd P_post;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd K;
    Eigen::MatrixXd I;
    double dt_;
    
    double dd_;
    double dz_;
    double rd_;
    double rz_;
};

#endif // CAEKF_H