#include "MPC.h"
#include <fstream>
constexpr double dt = 0.01;  
constexpr int half_horizon = 50;    
constexpr int horizon = half_horizon * 2; 
using trajectory = Eigen::Matrix<double, 2, horizon>; 
MPC::MPC(){

    YAML::Node config = YAML::LoadFile(Yaml);
    predict_time = config["AIM"]["predict_time"].as<double>();
    max_yaw_acc = config["AIM"]["max_yaw_acc"].as<double>();
    Eigen::MatrixXd A{{1, dt}, {0, 1}};
    Eigen::MatrixXd B{{0}, {dt}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(9e6,0);
    Eigen::Matrix<double, 1, 1> R(1e2);
    tiny_setup(&yaw_solver, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, horizon, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, horizon, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, horizon, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, horizon - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, horizon - 1, max_yaw_acc);
    tiny_set_bound_constraints(yaw_solver, x_min, x_max, u_min, u_max);
}

void MPC::TrajectoryPlanning(KinematicModel& m_status){
    double hit_time = cv::norm(Point3d(m_status.center.x,m_status.center.y,(m_status.height[0]+m_status.height[1])/2))/ (bullet_speed) + predict_time;
    double counter =  -(half_horizon + 1);
    double yaw_last = GetGimbalYaw(m_status,hit_time+dt*(counter));
    counter++;
    double yaw_now = GetGimbalYaw(m_status,hit_time+dt*(counter));
    trajectory traj;
    for (int col = 0; counter < half_horizon; col++) {
        counter++;
        double yaw_next = GetGimbalYaw(m_status, hit_time+dt * counter);
        double yaw_vel = _std_radian(yaw_next - yaw_last) / (2 * dt);
        traj.col(col) << yaw_now, yaw_vel;

            std::ofstream out("mpc_log.txt", std::ios::app); 
            out << R2D(yaw_now) << " ";

        yaw_last = yaw_now;
        yaw_now = yaw_next;
    }
    

    tiny_set_x_ref(yaw_solver,traj);
    yaw_solver->work->Xref = traj.block(0, 0, 2, horizon);
    tiny_solve(yaw_solver);
    double yaw = _std_radian(yaw_solver->work->x(0, half_horizon));
    double yaw_vel = yaw_solver->work->x(1, half_horizon);
    double yaw_acc = yaw_solver->work->u(0, half_horizon);
    for (int i = 0; i < horizon; i++){
        std::ofstream out("mpc_log.txt", std::ios::app); 
        out << R2D(yaw_solver->work->x(0, i)) << "\n";
    }

}

double MPC::GetGimbalYaw(KinematicModel& m_status,double time){
    Armors armors = m_status.GetArmors(time);
    double min_distance = 100;
    cv::Point3d point;
    for(auto armor : armors){
        if(min_distance>cv::norm(armor.abs_pose)){
            point = armor.abs_pose;
        } 
    }
    point = C_C.abs2cam(point);
    return std::atan2(point.y, point.x);
}

