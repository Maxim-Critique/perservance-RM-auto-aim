#include "MPC.h"
#include <fstream>
constexpr double dt = 0.01;  
constexpr int half_horizon = 50;    
constexpr int horizon = half_horizon * 2; 
using trajectory = Eigen::Matrix<double, 2, horizon>; 
MPC::MPC(){

    YAML::Node config = YAML::LoadFile(YamlPath);
    predict_time = config["Planer"]["predict_time"].as<double>();
    max_yaw_acc = config["Planer"]["max_yaw_acc"].as<double>();
    Eigen::MatrixXd A{{1, dt}, {0, 1}};
    Eigen::MatrixXd B{{0}, {dt}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(9e6,0);
    Eigen::Matrix<double, 1, 1> R(10);
    tiny_setup(&yaw_solver, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, horizon, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, horizon, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, horizon, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, horizon - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, horizon - 1, max_yaw_acc);
    yaw_solver->settings->en_input_bound = 1;
    yaw_solver->settings->en_state_bound = 1;
    tiny_set_bound_constraints(yaw_solver, x_min, x_max, u_min, u_max);
}

Gimbal_Angle MPC::TrajectoryPlanning(Eigen::VectorXd states){
    
    m_status = std::make_shared<StandardModel>(states);
    auto trajectory_ = GetTrajectory();
    auto hit_time = trajectory_.fly_time + predict_time;
    double counter =  -(half_horizon + 1);
    double yaw_last = GetGimbalYaw(hit_time+dt*(counter));
    counter++;
    double yaw_now = GetGimbalYaw(hit_time+dt*(counter));
    double yaw0 = GetGimbalYaw(hit_time);
    trajectory traj;
    for (int col = 0; counter < half_horizon; col++) {
        counter++;
        double yaw_next = GetGimbalYaw( hit_time+dt * counter);
        double yaw_vel = _std_radian(yaw_next - yaw_last) / (2 * dt);
        traj.col(col) << yaw_now-yaw0, yaw_vel;
        yaw_last = yaw_now;
        yaw_now = yaw_next;
    }
    
    
    tiny_set_x_ref(yaw_solver,traj);
    yaw_solver->work->Xref = traj.block(0, 0, 2, horizon);
    tiny_solve(yaw_solver);
    
    target_yaw = _std_radian(yaw_solver->work->x(0, half_horizon)+yaw0);
    target_vel = yaw_solver->work->x(1, half_horizon);
    target_acc = yaw_solver->work->u(0, half_horizon);
    Gimbal_Angle gimbal_angle;
    gimbal_angle.manage_yaw = target_yaw;
    gimbal_angle.yaw_vel = target_vel;
    gimbal_angle.yaw_acc = target_acc;
    gimbal_angle.target_yaw = yaw0;
    gimbal_angle.target_pitch = trajectory_.pitch;
    
    return gimbal_angle;

}

double MPC::GetGimbalYaw(double time){
    Armors armors = m_status->GetArmors(time);
    double min_distance = 100;
    Eigen::Vector3d point;
    for(auto armor : armors){
        if(min_distance>armor.abs_pose.norm()){
            point = armor.abs_pose;
            min_distance = armor.abs_pose.norm();
        } 
    }
    return std::atan2(point.y(), point.x());
}

tools::Trajectory MPC::GetTrajectory(){
    Armors armors = m_status->GetArmors(0);
    double min_distance = 100;
    Eigen::Vector3d point;
    for(auto armor : armors){
        if(min_distance>armor.abs_pose.norm()){
            point = armor.abs_pose;
            min_distance = armor.abs_pose.norm();
        } 
    }
    auto distance = sqrt(point.x()*point.x()+point.y()*point.y());
    return tools::Trajectory(bullet_speed,distance,point.z());
}

