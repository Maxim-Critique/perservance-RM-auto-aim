#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "tiny_api.hpp"
#include "Predict.h"
#include "trajectory.hpp"
class MPC
{
public:
    MPC();
    Gimbal_Angle TrajectoryPlanning(std::shared_ptr<KinematicModel> model);
    
private:
    double GetGimbalYaw(double time);
    tools::Trajectory GetTrajectory();
    std::shared_ptr<KinematicModel> m_status;
    TinySolver *yaw_solver;
    double predict_time;
    double max_yaw_acc;      // 最大角加速度
    double target_yaw;       // 目标yaw
    double target_vel;       // 目标角速度
    double target_acc;       // 目标角加速度
};