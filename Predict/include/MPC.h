#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "tiny_api.hpp"
#include "Predict.h"
class MPC
{
public:
    MPC();
    Gimbal_Angle TrajectoryPlanning(KinematicModel& m_status);
    
private:
    double GetGimbalYaw(KinematicModel& m_status,double time);
    double GetGimbalPitch(KinematicModel& m_status,double time);
    TinySolver *yaw_solver;
    double predict_time;
    double max_yaw_acc;      // 最大角加速度
    double target_yaw;       // 目标yaw
    double target_vel;       // 目标角速度
    double target_acc;       // 目标角加速度
};