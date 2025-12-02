#include"test.h"
#include "AIM.h"
int MPCTest(){
    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(10);
    std::shared_ptr<KinematicModel> m_status;
    target_state<<3,0,0,0,0,0,0.2,0.2,0,10;
    std::unique_ptr<MPC> mpc = std::make_unique<MPC>();
    io::SerialPort serial_("/dev/ttyACM0");
    Data_Get dg;
    while(true){ 
        target_state(8)+=target_state(9)*0.01;
        m_status = std::make_shared<StandardModel>(target_state);
        auto trajectory = mpc->TrajectoryPlanning(*m_status);
        dg.Vision2Control(trajectory.manage_yaw*1e4,trajectory.target_pitch*1e4,trajectory.yaw_vel*1e4,trajectory.yaw_acc*1e4,1,0);
        serial_.SendData(dg);
    }
    
    
}