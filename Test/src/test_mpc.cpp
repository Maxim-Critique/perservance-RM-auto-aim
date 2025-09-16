#include"test.h"
#include "AIM.h"
int MPCTest(){
    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(10);
    std::shared_ptr<KinematicModel> m_status;
    target_state<<3,0,0,0,0,0,0.2,0.2,0,15;
    std::unique_ptr<MPC> mpc = std::make_unique<MPC>();
    while(true){ 
        target_state(8)+=target_state(9)*0.01;
        m_status = std::make_shared<StandardModel>(target_state);
        mpc->TrajectoryPlanning(*m_status);
    }
    
    
}