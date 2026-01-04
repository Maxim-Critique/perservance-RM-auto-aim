#include "test.h"
#include "AIM.h"
#include "Planner.h"
int MPCTest(){
    
    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(10);
    target_state<<3,0,0,0,0,0,0.2,0.2,0,10;
    std::unique_ptr<Planner> planner = std::make_unique<Planner>();
    io::SerialPort serial_("/dev/ttyACM0");
    Data_Get dg;
    std::ofstream out("mpc_log.txt", std::ios::app); 
    int i = 0;
    while(true){ 
        target_state(8)+=target_state(9)*0.01;
        auto trajectory = planner->TrajectoryPlanning(target_state);
        // dg.Vision2Control(tools::R2D(trajectory.manage_yaw)*1e4,tools::R2D(trajectory.target_pitch)*1e4,tools::R2D(trajectory.yaw_vel)*1e4,trajectory.yaw_acc*1e4,1,0);
        // serial_.SendData(dg);
        out << trajectory.target_yaw << " " << trajectory.manage_yaw << "\n";
        i++;
        if(i>200)break;
        
    }
    return 0;
}