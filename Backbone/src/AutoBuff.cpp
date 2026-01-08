#include "AutoBuff.h"

AutoBuff::AutoBuff(){
    discover_ = std::make_unique<auto_buff::BuffDetector>();
    slover_ = std::make_unique<auto_buff::Solver>();
    target_ = std::make_unique<auto_buff::SmallTarget>();
    planner_ = std::make_unique<auto_buff::Planner>();
}

void AutoBuff::AutoAim(cv::Mat &img,const Data_Save &ds,Data_Get &dg){
    if(ds.armor_or_buff == last_mode){
        if(ds.armor_or_buff == 1)target_ = std::make_unique<auto_buff::SmallTarget>();
        else{target_ = std::make_unique<auto_buff::BigTarget>();}
        last_mode = ds.armor_or_buff;
    }
    dg.Vision2Control();
    auto power_runes = discover_->Detect(img);
    slover_->UpdateStatus(ds);
    slover_->Solve(power_runes);
    target_->get_target(power_runes,ds.timestamp);
    auto target_copy =  target_->clone();
    auto gimbal_angle = planner_->Plan(*target_copy,ds.timestamp);
    Transmit(gimbal_angle,ds,dg);
}


void AutoBuff::Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg){
    int32_t yaw = (tools::R2D(gimbal_angle.target_yaw)) * 1e4;
    int32_t pitch = (tools::R2D(gimbal_angle.target_pitch)) * 1e4;
 
    bool yaw_delta = abs(tools::R2D(gimbal_angle.target_yaw) - tools::R2D(ds.camera_yaw_angle)) < 1;
    bool pitch_delta = abs(tools::R2D(gimbal_angle.target_pitch) - tools::R2D(ds.camera_pitch_angle)) < 1;
    bool fire_delta = (yaw_delta && pitch_delta);
    bool follow_dalta = gimbal_angle.manage_yaw;
    dg.Vision2Control(yaw,pitch,0,0,follow_dalta,fire_delta);
}
