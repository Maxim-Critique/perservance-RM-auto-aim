#include "AutoBuff.h"
#include <fstream>
AutoBuff::AutoBuff(){
    discover_ = std::make_unique<auto_buff::BuffDetector>();
    slover_ = std::make_unique<auto_buff::Solver>();
    target_ = std::make_unique<auto_buff::SmallTarget>();
    planner_ = std::make_unique<auto_buff::Planner>();
}

void AutoBuff::AutoAim(cv::Mat &img,const Data_Save &ds,Data_Get &dg){

    // if(ds.armor_or_buff != last_mode){
    //     if(ds.armor_or_buff == 1) target_ = std::make_unique<auto_buff::SmallTarget>();
    //     else target_ = std::make_unique<auto_buff::BigTarget>();
    //     last_mode = ds.armor_or_buff;
    // }
    dg.Vision2Control();
    auto power_runes = discover_->Detect(img);
    slover_->UpdateStatus(ds);
    slover_->Solve(power_runes);
    target_->get_target(power_runes,ds.timestamp);
    auto target_copy =  target_->clone();
    auto gimbal_angle = planner_->Plan(*target_copy,ds.timestamp);
    Transmit(gimbal_angle,ds,dg,img);
    // Show(img);
}

void AutoBuff::Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg,cv::Mat &img){
    
    int32_t yaw = (tools::R2D(gimbal_angle.target_yaw)) * 1e4;
    int32_t pitch = (tools::R2D(gimbal_angle.target_pitch)) * 1e4;
    bool yaw_delta = abs(tools::R2D(gimbal_angle.target_yaw) - tools::R2D(ds.camera_yaw_angle)) < 1;
    bool pitch_delta = abs(tools::R2D(gimbal_angle.target_pitch) - tools::R2D(ds.camera_pitch_angle)) < 1;
    bool fire_delta = (yaw_delta && pitch_delta);
    bool follow_dalta = gimbal_angle.manage_yaw && yaw!=0 && pitch !=0;
    if(fire_delta) cv::putText(img, "Fire", cv::Point(40, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 4);
    dg.Vision2Control(yaw,pitch,0,0,follow_dalta,follow_dalta);
}

void AutoBuff::Show(cv::Mat &img){
    auto Rxyz_in_world_now = target_->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
    auto image_points = slover_->reproject_buff(Rxyz_in_world_now, target_->ekf_x()[4], target_->ekf_x()[5]);
    cv::circle(img, image_points[4], 8, cv::Scalar(128, 0, 128), -1);
}
