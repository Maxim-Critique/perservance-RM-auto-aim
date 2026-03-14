// #include "Buff_Task.h"
// #include "Base.h"
// #include <Eigen/Dense>
// #include "math.hpp"
// #include <thread>
// #include <mutex>
// #include <deque>
// #include <algorithm>
// #include <condition_variable>
// #include <cmath> 
// Data_Save ds;
// extern SerialPort serial_;

// double VectorizedVaneArctan2(Point3d point) {
//     vector<double> result;
//     double r = sqrt(point.x * point.x + point.y * point.y);
//     double angle = 0.0;

//     // ���ݲ�ͬ���޴����Ƕ�
//     if (point.x > 0) {
//         angle = atan2(point.z, r);
//     } else if (point.x < 0) {
//         angle = atan2(point.z, -r);
//     } else {  // x = 0 ���������
//         if (point.z > 0) {
//             angle = PI / 2.0;
//         } else if (point.z < 0) {
//             angle = -PI / 2.0;
//         } else {
//             angle = 0.0;
//         }
//     }

//     // ȷ�������[0, 2��]��Χ��
//     if (angle < 0) {
//         angle += 2.0 * PI;
//     }
//     return angle;
// }



// Buff_Task::Buff_Task() {
//     // try {
//     //     YAML::Node config = YAML::LoadFile(YamlPath);
//     //     compensate_pitch = config["Buff"]["pitch"].as<double>();
//     //     compensate_yaw = config["Buff"]["yaw"].as<double>();
//     //     show = config["Camera"]["show"].as<double>();
//     // } catch (const YAML::Exception& e) {
//     //     throw std::runtime_error("YAML Error: " + std::string(e.what()));
//     // }
     
//     // Buff_discover = std::make_unique<Buff_Discover>(); 
// }

// Buff_Task::~Buff_Task() {}

// void Buff_Task::GetThetaPhi(){
//     Point3d point = Point3d(Buff_discover->Aimpose.x - Buff_discover->Rpose.x,Buff_discover->Aimpose.y - Buff_discover->Rpose.y,Buff_discover->Aimpose.z - Buff_discover->Rpose.z);
//     theta = VectorizedVaneArctan2(point);
//     phi = atan2(point.x, point.y);
//     if(phi<0){
//         phi+=PI;
//     }
//     if(!rotatedirection){
//         if(!last_angle){
//             if(_std_radian(theta-last_angle)<2*PI/5){
//                 if(_std_radian(theta-last_angle)>0){
//                     rotatedirection_number++;
//                 }else if(_std_radian(theta-last_angle)<0){
//                     rotatedirection_number--;
//                 }
//             }
//         }
//         if(abs(rotatedirection_number) >= 20){
//             rotatedirection = rotatedirection_number/abs(rotatedirection_number);
//         }
//         last_angle = theta;
//     }
// }

// bool Buff_Task::SeizeGlory(Mat &src, time_point time) {   
//     if (src.empty()) {
//         printf("could not load image\n");
//         return false;
//     }
//     Mat test = Buff_discover->Buff_img_deal(src);
//     imshow("test",test);
//     bool Found = Buff_discover->FoundBuff(test, src);

//     // if(Found){
//     //     double time_interval = (time - last_time)/1e8;
//     //     last_time = time;
//     //     GetThetaPhi(); 
//     //     if(ds.go_back - 1) {
//     //         // if(!init){
//     //         //     EKF = std::make_unique<CAlphaEKF>(this->theta);
//     //         //     init = true;
//     //         // }
//     //         // EKF->predict(time_interval);
//     //         // Eigen::Vector4d state = EKF->update(this->theta-PI);
//     //         // state[0]+=PI;
            
//     //         // // painting.KF_Curve(tools::R2D(theta),tools::R2D(state[0]));
//     //         // painting.Speed_Curve(state[1]*50);
//     //         // Gauss->fit(state[0],state[1],time);
//     //         // double znacun = theta;
//     //         // theta = Gauss->getAngle(time+0.5*1e8);
//     //         // painting.KF_Curve(tools::R2D(znacun),tools::R2D(theta));
//     //         // std::ofstream out("ekf_data.txt", std::ios::app);  // ׷��ģʽ
//     //         // out << time*1e-8 << " "<<state[1]<<"\n";
//     //         theta -= tools::D2R(30);
//     //     }else{
//     //         theta += tools::D2R(30);
//     //     }
//     //     Point2d prediction = Point2d(0.65 * cos(theta),0.65 * sin(theta));
//     //     Buff_discover->Rpose = last_center *0.9+Buff_discover->Rpose*0.1;
//     //     last_center = Buff_discover->Rpose;

//     //     Point3d predict_pose = Point3d(Buff_discover->Rpose.x+prediction.x*sin(phi),Buff_discover->Rpose.y+prediction.x*cos(phi),Buff_discover->Rpose.z+prediction.y);
//     //     // Point3d predict_pose = Buff_discover->Rpose;
//     //     predict_pose = tools::abs2cam(predict_pose);
//     //     // Point2d zi = pnp_armor.ProjectTo2D(Point3d(-predict_pose.y,-predict_pose.z,predict_pose.x+error_x));
//     //     // cv::circle(src, zi, 40, cv::Scalar(0, 0, 255), -1); 
//     //     Transmit(predict_pose);
//     //     return true;
//     // }
//     // return false;
// }

// void Buff_Task::Transmit(Point3d Aim){
//     Gimbal_Angle angle = tools::GetAngle(Aim);
//     int32_t yaw = (tools::R2D(angle.target_yaw) + compensate_yaw) * 1e4;
//     int32_t pitch = (tools::R2D(angle.target_pitch) + compensate_pitch) * 1e4;
//     // Point2d zi = pnp_armor.ProjectTo2D(Point3d(-Aim.y,-Aim.z,Aim.x+error_x));
//     bool yaw_delta = abs(tools::R2D(angle.target_yaw) - tools::R2D(ds.camera_yaw_angle)) < 20;
//     bool pitch_delta = abs(tools::R2D(angle.target_pitch)  + compensate_pitch - tools::R2D(ds.camera_pitch_angle) -1) < 1.5;
//     if(yaw_delta)
//     {
//         dg.Vision2Control(yaw,pitch,0,0,3,0);
//     }else{
//         dg.Vision2Control(0,0,0,0,0,0);
//     }
//     serial_.SendData(dg);
// }

