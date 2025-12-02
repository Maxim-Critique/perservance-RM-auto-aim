#include "AutoArmor.h"

AutoArmor::AutoArmor(){
    readYAML(YamlPath);
    find_ptr = std::make_unique<Tradition>();
    planner_ptr = std::make_unique<MPC>();
    solve_ptr = std::make_unique<Slover>();
    last_time = std::chrono::steady_clock::now()-std::chrono::seconds(4);
}
void AutoArmor::AutoAim(Mat &img,const Data_Save &ds,Data_Get &dg){
    dg.Vision2Control();
    LightBlobs light_blobs = find_ptr->FindLight(img,ds.color);
    if(light_blobs.size()<2)return;
    auto armors = find_ptr->FindArmor(img,light_blobs);
    classifier.extractNumbers(img,armors);
    classifier.classify(armors);
    if(armors.empty())return;
    auto armors_ = ChooseProcess(armors,ds);
    if(armors_.empty())return;
    ShowArmors(armors_,img);
    UpdateTimeInterval(ds.timestamp);
    if(time_interval == 0)return;
    auto status = aim_ptr->Estimate(armors_, time_interval);
    auto gimbal_angle = planner_ptr->TrajectoryPlanning(*status);
    Transmit(gimbal_angle,dg);
}

void AutoArmor::UpdateTimeInterval(time_point timestamp){
    auto duration = timestamp - last_time;
    time_interval = duration.count() / 1e9;
    cout<<time_interval<<endl;
    last_time = timestamp;
}

void AutoArmor::Transmit(const Gimbal_Angle &gimbal_angle,Data_Get &dg){
    int32_t yaw = (tools::R2D(gimbal_angle.manage_yaw) + compensate_yaw) * 1e4;
    int32_t pitch = (tools::R2D(gimbal_angle.target_pitch) + compensate_pitch) * 1e4;
    bool yaw_delta = abs(tools::R2D(gimbal_angle.target_yaw) + compensate_yaw ) < 1;
    bool pitch_delta = abs(tools::R2D(gimbal_angle.target_pitch)  + compensate_pitch ) < 1;
    bool fire_delta = (yaw_delta && pitch_delta);
    dg.Vision2Control(yaw,pitch,gimbal_angle.yaw_vel*1e4,0,1,fire_delta);
}

// 处理倒影
void SortArmors(Armors& armors) {
    std::sort(armors.begin(), armors.end(), 
        [](const Armor& a, const Armor& b) {
            return a.points[0].y < b.points[0].y;
        });
}

// 从多个装甲板中按照ID选择进整车观测的装甲板
Armors AutoArmor::ChooseProcess(Armors &armors,const Data_Save &ds){
    Armors armors_;
    SortArmors(armors);
    solve_ptr->UpdateStatus(ds);
    if(ds.timestamp-last_time>std::chrono::seconds(2)){
        for(auto armor : armors){
            solve_ptr->Solve(armor);
            if(armor.gim_pose.norm()<5){
                armors_.push_back(armor); 
                aim_ptr = std::make_unique<Aim>(armor);
                break;
            }
        }
    }else{
        for(auto armor : armors){
            if(armor.number == aim_ptr->aim_id){
                if(armors_.size()){
                    if(abs(armors[0].center.x-armor.center.x>(armor.points[3].x-armor.points[0].x)/2)){
                        solve_ptr->Solve(armor);
                        armors_.push_back(armor);
                    }
                }else{
                    solve_ptr->Solve(armor);
                    armors_.push_back(armor);
                }
            }
            if(armors_.size()>2){
                break;
            }
        }
    }
    return armors_;
}

int AutoArmor::FoundID(string x) {
    if (x == "outpost") {
        return 6;
    } else if (x == "guard") {
        return 7;
    } else if (x == "base") {
        return 8;
    } else {
        return std::stoi(x);
    }
}

YAML::Node AutoArmor::readYAML(const std::string& filename){
    try {
        YAML::Node config = YAML::LoadFile(filename);
        compensate_pitch = config["Transmit"]["compensate_pitch"].as<double>();
        compensate_yaw = config["Transmit"]["compensate_yaw"].as<double>();
        show = config["Camera"]["show"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()) + "AutoArmor");
    }
    return YAML::LoadFile(filename);
}

void AutoArmor::ShowArmors(const Armors &armors,Mat &img){
    for(auto &armor :armors){
        if(!armor.points.empty()){
            for(int i=0;i<4;i++){
                cv::line(img, armor.points[i], armor.points[(i+1)%4], cv::Scalar(125, 0, 125), 2);
                cv::putText(img, std::to_string(i), cv::Point(armor.points[i].x + 5, armor.points[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }
            if(!armor.re_points.empty()){  
                for(int i=0;i<4;i++){
                    cv::line(img, armor.re_points[i],armor.re_points[(i+1)%4],cv::Scalar(0, 255, 0), 2);
                    cv::putText(img, std::to_string(i), cv::Point(armor.re_points[i].x + 5, armor.re_points[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
                }  
            }
        }
    }
}