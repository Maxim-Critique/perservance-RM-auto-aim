#include "AutoArmor.h"

AutoArmor::AutoArmor(){
    readYAML(YamlPath);
    find_ptr = std::make_unique<Tradition>();
    planner_ptr = std::make_unique<Planner>();
    solve_ptr = std::make_unique<Slover>();
    last_time = std::chrono::steady_clock::now()-std::chrono::seconds(4);
    time_interval = 4.0; 
}
void AutoArmor::AutoAim(Mat &img,const Data_Save &ds,Data_Get &dg){
    dg.Vision2Control();
    LightBlobs light_blobs = find_ptr->FindLight(img,ds.color);
    if(light_blobs.size()<2){
        tools::logger()->warn("未识别到足够灯条，灯条数量: {}", light_blobs.size());
        return;
    }
    auto armors = find_ptr->FindArmor(img,light_blobs);
    if(armors.empty()){
        tools::logger()->warn("灯条配对后未生成任何装甲板。");
        return;
    }
    classifier.extractNumbers(img,armors);
    classifier.classify(armors);
    if(armors.empty()){
        tools::logger()->warn("装甲板数字识别与分类后无有效目标。");
        return;
    }
    auto armors_ = ChooseProcess(armors,ds);
    if(armors_.empty())return;
    tools::logger()->info("装甲板数字识别成功。");
    ShowArmors(armors_,img);
    auto target_state = aim_ptr->Estimate(armors_, time_interval);
    auto gimbal_angle = planner_ptr->TrajectoryPlanning(target_state);
    Transmit(gimbal_angle,ds,dg);
}

void AutoArmor::Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg){
    int32_t yaw = (tools::R2D(gimbal_angle.manage_yaw) + compensate_yaw) * 1e4;
    int32_t pitch = (tools::R2D(gimbal_angle.target_pitch) + compensate_pitch) * 1e4;
    bool yaw_delta = abs(tools::R2D(gimbal_angle.target_yaw) + compensate_yaw - tools::R2D(ds.camera_yaw_angle)) < 2;
    bool pitch_delta = abs(tools::R2D(gimbal_angle.target_pitch) + compensate_pitch - tools::R2D(ds.camera_pitch_angle)) < 2;
    bool fire_delta = (yaw_delta && pitch_delta);
    dg.Vision2Control(yaw,pitch,m_aim_id,1,fire_delta);
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
    time_interval = (ds.timestamp-last_time).count() / 1e9;
    solve_ptr->UpdateStatus(ds);

    if(time_interval>0.2){
        for(auto armor : armors){
            solve_ptr->Solve(armor);
            if(armor.gim_pose.norm()<10){
                armors_.push_back(armor); 
                aim_ptr = std::make_unique<Aim>(armor);
                last_time = ds.timestamp;
                break;
            }
        }
    }else{
        if(!aim_ptr)return armors_;
        
        for(auto armor : armors){
            if(armor.number == aim_ptr->aim_id){
                solve_ptr->Solve(armor);
                if(armor.gim_pose.norm()<10){
                    armors_.push_back(armor);
                    last_time = ds.timestamp;
                }
            }
            if(armors_.size()>2){
                break;
            }
        }
    }
    // if(!armors_.empty())m_aim_id = FoundID(aim_ptr->aim_id);
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
        cv::putText(img, cv::format("X:%.2f Y:%.2f Z:%.2f,Yaw:%.1f", 
            armor.abs_pose.x(), 
            armor.abs_pose.y(), 
            armor.abs_pose.z(),
            tools::R2D(armor.yaw-armor.predict_yaw)),
            armor.points[1], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }
    
}