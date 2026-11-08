#include "AutoArmor.h"
#include <filesystem>

AutoArmor::AutoArmor(const std::string &path, bool input_rgb) : input_rgb_(input_rgb) {
    readYAML(path);
    const auto yaml = YAML::LoadFile(path);
    const auto backend = yaml["Algorithm"] ? yaml["Algorithm"]["backend"].as<std::string>() : "legacy";
    planner_ptr = std::make_unique<Planner>(path);
    if (backend == "tongji") {
        auto algorithm = std::filesystem::path(yaml["Algorithm"]["config"].as<std::string>());
        if (algorithm.is_relative()) algorithm = std::filesystem::path(path).parent_path() / algorithm;
        tongji_ = std::make_unique<fusion::TongjiAdapter>(path, algorithm.string());
    } else if (backend == "legacy") {
        find_ptr = std::make_unique<ArmorDetector>("../model/mlp.onnx", "../model/label.txt", 0.7, 180,
            std::vector<std::string>{"negative","outpost","base"});
        solve_ptr = std::make_unique<Slover>();
    } else throw std::invalid_argument("Unknown Algorithm.backend: " + backend);
    last_time = std::chrono::steady_clock::now()-std::chrono::seconds(4);
    time_interval = 4.0;
}
void AutoArmor::Reset() {
    if (tongji_) tongji_->reset();
    aim_ptr.reset();
    last_time = std::chrono::steady_clock::now()-std::chrono::seconds(4);
}
bool AutoArmor::AutoAim(Mat &img,const Data_Save &ds,Data_Get &dg){
    dg.Vision2Control();
    if (tongji_) {
        auto model = tongji_->process(img, ds, input_rgb_);
        if (!model) return false;
        m_aim_id = 0; // A405b408 的 FoundID 更新被注释：保持实际协议 ID 输出为 0。
        auto gimbal_angle = planner_ptr->TrajectoryPlanning(model);
        if (!std::isfinite(gimbal_angle.manage_yaw) || !std::isfinite(gimbal_angle.target_pitch) ||
            !std::isfinite(gimbal_angle.target_yaw)) return false;
        Transmit(gimbal_angle, ds, dg);
        return true;
    }
    if (img.empty()) return false;
    if(ds.armor_or_buff != last_mode_){
        if(ds.armor_or_buff == 3) find_ptr = std::make_unique<ArmorDetector>(
            "../model/outpost.onnx", "../model/label.txt", 0.7, 100,
            std::vector<std::string>{"negative","base"}, 21, 0);
        else find_ptr = std::make_unique<ArmorDetector>(
            "../model/mlp.onnx", "../model/label.txt", 0.7, 180,
            std::vector<std::string>{"negative","outpost","base"}, 19, -2);
        last_mode_ = ds.armor_or_buff;
    }


    auto armors = find_ptr->Detect(img, ds.color);
    if(armors.empty())return false;
    auto armors_ = ChooseProcess(armors,ds);
    if(armors_.empty())return false;
    if(show)ShowArmors(armors_,img);
    auto model = aim_ptr->Estimate(armors_, time_interval);

    if (ds.armor_or_buff == 3 &&(std::abs(model->palstance) < 2.0 || std::abs(model->palstance) > 3.0))return true;

    auto gimbal_angle = planner_ptr->TrajectoryPlanning(model);
    Transmit(gimbal_angle,ds,dg);
    return true;
}

void AutoArmor::Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg){
    int32_t yaw = (tools::R2D(gimbal_angle.manage_yaw) + compensate_yaw) * 1e4;
    int32_t pitch = (tools::R2D(gimbal_angle.target_pitch) + compensate_pitch) * 1e4;
    int32_t yaw_val = tools::R2D(gimbal_angle.yaw_vel)*1e4;
    bool yaw_delta = abs(tools::R2D(gimbal_angle.target_yaw) + compensate_yaw - tools::R2D(ds.camera_yaw_angle)) < 2.5;
    bool pitch_delta = abs(tools::R2D(gimbal_angle.target_pitch) + compensate_pitch - tools::R2D(ds.camera_pitch_angle)) < 2.5;
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

    if(time_interval>1.5|| !aim_ptr){
        for(auto armor : armors){
            solve_ptr->Solve(armor);
            if(armor.gim_pose.norm()<12){
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
                if(armor.gim_pose.norm()<12){
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
            armor.gim_pose.x(),
            armor.gim_pose.y(),
            armor.gim_pose.z(),
            tools::R2D(armor.yaw-armor.predict_yaw)),
            armor.points[1], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }

}
