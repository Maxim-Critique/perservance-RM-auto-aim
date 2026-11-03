#include "AutoArmor.h"

#include "logger.hpp"
#include <filesystem>

AutoArmor::AutoArmor(const std::string & config_path, bool realtime) : realtime_(realtime)
{
    const auto yaml = YAML::LoadFile(config_path);
    config_ = sentry_aim::Config::from_yaml(yaml);
    show_ = yaml["Camera"]["show"].as<int>(0) != 0;
    solve_ptr_ = std::make_unique<Slover>(config_path);
    model_dir_ = (std::filesystem::path(config_path).parent_path() / "model").string();
    tracker_ = std::make_unique<sentry_aim::Tracker>(config_);
    aimer_ = std::make_unique<sentry_aim::Aimer>(config_);
    shooter_ = std::make_unique<sentry_aim::Shooter>(config_);
}

void AutoArmor::Reset()
{
    tracker_->reset();
    aimer_->reset();
    shooter_->reset();
    last_mode_ = last_color_ = -1;
}

bool AutoArmor::AutoAim(Mat & img, const Data_Save & ds, Data_Get & dg)
{
    dg.Vision2Control();
    const auto enemy = sentry_aim::color_from_a(ds.color);
    if (img.empty() || !enemy || !std::isfinite(ds.camera_yaw_angle) ||
        !std::isfinite(ds.camera_pitch_angle) || !std::isfinite(ds.camera_roll_angle)) {
        tracker_->reset();
        aimer_->reset();
        shooter_->reset();
        return false;
    }
    if (ds.armor_or_buff != last_mode_ || ds.color != last_color_) {
        const bool outpost = ds.armor_or_buff == 3;
        find_ptr_ = std::make_unique<ArmorDetector>(
            model_dir_ + (outpost ? "/outpost.onnx" : "/mlp.onnx"),
            model_dir_ + "/label.txt", 0.7, outpost ? 100 : 180,
            outpost ? std::vector<std::string>{"negative", "base"} :
                      std::vector<std::string>{"negative", "outpost", "base"});
        tracker_->reset();
        aimer_->reset();
        shooter_->reset();
        last_mode_ = ds.armor_or_buff;
        last_color_ = ds.color;
    }
    const auto armors = find_ptr_->Detect(img, ds.color);
    solve_ptr_->UpdateStatus(ds);
    std::vector<sentry_aim::Armor> observations;
    Armors solved;
    for (auto armor : armors) {
        if (sentry_aim::name_from_a(armor.number) == sentry_aim::ArmorName::not_armor) continue;
        try {
            if (!solve_ptr_->Solve(armor)) continue;
            const auto observation = sentry_aim::observation_from_a(armor, ds.color, config_);
            if (observation) {
                observations.push_back(*observation);
                solved.push_back(std::move(armor));
            }
        } catch (const cv::Exception & e) {
            tools::logger()->warn("[AutoArmor] Invalid PnP observation: {}", e.what());
        }
    }
    // Empty detections still advance the tracker, allowing prediction through brief occlusion.
    const auto * target = tracker_->track(std::move(observations), ds.timestamp, *enemy, img.size());
    if (show_) ShowArmors(solved, img);
    if (tracker_->generation() != generation_) {
        aimer_->reset();
        shooter_->reset();
        generation_ = tracker_->generation();
    }
    if (!target) {
        aimer_->reset();
        shooter_->reset();
        return false;
    }
    const double latency = realtime_ ? std::max(0.0,
        tools::delta_time(std::chrono::steady_clock::now(), ds.timestamp)) : 0.0;
    const auto command = aimer_->aim(*target, ds.timestamp, bullet_speed, latency);
    const bool fire = shooter_->shoot(command, aimer_->debug_aim_point, target,
        Eigen::Vector3d(ds.camera_yaw_angle, ds.camera_pitch_angle, ds.camera_roll_angle),
        tracker_->state() == sentry_aim::Tracker::State::tracking, tracker_->observed());
    sentry_aim::write_command(command, target->name, fire, dg);
    return command.control;
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
