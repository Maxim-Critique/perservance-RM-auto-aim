#include "IAmFuture.h"

Auto_aim::Auto_aim(){
    readYAML(Yaml);
    find_ptr = std::make_unique<Tradition>();
}

int Auto_aim::IAmFuture(Mat &img,uint64_t timestamp){
   
    //尝试次数大于20重置
    if(try_times == 20){ReEKF();}
    LightBlobs light_blobs = find_ptr->FindLight(img);
    if(light_blobs.size()>=2){
        Reboot();
        return 0;
    }
    //传统找装甲板
    Armors armors = find_ptr->FindArmor(img,light_blobs);
    //MLP神经网络
    classifier.extractNumbers(img,armors);
    classifier.classify(armors);
    if(armors.empty()){
        Reboot();
        return 0;
    }
    armors = ChooseProcess(armors,img);
    if(armors.empty()){
        Reboot();
        return 0;
    }
    UpdateTimeInterval(timestamp);
    if(time_interval == 0){
        Reboot();
        return 0;
    }
    auto gimbal_angle = aim_ptr->Shoot(armors,img,time_interval);
    Transmit(gimbal_angle);
}

void Auto_aim::UpdateTimeInterval(uint64_t timestamp){
    time_interval = (timestamp - last_time)/1e8;
    last_time = timestamp;
}

void Auto_aim::Transmit(const Gimbal_Angle &gimbal_angle){

    if(abs(initial_abs.target_yaw-gimbal_angle.target_yaw)<PI/6){
        int32_t yaw = (R2D(gimbal_angle.manage_yaw) + compensate_yaw) * 1e4;
        int32_t pitch = (R2D(gimbal_angle.target_pitch) + compensate_pitch) * 1e4;
        std::unique_lock<std::mutex> lock(ds_mutex);
        bool yaw_delta = abs(R2D(gimbal_angle.target_yaw) + compensate_yaw - R2D(ds.camera_yaw_angle)) < 1;
        bool pitch_delta = abs(R2D(gimbal_angle.target_pitch)  + compensate_pitch - R2D(ds.camera_pit_angle)) < 1;
        lock.unlock();

        if(yaw_delta && pitch_delta)
        {
            dg.Vision2Control(yaw,pitch,gimbal_angle.yaw_vel*1e4,0,1,1);
        }
        else
        {
           dg.Vision2Control(yaw,pitch,gimbal_angle.yaw_vel*1e4,0,1,0);
        }
    }
    else{
        dg.Vision2Control(0,0,0,0,4,0);
    }
    serial_.send_data(dg);
}

//处理倒影
void SortArmors(Armors& armors) {
    std::sort(armors.begin(), armors.end(), 
        [](const Armor& a, const Armor& b) {
            return a.points[0].y < b.points[0].y;
        });
}

//从多个装甲板中按照ID选择进整车观测的装甲板
Armors Auto_aim::ChooseProcess(Armors armors,Mat &img){
    Armors after_armors;
    SortArmors(armors);
    std::unique_ptr<PNP> pnp = std::make_unique<PNP>();
    if(aim_id.empty()){
        for(auto armor : armors){
            pnp->Solve(armor, img, initial_abs);
            if(cv::norm(armor.cam_pose)<5){
                after_armors.push_back(armor); 
                aim_id = armor.number;
                aim_ptr = std::make_unique<Aim>(armor);
                break;
            }
        }
    }else{
        for(auto armor : armors){
            if(armor.number == aim_id){
                if(after_armors.size()){
                    if(abs(armors[0].center.x-armor.center.x>(armor.points[3].x-armor.points[0].x)/2)){
                        pnp->Solve(armor, img, initial_abs);
                        after_armors.push_back(armor);
                    }
                }else{
                    pnp->Solve(armor, img, initial_abs);
                    after_armors.push_back(armor);
                }
            }
            if(after_armors.size()>1){
                break;
            }
        }
    }

    return after_armors;
}


Rect Auto_aim::MakeROI(Point2d found_armor) {
    const int IMG_WIDTH = 1440;
    const int IMG_HEIGHT = 1080;
    const int ROI_WIDTH = 640;
    const int ROI_HEIGHT = 480;

    Point2f roi_center = found_armor;
    // 计算ROI边界
    int x0 = cvRound(roi_center.x - ROI_WIDTH / 2);
    int y0 = cvRound(roi_center.y - ROI_HEIGHT / 2);
    
    // 边界保护
    x0 = std::clamp(x0, 0, IMG_WIDTH - ROI_WIDTH);
    y0 = std::clamp(y0, 0, IMG_HEIGHT - ROI_HEIGHT);
    return Rect(x0, y0, ROI_WIDTH, ROI_HEIGHT);
}

int Auto_aim::FoundID(string x) {
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

void Auto_aim::Reboot(){
    roi = Rect();
    try_times++;
    dg.Vision2Control(0,0,0,0,4,0);
    serial_.send_data(dg);
}

void Auto_aim::ReEKF(){
    roi = Rect();
    try_times = 0;
    aim_id.clear();
} 

YAML::Node Auto_aim::readYAML(const std::string& filename){
    try {
        YAML::Node config = YAML::LoadFile(filename);
        compensate_pitch = config["Transmit"]["compensate_pitch"].as<double>();
        compensate_yaw = config["Transmit"]["compensate_yaw"].as<double>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()) + "Auto_aim");
    }
    return YAML::LoadFile(filename);
}




bool Auto_aim::ShootAble()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    double now = (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
    if (now > m_next_shoot_time)
    {
        m_next_shoot_time = now + 0.02;
        return true;
    }
    return false;
}

void Auto_aim::CheckAimer(Armor armor){
    if (std::isnan(armor.cam_pose.x) || std::isnan(armor.cam_pose.y) || std::isnan(armor.cam_pose.z)||
    std::isnan(-armor.cam_pose.x) || std::isnan(-armor.cam_pose.y) || std::isnan(-armor.cam_pose.z)) 
    {
        ReEKF();
    }
}