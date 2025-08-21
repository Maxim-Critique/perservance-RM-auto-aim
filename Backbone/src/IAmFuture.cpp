#include "IAmFuture.h"

Auto_aim::Auto_aim(){
    readYAML(Yaml);
    find_ptr = std::make_unique<Tradition>();
}

int Auto_aim::IAmFuture(Mat &img,uint64_t timestamp){
   
    //尝试次数大于20重置
    if(try_times == 20){ReEKF();}
    //传统找装甲板
    Armors armors = find_ptr->Find_light(img,roi);
    //MLP神经网络
    classifier.extractNumbers(img,armors);
    classifier.classify(armors);
    if(armors.empty()){
        Reboot();
        return 0;
    }
    armors = ChooseProcess(armors,img);
    if(!armors.size()){
        Reboot();
        return 0;
    }
    UpdateTimeInterval(timestamp);
    if(time_interval == 0){
        Reboot();
        return 0;
    }
    Armor armor = aim_ptr->Shoot(armors,img,time_interval);
    Transmit(armor);

}

void Auto_aim::UpdateTimeInterval(uint64_t timestamp){
    time_interval = (timestamp - last_time)/1e8;
    last_time = timestamp;
    //跑视频
    time_interval = 0.006;
}

void Auto_aim::Transmit(const Armor &armor){
    try_times = 0;
    gimbal_angle = C_C.GetAngle(armor.cam_pose);
     
    if(Protection()){
        int32_t yaw = (R2D(gimbal_angle.yaw) + compensate_yaw) * 1e4;
        int32_t pitch = (R2D(gimbal_angle.pitch) + compensate_pitch) * 1e4;
        std::unique_lock<std::mutex> lock(ds_mutex);
        bool yaw_delta = abs(R2D(gimbal_angle.yaw) + compensate_yaw - R2D(ds.camera_yaw_angle)) < 1;
        bool pitch_delta = abs(R2D(gimbal_angle.pitch)  + compensate_pitch - R2D(ds.camera_pit_angle)) < 1;
        lock.unlock();
        bool distance =  GetDistance(armor.cam_pose)>1.5&&GetDistance(armor.cam_pose)<4;
        int distance_delta = distance*10 + aim_ptr->control_model;
        
        if(aim_ptr->control_model == 3){
            Gimbal_Angle center = C_C.GetAngle(C_C.abs2cam(aim_ptr->predict_point));
            yaw = (R2D(center.yaw) + compensate_yaw) * 1e4;
            pitch = (R2D(center.pitch) + compensate_pitch) * 1e4;
        }
        if(yaw_delta && pitch_delta)
        {
            dg.Vision2Control(yaw,pitch,aim_ptr->ui.x*100,aim_ptr->ui.y*100,distance_delta,1);
        }
        else
        {
           dg.Vision2Control(yaw,pitch,aim_ptr->ui.x*100,aim_ptr->ui.y*100,distance_delta,0);
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
    if(aim_id.empty()){
        for(auto armor : armors){
            solveYawPnP(armor,img,initial_abs);
            if(GetDistance(armor.cam_pose)<5){
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
                        solveYawPnP(armor,img,initial_abs);
                        after_armors.push_back(armor);
                    }
                }else{
                    solveYawPnP(armor,img,initial_abs);
                    after_armors.push_back(armor);
                }
            }
            if(after_armors.size()>2){
                break;
            }
        }
    }

    return after_armors;
}

//防甩头
bool Auto_aim::Protection(){
    // cout<<R2D(initial_abs_yaw)<<"       "<<R2D(gimbal_angle.yaw)<<endl;
    bool sign = abs(initial_abs.yaw - gimbal_angle.yaw) < PI/6;
    return sign;
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
    if (x == "1") {
        return 1;
    } else if (x == "3") {
        return 3;
    } else if (x == "guard") {
        return 7;
    } else {
        // 如果输入的字符串不匹配，可以返回一个默认值，比如 -1
        return -1;
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
        throw std::runtime_error("YAML Error: " + std::string(e.what()));
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