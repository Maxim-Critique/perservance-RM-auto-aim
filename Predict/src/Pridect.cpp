#include "Pridict.h"
#include <fstream>


KinematicModel::KinematicModel()
:   index(0)
{
    height.clear();
    radius.clear();
}
KinematicModel::~KinematicModel()
{}

StandardModel::StandardModel()
:   StandardModel(Eigen::Matrix<double, 10, 1>::Zero())
{}
StandardModel::StandardModel(const Eigen::Matrix<double, 10, 1> &X)
:   KinematicModel()
{
    height.resize(2);
    radius.resize(2);
    number = 4;

    center = cv::Point2d(X(0), X(2));        
    velocity = cv::Point2d(X(1), X(3));
    height[0] = X(4);     //z1
    height[1] = X(5);     //z2
    radius[0] = X(6);     //r1
    radius[1] = X(7);     //r2
    phase = X(8);         //yaw
    palstance = X(9);     //角速度
}
StandardModel::~StandardModel()
{
    last_execution_time = std::chrono::steady_clock::time_point::min();;
    Window_time = -1;
}

StandardModel StandardModel::operator=(const StandardModel &status)
{
    int tmp_index = this->index;
    *this = status;
    this->index = tmp_index;
    return *this;
}

Armors StandardModel::getArmors(double predict_time)
{
    Armors armors(number);
    for (int i = 0; i < number; i++)
    {
        double angle = _std_radian(phase + palstance * predict_time + i * PI / 2);
        cv::Point2d point = center + velocity * predict_time +
                            cv::Point2d(radius[i % 2] * cos(angle), radius[i % 2] * sin(angle));
        armors[i].abs_pose = cv::Point3d(point.x, point.y, height[i % 2]);
        armors[i].predict_yaw = angle;
    }
    return armors;
}

Armor StandardModel::getClosestArmor(double predict_time,int &last_index,int &frames_since_last_switch)
{   
    std::unique_lock<std::mutex> lock(ds_mutex);
    double yaw = ds.camera_yaw_angle;
    lock.unlock();
    int SWITCH_FRAME_INTERVAL = 4;
    bool should_switch = false;
    Armors armors = getArmors(predict_time);
    bool flag = 0;
    // 查找符合条件的装甲板
    for (int i = 0; i < number; i++) 
    {
        double angle = _std_radian(armors[i].predict_yaw - PI - yaw);
        if (abs(angle) <= PI/4) 
        {
            if(abs(angle) > D2R(30)){
                if(angle > 0 && (abs(palstance) / palstance) == 1){
                    if(i == 0){
                        index = 3;
                    }else{
                        index = i - 1;
                    }
                }
                else if(angle < 0 && (abs(palstance) / palstance) == -1){
                    index = (i + 1) % number;
                }
                else{
                    index = i;
                }
            }else{
                index = i;
            } 
            // cout<<" index "<<index<<" last_index "<<last_index<< " AfterChange ";
            should_switch = (frames_since_last_switch >= SWITCH_FRAME_INTERVAL);
            if(index != last_index){
                if(should_switch){
                    frames_since_last_switch = 0;
                    last_index = index;
                    flag = 1;
                }else{
                    index = last_index;
                    frames_since_last_switch ++;
                }
            }else{
                frames_since_last_switch ++;
            }
            break;
        }
    }
    // cout<<" index "<<index<<" last_index "<<last_index<<" frames_since_last_switch "<<frames_since_last_switch<<"   ";
    std::ofstream out("ekf_data.txt", std::ios::app); 
    out << flag << "\n";
    armors[index].yaw = phase;
    return armors[index];
}

Armor StandardModel::getFacingArmor(double predict_time,int &last_index,int &frames_since_last_switch,Point3d &center_pose)
{
    std::unique_lock<std::mutex> lock(ds_mutex);
    double yaw = ds.camera_yaw_angle;
    lock.unlock();
    int SWITCH_FRAME_INTERVAL = 4;
    bool should_switch = false;
    Armors armors = getArmors(predict_time);
    bool flag = 0;
    // 查找符合条件的装甲板
    for (int i = 0; i < number; i++) 
    {
        double angle = _std_radian(armors[i].predict_yaw - PI - yaw);
        if (abs(angle) <= PI/4) 
        {
            if(abs(angle) > D2R(30)){
                if(angle > 0 && (abs(palstance) / palstance) == 1){
                    if(i == 0){
                        index = 3;
                    }else{
                        index = i - 1;
                    }
                }
                else if(angle < 0 && (abs(palstance) / palstance) == -1){
                    index = (i + 1) % number;
                }
                else{
                    index = i;
                }
            }else{
                index = i;
            } 
            // cout<<" index "<<index<<" last_index "<<last_index<< " AfterChange ";
            should_switch = (frames_since_last_switch >= SWITCH_FRAME_INTERVAL);
            if(index != last_index){
                if(should_switch){
                    frames_since_last_switch = 0;
                    last_index = index;
                    flag = 1;
                }else{
                    index = last_index;
                    frames_since_last_switch ++;
                }
            }else{
                frames_since_last_switch ++;
            }
            break;
        }
    }
    // cout<<" index "<<index<<" last_index "<<last_index<<" frames_since_last_switch "<<frames_since_last_switch<<"   ";
    std::ofstream out("ekf_data.txt", std::ios::app); 
    out << flag << "\n";
    center_pose = Point3d(center.x+velocity.x*predict_time,center.y+velocity.y*predict_time,(height[0]+height[1])/2);
    armors[index].yaw = phase;
    return armors[index];
    
}

void StandardModel::print(std::string tag)
{
    std::string prefix = "[StandardModel] " + tag + " ";
    std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
    std::cout << prefix << "height: " << height[0] << " " << height[1] << std::endl;
    std::cout << prefix << "radius: " << radius[0] << " " << radius[1] << std::endl;
    std::cout << prefix << "angle: " << _std_radian(phase)*180/PI << " += " << _std_radian(palstance)*180/PI << std::endl;
    std::cout << prefix << "index: " << index << std::endl;
}

